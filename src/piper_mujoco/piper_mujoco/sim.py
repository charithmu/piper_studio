"""PiperSim: an ArmBackend on MuJoCo. Position actuators, same MJCF and servo gains as the ROS MuJoCo backend (data/ is generated from it)."""

from __future__ import annotations

import math
import time
from importlib import resources
from pathlib import Path

import numpy as np

import mujoco
import piper_assets
from piper_core.backend import (ARM_JOINTS, MAX_STEP, ArmState, BackendError, ExecResult, Outcome, Trajectory, check_start, check_trajectory,
                                resample)
from piper_core.guard import GuardConfig, check_command

GRIPPER = "gripper"
FINGERS = (("gripper_joint1", 0.5), ("gripper_joint2", -0.5))      # mimic joints of the description: multiplier x gripper width
SETTLE_S, SETTLE_TOL = 3.0, 1e-4                                   # after the last setpoint: hold it until within 0.1 mrad, at most 3 s


def data_dir() -> Path:
    path = Path(str(resources.files("piper_mujoco") / "data"))
    if not (path / "manifest.json").exists():
        raise FileNotFoundError(f"{path} has no generated data: run tools/release/build_mujoco.py (needs the ROS environment) or install the wheel")
    return path


class PiperSim:
    joint_names = list(ARM_JOINTS)

    def __init__(self, scene: str = "empty", gripper_width: float = 0.07, physics_dt: float = 0.001, ctrl_hz: float = 500.0,
                 realtime: bool = False, render: bool = False):
        if scene not in ("empty", "table"):
            raise ValueError("scene must be 'empty' or 'table'")
        self.limits_info = piper_assets.limits()
        if not 0.0 <= gripper_width <= self.limits_info.gripper_stroke:
            raise ValueError(f"gripper_width must be in [0, {self.limits_info.gripper_stroke}]")
        self.n_sub = round(1.0 / (ctrl_hz * physics_dt))
        if self.n_sub < 1 or abs(self.n_sub * ctrl_hz * physics_dt - 1.0) > 1e-6:
            raise ValueError("1 / (ctrl_hz * physics_dt) must be a whole number of physics steps")
        self.scene, self.ctrl_dt, self.realtime, self.render_enabled = scene, 1.0 / ctrl_hz, realtime, render
        self.gripper_width = gripper_width
        self.model = mujoco.MjModel.from_xml_path(str(data_dir() / "model" / f"scene_{scene}.xml"))
        self.model.opt.timestep = physics_dt
        self.data = mujoco.MjData(self.model)
        self.limits = dict(self.limits_info.hard)
        self.margin = self.limits_info.margin
        self._qadr = [self.model.joint(j).qposadr[0] for j in ARM_JOINTS]
        self._vadr = [self.model.joint(j).dofadr[0] for j in ARM_JOINTS]
        self._act = [self.model.actuator(j).id for j in ARM_JOINTS]
        self._grip_act = self.model.actuator(GRIPPER).id
        self._geom_names = {}
        link1 = self.model.body("link1").id            # the arm: link1 and everything below it (the fused base is world, never a contact partner)
        self._arm_body = np.zeros(self.model.nbody, dtype=bool)
        for b in range(self.model.nbody):
            k = b
            while k > 0 and k != link1:
                k = self.model.body_parentid[k]
            self._arm_body[b] = k == link1
        self._renderer = None
        self._wall0 = None
        self.reset()

    # ------------------------------------------------------------------ world
    @property
    def time(self) -> float:
        return float(self.data.time)

    def reset(self) -> None:
        """Back to the zero pose with the gripper at the constructor's width and every object where it started."""
        mujoco.mj_resetData(self.model, self.data)
        zero = piper_assets.zero_pose()
        for a, q in zip(self._qadr, zero):
            self.data.qpos[a] = q
        self._set_gripper_qpos(self.gripper_width)
        self.data.ctrl[self._act] = zero
        self.data.ctrl[self._grip_act] = self.gripper_width
        mujoco.mj_forward(self.model, self.data)
        self._wall0 = None

    def _set_gripper_qpos(self, width: float) -> None:
        self.data.qpos[self.model.joint(GRIPPER).qposadr[0]] = width
        for name, mult in FINGERS:
            self.data.qpos[self.model.joint(name).qposadr[0]] = mult * width

    def object_poses(self) -> dict:
        """{name: (position (3,), quaternion wxyz (4,))} of the free objects, in the base frame (empty in the 'empty' scene)."""
        out = {}
        for i in range(self.model.njnt):
            if self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                b = self.model.jnt_bodyid[i]
                out[self.model.body(b).name] = (self.data.xpos[b].copy(), self.data.xquat[b].copy())
        return out

    def render(self, width: int = 640, height: int = 480, camera: str | int = -1) -> np.ndarray:
        """RGB image (height, width, 3). Needs an OpenGL backend (MUJOCO_GL=egl or osmesa on a machine without a display)."""
        if self._renderer is None or (self._renderer.width, self._renderer.height) != (width, height):
            self._renderer = mujoco.Renderer(self.model, height, width)
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    # ------------------------------------------------------------------ lifecycle (ArmBackend)
    def start(self) -> Outcome:
        self.reset()
        return Outcome(True, "ready: at zero")

    def stop(self) -> Outcome:
        r = self.move_to(piper_assets.zero_pose())
        return Outcome(r.ok, "at zero" if r.ok else r.message)

    def state(self) -> ArmState:
        q = np.array([self.data.qpos[a] for a in self._qadr])
        dq = np.array([self.data.qvel[v] for v in self._vadr])
        return ArmState(self.time, q, dq, self.time, self.gripper_opening(), math.nan)

    def hold(self) -> None:
        self.data.ctrl[self._act] = self.state().q

    # ------------------------------------------------------------------ stepping
    def _step(self, contacts: list | None = None, t_log: float = 0.0) -> None:
        for _ in range(self.n_sub):
            mujoco.mj_step(self.model, self.data)
        if contacts is not None:
            self._collect_contacts(contacts, t_log)
        if self.realtime:
            if self._wall0 is None:
                self._wall0 = time.monotonic() - self.time + self.ctrl_dt
            ahead = self._wall0 + self.time - time.monotonic()
            if ahead > 0:
                time.sleep(ahead)

    def _geom_name(self, g: int) -> str:
        if g not in self._geom_names:
            name = self.model.geom(g).name
            self._geom_names[g] = name or f"{self.model.body(self.model.geom_bodyid[g]).name}#{g}"
        return self._geom_names[g]

    def _collect_contacts(self, contacts: list, t: float) -> None:
        """Penetrations that involve the arm (self-collision, arm-table, arm-object); objects resting on the table are not logged."""
        for c in self.data.contact[: self.data.ncon]:
            if c.dist < 0.0 and (self._arm_body[self.model.geom_bodyid[c.geom1]] or self._arm_body[self.model.geom_bodyid[c.geom2]]):
                contacts.append((t, self._geom_name(c.geom1), self._geom_name(c.geom2), float(-c.dist)))

    def _guard(self, q) -> str | None:
        measured = dict(zip(ARM_JOINTS, self.state().q))
        v = check_command(ARM_JOINTS, list(map(float, q)), self.limits, measured, GuardConfig(ARM_JOINTS, MAX_STEP, self.margin))
        return None if v.ok else v.reason

    # ------------------------------------------------------------------ commands
    def command(self, q, dq=None) -> None:
        """One setpoint through the guard, then one control period of simulation. BackendError if the guard refuses it."""
        reason = self._guard(q)
        if reason:
            raise BackendError(reason)
        self.data.ctrl[self._act] = np.asarray(q, dtype=float)
        self._step()

    def gripper_opening(self) -> float:
        """The measured gripper opening in metres (0 closed, 0.07 open)."""
        return float(self.data.qpos[self.model.joint(GRIPPER).qposadr[0]])

    def gripper_force(self) -> float:
        """NaN: the simulated gripper has no force read-back (the real one does, about -2 N at 3 N commanded: docs/real_arm/arms/piper-01.md)."""
        return math.nan

    def set_gripper(self, width: float, effort: float | None = None, timeout: float | None = None) -> float:
        """Move the gripper to `width` (0 to the stroke) and wait until it has settled (at most `timeout`, default 2 s). `effort` is ignored by the simulator. Returns the measured width."""
        if not 0.0 <= width <= self.limits_info.gripper_stroke:
            raise BackendError(f"gripper width {width} outside [0, {self.limits_info.gripper_stroke}]")
        self.data.ctrl[self._grip_act] = width
        gq = self.model.joint(GRIPPER).qposadr[0]
        for _ in range(int((2.0 if timeout is None else timeout) / self.ctrl_dt)):
            self._step()
            if abs(self.data.qpos[gq] - width) < 1e-4 and abs(self.data.qvel[self.model.joint(GRIPPER).dofadr[0]]) < 1e-3:
                break
        return self.gripper_opening()

    def move_to(self, q, duration: float | None = None) -> ExecResult:
        """A minimum-jerk move from the present pose to `q` (our own convenience move; planner trajectories go through execute())."""
        q0, q1 = self.state().q, np.asarray(q, dtype=float)
        dur = duration or max(1.0, 1.875 * float(np.max(np.abs(q1 - q0))) / 1.0)      # peak speed of a minimum-jerk move = 1.875 x distance / duration
        t = np.arange(0.0, dur + 1e-9, 0.02)
        s = t / t[-1]
        pos, vel = 10 * s**3 - 15 * s**4 + 6 * s**5, (30 * s**2 - 60 * s**3 + 30 * s**4) / t[-1]
        d = (q1 - q0)[None, :]
        return self.execute(Trajectory(list(ARM_JOINTS), t, q0 + pos[:, None] * d, vel[:, None] * d))

    def execute(self, traj, *, margin_percent: float = 20.0, blocking: bool = True) -> ExecResult:
        """Run a trajectory as given (see piper_core.backend). `margin_percent` is ignored: the setpoints go straight to the position actuators."""
        if not blocking:
            raise NotImplementedError("PiperSim.execute is blocking only")
        try:
            traj = traj if isinstance(traj, Trajectory) else Trajectory.from_pairs(traj)
        except (ValueError, TypeError) as e:
            return ExecResult(False, "bad_trajectory", str(e))
        if list(traj.joint_names) != list(ARM_JOINTS):
            return ExecResult(False, "bad_trajectory", f"joint_names must be {list(ARM_JOINTS)}")
        refused = (check_trajectory(traj, self.limits, margin=self.margin, max_step=MAX_STEP, max_velocity=self.limits_info.velocity,
                                    rate_hz=1.0 / self.ctrl_dt)
                   or check_start(traj, self.state().q))
        if refused:
            return ExecResult(False, *refused)
        grid, setpoints = resample(traj, 1.0 / self.ctrl_dt)
        t0, log_t, log_c, log_m, contacts, events = self.time, [], [], [], [], []
        for t, q in zip(grid, setpoints):
            reason = self._guard(q)
            if reason:
                self.hold()
                events.append((float(t), "guard:rejected", reason))
                return ExecResult(False, "guard:rejected", f"setpoint at t={t:.3f}: {reason}; holding", np.array(log_t),
                                  np.array(log_c).reshape(-1, 6), np.array(log_m).reshape(-1, 6), events, contacts)
            self.data.ctrl[self._act] = q
            self._step(contacts, float(t))
            log_t.append(float(t)); log_c.append(q.copy()); log_m.append(self.state().q)
        goal = setpoints[-1]
        extra = grid[-1]
        for _ in range(int(SETTLE_S / self.ctrl_dt)):                 # keep holding the last setpoint until the servos have arrived
            if float(np.max(np.abs(self.state().q - goal))) < SETTLE_TOL and float(np.max(np.abs(self.state().dq))) < 1e-3:
                break
            extra += self.ctrl_dt
            self._step(contacts, float(extra))
            log_t.append(float(extra)); log_c.append(goal.copy()); log_m.append(self.state().q)
        return ExecResult(True, "done", "", np.array(log_t), np.array(log_c), np.array(log_m), events, contacts)
