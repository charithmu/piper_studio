"""The real Piper without ROS: the same rules as the ROS stack, straight on the vendor SDK (pyAgxArm), for planners such as cuRobo.

    arm = RealArm.connect("can0")          # one firmware query; picks the SDK profile for the arm's firmware
    arm.start()                            # preflight, motors on, leave the slumped pose for zero, check zero
    arm.execute(trajectory)                # [(t_seconds, [q1..q6]), ...] from a planner, every point through the guard
    arm.stop()                             # zero, wait, park, wait, motors off

The rules are the ones `command_guard` applies (`piper_core.check_command`, `zero_pose`, `classify_state`): nothing is commanded outside
the limits, nothing from the slumped pose, nothing while the arm reports a fault; the only way out of the slumped pose is the one checked
move in `start()`. Joint names are the official ones (joint1 .. joint6), angles are radians.

NOT YET RUN ON THE REAL ARM. It is written against the SDK calls `tools/real_arm/arm_zero.py` used on 2026-10-14 and tested against a
fake SDK arm (test_real_arm.py). The first run needs the user at the arm: `execute()` streams one `move_j` per trajectory point, which
the firmware smooths (documented as streamable in docs/real_arm/REAL_ARM.md, never tried with 25 ms points); the speed percent derived
from the trajectory (assumes 100 % = the firmware's 3.0 rad/s) is unverified.
"""

from __future__ import annotations

import math
import time

import numpy as np

from piper_core.backend import (ARM_JOINTS, MAX_STEP, ArmState, BackendError, ExecResult, Outcome, Trajectory, check_start,
                               check_trajectory)
from piper_core.guard import GuardConfig, check_command
from piper_core.poses import classify_state, zero_pose

JOINTS = tuple(f"joint{i}" for i in range(1, 7))
PARK_JOINT5 = 0.3840           # rad (22.0 deg): park = zero with joint 5 lowered (SRDF `park`, test_named_poses)
FIRMWARE_MAX_VELOCITY = 3.0    # rad/s, read from the arm (piper-01.readout.json)
DONE_TOLERANCE = 0.01          # rad (0.57 deg): "at the goal"
DONE_HOLD_S = 1.0
MODE_J = 1                     # mode_feedback value for CAN control + MOVE_J
DRIVER_FAULT_BITS = ("voltage_too_low", "motor_overheating", "driver_overcurrent", "driver_overheating",
                     "collision_status", "driver_error_status", "stall_status")


class RealArmError(BackendError):
    """A refused or aborted action. The arm is left holding (motors on) unless the message says otherwise."""


class RealArm:
    def __init__(self, sdk_arm, limits: dict[str, tuple[float, float]] | None = None, margin: float = 0.01, speed_percent: int = 30,
                 clock=time.monotonic, sleep=time.sleep):
        """`sdk_arm` is a connected pyAgxArm arm. `limits` {joint1: (lo, hi), ...} default to the limits stored in the arm."""
        self.sdk, self.margin, self.speed_percent = sdk_arm, margin, speed_percent
        self._now, self._sleep = clock, sleep
        self.limits = limits or self._stored_limits()
        self.enabled = False

    @classmethod
    def connect(cls, canbus: str = "can0", **kwargs) -> "RealArm":
        """Open the arm with the SDK profile that matches its firmware (one firmware query, no motion)."""
        from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config, resolve_firmware_profile
        probe = AgxArmFactory.create_arm(create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel=canbus))
        probe.connect()
        try:
            time.sleep(0.5)
            fw = probe.get_firmware(timeout=3.0)
        finally:
            probe.disconnect()
        if not fw:
            raise RealArmError("no firmware answer (arm silent, another program on the bus, master mode or teach playback?)")
        profile = resolve_firmware_profile(ArmModel.PIPER, fw["software_version"])
        sdk = AgxArmFactory.create_arm(create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=profile, channel=canbus))
        sdk.connect()
        time.sleep(0.5)
        return cls(sdk, **kwargs)

    def close(self):
        self.sdk.disconnect()

    # ------------------------------------------------------------------ reads (send nothing)
    def _stored_limits(self) -> dict[str, tuple[float, float]]:
        out = {}
        for i, name in enumerate(JOINTS, 1):
            r = self.sdk.get_joint_angle_vel_limits(i, timeout=2.0)
            if r is None:
                raise RealArmError(f"stored limits of {name} unreadable")
            out[name] = (float(r.msg.min_angle_limit), float(r.msg.max_angle_limit))
        return out

    def positions(self) -> dict[str, float] | None:
        ja = self.sdk.get_joint_angles()
        return None if ja is None else dict(zip(JOINTS, (float(a) for a in ja.msg)))

    def problems(self) -> list[str]:
        """Everything that must stop a motion: no feedback, status or error code, driver fault bits, teach or master mode."""
        out = []
        st = self.sdk.get_arm_status()
        if st is None or self.positions() is None:
            return ["no joint or status feedback"]
        m = st.msg
        if int(m.ctrl_mode) not in (0, 1):
            out.append(f"control mode {int(m.ctrl_mode)} (not standby or CAN control)")
        if int(m.teach_status) != 0:
            out.append(f"teach state {int(m.teach_status)}")
        if int(m.arm_status) != 0 or int(m.err_code) != 0:
            out.append(f"arm_status={int(m.arm_status)} err_code={int(m.err_code)}")
        for i in range(1, 7):
            d = self.sdk.get_driver_states(i)
            if d is not None:
                out += [f"joint{i}:{k}" for k in DRIVER_FAULT_BITS if getattr(d.msg.foc_status, k, False)]
        return out

    def pose_state(self) -> str:
        """'zero' | 'valid' | 'slumped' | 'invalid' (piper_core.classify_state on the measured joints)."""
        return classify_state(self.positions() or {}, self.limits, JOINTS, self.margin, DONE_TOLERANCE)

    def zero(self) -> dict[str, float]:
        return zero_pose(self.limits, JOINTS, self.margin)

    def park(self) -> dict[str, float]:
        return {**self.zero(), "joint5": PARK_JOINT5}

    # ------------------------------------------------------------------ the guard, as the ROS stack applies it
    def _guard(self, target: dict[str, float], max_step: float = math.inf) -> Outcome:
        measured = self.positions()
        if measured is None:
            return Outcome(False, "no joint feedback")
        v = check_command(JOINTS, [target[j] for j in JOINTS], self.limits, measured, GuardConfig(JOINTS, max_step, self.margin))
        return Outcome(v.ok, v.reason, {"positions": v.positions})

    # ------------------------------------------------------------------ control
    def _configure(self, speed_percent: int) -> Outcome:
        self.sdk.set_speed_percent(int(min(max(speed_percent, 1), 100)))
        self.sdk.set_motion_mode(self.sdk.OPTIONS.MOTION_MODE.J)
        t0 = self._now()
        while self._now() - t0 < 2.5:
            st = self.sdk.get_arm_status()
            if st is not None and int(st.msg.mode_feedback) == MODE_J and int(st.msg.ctrl_mode) == 1:
                return Outcome(True)
            self._sleep(0.05)
        return Outcome(False, "the arm did not report CAN control + MOVE_J")

    def enable(self) -> Outcome:
        t0 = self._now()
        while self._now() - t0 < 5.0 and not self.sdk.enable():
            self._sleep(0.05)
        self.enabled = any(self.sdk.get_joint_enable_status(i) for i in range(1, 7))
        return Outcome(all(self.sdk.get_joint_enable_status(i) for i in range(1, 7)),
                       "" if self.enabled else "could not enable all six joints")

    def disable(self) -> Outcome:
        for _ in range(10):
            self.sdk.disable()
            if not any(self.sdk.get_joint_enable_status(i) for i in range(1, 7)):
                break
            self._sleep(0.1)
        self.enabled = any(self.sdk.get_joint_enable_status(i) for i in range(1, 7))
        return Outcome(not self.enabled, "motors off" if not self.enabled else "motors still enabled")

    def _hold(self):
        """Hold the present pose with move_j (inside the limits): used after any abort."""
        q = self.positions()
        if q:
            self.sdk.move_j([min(max(q[j], self.limits[j][0]), self.limits[j][1]) for j in JOINTS])

    def _wait_at(self, goal: dict[str, float], timeout: float = 60.0) -> Outcome:
        t0, since = self._now(), None
        while self._now() - t0 < timeout:
            self._sleep(0.1)
            bad = self.problems()
            if bad:
                self._hold()
                return Outcome(False, "aborted, holding: " + "; ".join(bad))
            q = self.positions()
            err = max(abs(q[j] - goal[j]) for j in JOINTS)
            since = (since or self._now()) if err < DONE_TOLERANCE else None
            if since and self._now() - since >= DONE_HOLD_S:
                return Outcome(True, "reached", {"error": err})
        self._hold()
        return Outcome(False, f"goal not reached within {timeout:.0f} s, holding")

    def _move(self, target: dict[str, float], speed_percent: int | None = None) -> Outcome:
        """One checked point-to-point move (firmware interpolation) from a VALID or ZERO arm. Never from the slumped pose (use start())."""
        if self.pose_state() not in ("zero", "valid"):
            return Outcome(False, f"the arm is {self.pose_state()}: call start() first")
        bad = self.problems()
        if bad:
            return Outcome(False, "; ".join(bad))
        g = self._guard(target)
        if not g:
            return g
        cfg = self._configure(speed_percent or self.speed_percent)
        if not cfg:
            return cfg
        self.sdk.move_j([g.details["positions"][k] for k in range(6)])
        return self._wait_at(dict(zip(JOINTS, g.details["positions"])))

    def start(self, speed_percent: int | None = None) -> Outcome:
        """Preflight, motors on, leave the slumped pose by one checked move to zero, check zero. Success: enabled, valid, at zero."""
        bad = self.problems()
        if bad:
            return Outcome(False, "not starting: " + "; ".join(bad))
        state = self.pose_state()
        if state == "invalid":
            return Outcome(False, "not starting: the arm is outside its limits by more than a slump or has no feedback")
        cfg = self._configure(speed_percent or self.speed_percent)
        if not cfg:
            return cfg
        en = self.enable()
        if not en:
            return en
        if state == "slumped":          # the one move that is allowed outside the limits: its target is zero
            self.sdk.move_j([self.zero()[j] for j in JOINTS])
            r = self._wait_at(self.zero())
            if not r:
                return r
        elif state == "valid":
            r = self._move(self.zero(), speed_percent)
            if not r:
                return r
        return Outcome(self.pose_state() == "zero", "ready: enabled, at zero" if self.pose_state() == "zero" else "not at zero after start")

    def stop(self, speed_percent: int | None = None, settle: float = 1.0) -> Outcome:
        """Zero, check, wait, park, wait, motors off. An arm that is already slumped is left alone."""
        if self.pose_state() == "slumped":
            return Outcome(True, "already powered off and slumped")
        r = self._move(self.zero(), speed_percent)
        if not r:
            return Outcome(False, f"zero: {r.message}; motors left on")
        self._sleep(settle)
        if self.pose_state() != "zero":
            return Outcome(False, "the arm did not stay at zero; motors left on")
        r = self._move(self.park(), speed_percent)
        if not r:
            return Outcome(False, f"park: {r.message}; motors left on")
        self._sleep(settle)
        return self.disable()

    # ------------------------------------------------------------------ ArmBackend (piper_core.backend)
    joint_names = list(JOINTS)

    def state(self) -> ArmState:
        """The measured joints as an ArmState. dq is NaN: the SDK feedback used here has no velocity."""
        q = self.positions()
        if q is None:
            raise RealArmError("no joint feedback")
        now = self._now()
        ja = self.sdk.get_joint_angles()
        stamp = float(getattr(ja, "timestamp", None) or now)
        return ArmState(now, np.array([q[j] for j in JOINTS]), np.full(6, np.nan), stamp)

    def hold(self) -> None:
        self._hold()

    def command(self, q, dq=None) -> None:
        """One setpoint through the guard (limits, margin, step from the measured pose, faults). BackendError if it is refused."""
        if self.pose_state() not in ("zero", "valid"):
            raise RealArmError(f"the arm is {self.pose_state()}: call start() first")
        bad = self.problems()
        if bad:
            raise RealArmError("; ".join(bad))
        g = self._guard(dict(zip(JOINTS, map(float, q))), max_step=MAX_STEP)
        if not g:
            raise RealArmError(g.message)
        self.sdk.move_j(g.details["positions"])

    def move_to(self, q, duration: float | None = None) -> ExecResult:
        """One checked point-to-point move (firmware interpolation) from a valid arm. `q`: six values or {joint: value}."""
        target = dict(q) if isinstance(q, dict) else dict(zip(JOINTS, map(float, q)))
        pct = None
        if duration:
            now = self.positions()
            if now:
                pct = math.ceil(100.0 * max(abs(target[j] - now[j]) for j in JOINTS) / duration / FIRMWARE_MAX_VELOCITY)
        r = self._move(target, pct)
        qn = self.positions() or {}
        return ExecResult(r.ok, "done" if r else "guard:rejected", "" if r else r.message, np.array([self._now()]),
                          np.array([[target[j] for j in JOINTS]]), np.array([[qn.get(j, np.nan) for j in JOINTS]]))

    def execute(self, traj, *, margin_percent: float = 20.0, blocking: bool = True) -> ExecResult:
        """Follow a trajectory with one move_j per trajectory point (see the module docstring for the rules and the caveat).

        The whole trajectory is checked before anything moves (`check_trajectory`: limits, margin, step, velocity; `check_start`);
        nothing is clamped. While it runs, every point also passes the guard against the MEASURED pose (step of at most 0.15 rad,
        faults) and a rejection stops and holds. The firmware speed percent is the fastest segment (100 % = 3.0 rad/s, unverified)
        plus `margin_percent`. `traj` may also be [(t, [q1..q6]), ...]. Logs the commanded and measured joints at each point.
        """
        if not blocking:
            raise NotImplementedError("RealArm.execute is blocking only")
        try:
            traj = traj if isinstance(traj, Trajectory) else Trajectory.from_pairs(traj)
        except (ValueError, TypeError) as e:
            return ExecResult(False, "bad_trajectory", str(e))
        if list(traj.joint_names) != list(JOINTS):
            return ExecResult(False, "bad_trajectory", f"joint_names must be {list(JOINTS)}")
        if self.pose_state() not in ("zero", "valid"):
            return ExecResult(False, "not_ready", f"the arm is {self.pose_state()}: call start() first")
        bad = self.problems()
        if bad:
            return ExecResult(False, "fault", "; ".join(bad))
        refused = check_trajectory(traj, self.limits, margin=self.margin, max_step=MAX_STEP, max_velocity=FIRMWARE_MAX_VELOCITY)
        if refused:
            return ExecResult(False, *refused)
        start = self.positions()
        refused = check_start(traj, [start[j] for j in JOINTS])
        if refused:
            return ExecResult(False, *refused)
        seg = np.abs(np.diff(traj.q, axis=0)).max(axis=1) / np.diff(traj.t)
        cfg = self._configure(math.ceil(100.0 * float(seg.max()) / FIRMWARE_MAX_VELOCITY * (1 + margin_percent / 100.0)) or 1)
        if not cfg:
            return ExecResult(False, "fault", cfg.message)
        log_t, log_c, log_m, events = [], [], [], []

        def result(ok, reason, detail=""):
            return ExecResult(ok, reason, detail, np.array(log_t), np.array(log_c).reshape(-1, 6), np.array(log_m).reshape(-1, 6), events)

        t_start = self._now()
        for t, q in zip(traj.t[1:], traj.q[1:]):
            wait = t_start + t - self._now()
            if wait > 0:
                self._sleep(wait)
            bad = self.problems()
            if bad:
                self._hold()
                events.append((float(t), "fault", "; ".join(bad)))
                return result(False, "fault", "aborted, holding: " + "; ".join(bad))
            g = self._guard(dict(zip(JOINTS, map(float, q))), max_step=MAX_STEP)
            if not g:
                self._hold()
                events.append((float(t), "guard:rejected", g.message))
                return result(False, "guard:rejected", f"point at t={t:.3f}: {g.message}; holding")
            self.sdk.move_j(g.details["positions"])
            now = self.positions() or {}
            log_t.append(self._now() - t_start)
            log_c.append(g.details["positions"])
            log_m.append([now.get(j, np.nan) for j in JOINTS])
        end = self._wait_at(dict(zip(JOINTS, map(float, traj.q[-1]))), timeout=10.0)
        if not end:
            events.append((float(traj.t[-1]), "end", end.message))
        return result(end.ok, "done" if end.ok else "timeout", end.message)
