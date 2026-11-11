"""PiperIsaacSim: an ArmBackend on Isaac Sim, without ROS. A thin client of the `isaac/run_server.py` process (Isaac's own Python environment, GPU).

The rules live HERE and in piper_core, not in the server: the whole trajectory is checked and resampled with `piper_core` (so accept and reject are the
same code as PiperSim and the real arm), the setpoints go to the server in chunks of CHUNK control steps, and between chunks the guard is applied against
the measured state. The server only applies setpoints, steps physics in simulation time and reports what it measured.
"""

from __future__ import annotations

import atexit
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import tempfile
import time
from importlib import resources
from pathlib import Path

import numpy as np

import piper_assets
from piper_core.backend import (ARM_JOINTS, MAX_STEP, ArmState, BackendError, ExecResult, Outcome, Trajectory, check_start, check_trajectory,
                                resample)
from piper_core.guard import GuardConfig, check_command

CHUNK = 25                                          # control steps per request (100 ms at 250 Hz): the guard sees the measured state at every chunk start
SETTLE_S, SETTLE_TOL, SETTLE_VEL = 3.0, 1e-4, 1e-3  # after the last setpoint: hold it until within 0.1 mrad and slower than 1 mrad/s, at most 3 s (as PiperSim)


def runner_dir() -> Path:
    """The Isaac-side files (world.py, run_server.py, servo_gains.py): $PIPER_ISAAC_RUNNER, else the repo's `isaac/` (a source checkout), else the copy inside the wheel."""
    for p in (os.environ.get("PIPER_ISAAC_RUNNER"), str(Path(__file__).resolve().parents[3] / "isaac"), str(resources.files("piper_isaac") / "runner")):
        if p and (Path(p) / "run_server.py").exists():
            return Path(p)
    raise FileNotFoundError("no Isaac runner: install the wheel, run from the repo, or set PIPER_ISAAC_RUNNER")


def _data_dirs() -> list:
    return [Path(str(resources.files("piper_isaac") / "data")), Path(os.environ.get("PIPER_ISAAC_DATA", Path.home() / "data/ml/isaac/piper_studio"))]


def usd_path(camera: bool = True) -> Path:
    """The Piper USD with the ZED X Mini bracket and camera (camera=True, the default) or the robot only: $PIPER_ISAAC_USD, else inside the wheel, else the build of
    `scripts/isaac.sh build-usd [no_camera]`."""
    rel = "usd/piper_isaac/piper_isaac.usda" if camera else "usd_no_camera/piper_isaac_no_camera/piper_isaac_no_camera.usda"
    cands = [os.environ.get("PIPER_ISAAC_USD")] + [str(d / rel) for d in _data_dirs()]
    for p in cands:
        if p and Path(p).exists():
            return Path(p)
    raise FileNotFoundError("no Piper USD: install the wheel or run `scripts/isaac.sh build-usd [no_camera]` (needs the ROS environment once)")


def camera_pose_path() -> Path:
    for d in _data_dirs():
        if (d / "camera_pose.json").exists():
            return d / "camera_pose.json"
    raise FileNotFoundError("no camera_pose.json: install the wheel or run `scripts/isaac.sh build-usd`")


class PiperIsaacSim:
    joint_names = list(ARM_JOINTS)

    def __init__(self, scene: str = "empty", gripper_width: float = 0.07, ctrl_hz: float = 250.0, physics_hz: float = 250.0, render: bool = False,
                 usd: str | Path | None = None, isaac_env: str | Path | None = None, connect: str | Path | None = None, startup_timeout: float = 240.0,
                 gui: bool = False, camera: bool = True):
        """Starts the Isaac server (one process per instance, through gpu-run) and connects. `connect` attaches to an already running server instead
        (tests). `isaac_env` is the script that activates Isaac's environment (default $ISAAC_ENV_SH, else ~/projects/sim/robosim/env.sh).
        `camera=True` (the default): the arm carries the ZED X Mini on its bracket and `camera_image()` renders from its left lens; `camera=False`: the robot only.
        `render=True` enables recording (`record_start`) and renders only at the video frame rate. `gui=True` opens the full editor and enables recording."""
        if scene not in ("empty", "table"):
            raise ValueError("scene must be 'empty' or 'table'")
        self.limits_info = piper_assets.limits()
        if not 0.0 <= gripper_width <= self.limits_info.gripper_stroke:
            raise ValueError(f"gripper_width must be in [0, {self.limits_info.gripper_stroke}]")
        self.scene, self.ctrl_hz, self.ctrl_dt, self.physics_hz, self.gripper_width = scene, float(ctrl_hz), 1.0 / ctrl_hz, float(physics_hz), gripper_width
        self.limits = dict(self.limits_info.hard)
        self.margin = self.limits_info.margin
        self.gui = gui
        self.camera_enabled = bool(camera)
        self.render_enabled = render or gui
        self._proc = self._sock = self._file = self._dir = None
        self._external = connect is not None
        atexit.register(self.close)
        try:
            if connect is None:
                self._launch(Path(usd) if usd else usd_path(self.camera_enabled), isaac_env, self.render_enabled, startup_timeout)
            else:
                self._connect(str(connect), startup_timeout)
            hello = self._rpc(cmd="hello")
        except Exception:
            self.close()
            raise
        self.server = hello
        self.contacts_supported = bool(hello.get("contacts"))
        self.reset()

    # ------------------------------------------------------------------ process
    def _launch(self, usd, isaac_env, render, timeout):
        env_sh = str(isaac_env or os.environ.get("ISAAC_ENV_SH") or Path.home() / "projects/sim/robosim/env.sh")
        if not Path(env_sh).exists():
            raise FileNotFoundError(f"Isaac environment script {env_sh} not found (set ISAAC_ENV_SH)")
        self._dir = tempfile.mkdtemp(prefix="pisaac_")           # short path (unix socket limit), also the cwd: Isaac drops ucode.elf there
        (Path(self._dir) / "scene.json").write_text(json.dumps(piper_assets.scene()))
        sock, pid = os.path.join(self._dir, "s"), os.path.join(self._dir, "pid")
        home = str(Path.home())
        display_env = [f"{k}={os.environ[k]}" for k in ("DISPLAY", "XAUTHORITY") if k in os.environ] if self.gui else []
        cmd = ["env", "-i", f"HOME={home}", f"PATH=/usr/bin:/bin:{home}/.local/bin", f"PIPER_ISAAC_PIDFILE={pid}", *display_env, "bash", "-c", 'source "$0"; exec gpu-run "$@"', env_sh,
               "python", str(runner_dir() / "run_server.py"), "--usd", str(usd), "--socket", sock, "--scene", self.scene, "--scene-json", os.path.join(self._dir, "scene.json"),
               "--zero", json.dumps(list(piper_assets.zero_pose())), "--gripper-width", str(self.gripper_width), "--ctrl-hz", str(self.ctrl_hz),
               "--physics-hz", str(self.physics_hz)] + (["--render"] if render else []) + (["--gui"] if self.gui else [])
        cmd += ["--camera", "zedxm", "--camera-pose", str(camera_pose_path())] if self.camera_enabled else ["--camera", "none"]
        self._log = open(os.path.join(self._dir, "server.log"), "w")
        self._pidfile = pid
        self._proc = subprocess.Popen(cmd, cwd=self._dir, stdout=self._log, stderr=subprocess.STDOUT, start_new_session=True)
        self._connect(sock, timeout)

    def _connect(self, path, timeout):
        t0 = time.time()
        while True:
            if self._proc is not None and self._proc.poll() is not None:
                raise BackendError(f"the Isaac server exited ({self._proc.returncode}); log tail:\n{self._log_tail()}")
            if os.path.exists(path):
                try:
                    s = socket.socket(socket.AF_UNIX)
                    s.connect(path)
                    self._sock, self._file = s, s.makefile("rwb")
                    return
                except OSError:
                    s.close()
            if time.time() - t0 > timeout:
                raise BackendError(f"no Isaac server after {timeout:.0f} s; log tail:\n{self._log_tail()}")
            time.sleep(0.2)

    def _log_tail(self, n=3000):
        try:
            return Path(self._dir, "server.log").read_text()[-n:]
        except Exception:  # noqa: BLE001
            return "(no log)"

    def _rpc(self, **c):
        self._file.write((json.dumps(c) + "\n").encode())
        self._file.flush()
        line = self._file.readline()
        if not line:
            raise BackendError(f"the Isaac server closed the connection; log tail:\n{self._log_tail()}")
        r = json.loads(line)
        if not r.get("ok"):
            raise BackendError(f"Isaac server: {r.get('error')}")
        return r

    def close(self) -> None:
        """Stop the server (quit, then SIGTERM to the PID it wrote, then SIGKILL), remove its run directory. Safe to call twice."""
        atexit.unregister(self.close)
        try:
            if self._file is not None and not self._external:
                self._rpc(cmd="quit")
        except Exception:  # noqa: BLE001
            pass
        for f in (self._file, self._sock):
            try:
                f.close()
            except Exception:  # noqa: BLE001
                pass
        self._file = self._sock = None
        if self._proc is not None:
            pid = None
            try:
                pid = int(Path(self._pidfile).read_text())
            except Exception:  # noqa: BLE001
                pass
            try:
                self._proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                for p in (pid, None):
                    try:
                        os.kill(p, signal.SIGTERM) if p else self._proc.terminate()
                    except Exception:  # noqa: BLE001
                        pass
                try:
                    self._proc.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    try:
                        os.kill(pid, signal.SIGKILL) if pid else self._proc.kill()
                    except Exception:  # noqa: BLE001
                        pass
            self._proc = None
            try:
                self._log.close()
            except Exception:  # noqa: BLE001
                pass
        if self._dir:
            shutil.rmtree(self._dir, ignore_errors=True)
            self._dir = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------ world
    @property
    def time(self) -> float:
        return float(self._rpc(cmd="state")["t"])

    def reset(self) -> None:
        """Back to the zero pose with the gripper at the constructor's width and every object where it started. Simulation time restarts at 0."""
        self._rpc(cmd="reset", zero=list(piper_assets.zero_pose()), width=self.gripper_width)

    # ------------------------------------------------------------------ wrist camera (the ZED X Mini's left lens)
    def camera_info(self) -> dict:
        """piper_assets.camera(): intrinsics K, width, height, hfov_deg, depth range, optical frame name and its pose in gripper_base / the TCP."""
        self._need_camera()
        return piper_assets.camera()

    def camera_pose(self) -> np.ndarray:
        """4x4 pose of the camera's optical frame (z forward, x right, y down) in the base frame, now."""
        self._need_camera()
        return np.array(self._rpc(cmd="camera_pose")["T"])

    def camera_image(self, depth: bool = False):
        """What the wrist camera sees: RGB (height, width, 3) uint8, or (rgb, depth) with depth in metres along the optical axis (NaN outside the lens's depth
        range). The size is the camera model's (960 x 600; K of `camera_info()`). Simulation time does not advance."""
        import base64
        self._need_camera()
        r = self._rpc(cmd="camera_image")
        h, w = r["height"], r["width"]
        rgb = np.frombuffer(base64.b64decode(r["rgb"]), dtype=np.uint8).reshape(h, w, 3).copy()
        if not depth:
            return rgb
        return rgb, np.frombuffer(base64.b64decode(r["depth"]), dtype=np.float32).reshape(h, w).copy()

    def _need_camera(self) -> None:
        if not self.camera_enabled:
            raise RuntimeError("this PiperIsaacSim was created with camera=False (the robot only)")

    def object_poses(self) -> dict:
        """{name: (position (3,), quaternion wxyz (4,))} of the free objects, in the base frame (empty in the 'empty' scene)."""
        return {n: (np.array(v["pos"]), np.array(v["quat_wxyz"])) for n, v in self._rpc(cmd="objects").items() if n != "ok"}

    def record_start(self, path, fps=30, width=960, height=540, pos=(1.05, -0.95, 0.62), look_at=(0.12, 0.0, 0.2)) -> None:
        """Record the overview camera to an mp4 (simulation time; needs render=True)."""
        self._rpc(cmd="record_start", path=str(path), fps=fps, width=width, height=height, pos=list(pos), look_at=list(look_at))

    def record_stop(self) -> None:
        self._rpc(cmd="record_stop")

    # ------------------------------------------------------------------ lifecycle (ArmBackend)
    def start(self) -> Outcome:
        self.reset()
        return Outcome(True, "ready: at zero")

    def stop(self) -> Outcome:
        r = self.move_to(piper_assets.zero_pose())
        return Outcome(r.ok, "at zero" if r.ok else r.message)

    def state(self) -> ArmState:
        r = self._rpc(cmd="state")
        return ArmState(r["t"], np.array(r["q"]), np.array(r["dq"]), r["t"], float(r["gripper"]), math.nan)

    def hold(self) -> None:
        self._rpc(cmd="hold")

    def gripper_opening(self) -> float:
        """The measured gripper opening in metres (0 closed, 0.07 open)."""
        return float(self._rpc(cmd="state")["gripper"])

    # ------------------------------------------------------------------ commands
    def _guard(self, q, measured) -> str | None:
        v = check_command(ARM_JOINTS, list(map(float, q)), self.limits, dict(zip(ARM_JOINTS, map(float, measured))), GuardConfig(ARM_JOINTS, MAX_STEP, self.margin))
        return None if v.ok else v.reason

    def command(self, q, dq=None) -> None:
        """One setpoint through the guard, then one control period of simulation. BackendError if the guard refuses it."""
        reason = self._guard(q, self.state().q)
        if reason:
            raise BackendError(reason)
        self._rpc(cmd="command", q=list(map(float, q)))

    def gripper_force(self) -> float:
        """NaN: the simulated gripper has no force read-back (the real one does, about -2 N at 3 N commanded: docs/real_arm/arms/piper-01.md)."""
        return math.nan

    def set_gripper(self, width: float, effort: float | None = None, timeout: float | None = None) -> float:
        """Move the gripper to `width` (0 to the stroke) and wait until it has settled (at most `timeout`, default 2 s of simulation). `effort` is ignored. Returns the measured width."""
        if not 0.0 <= width <= self.limits_info.gripper_stroke:
            raise BackendError(f"gripper width {width} outside [0, {self.limits_info.gripper_stroke}]")
        self._rpc(cmd="gripper", width=float(width), max_s=2.0 if timeout is None else float(timeout))
        return self.gripper_opening()

    def move_to(self, q, duration: float | None = None) -> ExecResult:
        """A minimum-jerk move from the present pose to `q` (a convenience move, same as PiperSim's; planner trajectories go through execute())."""
        q0, q1 = self.state().q, np.asarray(q, dtype=float)
        dur = duration or max(1.0, 1.875 * float(np.max(np.abs(q1 - q0))) / 1.0)      # peak speed of a minimum-jerk move = 1.875 x distance / duration
        t = np.arange(0.0, dur + 1e-9, 0.02)
        s = t / t[-1]
        pos, vel = 10 * s**3 - 15 * s**4 + 6 * s**5, (30 * s**2 - 60 * s**3 + 30 * s**4) / t[-1]
        d = (q1 - q0)[None, :]
        return self.execute(Trajectory(list(ARM_JOINTS), t, q0 + pos[:, None] * d, vel[:, None] * d))

    def execute(self, traj, *, margin_percent: float = 20.0, blocking: bool = True) -> ExecResult:
        """Run a trajectory as given (see piper_core.backend). `margin_percent` is ignored: the setpoints go straight to the position drives."""
        if not blocking:
            raise NotImplementedError("PiperIsaacSim.execute is blocking only")
        try:
            traj = traj if isinstance(traj, Trajectory) else Trajectory.from_pairs(traj)
        except (ValueError, TypeError) as e:
            return ExecResult(False, "bad_trajectory", str(e))
        if list(traj.joint_names) != list(ARM_JOINTS):
            return ExecResult(False, "bad_trajectory", f"joint_names must be {list(ARM_JOINTS)}")
        state = self.state()
        refused = (check_trajectory(traj, self.limits, margin=self.margin, max_step=MAX_STEP, max_velocity=self.limits_info.velocity, rate_hz=self.ctrl_hz)
                   or check_start(traj, state.q))
        if refused:
            return ExecResult(False, *refused)
        grid, setpoints = resample(traj, self.ctrl_hz)
        log_t, log_c, log_m, contacts, events = [], [], [], [], []
        measured = state.q
        for a in range(0, len(grid), CHUNK):
            rows = setpoints[a:a + CHUNK]
            reason = self._guard(rows[0], measured)
            if reason:
                self.hold()
                events.append((float(grid[a]), "guard:rejected", reason))
                return ExecResult(False, "guard:rejected", f"setpoint at t={grid[a]:.3f}: {reason}; holding", np.array(log_t), np.array(log_c).reshape(-1, 6),
                                  np.array(log_m).reshape(-1, 6), events, contacts)
            r = self._rpc(cmd="run", q=rows.tolist())
            m = np.array(r["q_meas"])
            log_t += grid[a:a + len(rows)].tolist()
            log_c += list(rows)
            log_m += list(m)
            contacts += [(float(grid[a + s]), x, y, d) for s, x, y, d in r["contacts"]]
            measured = m[-1]
        goal, t_end = setpoints[-1], float(grid[-1])
        r = self._rpc(cmd="settle", goal=goal.tolist(), tol=SETTLE_TOL, vel_tol=SETTLE_VEL, max_s=SETTLE_S)
        for k, row in enumerate(r["q_meas"]):
            log_t.append(t_end + (k + 1) * self.ctrl_dt)
            log_c.append(goal.copy())
            log_m.append(np.array(row))
        contacts += [(t_end + (s + 1) * self.ctrl_dt, x, y, d) for s, x, y, d in r["contacts"]]
        return ExecResult(True, "done", "", np.array(log_t), np.array(log_c), np.array(log_m), events, contacts)
