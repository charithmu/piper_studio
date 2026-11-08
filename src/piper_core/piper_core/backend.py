"""The backend contract: how a planner hands a trajectory to an arm, and gets back what happened. No ROS; numpy only.

    from piper_core.backend import Trajectory, ArmBackend
    traj = Trajectory.from_pairs([(0.0, q0), (0.025, q1), ...])        # or Trajectory(joint_names, t, q, dq=...)
    result = backend.execute(traj)                                       # an ExecResult, never an exception for a refused trajectory
    Trajectory.from_json(traj.to_json())                                 # hand-off to another Python environment (schema piper.trajectory/1)

Every backend (the real arm `piper_core.RealArm`, the MuJoCo `piper_mujoco.PiperSim`, the ROS executor) implements `ArmBackend` and obeys
the same rules, which are applied in `check_trajectory` and `check_start` so that they are the same code everywhere:

  * Nothing is changed. A trajectory is executed as given or refused whole, before anything moves; nothing is clamped or rescaled.
  * Joint names are the six arm joints without prefix (joint1 .. joint6). Angles in rad, times in seconds, starting at 0, strictly increasing.
    `t` is used as given: the step between points need not be uniform (cuRobo's 25 ms plan is 24.956 ms).
  * Between points the backend resamples to its own control rate (`resample`): cubic Hermite when `dq` is given, linear otherwise.
    That reproduces the planner's own curve; it does not change the path.
  * The rules are checked on the resampled setpoints: finite, inside the limits and not nearer than `margin` (0.01 rad) to a limit
    ("guard:limit" outside, "guard:margin" in the band; plan with `planning` limits, which are the hard ones shrunk by the margin),
    at most `max_step` (0.15 rad) per control step ("guard:step") and at most the velocity limit ("guard:velocity").
  * The first point must be within 0.05 rad of the present pose, else "start_too_far" and nothing moves.
  * A guard rejection while executing (measured state, fault) stops the motion and holds the pose (`ok=False`, reason "guard:...").
  * `margin_percent` is speed headroom for backends whose servo takes a speed setting (the real arm): the speed is set to the fastest
    segment of the trajectory plus that percentage. Backends that follow the setpoints directly (the simulators) ignore it.
  * `execute` can be called repeatedly in one process.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np

SCHEMA = "piper.trajectory/1"
ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
START_TOLERANCE = 0.05       # rad: how far the first point may be from the present pose
LIMIT_TOLERANCE = 1e-6       # rad: float32 noise of a planner that was given the `planning` limits is not a violation
MAX_STEP = 0.15              # rad per control step
MAX_VELOCITY = 3.0           # rad/s, firmware (docs/real_arm/arms/piper-01.md)
CONTROL_HZ = 500.0           # rate at which the rules are checked


class BackendError(RuntimeError):
    """A refused single command (`command`, `move_to`) or a backend that is not ready."""


@dataclass
class Outcome:
    """Result of a lifecycle step (`start`, `stop`). Truthy when it worked."""
    ok: bool
    message: str = ""
    details: dict = field(default_factory=dict)

    def __bool__(self):
        return self.ok


@dataclass
class Trajectory:
    joint_names: list[str]
    t: np.ndarray                          # (N,) seconds, starts at 0, strictly increasing
    q: np.ndarray                          # (N, 6) rad
    dq: np.ndarray | None = None           # (N, 6) rad/s: with it the backend uses cubic Hermite
    ddq: np.ndarray | None = None          # (N, 6) rad/s^2: carried along, not used
    meta: dict = field(default_factory=dict)   # free text: planner, commit, dt

    def __post_init__(self):
        self.joint_names = list(self.joint_names)
        self.t = np.asarray(self.t, dtype=float)
        self.q = np.asarray(self.q, dtype=float)
        self.dq = None if self.dq is None else np.asarray(self.dq, dtype=float)
        self.ddq = None if self.ddq is None else np.asarray(self.ddq, dtype=float)
        n, m = len(self.joint_names), len(self.t)
        if self.t.ndim != 1 or self.q.shape != (m, n):
            raise ValueError(f"t must be (N,) and q (N, {n}); got {self.t.shape} and {self.q.shape}")
        for name in ("dq", "ddq"):
            a = getattr(self, name)
            if a is not None and a.shape != (m, n):
                raise ValueError(f"{name} must be (N, {n}); got {a.shape}")

    @classmethod
    def from_pairs(cls, pairs, joint_names=ARM_JOINTS, meta=None) -> "Trajectory":
        """[(t, [q1..q6]), ...] as a planner or `RealArm.execute` used to take it."""
        pairs = list(pairs)
        return cls(list(joint_names), [float(t) for t, _ in pairs], [list(map(float, q)) for _, q in pairs], meta=dict(meta or {}))

    def to_json(self) -> str:
        d = {"schema": SCHEMA, "joint_names": self.joint_names, "t": self.t.tolist(), "q": self.q.tolist(), "meta": self.meta}
        if self.dq is not None:
            d["dq"] = self.dq.tolist()
        if self.ddq is not None:
            d["ddq"] = self.ddq.tolist()
        return json.dumps(d)

    @classmethod
    def from_json(cls, text: str) -> "Trajectory":
        d = json.loads(text)
        if d.get("schema") != SCHEMA:
            raise ValueError(f"not a {SCHEMA} document (schema={d.get('schema')!r})")
        return cls(d["joint_names"], d["t"], d["q"], d.get("dq"), d.get("ddq"), d.get("meta", {}))


@dataclass
class ArmState:
    t: float                # time of the reading, seconds on the backend's clock
    q: np.ndarray           # (6,) rad
    dq: np.ndarray          # (6,) rad/s; NaN where the backend cannot measure it (the real arm)
    stamp: float            # when the feedback was produced; the guard rejects stale feedback by this


@dataclass
class ExecResult:
    ok: bool
    reason: str = "done"        # "done" | "guard:limit" | "guard:margin" | "guard:step" | "guard:velocity" | "guard:rejected" |
    #                             "start_too_far" | "bad_trajectory" | "not_ready" | "fault" | "timeout" | "stopped"
    detail: str = ""            # which joint / time / message
    t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    q_cmd: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    q_meas: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    guard_events: list = field(default_factory=list)    # (t, rule, detail)
    contacts: list = field(default_factory=list)        # (t, geom_a, geom_b, depth); empty on the real arm

    def __bool__(self):
        return self.ok

    def to_json(self) -> str:
        return json.dumps({"schema": "piper.exec_result/1", "ok": self.ok, "reason": self.reason, "detail": self.detail, "t": self.t.tolist(),
                           "q_cmd": self.q_cmd.tolist(), "q_meas": self.q_meas.tolist(), "guard_events": self.guard_events, "contacts": self.contacts})

    @classmethod
    def from_json(cls, text: str) -> "ExecResult":
        d = json.loads(text)
        if d.get("schema") != "piper.exec_result/1":
            raise ValueError(f"not a piper.exec_result/1 document (schema={d.get('schema')!r})")
        return cls(d["ok"], d["reason"], d["detail"], np.array(d["t"]), np.array(d["q_cmd"]).reshape(-1, 6),
                   np.array(d["q_meas"]).reshape(-1, 6), [tuple(e) for e in d["guard_events"]], [tuple(c) for c in d["contacts"]])

    @property
    def message(self) -> str:
        return self.reason if not self.detail else f"{self.reason}: {self.detail}"


@runtime_checkable
class ArmBackend(Protocol):
    """What a planner may rely on. Construction is backend specific (`RealArm.connect("can0")`, `PiperSim(...)`)."""

    joint_names: list[str]

    def start(self) -> Outcome: ...                       # powered, valid, at the zero pose
    def stop(self) -> Outcome: ...                        # back to zero, then powered off (the real arm parks first)
    def close(self) -> None: ...
    def state(self) -> ArmState: ...
    def execute(self, traj: Trajectory, *, margin_percent: float = 20.0, blocking: bool = True) -> ExecResult: ...
    def command(self, q, dq=None) -> None: ...            # one real-time setpoint through the guard; BackendError if refused
    def hold(self) -> None: ...                           # keep the present position
    def move_to(self, q, duration: float | None = None) -> ExecResult: ...


def resample(traj: Trajectory, rate_hz: float = CONTROL_HZ) -> tuple[np.ndarray, np.ndarray]:
    """(t, q) on a uniform grid of 1/rate_hz, plus the exact final time. Cubic Hermite with `dq`, else linear. `traj.t` as given."""
    t = traj.t
    if len(t) == 1:
        return t.copy(), traj.q.copy()
    grid = np.arange(0.0, t[-1], 1.0 / rate_hz)
    grid = np.append(grid[grid < t[-1] - 1e-12], t[-1])
    idx = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 2)
    t0, t1 = t[idx], t[idx + 1]
    h = (t1 - t0)[:, None]
    s = ((grid - t0)[:, None]) / h
    q0, q1 = traj.q[idx], traj.q[idx + 1]
    if traj.dq is None:
        return grid, q0 + s * (q1 - q0)
    m0, m1 = traj.dq[idx], traj.dq[idx + 1]
    h00, h10, h01, h11 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s, -2 * s**3 + 3 * s**2, s**3 - s**2
    return grid, h00 * q0 + h10 * h * m0 + h01 * q1 + h11 * h * m1


def check_trajectory(traj: Trajectory, limits: dict[str, tuple[float, float]], *, margin: float = 0.01, max_step: float = MAX_STEP,
                     max_velocity: float = MAX_VELOCITY, rate_hz: float = CONTROL_HZ) -> tuple[str, str] | None:
    """None if the whole trajectory passes, else (reason, detail). Nothing is modified. See the module docstring for the rules."""
    if not traj.joint_names or any(j not in limits for j in traj.joint_names):
        return "bad_trajectory", f"joint names {traj.joint_names} are not all known joints {sorted(limits)}"
    if len(traj.t) < 2:
        return "bad_trajectory", "a trajectory needs at least 2 points"
    if not (np.all(np.isfinite(traj.t)) and np.all(np.isfinite(traj.q))):
        return "bad_trajectory", "t or q is not finite"
    if abs(traj.t[0]) > 1e-9 or np.any(np.diff(traj.t) <= 0):
        return "bad_trajectory", "t must start at 0 and strictly increase"
    if traj.dq is not None and not np.all(np.isfinite(traj.dq)):
        return "bad_trajectory", "dq is not finite"
    t, q = resample(traj, rate_hz)
    for k, name in enumerate(traj.joint_names):       # the planner's own points and the resampled setpoints between them
        lo, hi = limits[name]
        for tt, col in ((traj.t, traj.q[:, k]), (t, q[:, k])):
            bad = np.nonzero((col < lo - LIMIT_TOLERANCE) | (col > hi + LIMIT_TOLERANCE))[0]
            if len(bad):
                return "guard:limit", f"{name}={col[bad[0]]:.4f} outside [{lo:.4f}, {hi:.4f}] at t={tt[bad[0]]:.3f}"
            bad = np.nonzero((col < lo + margin - LIMIT_TOLERANCE) | (col > hi - margin + LIMIT_TOLERANCE))[0]
            if len(bad):
                return "guard:margin", f"{name}={col[bad[0]]:.4f} within {margin} rad of a limit [{lo:.4f}, {hi:.4f}] at t={tt[bad[0]]:.3f}"
    dq_ = np.abs(np.diff(q, axis=0))
    dt = np.diff(t)[:, None]
    worst = np.unravel_index(np.argmax(dq_), dq_.shape)
    if dq_[worst] > max_step:
        return "guard:step", f"{traj.joint_names[worst[1]]} moves {dq_[worst]:.3f} rad in one control step (max {max_step})"
    v = dq_ / dt
    worst = np.unravel_index(np.argmax(v), v.shape)
    if v[worst] > max_velocity:
        return "guard:velocity", f"{traj.joint_names[worst[1]]} needs {v[worst]:.2f} rad/s at t={t[worst[0]]:.3f} (max {max_velocity})"
    return None


def check_start(traj: Trajectory, q_now, tolerance: float = START_TOLERANCE) -> tuple[str, str] | None:
    """None if the first point is within `tolerance` of the present pose, else ("start_too_far", detail)."""
    err = float(np.max(np.abs(np.asarray(q_now, dtype=float) - traj.q[0])))
    if not math.isfinite(err) or err > tolerance:
        return "start_too_far", f"first point is {err:.3f} rad from the present pose (max {tolerance})"
    return None
