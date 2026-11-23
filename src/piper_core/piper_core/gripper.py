"""The gripper's contract, shared by the real arm and every simulator (one implementation: ARCHITECTURE "one owner, thin wrappers").

What a gripper move is, from a caller's side: `set_gripper(width, effort)` sends one target and blocks until the move is over, then says why
(`GripperResult.reason`): "reached" | "stalled" (stopped before the target: an object or the end stop) | "no_motion" | "timeout".
The end condition is decided from measured widths only, by `GripperWatch`, with the same numbers on the real arm and in MuJoCo and Isaac.
The numbers below are the real gripper's (docs/real_arm/REPORT.md section 4); the simulators' physical model of the same gripper is
`piper_description/config/actuation.yaml`, and a test keeps the two equal.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

GRIPPER_STROKE = 0.07          # m, firmware-reported (max_range_config), docs/real_arm/arms/piper-01.md
GRIPPER_DEFAULT_EFFORT = 1.0   # N, the vendor default (agx_arm_ctrl `gripper_default_effort`)
GRIPPER_MAX_EFFORT = 3.0       # N, the vendor's own cap (agx_arm_ctrl AgxGripperWrapper.FORCE_MAX 3.0 N; command range 0.5 to 3.0 N); also our cap. Raise it deliberately: arm.gripper_max_effort = 5.0




@dataclass
class GripperResult:
    reason: str        # "reached" | "stalled" (stopped before the target: an object or the end stop) | "no_motion" | "timeout"
    width: float       # m, last measured
    force: float | None
    duration: float = 0.0                                   # s from the command to the end condition (includes the 0.3 s the width must hold for "reached")
    trace: list = field(default_factory=list)               # (t s from the command, width m, force N) about every 20 ms


class GripperWatch:
    """Decides when a gripper move is over, from measured widths only. Closing on an object never reaches the target, so 'stopped moving' counts."""
    TOL, HOLD, WINDOW, STILL, MOVED, NO_MOTION_S = 0.001, 0.3, 0.5, 0.0003, 0.0015, 1.0

    def __init__(self, target: float, clock=time.monotonic, width0: float | None = None):
        self.target, self.now, self.w0, self.t0 = target, clock, width0, clock()
        self.hist, self.near_since = deque(), None

    def update(self, w: float) -> str | None:
        t = self.now()
        if self.w0 is None:
            self.w0 = w
        self.hist.append((t, w))
        while self.hist and t - self.hist[0][0] > 2 * self.WINDOW:
            self.hist.popleft()
        if abs(w - self.target) <= self.TOL:
            self.near_since = t if self.near_since is None else self.near_since
            if t - self.near_since >= self.HOLD:
                return "reached"
        else:
            self.near_since = None
        if abs(w - self.w0) >= self.MOVED and any(t - x >= self.WINDOW for x, _ in self.hist):
            recent = [v for x, v in self.hist if t - x <= self.WINDOW]
            if max(recent) - min(recent) < self.STILL:
                return "stalled"
        if t - self.t0 >= self.NO_MOTION_S and abs(w - self.w0) < self.MOVED:
            return "no_motion"
        return None




# ---------------------------------------------------------------------------------------------------------------------------------------
# The simulators' gripper move. MuJoCo (`piper_mujoco.PiperSim`) and Isaac Sim (`isaac/run_server.py`) both call `run_gripper_move`, so the
# gradual close, the force limit, the end condition and the read-back are ONE implementation. This part imports nothing but the standard library
# (the Isaac server runs in an environment without Piper packages and loads this file by path).

def grip_force_limit(effort: float, plateau: float) -> float:
    """The force the simulated gripper applies while gripping: the commanded effort, but never more than the real gripper's plateau (measured, about 2.1 N at 3 N commanded)."""
    return min(float(effort), float(plateau))


def ramp_target(current: float, goal: float, speed: float, dt: float) -> float:
    """One control step of the gradual move: the position target approaches `goal` at `speed` (m/s) instead of jumping there."""
    step = speed * dt
    return goal if abs(goal - current) <= step else current + step * (1.0 if goal > current else -1.0)


def check_gripper_command(width: float, effort: float | None, stroke: float, max_effort: float, default_effort: float):
    """(effort, error): the effort to use and None, or (None, the refusal text), by the same rules as `RealArm.gripper_command` (nothing is sent on a refusal)."""
    import math
    if not (math.isfinite(width) and -1e-9 <= width <= stroke + 1e-9):
        return None, f"gripper width {width} m is outside [0, {stroke}]"
    effort = default_effort if effort is None else float(effort)
    if not (math.isfinite(effort) and 0.0 < effort <= max_effort):
        return None, f"gripper effort {effort} N is outside (0, {max_effort}]"
    return effort, None


def run_gripper_move(*, width: float, effort: float, timeout: float, speed: float, plateau: float, dt: float, step, opening, force, time_now,
                     set_force_limit, current_target: float) -> tuple[GripperResult, float]:
    """Move a simulated gripper to `width` the way the real one moves, and say how it ended.

    The position target ramps at `speed` (a gradual close, contact detecting by the end condition), the force limit is
    `grip_force_limit(effort, plateau)`, and `GripperWatch` decides the end from measured widths on the simulation clock.
    Callables of the simulator: `step(target)` advances one control period with the gripper's position target; `opening()` the measured opening (m, the
    two fingers' travel); `force()` the gripper's force (N, negative while gripping, as the real one reads); `time_now()` simulation seconds;
    `set_force_limit(N)`. Returns (GripperResult, the last target); the simulator keeps holding that target and force limit afterwards."""
    set_force_limit(grip_force_limit(effort, plateau))
    watch = GripperWatch(width, clock=time_now, width0=opening())
    t0, trace, target, reason, next_trace = time_now(), [], current_target, None, 0.0
    while reason is None:
        target = ramp_target(target, width, speed, dt)
        step(target)
        w = opening()
        elapsed = time_now() - t0
        if elapsed >= next_trace:
            trace.append((elapsed, w, force()))
            next_trace += 0.02
        reason = watch.update(w)
        if reason is None and elapsed >= timeout:
            reason = "timeout"
    return GripperResult(reason, w, force(), time_now() - t0, trace), target
