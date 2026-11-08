"""The command guard: the rules every joint command for the arm must pass before it reaches the driver.

One implementation, used by the ROS node (piper_control `command_guard.py`) and by any direct control path. Rejecting is always safe:
the driver keeps its last valid target. Freshness of the measured state is checked by the caller (it owns the clock).

A command is accepted only if
  * every value is finite (ros2_control publishes NaN while no controller owns the interfaces, and the driver turns NaN into 0.0,
    the vendor's exact-zero pose),
  * arm joints are inside the URDF limits, never outside. A value inside the limits but closer than `limit_margin` to one is clamped
    to the margin (the arm tracks a little short of its target, so "hold the measured pose" at zero lands just inside the margin);
    a value outside the limits is rejected. The slumped pose of a powered-off arm is outside the limits, so nothing can be commanded from
    it: `piper_core.poses.classify_state` recognises it and the start sequence (command_guard `go_zero`) leaves it first,
  * the arm is itself inside its limits (`MEASURED_TOLERANCE`): a command is never forwarded while the measured state is the slumped pose,
    so a stale command left in ros2_control cannot pull a freshly enabled arm anywhere; only `go_zero` leaves that pose,
  * other joints (the gripper) are inside their limits,
  * every arm joint has a measured position and the target is within `max_step` of it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


MEASURED_TOLERANCE = 1e-3     # rad a measured arm joint may sit outside its limit (sensor noise at a limit); the slumped pose is ~0.04


@dataclass(frozen=True)
class GuardConfig:
    arm_joints: tuple[str, ...]
    max_step: float = 0.15        # rad per command, arm joints
    limit_margin: float = 0.01    # rad kept inside the URDF limits, arm joints


@dataclass
class Verdict:
    ok: bool
    reason: str = ""
    positions: list[float] = field(default_factory=list)   # the (possibly clamped) values, only when ok


def check_command(names, positions, limits: dict[str, tuple[float, float]], measured: dict[str, float],
                  cfg: GuardConfig) -> Verdict:
    """Validate one command (parallel `names` / `positions`) against the limits and the measured state."""
    if not limits:
        return Verdict(False, "no joint limits known")
    for name in cfg.arm_joints:
        if name in limits and name in measured and math.isfinite(measured[name]):
            lo, hi = limits[name]
            if not lo - MEASURED_TOLERANCE <= measured[name] <= hi + MEASURED_TOLERANCE:
                return Verdict(False, f"the arm is outside its limits ({name}={measured[name]:.4f}): powered-off pose, call start first")
    out = list(positions)
    for k, (name, pos) in enumerate(zip(names, positions)):
        if not math.isfinite(pos):
            return Verdict(False, f"{name} is not finite (no active controller?)")
        lo, hi = limits.get(name, (-math.inf, math.inf))
        if name in cfg.arm_joints:
            if not lo <= pos <= hi:
                return Verdict(False, f"{name}={pos:.4f} outside the limits [{lo:.4f}, {hi:.4f}]")
            out[k] = pos = min(max(pos, lo + cfg.limit_margin), hi - cfg.limit_margin)
            if name not in measured or not math.isfinite(measured[name]):
                return Verdict(False, f"no measured state for {name}")
            if abs(pos - measured[name]) > cfg.max_step:
                return Verdict(False, f"{name} step {pos - measured[name]:+.3f} rad exceeds max_step {cfg.max_step}")
        elif not lo - 1e-6 <= pos <= hi + 1e-6:
            return Verdict(False, f"{name}={pos:.4f} outside [{lo:.4f}, {hi:.4f}]")
    return Verdict(True, "", out)
