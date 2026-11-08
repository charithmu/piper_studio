"""The command guard: the rules every joint command for the arm must pass before it reaches the driver.

One implementation, used by the ROS node (piper_control `command_guard.py`) and by any direct control path. Rejecting is always safe:
the driver keeps its last valid target. Freshness of the measured state is checked by the caller (it owns the clock).

A command is accepted only if
  * every value is finite (ros2_control publishes NaN while no controller owns the interfaces, and the driver turns NaN into 0.0,
    the folded rest pose),
  * arm joints are inside the URDF limits, kept `limit_margin` inside; a value up to `limit_slack` outside that range is clamped to it
    instead of rejected (a real Piper rests about 0.04 rad past the limits of joints 2 and 3, so "hold the measured pose" and "go to zero"
    would otherwise be refused or sit exactly on the arm's stored limit),
  * other joints (the gripper) are inside their limits,
  * every arm joint has a measured position and the target is within `max_step` of it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class GuardConfig:
    arm_joints: tuple[str, ...]
    max_step: float = 0.15        # rad per command, arm joints
    limit_margin: float = 0.01    # rad kept inside the URDF limits, arm joints
    limit_slack: float = 0.1      # rad outside the margin range that is clamped, not rejected


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
    out = list(positions)
    for k, (name, pos) in enumerate(zip(names, positions)):
        if not math.isfinite(pos):
            return Verdict(False, f"{name} is not finite (no active controller?)")
        lo, hi = limits.get(name, (-math.inf, math.inf))
        if name in cfg.arm_joints:
            lo, hi = lo + cfg.limit_margin, hi - cfg.limit_margin
            if not lo - cfg.limit_slack <= pos <= hi + cfg.limit_slack:
                return Verdict(False, f"{name}={pos:.4f} outside [{lo:.4f}, {hi:.4f}] by more than {cfg.limit_slack}")
            out[k] = pos = min(max(pos, lo), hi)
            if name not in measured or not math.isfinite(measured[name]):
                return Verdict(False, f"no measured state for {name}")
            if abs(pos - measured[name]) > cfg.max_step:
                return Verdict(False, f"{name} step {pos - measured[name]:+.3f} rad exceeds max_step {cfg.max_step}")
        elif not lo - 1e-6 <= pos <= hi + 1e-6:
            return Verdict(False, f"{name}={pos:.4f} outside [{lo:.4f}, {hi:.4f}]")
    return Verdict(True, "", out)
