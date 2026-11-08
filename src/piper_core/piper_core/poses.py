"""Named poses from an expanded SRDF, the `zero` pose, and the classification of the arm's state at start-up.

The words (one meaning each, see docs/real_arm/arms/piper-01.md):
  zero     the one valid start and end pose of every session: every joint 0, except a joint whose limit is 0 is held `margin` inside it
           (a pose exactly on a limit is rejected by planners and unreachable for the arm). Downstream users only ever see this pose.
  park     (real arm, Piper Studio internal) zero with joint 5 lowered; the stop sequence parks there before the motors go off.
  slumped  the pose a powered-off real arm sags into: joints 2 and 3 a little outside their limits. A measured fact, never a target.
"""

import math
import xml.etree.ElementTree as ET

SLUMP_TOLERANCE = 0.1    # rad: how far outside its limits a joint may be and still count as "slumped" (measured: about 0.04)


def named_poses_from_srdf(srdf: str, group: str = "arm") -> dict[str, dict[str, float]]:
    """{pose name: {joint name: value}} for the group's <group_state> entries. `srdf` is the expanded SRDF text (not the xacro)."""
    root = ET.fromstring(srdf)
    return {s.get("name"): {j.get("name"): float(j.get("value")) for j in s.findall("joint")}
            for s in root.findall("group_state") if s.get("group") == group}


def zero_pose(limits: dict[str, tuple[float, float]], arm_joints, margin: float = 0.01) -> dict[str, float]:
    """The zero pose: 0 for every joint, moved `margin` inside the limit where 0 is on or outside it (the guard keeps commands that far in)."""
    return {j: min(max(0.0, limits[j][0] + margin), limits[j][1] - margin) for j in arm_joints}


def classify_state(measured: dict[str, float], limits: dict[str, tuple[float, float]], arm_joints, margin: float = 0.01,
                   zero_tolerance: float = 0.01) -> str:
    """'zero' | 'valid' | 'slumped' | 'invalid' for a measured arm state.

    zero     every joint within `zero_tolerance` of the zero pose
    valid    every joint inside its limits (anywhere in the workspace)
    slumped  at least one joint outside its limits, none by more than SLUMP_TOLERANCE: a powered-off arm resting on its stops
    invalid  missing or non-finite values, or outside the limits by more than that
    """
    joints = list(arm_joints)
    if any(j not in measured or not math.isfinite(measured[j]) for j in joints):
        return "invalid"
    zero = zero_pose(limits, joints, margin)
    if all(abs(measured[j] - zero[j]) <= zero_tolerance for j in joints):
        return "zero"
    out = [max(limits[j][0] - measured[j], measured[j] - limits[j][1], 0.0) for j in joints]
    if max(out) == 0.0:
        return "valid"
    return "slumped" if max(out) <= SLUMP_TOLERANCE else "invalid"
