"""Operations on a URDF string that need no ROS (numpy only)."""

from __future__ import annotations

import xml.etree.ElementTree as ET

DEFAULT_PREFIX = "piper_"


def _rpy_matrix(r: float, p: float, y: float):
    import numpy as np
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def fixed_gripper(urdf: str, width: float, prefix: str = DEFAULT_PREFIX) -> str:
    """The description with the gripper held at `width` metres (0 closed, 0.07 fully open): for planners that need a fixed-joint robot.

    The `gripper` joint and the two mimic finger joints become fixed joints, each origin moved along its axis by the value the joint would
    have at that width (finger joints: multiplier x width). Nothing else changes: same links, same names, same meshes and collision geometry.
    Used by `piper_sim/scripts/export_urdf.py --gripper-width` (cuRobo's builder cannot take the mimic joints).
    """
    import numpy as np
    root = ET.fromstring(urdf)
    main = root.find(f"joint[@name='{prefix}gripper']")
    if main is None:
        raise ValueError("this description has no gripper")
    lo, hi = float(main.find("limit").get("lower")), float(main.find("limit").get("upper"))
    if not lo <= width <= hi:
        raise ValueError(f"gripper width {width} outside [{lo}, {hi}]")
    for name in ("gripper", "gripper_joint1", "gripper_joint2"):
        joint = root.find(f"joint[@name='{prefix}{name}']")
        mimic = joint.find("mimic")
        q = width * (float(mimic.get("multiplier", 1.0)) if mimic is not None else 1.0)
        axis = np.array([float(v) for v in joint.find("axis").get("xyz").split()])
        origin = joint.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz").split()])
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        origin.set("xyz", " ".join(f"{v:.9g}" for v in xyz + _rpy_matrix(*rpy) @ (axis * q)))
        joint.set("type", "fixed")
        for tag in ("axis", "limit", "mimic"):
            for el in joint.findall(tag):
                joint.remove(el)
    return ET.tostring(root, encoding="unicode")
