"""Generate the Piper robot description (xacro expansion) for any hardware backend.

    from piper_description import robot_description
    urdf = robot_description("mock", camera="d435")

Simulator-specific model processing (Gazebo, MuJoCo, Isaac) lives in piper_sim, not here.
"""

from __future__ import annotations

import hashlib
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_PREFIX = "piper_"  # prefix of every link, joint and frame name (piper.urdf.xacro `prefix`; "" gives the official names)


def normalize_prefix(prefix: str) -> str:
    """`ros2 launch` rejects an empty value (`prefix:=`), so the launch files accept `none` for "no prefix" (the official names)."""
    return "" if prefix.strip().lower() in ("none", '""', "''") else prefix


def xacro_file() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("piper_description")) / "urdf" / "piper.urdf.xacro"


def robot_description(hardware: str = "mock", **mappings) -> str:
    """Expand piper.urdf.xacro for `hardware` (mock | real | gazebo | mujoco | isaac); `mappings` are its xacro arguments (prefix, gripper, camera, ...)."""
    import xacro
    args = {"hardware": hardware, **{k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in mappings.items()}}
    return xacro.process_file(str(xacro_file()), mappings=args).toxml()


def apply_prefix(text: str, prefix: str = DEFAULT_PREFIX) -> str:
    """Fill the `${prefix}` placeholder of a config template (controllers.yaml, moveit yaml, MuJoCo inputs)."""
    return text.replace("${prefix}", prefix)


def render_config(path: str | Path, prefix: str = DEFAULT_PREFIX) -> Path:
    """Write the template at `path` with the prefix filled in to a temp file (content-addressed, so repeated launches reuse it)."""
    text = apply_prefix(Path(path).read_text(), prefix)
    out = Path(tempfile.gettempdir()) / "piper_studio_config" / hashlib.sha256(text.encode()).hexdigest()[:16] / Path(path).name
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text)
    return out


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
