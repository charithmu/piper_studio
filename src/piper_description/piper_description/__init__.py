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


from piper_core.urdf_tools import fixed_gripper  # noqa: E402,F401  (ROS-free, so planners can use it without this package)
