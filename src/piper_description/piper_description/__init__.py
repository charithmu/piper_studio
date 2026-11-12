"""Generate the Piper robot description (xacro expansion) for any hardware backend.

    from piper_description import robot_description
    urdf = robot_description("mock")                  # with the ZED X Mini wrist camera (the default)
    urdf = robot_description("mock", camera="none")   # the robot only

Simulator-specific model processing (Gazebo, MuJoCo, Isaac) lives in piper_sim, not here.
"""

from __future__ import annotations

import hashlib
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

DEFAULT_CAMERA = "zedxm"   # the wrist camera every description has unless `camera="none"` (robot only)
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


def camera_model(lens: str | None = None, width: int | None = None, height: int | None = None, camera: str = DEFAULT_CAMERA) -> dict:
    """The wrist camera's numbers (config/camera_models/<camera>.yaml, the one source of the simulated cameras): lens, width, height, hfov_deg, vfov_deg,
    depth_min, depth_max, rate, baseline, `K` (3x3 pinhole intrinsics of the rectified left image) and the names `optical_frame` / `link` / `camera_name` (without prefix)."""
    import math
    import yaml
    from ament_index_python.packages import get_package_share_directory
    m = yaml.safe_load((Path(get_package_share_directory("piper_description")) / "config" / "camera_models" / f"{camera}.yaml").read_text())
    lens = lens or m["lens"]
    L = m["lenses"][lens]
    w, h = int(width or m["image"]["width"]), int(height or m["image"]["height"])
    f = (w / 2) / math.tan(math.radians(L["hfov_deg"]) / 2)
    return {"model": m["model"], "lens": lens, "width": w, "height": h, "hfov_deg": L["hfov_deg"], "vfov_deg": L["vfov_deg"],
            "depth_min": L["depth_min"], "depth_max": L["depth_max"], "rate": m["rate"], "baseline": m["baseline"],
            "K": [[f, 0.0, w / 2], [0.0, f, h / 2], [0.0, 0.0, 1.0]], "optical_frame": m["optical_frame"], "link": "gripper_base", "camera_name": m["camera_name"]}


def camera_topics(prefix: str = DEFAULT_PREFIX) -> dict:
    """The wrist camera's frame and topics, the ZED ROS 2 wrapper's (camera_name = `<prefix>zed`, node `zed_node`): the same names in every simulator and from the real
    ZED X Mini (`ros2 launch piper_control zed_camera.launch.py`). Keys: camera_name, optical_frame (the left lens; colour, registered depth and camera_info are in it),
    color (sensor_msgs/Image), color_info (CameraInfo, valid for the depth too: it is registered to the left image), depth (32FC1, metres)."""
    name = prefix + "zed"
    root = f"/{name}/zed_node"
    return {"camera_name": name, "optical_frame": f"{name}_left_camera_frame_optical", "color": f"{root}/rgb/color/rect/image",
            "color_info": f"{root}/rgb/color/rect/camera_info", "depth": f"{root}/depth/depth_registered"}


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


def fixed_gripper(urdf: str, width: float, prefix: str = DEFAULT_PREFIX) -> str:
    """The description with the gripper held at `width` metres: the ROS-free implementation lives in `piper_core.urdf_tools`
    (planners use it without this package). Imported on use, because piper_core's tests depend on this package (no dependency cycle)."""
    from piper_core.urdf_tools import fixed_gripper as impl
    return impl(urdf, width, prefix)


GO2_URDF = Path.home() / "projects/ros/go2_studio/third_party/unitree_ros/robots/go2_description/urdf/go2_description.urdf"


def go2_piper_description(go2_urdf: str | Path = GO2_URDF, **mappings) -> str:
    """The Go2 carrying the Piper (urdf/go2_piper.urdf.xacro). NOMINAL mount until the dock plate is measured: `plate_xyz`, `plate_size`,
    `mount_rpy` are xacro arguments in the Go2 `base` frame. The Go2 URDF is an external input (default: go2_studio's unitree_ros checkout)."""
    import xacro
    args = {"go2_urdf": str(go2_urdf), **{k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in mappings.items()}}
    path = xacro_file().with_name("go2_piper.urdf.xacro")
    return xacro.process_file(str(path), mappings=args).toxml()


from .go2 import DEFAULT_SETUP, SETUPS, arm_base_pose, check_setup, cover_report, dog_cover, go2_piper_curobo, go2_stand_pose, go2_world, mount_boxes, setup_config  # noqa: E402,F401
