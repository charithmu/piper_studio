"""The real ZED X Mini through the Stereolabs ZED ROS 2 wrapper, set up to match this description (run on the ZED Box, or any machine with the camera and the wrapper).

    ros2 launch piper_control zed_camera.launch.py                         # camera_name piper_zed, frames of the Piper description, TF off, 960x600 (SVGA) 30 Hz
    ros2 launch piper_control zed_camera.launch.py prefix:=arm_ resolution:=HD1200 camera_model:=zedxm
    ros2 launch /path/to/zed_camera.launch.py                              # on a box that has only this file (no workspace): needs only the wrapper

What it fixes (docs/real_arm/ZED_BOX.md): `camera_name` = `<prefix>zed`, so the wrapper's frames and topics are the ones the simulators publish (`piper_description.camera_topics`);
the wrapper publishes NO TF and no URDF (the transforms come from this description's robot_state_publisher, one description only); positional tracking is off (it would
publish map/odom TF and cost GPU time); the image mode defaults to the simulators' (SVGA = 960x600, 30 Hz). Needs the wrapper's `zed_wrapper` package (zed-ros2-wrapper 5.x:
topics `rgb/color/rect/image`, `depth/depth_registered`; check the parameter names of the version installed). Nothing here is Piper-specific except the names, so this file imports no
workspace package and runs on the ZED Box (aarch64, Jetson) as it is. The camera pose comes from the bracket (nominal until the hand-eye calibration, CHECKLIST R3.6).
"""

import tempfile
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

OVERRIDE = """\
/**:
  ros__parameters:
    general:
      grab_resolution: '{resolution}'
      grab_frame_rate: {fps}
    pos_tracking:
      pos_tracking_enabled: false
      publish_tf: false
      publish_map_tf: false
"""


def zed_names(prefix: str = "piper_") -> dict:
    """camera_name and the topics/frame the wrapper gives it. Same as piper_description.camera_topics (a test keeps the two equal)."""
    name = f"{prefix}zed"
    root = f"/{name}/zed_node"
    return {"camera_name": name, "optical_frame": f"{name}_left_camera_frame_optical", "color": f"{root}/rgb/color/rect/image",
            "color_info": f"{root}/rgb/color/rect/camera_info", "depth": f"{root}/depth/depth_registered"}


def launch_setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    prefix = arg("prefix")
    prefix = "" if prefix in ("", "none") else (prefix if prefix.endswith("_") else prefix + "_")
    try:
        wrapper = Path(get_package_share_directory("zed_wrapper")) / "launch" / "zed_camera.launch.py"
    except PackageNotFoundError:
        raise RuntimeError("zed_wrapper is not installed or not sourced (the ZED ROS 2 wrapper, https://github.com/stereolabs/zed-ros2-wrapper; "
                           "on the ZED Box it comes with the ZED SDK image). See docs/real_arm/ZED_BOX.md.") from None
    override = Path(arg("params_file")) if arg("params_file") else None
    if override is None:
        override = Path(tempfile.mkdtemp(prefix="piper_zed_")) / "override.yaml"
        override.write_text(OVERRIDE.format(resolution=arg("resolution"), fps=arg("fps")))
    print(f"[zed_camera] camera_name={zed_names(prefix)['camera_name']}  frame={zed_names(prefix)['optical_frame']}  override={override}", flush=True)
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(wrapper)), launch_arguments={
        "camera_name": zed_names(prefix)["camera_name"], "camera_model": arg("camera_model"),
        "publish_urdf": "false", "publish_tf": "false", "publish_map_tf": "false",
        "ros_params_override_path": str(override)}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("prefix", default_value="piper_", description="name prefix of the description (camera_name = <prefix>zed); none for no prefix"),
        DeclareLaunchArgument("camera_model", default_value="zedxm", description="zedxm (ZED X Mini), zedx, ..."),
        DeclareLaunchArgument("resolution", default_value="SVGA", description="SVGA = 960x600 like the simulators; HD1080, HD1200"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("params_file", default_value="", description="your own wrapper override yaml instead of the generated one"),
        OpaqueFunction(function=launch_setup),
    ])
