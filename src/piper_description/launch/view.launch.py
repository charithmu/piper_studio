"""Inspect the description alone: robot_state_publisher + joint sliders + RViz (no controllers, MoveIt or simulator).

    ros2 launch piper_description view.launch.py                      # with the ZED X Mini wrist camera (default)
    ros2 launch piper_description view.launch.py camera:=none gripper:=false sliders:=false   # the robot only

The description arguments are the same names piper_sim's and piper_control's launch files pass to the xacro.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from piper_description import normalize_prefix, robot_description

# launch argument -> (default, description); forwarded to piper.urdf.xacro as name:=value
XACRO_ARGS = {
    "prefix": ("piper_", "prefix of every link, joint and frame name (none: no prefix, the official AgileX names)"),
    "gripper": ("true", "include the gripper (true|false)"),
    "camera": ("zedxm", "wrist camera: zedxm (ZED X Mini on the own bracket, default) | none (the robot only)"),
    "camera_calibration": ("", "a version of the bracket (a name in config/camera_mounts, e.g. zedxm_bracket_v2) or a yaml path with a measured pose (default: the newest version)"),
    "camera_lens": ("", "wide | narrow (default: config/camera_models/zedxm.yaml)"),
    "camera_width": ("", "simulated image width (px; default: the camera model's)"),
    "camera_height": ("", "simulated image height (px; default: the camera model's)"),
    "camera_hfov_deg": ("", "simulated horizontal field of view (deg; default: the lens's)"),
    "camera_rate": ("", "simulated frame rate (Hz; default: the camera model's)"),
    "tcp_xyz": ("0 0 0.138", "tool centre point offset from the flange"),
    "tcp_rpy": ("0 0 0", "tool centre point orientation"),
}


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    mappings = {n: arg(n) for n in XACRO_ARGS if arg(n)}        # empty = the default of piper.urdf.xacro
    mappings["prefix"] = normalize_prefix(mappings["prefix"])
    description = robot_description("mock", **mappings)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(LaunchConfiguration("sliders"))),
        Node(package="joint_state_publisher", executable="joint_state_publisher", condition=UnlessCondition(LaunchConfiguration("sliders"))),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", PathJoinSubstitution([FindPackageShare("piper_description"), "rviz", "view.rviz"])]),
    ]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(n, default_value=d, description=t) for n, (d, t) in XACRO_ARGS.items()],
        DeclareLaunchArgument("sliders", default_value="true", description="joint_state_publisher_gui sliders (false: joints stay at zero)"),
        DeclareLaunchArgument("rviz", default_value="true", description="start RViz"),
        OpaqueFunction(function=launch_setup),
    ])
