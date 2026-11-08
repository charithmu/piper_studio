"""Inspect the description alone: robot_state_publisher + joint sliders + RViz (no controllers, MoveIt or simulator).

    ros2 launch piper_description view.launch.py camera:=d435
    ros2 launch piper_description view.launch.py camera:=d435 camera_mount_mesh:=none gripper:=false sliders:=false

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
    "camera": ("none", "wrist camera model (urdf/cameras/<name>.xacro): none | d435"),
    "camera_mount": ("agilex_mid_stand", "camera mount (urdf/mounts/<name>.xacro): agilex_mid_stand | custom"),
    "camera_calibration": ("", "measured stand and camera pose: a name in config/camera_mounts or a yaml path (default: nominal)"),
    "camera_mount_mesh": ("", "stand mesh path, or 'none' for a box (default: the mount's own mesh)"),
    "camera_xyz": ("0 0 0", "camera_mount:=custom: camera position in gripper_base"),
    "camera_rpy": ("0 0 0", "camera_mount:=custom: camera orientation in gripper_base"),
    "camera_width": ("640", "simulated image width (px)"),
    "camera_height": ("480", "simulated image height (px)"),
    "camera_hfov_deg": ("69.4", "simulated horizontal field of view (deg)"),
    "camera_rate": ("30", "simulated frame rate (Hz)"),
    "tcp_xyz": ("0 0 0.138", "tool centre point offset from the flange"),
    "tcp_rpy": ("0 0 0", "tool centre point orientation"),
}


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    # camera_mount_mesh is forwarded only when set: an empty value would override the mount's own mesh.
    mappings = {n: arg(n) for n in XACRO_ARGS if n not in ("camera_mount_mesh", "camera_calibration") or arg(n)}
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
