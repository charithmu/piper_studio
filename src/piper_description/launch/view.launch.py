"""Inspect the description alone: robot_state_publisher + joint sliders + RViz (no controllers, MoveIt or simulator).

    ros2 launch piper_description view.launch.py camera:=d435
    ros2 launch piper_description view.launch.py camera:=d435 camera_bracket_mesh:=none gripper:=false sliders:=false

The description arguments are the same names piper_bringup's piper.launch.py passes to the xacro.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# launch argument -> (default, description); forwarded to piper.urdf.xacro as name:=value
XACRO_ARGS = {
    "gripper": ("true", "include the gripper (true|false)"),
    "camera": ("none", "wrist camera: none | d435"),
    "camera_bracket_mesh": ("", "camera stand mesh path, or 'none' for a box (default: AgileX stand packaged in piper_description)"),
    "camera_width": ("640", "simulated image width (px)"),
    "camera_height": ("480", "simulated image height (px)"),
    "camera_hfov_deg": ("69.4", "simulated horizontal field of view (deg)"),
    "camera_rate": ("30", "simulated frame rate (Hz)"),
    "tcp_xyz": ("0 0 0.138", "tool centre point offset from the flange"),
    "tcp_rpy": ("0 0 0", "tool centre point orientation"),
}


def generate_launch_description():
    cfg = {name: LaunchConfiguration(name) for name in XACRO_ARGS}
    cmd = ["xacro ", PathJoinSubstitution([FindPackageShare("piper_description"), "urdf", "piper.urdf.xacro"])]
    for name in XACRO_ARGS:
        if name == "camera_bracket_mesh":
            continue  # only forwarded when set: an empty value would override the packaged default
        value = cfg[name]
        # tcp_* hold spaces: quote them for the shell
        cmd += [f" {name}:='", value, "'"] if name.startswith("tcp_") else [f" {name}:=", value]
    # the optional mesh goes in via the generic substitution below
    description = ParameterValue(Command(cmd + [" ", _mesh_arg(cfg["camera_bracket_mesh"])]), value_type=str)
    return LaunchDescription([
        *[DeclareLaunchArgument(n, default_value=d, description=t) for n, (d, t) in XACRO_ARGS.items()],
        DeclareLaunchArgument("sliders", default_value="true", description="joint_state_publisher_gui sliders (false: joints stay at zero)"),
        DeclareLaunchArgument("rviz", default_value="true", description="start RViz"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(LaunchConfiguration("sliders"))),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             condition=_unless("sliders")),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")), arguments=[
            "-d", PathJoinSubstitution([FindPackageShare("piper_description"), "rviz", "view.rviz"])]),
    ])


def _mesh_arg(value):
    """' camera_bracket_mesh:=<value>' if set, else nothing (shell: a command substitution would be overkill)."""
    from launch.substitutions import PythonExpression
    return PythonExpression(["('camera_bracket_mesh:=' + '", value, "') if '", value, "' else ''"])


def _unless(name):
    from launch.conditions import UnlessCondition
    return UnlessCondition(LaunchConfiguration(name))
