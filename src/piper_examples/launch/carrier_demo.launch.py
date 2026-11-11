"""Example: a carrier robot that owns its ros2_control node, with the Piper control stack on top (backend:=external).

    ros2 launch piper_examples carrier_demo.launch.py [rviz:=true]

What a downstream project does, in order:
  1. expand its own robot description (the carrier, with the Piper macros and the Piper's ros2_control block);
  2. run its own controller manager (here the stock ros2_control_node with mock hardware; a simulator's plugin in a real project)
     with the Piper's controllers.yaml, filled with its prefix (piper_control.launch_lib.controllers_file);
  3. include piper_control's control.launch.py with backend:=external: it spawns the Piper controllers, starts MoveIt and Servo.
"""

import tempfile
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from piper_control.launch_lib import controllers_file

PREFIX = "arm_"


def launch_setup(context):
    share = Path(get_package_share_directory("piper_examples"))
    urdf = xacro.process_file(str(share / "urdf/carrier.urdf.xacro"), mappings={"prefix": PREFIX}).toxml()
    description_file = Path(tempfile.gettempdir()) / "piper_examples_carrier.urdf"
    description_file.write_text(urdf)
    controllers = controllers_file(PREFIX)
    control = Path(get_package_share_directory("piper_control")) / "launch/control.launch.py"
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": urdf}], output="log"),
        Node(package="controller_manager", executable="ros2_control_node", parameters=[controllers], output="screen",
             remappings=[("~/robot_description", "/robot_description")]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(control)), launch_arguments={
            "backend": "external", "prefix": PREFIX, "robot_description_file": str(description_file), "robot_name": "carrier", "camera": "zedxm",  # tells the SRDF about the camera links (collision pairs)
            
            "servo": "false", "rviz": LaunchConfiguration("rviz")}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("rviz", default_value="false"), OpaqueFunction(function=launch_setup)])
