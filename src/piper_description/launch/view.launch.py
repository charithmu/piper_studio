"""Inspect the description: robot_state_publisher + joint sliders + RViz."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gripper = LaunchConfiguration("gripper")
    description = ParameterValue(
        Command([
            "xacro ", PathJoinSubstitution([FindPackageShare("piper_description"), "urdf", "piper.urdf.xacro"]),
            " gripper:=", gripper,
        ]),
        value_type=str,
    )
    return LaunchDescription([
        DeclareLaunchArgument("gripper", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": description}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=[
            "-d", PathJoinSubstitution([FindPackageShare("piper_description"), "rviz", "view.rviz"])]),
    ])
