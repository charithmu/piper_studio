"""Builders for the control layer: description arguments, MoveIt configuration, controller spawners, MoveIt, Servo, RViz, real driver.

Everything here is independent of any simulator. A launch file composes these with its own hardware (a simulator, the real arm, mock).
"""

from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from piper_description import DEFAULT_PREFIX, normalize_prefix, render_config, xacro_file


def share(package: str) -> Path:
    return Path(get_package_share_directory(package))


# ---- description arguments (the same names piper_description's view.launch.py and the xacro use) -------------------------------------
def description_arguments() -> list:
    return [
        DeclareLaunchArgument("prefix", default_value=DEFAULT_PREFIX,
                              description="prefix of every link, joint and frame name (none: no prefix, the official AgileX names)"),
        DeclareLaunchArgument("gripper", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("camera", default_value="zedxm", choices=["zedxm", "none"],
                              description="wrist camera: zedxm (ZED X Mini on the own bracket, the default) | none (the robot only)"),
        DeclareLaunchArgument("camera_calibration", default_value="",
                              description="a version of the bracket (a name in piper_description/config/camera_mounts, e.g. zedxm_bracket_v2) or a yaml path with a measured pose (default: the newest)"),
        DeclareLaunchArgument("camera_lens", default_value="", description="wide | narrow (default: piper_description/config/camera_models/zedxm.yaml)"),
        DeclareLaunchArgument("camera_width", default_value="", description="simulated image width, px (default: the camera model's)"),
        DeclareLaunchArgument("camera_height", default_value="", description="simulated image height, px (default: the camera model's)"),
    ]


def description_mappings(arg) -> dict:
    """xacro mappings from the launch arguments; `arg(name)` returns a launch argument's value."""
    mappings = {"prefix": normalize_prefix(arg("prefix")), "gripper": arg("gripper")}
    mappings["camera"] = arg("camera")
    if arg("camera") != "none":
        mappings.update({k: arg(k) for k in ("camera_calibration", "camera_lens", "camera_width", "camera_height") if arg(k)})
    return mappings


def control_arguments() -> list:
    return [
        DeclareLaunchArgument("moveit", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("servo", default_value="true", choices=["true", "false"],
                              description="start MoveIt Servo (TCP-frame jogging via the streaming controller)"),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
    ]


def real_arm_arguments() -> list:
    return [
        DeclareLaunchArgument("canbus", default_value="can0", description="real only: CAN bus (network interface of the USB-CAN adapter)"),
        DeclareLaunchArgument("speed_percent", default_value="20",
                              description="real only: arm speed in percent (1-100) used by `start` and by streamed setpoints"),
        DeclareLaunchArgument("driver", default_value="true", choices=["true", "false"],
                              description="real only: start piper_real_driver (false: bring your own driver with the same topics and services)"),
    ]


# ---- MoveIt ---------------------------------------------------------------------------------------------------------------------------
def moveit_config(gripper: str, urdf: str, camera: str = "zedxm", prefix: str = DEFAULT_PREFIX, robot_name: str = "piper"):
    moveit_dir = share("piper_control") / "config/moveit"
    config = (
        MoveItConfigsBuilder("piper", package_name="piper_control")
        .robot_description(file_path=str(xacro_file()), mappings={"hardware": "mock", "gripper": gripper, "prefix": prefix})
        .robot_description_semantic(
            file_path="config/moveit/piper.srdf.xacro",
            mappings={"prefix": prefix, "gripper": gripper, "camera": camera, "robot_name": robot_name})
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path=str(render_config(moveit_dir / "joint_limits.yaml", prefix)))
        .trajectory_execution(file_path=str(render_config(moveit_dir / "moveit_controllers.yaml", prefix)))
        .pilz_cartesian_limits(file_path="config/moveit/pilz_cartesian_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"],
                            default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    config.robot_description = {"robot_description": urdf}  # the exact model the backend runs
    return config


def controllers_file(prefix: str) -> str:
    """controllers.yaml with the prefix filled in."""
    return str(render_config(share("piper_control") / "config/controllers.yaml", prefix))


# ---- nodes ----------------------------------------------------------------------------------------------------------------------------
def robot_state_publisher(config, sim_time: dict) -> Node:
    return Node(package="robot_state_publisher", executable="robot_state_publisher",
                parameters=[config.robot_description, sim_time], output="log")


# A loaded machine (a software-rendered simulator GUI, say) can take longer than the defaults to answer controller_manager services.
SPAWNER_TIMEOUTS = ["--controller-manager-timeout", "60", "--service-call-timeout", "30", "--switch-timeout", "30"]


def spawners(gripper: str, real: bool) -> list:
    """Spawn the controllers. On the real arm the command controllers start inactive (activate with `piper activate`)."""
    commanders = ["arm_controller"] + (["gripper_controller"] if gripper == "true" else [])
    if real:
        active, inactive = ["joint_state_broadcaster"], commanders + ["arm_position_controller"]
    else:
        active, inactive = ["joint_state_broadcaster", *commanders], ["arm_position_controller"]
    return [
        Node(package="controller_manager", executable="spawner",
             arguments=[*active, *SPAWNER_TIMEOUTS], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=[*inactive, "--inactive", *SPAWNER_TIMEOUTS], output="screen"),
    ]


def controller_manager_node(controllers: str, sim_time: dict) -> Node:
    return Node(package="controller_manager", executable="ros2_control_node",
                parameters=[controllers, sim_time], output="screen",
                remappings=[("~/robot_description", "/robot_description")])


def moveit_nodes(config, sim_time: dict, moveit: str, servo: str, rviz: str) -> list:
    nodes = [
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[config.to_dict(), sim_time], output="screen", condition=IfCondition(LaunchConfiguration("moveit"))),
        Node(package="rviz2", executable="rviz2", output="log",
             arguments=["-d", str(share("piper_control") / "rviz/moveit.rviz")],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines, config.joint_limits, sim_time],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]
    if servo == "true":
        servo_params = yaml.safe_load((share("piper_control") / "config/servo.yaml").read_text())
        servo_params["is_primary_planning_scene_monitor"] = moveit != "true"  # move_group owns the scene if present
        nodes.append(Node(package="moveit_servo", executable="servo_node", name="servo_node", output="screen",
                          parameters=[{"moveit_servo": servo_params},
                                      {"update_period": servo_params["publish_period"],  # read by the acceleration-limit smoothing plugin
                                       "planning_group_name": servo_params["move_group_name"]},
                                      config.robot_description, config.robot_description_semantic,
                                      config.robot_description_kinematics, config.joint_limits, sim_time]))
    return nodes


def real_arm_nodes(prefix: str, gripper: str, canbus: str, speed_percent: str, driver: str) -> list:
    """command_guard (the only name translator between ros2_control and the driver) and our driver (piper_control.real_driver, on piper_core.RealArm).
    The motors are never enabled by starting it: the `start` service does that, and moves to zero."""
    nodes = [Node(package="piper_control", executable="command_guard.py", name="command_guard", output="screen",
                  parameters=[{"prefix": prefix}])]
    if driver == "true":
        nodes.append(Node(
            package="piper_control", executable="real_driver.py", name="piper_real_driver", output="screen",
            parameters=[{"canbus": canbus, "speed_percent": int(speed_percent), "gripper": gripper == "true", "pub_rate": 200.0}]))
    return nodes
