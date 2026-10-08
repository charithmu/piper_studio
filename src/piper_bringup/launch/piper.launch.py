"""Bring up the Piper arm on any backend with the same controllers and MoveIt configuration.

    ros2 launch piper_bringup piper.launch.py                       # mock hardware + MoveIt
    ros2 launch piper_bringup piper.launch.py rviz:=true
    ros2 launch piper_bringup piper.launch.py backend:=gazebo [gui:=true]
    ros2 launch piper_bringup piper.launch.py backend:=mujoco [gui:=true]
    ros2 launch piper_bringup piper.launch.py backend:=real can_port:=can0

backend:=real starts the official agx_arm_ctrl driver. Safety defaults:
  * the driver does NOT enable the motors unless auto_enable:=true; enable explicitly with
    `ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"` while someone is at the arm;
  * arm_controller and gripper_controller start INACTIVE. Activate them with `piper activate`
    (piper_py), which first checks that driver feedback is fresh and matches ros2_control state.
    A trajectory controller holds the position it reads at activation; activating before real
    feedback arrives would command the arm toward the URDF initial pose.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import tempfile

from piper_description import SIMULATORS, mujoco_model, physics_model, robot_description, xacro_file

BACKENDS = ["mock", "real", "gazebo", "mujoco"]


def share(package: str) -> Path:
    return Path(get_package_share_directory(package))


def moveit_config(backend: str, gripper: str, urdf: str):
    config = (
        MoveItConfigsBuilder("piper", package_name="piper_bringup")
        .robot_description(file_path=str(xacro_file()), mappings={"hardware": "mock", "gripper": gripper})
        .robot_description_semantic(
            file_path="config/moveit/piper.srdf.xacro",
            mappings={"effector_type": "agx_gripper" if gripper == "true" else "none"})
        .robot_description_kinematics(file_path="config/moveit/kinematics.yaml")
        .joint_limits(file_path="config/moveit/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit/moveit_controllers.yaml")
        .pilz_cartesian_limits(file_path="config/moveit/pilz_cartesian_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"],
                            default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    config.robot_description = {"robot_description": urdf}  # the exact model the backend runs
    return config


def gazebo(gui: bool, urdf: str):
    """Gazebo Harmonic: empty world, controller manager in gz_ros2_control.

    The physics model (piper_description.physics_model) has no URDF mimic constraints, since
    gz_ros2_control drives the mimic fingers itself from the full URDF in robot_state_publisher, and
    slightly widened hard stops, since DART cannot move a joint off a limit it rests on.
    """
    physics_model_urdf = physics_model(urdf)
    model_file = tempfile.NamedTemporaryFile("w", prefix="piper_gz_", suffix=".urdf", delete=False)
    model_file.write(physics_model_urdf)
    model_file.close()
    # gz-transport is not scoped by ROS_DOMAIN_ID; give each domain its own Gazebo partition so
    # parallel simulations (and tests) cannot cross-talk. Inspect with: GZ_PARTITION=<value> gz topic -l
    partition = os.environ.get("GZ_PARTITION") or f"piper_d{os.environ.get('ROS_DOMAIN_ID', '0')}"
    return [
        SetEnvironmentVariable("GZ_PARTITION", partition),
        # package:// mesh URIs resolve against GZ_SIM_RESOURCE_PATH entries that contain the package dir.
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share("agx_arm_description").parent)),
        # Server inside a ROS node: launch shutdown (SIGINT or SIGTERM) reliably stops it.
        # (gz_sim.launch.py runs `gz sim` through a shell that leaves the server orphaned on SIGTERM.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share("ros_gz_sim") / "launch/gz_server.launch.py")),
            launch_arguments={"world_sdf_file": "empty.sdf", "verbosity_level": "2"}.items()),
        *([ExecuteProcess(cmd=["gz", "sim", "-g", "-v", "2"], output="log")] if gui else []),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-file", model_file.name, "-name", "piper"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),
    ]


def mujoco(controllers: str):
    """MuJoCo: mujoco_ros2_control's controller manager steps the generated model."""
    return [Node(package="mujoco_ros2_control", executable="ros2_control_node", output="screen",
                 parameters=[controllers, {"use_sim_time": True}])]


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    backend, gripper = arg("backend"), arg("gripper")
    if backend not in BACKENDS:
        raise RuntimeError(f"backend:={backend} is not supported; choose one of {BACKENDS}")
    controllers = str(share("piper_bringup") / "config/controllers.yaml")
    urdf = robot_description(backend, gripper=gripper, controllers_file=controllers)
    if backend == "mujoco":
        cfg = share("piper_bringup") / "config/mujoco"
        scene = mujoco_model(urdf, cfg / "inputs.xml", cfg / "scene.xml")
        urdf = robot_description(backend, gripper=gripper, controllers_file=controllers,
                                 mujoco_model=scene, mujoco_headless=arg("gui") != "true")
    config = moveit_config(backend, gripper, urdf)
    sim_time = {"use_sim_time": backend in SIMULATORS}
    commanders = ["arm_controller"] + (["gripper_controller"] if gripper == "true" else [])
    if backend == "real":
        active, inactive = ["joint_state_broadcaster"], commanders + ["arm_position_controller"]
    else:
        active, inactive = ["joint_state_broadcaster", *commanders], ["arm_position_controller"]

    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description, sim_time], output="log"),
        Node(package="controller_manager", executable="spawner",
             arguments=[*active, "--controller-manager-timeout", "30"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=[*inactive, "--inactive", "--controller-manager-timeout", "30"], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[config.to_dict(), sim_time], output="screen",
             condition=IfCondition(LaunchConfiguration("moveit"))),
        Node(package="rviz2", executable="rviz2", output="log",
             arguments=["-d", str(share("piper_bringup") / "rviz/moveit.rviz")],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines, config.joint_limits,
                         sim_time],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]
    if backend in ("mock", "real"):
        actions.append(Node(package="controller_manager", executable="ros2_control_node",
                            parameters=[controllers], output="screen",
                            remappings=[("~/robot_description", "/robot_description")]))
    if backend == "gazebo":
        actions += gazebo(arg("gui") == "true", urdf)
    if backend == "mujoco":
        actions += mujoco(controllers)
    if backend == "real":
        actions.append(Node(package="piper_bringup", executable="command_guard.py", name="command_guard",
                            output="screen"))
    if backend == "real" and arg("driver") == "true":
        actions.append(Node(
            package="agx_arm_ctrl", executable="agx_arm_ctrl_single", name="agx_arm_ctrl",
            output="screen",
            parameters=[{
                "can_port": arg("can_port"),
                "arm_type": "piper",
                "effector_type": "agx_gripper" if gripper == "true" else "none",
                "auto_enable": arg("auto_enable") == "true",
                "pub_rate": 200,
            }],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mock", choices=BACKENDS),
        DeclareLaunchArgument("gripper", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("moveit", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("gui", default_value="false", choices=["true", "false"],
                              description="simulators: show the simulator GUI"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("auto_enable", default_value="false", choices=["true", "false"],
                              description="real only: enable motors at driver start"),
        DeclareLaunchArgument("driver", default_value="true", choices=["true", "false"],
                              description="real only: start agx_arm_ctrl (false: bring your own, e.g. fake_driver)"),
        OpaqueFunction(function=launch_setup),
    ])
