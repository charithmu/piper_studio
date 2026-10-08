"""Bring up the Piper arm on any backend with the same controllers and MoveIt configuration.

    ros2 launch piper_bringup piper.launch.py                       # mock hardware + MoveIt
    ros2 launch piper_bringup piper.launch.py rviz:=true
    ros2 launch piper_bringup piper.launch.py backend:=real can_port:=can0

backend:=real starts the official agx_arm_ctrl driver. Safety defaults:
  * the driver does NOT enable the motors unless auto_enable:=true; enable explicitly with
    `ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"` while someone is at the arm;
  * arm_controller and gripper_controller start INACTIVE. Activate them with `piper activate`
    (piper_py), which first checks that driver feedback is fresh and matches ros2_control state.
    A trajectory controller holds the position it reads at activation; activating before real
    feedback arrives would command the arm toward the URDF initial pose.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

BACKENDS = ["mock", "real"]


def moveit_config(backend: str, gripper: str):
    description = Path(get_package_share_directory("piper_description")) / "urdf/piper.urdf.xacro"
    return (
        MoveItConfigsBuilder("piper", package_name="piper_bringup")
        .robot_description(file_path=str(description), mappings={"hardware": backend, "gripper": gripper})
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


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    backend, gripper = arg("backend"), arg("gripper")
    if backend not in BACKENDS:
        raise RuntimeError(f"backend:={backend} is not supported; choose one of {BACKENDS}")
    config = moveit_config(backend, gripper)
    controllers = str(Path(get_package_share_directory("piper_bringup")) / "config/controllers.yaml")
    commanders = ["arm_controller"] + (["gripper_controller"] if gripper == "true" else [])
    if backend == "real":
        active, inactive = ["joint_state_broadcaster"], commanders + ["arm_position_controller"]
    else:
        active, inactive = ["joint_state_broadcaster", *commanders], ["arm_position_controller"]

    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description], output="log"),
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[controllers], output="screen",
             remappings=[("~/robot_description", "/robot_description")]),
        Node(package="controller_manager", executable="spawner",
             arguments=[*active, "--controller-manager-timeout", "30"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=[*inactive, "--inactive", "--controller-manager-timeout", "30"], output="screen"),
        Node(package="moveit_ros_move_group", executable="move_group",
             parameters=[config.to_dict()], output="screen",
             condition=IfCondition(LaunchConfiguration("moveit"))),
        Node(package="rviz2", executable="rviz2", output="log",
             arguments=["-d", str(Path(get_package_share_directory("piper_bringup")) / "rviz/moveit.rviz")],
             parameters=[config.robot_description, config.robot_description_semantic,
                         config.robot_description_kinematics, config.planning_pipelines, config.joint_limits],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ]
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
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("auto_enable", default_value="false", choices=["true", "false"],
                              description="real only: enable motors at driver start"),
        DeclareLaunchArgument("driver", default_value="true", choices=["true", "false"],
                              description="real only: start agx_arm_ctrl (false: bring your own, e.g. fake_driver)"),
        OpaqueFunction(function=launch_setup),
    ])
