"""Bring up the Piper arm on any backend with the same controllers and MoveIt configuration.

    ros2 launch piper_bringup piper.launch.py                       # mock hardware + MoveIt
    ros2 launch piper_bringup piper.launch.py rviz:=true
    ros2 launch piper_bringup piper.launch.py backend:=gazebo [gui:=true]
    ros2 launch piper_bringup piper.launch.py backend:=mujoco [gui:=true]
    ros2 launch piper_bringup piper.launch.py backend:=isaac [gui:=true]   # runs Isaac Sim (GPU) via scripts/isaac.sh
    ros2 launch piper_bringup piper.launch.py backend:=real can_port:=can0
    ... servo:=false                                              # without MoveIt Servo (TCP-frame jogging)

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

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import tempfile

from piper_description import SIMULATORS, mujoco_model, physics_model, robot_description, xacro_file
from piper_description.scene import gazebo_world, load_objects, mujoco_scene

BACKENDS = ["mock", "real", "gazebo", "mujoco", "isaac"]


def share(package: str) -> Path:
    return Path(get_package_share_directory(package))


def moveit_config(backend: str, gripper: str, urdf: str, camera: str = "none"):
    config = (
        MoveItConfigsBuilder("piper", package_name="piper_bringup")
        .robot_description(file_path=str(xacro_file()), mappings={"hardware": "mock", "gripper": gripper})
        .robot_description_semantic(
            file_path="config/moveit/piper.srdf.xacro",
            mappings={"effector_type": "agx_gripper" if gripper == "true" else "none", "camera": camera})
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


def gazebo(gui: bool, urdf: str, objects: list, camera: bool):
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
    world_file = tempfile.NamedTemporaryFile("w", prefix="piper_world_", suffix=".sdf", delete=False)
    world_file.write(gazebo_world(objects))
    world_file.close()
    camera_bridge = [Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
                          arguments=["/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                                     "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                                     "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
                          remappings=[("/camera/image", "/camera/color/image_raw"),
                                      ("/camera/depth_image", "/camera/aligned_depth_to_color/image_raw"),
                                      ("/camera/camera_info", "/camera/color/camera_info")])] if camera else []
    partition = os.environ.get("GZ_PARTITION") or f"piper_d{os.environ.get('ROS_DOMAIN_ID', '0')}"
    return [
        SetEnvironmentVariable("GZ_PARTITION", partition),
        # package:// mesh URIs resolve against GZ_SIM_RESOURCE_PATH entries that contain the package dir.
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share("agx_arm_description").parent)),
        # Server inside a ROS node: launch shutdown (SIGINT or SIGTERM) reliably stops it.
        # (gz_sim.launch.py runs `gz sim` through a shell that leaves the server orphaned on SIGTERM.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share("ros_gz_sim") / "launch/gz_server.launch.py")),
            launch_arguments={"world_sdf_file": world_file.name, "verbosity_level": "2"}.items()),
        *([ExecuteProcess(cmd=["gz", "sim", "-g", "-v", "2"], output="log")] if gui else []),
        Node(package="ros_gz_sim", executable="create", output="screen",
             arguments=["-file", model_file.name, "-name", "piper"]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
             arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]),
        *camera_bridge,
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
    camera = arg("camera")
    cam_map = {"camera": camera, "camera_bracket_mesh": arg("camera_bracket_mesh")} if camera != "none" else {}
    urdf = robot_description(backend, gripper=gripper, controllers_file=controllers, **cam_map)
    objects = load_objects(share("piper_bringup") / "config/scene_objects.yaml") if arg("scene") == "true" else []
    if backend == "mujoco":
        cfg = share("piper_bringup") / "config/mujoco"
        scene_text = Path(tempfile.gettempdir()) / f"piper_scene_{os.getpid()}.xml"
        scene_text.write_text(mujoco_scene(cfg / "scene.xml", objects))
        scene = mujoco_model(urdf, cfg / "inputs.xml", scene_text,
                             camera={"width": 640, "height": 480, "hfov_deg": 69.4} if camera != "none" else None)
        urdf = robot_description(backend, gripper=gripper, controllers_file=controllers,
                                 mujoco_model=scene, mujoco_headless=arg("gui") != "true", **cam_map)
    config = moveit_config(backend, gripper, urdf, camera)
    sim_time = {"use_sim_time": backend in SIMULATORS}
    commanders = ["arm_controller"] + (["gripper_controller"] if gripper == "true" else [])
    if backend == "real":
        active, inactive = ["joint_state_broadcaster"], commanders + ["arm_position_controller"]
    else:
        active, inactive = ["joint_state_broadcaster", *commanders], ["arm_position_controller"]

    spawners = [
        Node(package="controller_manager", executable="spawner",
             arguments=[*active, "--controller-manager-timeout", "60"], output="screen"),
        Node(package="controller_manager", executable="spawner",
             arguments=[*inactive, "--inactive", "--controller-manager-timeout", "60"], output="screen"),
    ]
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description, sim_time], output="log"),
        *([] if backend == "isaac" else spawners),  # isaac: started after /clock, below
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
    if arg("servo") == "true":
        servo_params = yaml.safe_load((share("piper_bringup") / "config/servo.yaml").read_text())
        servo_params["is_primary_planning_scene_monitor"] = arg("moveit") != "true"  # move_group owns the scene if present
        actions.append(Node(package="moveit_servo", executable="servo_node", name="servo_node", output="screen",
                            parameters=[{"moveit_servo": servo_params},
                                        {"update_period": servo_params["publish_period"],  # read by the acceleration-limit smoothing plugin
                                         "planning_group_name": servo_params["move_group_name"]},
                                        config.robot_description,
                                        config.robot_description_semantic, config.robot_description_kinematics,
                                        config.joint_limits, sim_time]))
    if backend in ("mock", "real", "isaac"):
        control = Node(package="controller_manager", executable="ros2_control_node",
                       parameters=[controllers, sim_time], output="screen",
                       remappings=[("~/robot_description", "/robot_description")])
        if backend == "isaac":
            # Controllers on sim time cannot activate before Isaac's /clock ticks; Isaac takes ~30 s to start.
            wait = Node(package="piper_bringup", executable="wait_for_clock.py", output="screen",
                        arguments=["240"], parameters=[{"use_sim_time": False}])
            actions.append(wait)
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=wait, on_exit=[control, *spawners])))
        else:
            actions.append(control)
    if backend == "isaac" and arg("isaac_runner") == "true":
        # Isaac Sim is a separate process in its own environment; it must use this launch's ROS_DOMAIN_ID.
        actions.append(ExecuteProcess(
            cmd=[arg("isaac_script"), "run", *(["--gui"] if arg("gui") == "true" else []),
                 *(["--video", arg("isaac_video")] if arg("isaac_video") else [])],
            output="screen", sigterm_timeout="20", sigkill_timeout="30"))
    if backend == "gazebo":
        actions += gazebo(arg("gui") == "true", urdf, objects, camera != "none")
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
        DeclareLaunchArgument("scene", default_value="true", choices=["true", "false"],
                              description="simulators: add the shared static objects (config/scene_objects.yaml)"),
        DeclareLaunchArgument("camera", default_value="none", choices=["none", "d435"],
                              description="wrist RealSense D435 (simulated image/depth topics under /camera)"),
        DeclareLaunchArgument("camera_bracket_mesh", default_value="",
                              description="optional path of a bracket mesh (e.g. AgileX's realsense_mid_stand.dae)"),
        DeclareLaunchArgument("servo", default_value="true", choices=["true", "false"],
                              description="start MoveIt Servo (TCP-frame jogging via the streaming controller)"),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("gui", default_value="false", choices=["true", "false"],
                              description="simulators: show the simulator GUI"),
        DeclareLaunchArgument("isaac_runner", default_value="true", choices=["true", "false"],
                              description="isaac only: start Isaac Sim (false: you run scripts/isaac.sh yourself)"),
        DeclareLaunchArgument("isaac_script", default_value=os.path.join(os.environ.get("PIPER_STUDIO_WS", "."), "scripts/isaac.sh"),
                              description="isaac only: path of scripts/isaac.sh"),
        DeclareLaunchArgument("isaac_video", default_value="",
                              description="isaac only: record the Isaac camera view to this mp4"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("auto_enable", default_value="false", choices=["true", "false"],
                              description="real only: enable motors at driver start"),
        DeclareLaunchArgument("driver", default_value="true", choices=["true", "false"],
                              description="real only: start agx_arm_ctrl (false: bring your own, e.g. fake_driver)"),
        OpaqueFunction(function=launch_setup),
    ])
