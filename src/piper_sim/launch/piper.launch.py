"""Bring up the Piper arm on any backend (mock, real, Gazebo, MuJoCo, Isaac Sim) with the same controllers and MoveIt configuration.

    ros2 launch piper_sim piper.launch.py                       # mock hardware + MoveIt
    ros2 launch piper_sim piper.launch.py rviz:=true
    ros2 launch piper_sim piper.launch.py backend:=gazebo [gui:=true]
    ros2 launch piper_sim piper.launch.py backend:=mujoco [gui:=true]
    ros2 launch piper_sim piper.launch.py backend:=isaac [gui:=true]   # runs Isaac Sim (GPU) via scripts/isaac.sh
    ros2 launch piper_sim piper.launch.py backend:=real canbus:=can0 speed_percent:=20
    ... servo:=false                                              # without MoveIt Servo (TCP-frame jogging)

backend:=real starts the official agx_arm_ctrl driver. Safety defaults:
  * the arm speed is set explicitly (speed_percent, default 20 %) instead of whatever the arm powered up with;
  * the driver does NOT enable the motors unless auto_enable:=true; enable explicitly with
    `ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"` while someone is at the arm;
  * arm_controller and gripper_controller start INACTIVE. Activate them with `piper activate`
    (piper_py), which first checks that driver feedback is fresh and matches ros2_control state.
    A trajectory controller holds the position it reads at activation; activating before real
    feedback arrives would command the arm toward the URDF initial pose.
"""

import json
import os
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from piper_control import launch_lib as lib
from piper_control.launch_lib import share
from piper_description import normalize_prefix
from piper_sim import SIMULATORS, mujoco_model, physics_model, robot_description
from piper_sim.scene import gazebo_world, load_objects, mujoco_scene

BACKENDS = ["mock", "real", "gazebo", "mujoco", "isaac"]


def gazebo(gui: bool, urdf: str, objects: list, camera: bool):
    """Gazebo Harmonic: empty world, controller manager in gz_ros2_control.

    The physics model (piper_sim.physics_model) has no URDF mimic constraints, since
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
    world_file.write(gazebo_world(objects, overview=bool(camera)))
    world_file.close()
    camera_bridge = [Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
                          arguments=["/overview/image@sensor_msgs/msg/Image[gz.msgs.Image",
                                     "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                                     "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                                     "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
                          remappings=[("/overview/image", "/overview/image_raw"),
                                      ("/camera/image", "/camera/color/image_raw"),
                                      ("/camera/depth_image", "/camera/aligned_depth_to_color/image_raw"),
                                      ("/camera/camera_info", "/camera/color/camera_info")])] if camera else []
    partition = os.environ.get("GZ_PARTITION") or f"piper_d{os.environ.get('ROS_DOMAIN_ID', '0')}"
    return [
        SetEnvironmentVariable("GZ_PARTITION", partition),
        # package:// mesh URIs resolve against GZ_SIM_RESOURCE_PATH entries that contain the package dir.
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share("agx_arm_description").parent)),
        # Headless server with EGL rendering (camera sensors need it); the wrapper stops the whole process group.
        ExecuteProcess(cmd=[str(share("piper_sim").parent.parent / "lib/piper_sim/gz_server.sh"), world_file.name],
                       output="screen", sigterm_timeout="2", sigkill_timeout="15"),
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
    backend, gripper, camera, prefix = arg("backend"), arg("gripper"), arg("camera"), normalize_prefix(arg("prefix"))
    if backend not in BACKENDS:
        raise RuntimeError(f"backend:={backend} is not supported; choose one of {BACKENDS}")
    controllers = lib.controllers_file(prefix)
    mappings = lib.description_mappings(arg)
    urdf = robot_description(backend, controllers_file=controllers, **mappings)
    objects = load_objects(share("piper_sim") / "config/scene_objects.yaml") if arg("scene") == "true" else []
    if backend == "mujoco":
        cfg = share("piper_sim") / "config/mujoco"
        scene_text = Path(tempfile.gettempdir()) / f"piper_scene_{os.getpid()}.xml"
        scene_text.write_text(mujoco_scene(cfg / "scene.xml", objects))
        scene = mujoco_model(urdf, cfg / "inputs.xml", scene_text, prefix=prefix,
                             camera={"width": 640, "height": 480, "hfov_deg": 69.4} if camera != "none" else None)
        urdf = robot_description(backend, controllers_file=controllers, mujoco_model=scene, mujoco_headless=arg("gui") != "true", **mappings)
    config = lib.moveit_config(gripper, urdf, camera, prefix)
    sim_time = {"use_sim_time": backend in SIMULATORS}
    spawners = lib.spawners(gripper, backend == "real")
    actions = [lib.robot_state_publisher(config, sim_time),
               *([] if backend == "isaac" else spawners),  # isaac: started after /clock, below
               *lib.moveit_nodes(config, sim_time, arg("moveit"), arg("servo"), arg("rviz"))]
    if backend in ("mock", "real", "isaac"):
        control = lib.controller_manager_node(controllers, sim_time)
        if backend == "isaac":
            # Controllers on sim time cannot activate before Isaac's /clock ticks; Isaac takes ~30 s to start.
            wait = Node(package="piper_sim", executable="wait_for_clock.py", output="screen",
                        arguments=["240"], parameters=[{"use_sim_time": False}])
            actions.append(wait)
            actions.append(RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=[control, *spawners])))
        else:
            actions.append(control)
    if backend == "isaac" and arg("isaac_runner") == "true":
        # Isaac Sim is a separate process in its own environment; it must use this launch's ROS_DOMAIN_ID.
        scene_json = []
        if objects:
            scene_file = Path(tempfile.gettempdir()) / f"piper_isaac_scene_{os.getpid()}.json"
            scene_file.write_text(json.dumps(objects))
            scene_json = ["--scene", str(scene_file)]
        actions.append(ExecuteProcess(
            cmd=[arg("isaac_script"), "run", *(["--gui"] if arg("gui") == "true" else []),
                 *(["--video", arg("isaac_video")] if arg("isaac_video") else []),
                 "--prefix", prefix, *(["--camera", camera] if camera != "none" else []), *scene_json],
            output="screen", sigterm_timeout="20", sigkill_timeout="30"))
    if backend == "gazebo":
        actions += gazebo(arg("gui") == "true", urdf, objects, camera != "none")
    if backend == "mujoco":
        actions += mujoco(controllers)
    if backend == "real":
        actions += lib.real_arm_nodes(prefix, gripper, arg("canbus"), arg("speed_percent"), arg("auto_enable"), arg("driver"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mock", choices=BACKENDS),
        DeclareLaunchArgument("scene", default_value="true", choices=["true", "false"],
                              description="simulators: add the shared static objects (config/scene_objects.yaml)"),
        *lib.description_arguments(), *lib.control_arguments(),
        DeclareLaunchArgument("gui", default_value="false", choices=["true", "false"],
                              description="simulators: show the simulator GUI"),
        DeclareLaunchArgument("isaac_runner", default_value="true", choices=["true", "false"],
                              description="isaac only: start Isaac Sim (false: you run scripts/isaac.sh yourself)"),
        DeclareLaunchArgument("isaac_script", default_value=os.path.join(os.environ.get("PIPER_STUDIO_WS", "."), "scripts/isaac.sh"),
                              description="isaac only: path of scripts/isaac.sh"),
        DeclareLaunchArgument("isaac_video", default_value="",
                              description="isaac only: record the Isaac camera view to this mp4"),
        *lib.real_arm_arguments(),
        OpaqueFunction(function=launch_setup),
    ])
