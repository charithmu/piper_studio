"""The Piper control stack on mock hardware, on the real arm, or on top of a controller manager that someone else runs.

    ros2 launch piper_control control.launch.py                    # mock hardware + MoveIt (headless)
    ros2 launch piper_control control.launch.py rviz:=true
    ros2 launch piper_control control.launch.py backend:=real canbus:=can0 speed_percent:=20
    ros2 launch piper_control control.launch.py backend:=external sim_time:=true   # your own simulator or hardware owns the controller manager

Simulators (Gazebo, MuJoCo, Isaac) are in piper_sim's piper.launch.py, which composes the same pieces (piper_control.launch_lib).

backend:=real starts the official agx_arm_ctrl driver and command_guard. Safety defaults:
  * the arm speed is set explicitly (speed_percent, default 20 %) instead of whatever the arm powered up with;
  * the driver does NOT enable the motors unless auto_enable:=true; enable explicitly with `piper enable` (or the /enable_agx_arm service);
  * arm_controller and gripper_controller start INACTIVE. Activate them with `piper activate` (piper_py), which first checks that driver
    feedback is fresh and matches ros2_control state. A trajectory controller holds the position it reads at activation; activating before
    real feedback arrives would command the arm toward the URDF initial pose.
backend:=external starts only the spawners (they wait for /controller_manager), MoveIt, Servo and RViz; robot_description must come from
whoever runs the hardware (use piper_description's macros with your hardware plugin and the controllers_file this launch renders;
see src/piper_examples for a carrier robot), and robot_description_file:= gives MoveIt the URDF of your whole robot.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from piper_control import launch_lib as lib
from piper_description import normalize_prefix, robot_description

BACKENDS = ["mock", "real", "external"]


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    backend = arg("backend")
    prefix, gripper, camera = normalize_prefix(arg("prefix")), arg("gripper"), arg("camera")
    controllers = lib.controllers_file(prefix)
    hardware = "mock" if backend == "external" else backend
    if arg("robot_description_file"):  # backend:=external: the model of the whole robot (e.g. a carrier with the Piper mounted on it)
        urdf = Path(arg("robot_description_file")).read_text()
    else:
        urdf = robot_description(hardware, controllers_file=controllers, **lib.description_mappings(arg))
    config = lib.moveit_config(gripper, urdf, camera, prefix, arg("robot_name"))
    sim_time = {"use_sim_time": arg("sim_time") == "true"}
    actions = [lib.robot_state_publisher(config, sim_time), *lib.spawners(gripper, backend == "real"),
               *lib.moveit_nodes(config, sim_time, arg("moveit"), arg("servo"), arg("rviz"))]
    if backend != "external":
        actions.append(lib.controller_manager_node(controllers, sim_time))
    if backend == "real":
        actions += lib.real_arm_nodes(prefix, gripper, arg("canbus"), arg("speed_percent"), arg("auto_enable"), arg("driver"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mock", choices=BACKENDS),
        DeclareLaunchArgument("sim_time", default_value="false", choices=["true", "false"],
                              description="use /clock (backend:=external with a simulator)"),
        DeclareLaunchArgument("robot_name", default_value="piper",
                              description="name of the URDF MoveIt loads (the <robot name> of robot_description_file); the SRDF must carry the same name"),
        DeclareLaunchArgument("robot_description_file", default_value="",
                              description="backend:=external: URDF file of your whole robot for MoveIt (default: the Piper alone)"),
        *lib.description_arguments(), *lib.control_arguments(), *lib.real_arm_arguments(),
        OpaqueFunction(function=launch_setup),
    ])
