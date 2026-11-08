# Using the Piper from another project

For projects that want the Piper without adopting the whole repo: an ML endpoint with its own simulator, a Go2 carrying the arm, a
planner that produces joint trajectories (cuRobo), or a script with no ROS at all. Design background: [ARCHITECTURE.md](ARCHITECTURE.md).
Status of the parts marked *(planned)*: [CLEANUP.md](CLEANUP.md).

You never need the simulators, scenes or demos of this repo. Take only the layers you need:

| You want | Take | ROS needed |
|---|---|---|
| The robot model in your own world or on your own robot | `piper_description` (+ the official AgileX submodule it includes) | for xacro only |
| The same controllers and MoveIt setup | + `piper_control` | yes |
| Python client for state, trajectories, gripper, MoveIt, Servo | + `piper_py` | yes |
| The safety rules (limits, command guard, named poses) in plain Python | `piper_core` | **no** |
| Direct control of the real arm without ROS | `piper_core.RealArm` *(planned, B3)*; today `tools/real_arm/` | no |

Get the sources with `git clone --recursive` (the official AgileX stack `external/agx_arm_ros` is a submodule and is required: the
meshes and the SRDF are generated from it). Pin a commit or tag; see `piper.repos`.

## 1. The model, mounted on your robot

`piper_description` provides xacro macros. All names start with `prefix` (default `piper_`; `prefix:=none` in launch files for no prefix), so they cannot clash with your robot.

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <!-- your own links ... -->
  <xacro:include filename="$(find piper_description)/urdf/piper_arm.xacro"/>
  <xacro:piper_arm parent="trunk" prefix="piper_" xyz="0.15 0 0.05" rpy="0 0 0" gripper="true"/>

  <xacro:include filename="$(find piper_description)/urdf/piper.tool.xacro"/>
  <xacro:piper_tool_frames prefix="piper_" gripper="true" tcp_xyz="0 0 0.138" tcp_rpy="0 0 0"/>

  <!-- optional wrist camera: one camera model + one mount -->
  <xacro:include filename="$(find piper_description)/urdf/cameras/d435.xacro"/>
  <xacro:include filename="$(find piper_description)/urdf/mounts/agilex_mid_stand.xacro"/>
  <xacro:include filename="$(find piper_description)/urdf/piper_camera.xacro"/>
  <xacro:piper_camera prefix="piper_" parent="piper_link6" hardware="mock" mount_mesh="" mount_xyz="0 0 0" mount_rpy="0 0 0"
                      width="640" height="480" hfov_deg="69.4" update_rate="30"/>
</robot>
```

- Root: the macro attaches `${prefix}base_link` to `parent` with a fixed joint `${prefix}mount_joint`.
- ros2_control: `piper_ros2_control` (`urdf/piper.ros2_control.xacro`) adds the hardware block; `hardware` is `mock | real | gazebo | mujoco | isaac`. For **your own
  simulator**, write your hardware plugin into that block (or your own `<ros2_control>`) with the same joint names and one `position` command interface per joint.
- A complete example, tested: `src/piper_examples` (carrier robot with its own controller manager, prefix `arm_`, mounted at an offset, with the camera). Its launch shows the three steps: expand your own description, run your own controller manager with `piper_control.launch_lib.controllers_file(prefix)`, include `piper_control control.launch.py backend:=external prefix:=arm_ robot_name:=<your robot name> robot_description_file:=<your URDF> camera:=d435` (MoveIt needs the URDF of your whole robot, its name in the SRDF, and `camera:=` to know the camera links).
- Standalone (the arm at the world origin): `piper_description/urdf/piper.urdf.xacro`, or `ros2 launch piper_description view.launch.py`.
- Another camera or mount (ZED, OAK-D Pro, a CAD-exported bracket): add `urdf/cameras/<model>.xacro` defining `piper_camera_model(prefix, parent, hardware, width, height, hfov_deg, update_rate, *origin)`
  or `urdf/mounts/<mount>.xacro` defining `piper_camera_mount(prefix, parent, mesh, xyz, rpy)` and properties `piper_mount_camera_xyz/rpy`, then select with `camera:=` / `camera_mount:=`.

## 2. The ROS interface (what `piper_control` + `piper_py` give you)

Names below are for the default prefix; with another prefix, joints and frames follow it. Controllers, actions and topics do not.

| Item | Name | Notes |
|---|---|---|
| Arm joints | `piper_joint1..6` | radians; limits come from the URDF (`piper_core.limits_from_urdf`) |
| Gripper | `piper_gripper` | opening **width in metres**, 0 to 0.1; the fingers (`piper_gripper_joint1/2`) are mimic joints |
| Frames | `world` (caller's), `piper_base_link`, `piper_link1..6`, `piper_tcp_link`, camera: `piper_camera_link`, `piper_camera_color_optical_frame`, `piper_camera_depth_optical_frame` | the TCP is the fingertip centre, `piper_gripper_base` + 0.138 m z |
| State | `/joint_states` | |
| Trajectories | action `/arm_controller/follow_joint_trajectory` | `joint_trajectory_controller` |
| Streaming targets | topic `/arm_position_controller/commands` | `position_controllers/JointGroupPositionController`; teleop, policies, Servo |
| Gripper | action `/gripper_controller/gripper_cmd` (`ParallelGripperCommand`) | width in metres |
| Planning | MoveIt `move_group`, group `arm` (end effector group `gripper`) | OMPL and Pilz (LIN, PTP, CIRC) |
| Servo | MoveIt Servo `~/delta_twist_cmds`, `~/pose_target_cmds`, `~/delta_joint_cmds` | publishes to the streaming controller |
| Named poses | SRDF group states `zero`, `rest`, `ready` | `zero`: start and end pose of every session; `rest`: park pose of the real arm |
| Camera | `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/aligned_depth_to_color/image_raw` | RealSense topic names, in simulation and (planned) with the real driver |

**One command owner:** exactly one of `arm_controller` and `arm_position_controller` is active; switch explicitly (`Piper.use_streaming()` / `use_trajectories()`).
`controllers.yaml`, the MoveIt yaml and the MuJoCo inputs are templates with a `${prefix}` token; fill them with `piper_description.render_config(path, prefix)`.

Python client (`piper_py`): `Piper(prefix="piper_")`, then `joint_state()`, `move_joints()`, `stream_joints()`, `gripper()`, `move_named()`, `move_pose()`,
`servo_twist()`, `image()`, `tcp_pose()`. Every call returns a `Result` and **success means executed**, not planned.

## 3. Without ROS: `piper_core`

Plain Python, nothing to install but the package (`pip install src/piper_core`).

```python
from piper_core import GuardConfig, check_command, limits_from_urdf

limits = limits_from_urdf(open("piper.urdf").read())               # from the generated URDF
cfg = GuardConfig(arm_joints=tuple(f"joint{i}" for i in range(1, 7)))
verdict = check_command(names, positions, limits, measured, cfg)   # verdict.ok, verdict.reason, verdict.positions (clamped)
```

Apply the guard to every point of a planner's joint trajectory before it goes to the arm. It is the same function the ROS `command_guard` uses. A direct driver
wrapper (`RealArm`: state, gripper, enable/disable, `execute(trajectory)` with the guard) is *planned*; until then use `tools/real_arm/` (SDK scripts) and read
`docs/real_arm/REAL_ARM.md` before touching the real arm.

## 4. Real arm rules (do not weaken in a wrapper)

Motors are not enabled at start. The arm speed is set explicitly. Command controllers start inactive and are activated only after fresh, consistent feedback
(`piper activate`). Commands pass the guard. Every session starts and ends at `zero`; the real arm is parked at `rest` before the motors are disabled.
Anything that can move the real arm needs the user present and approving.

## 5. Versioning

This file is the contract. Renaming a controller, topic, frame, macro parameter or macro file is a breaking change: record it in `docs/CLEANUP.md` and
`comms/FOR_OTHERS.md`. Adding a macro parameter with a default, a camera or a mount is not. The names changed once, on 2026-11-08 (the `piper_` prefix).
