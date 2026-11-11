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
| The safety rules (limits, command guard, named poses) in plain Python | `piper_core` (pip wheel) | **no** |
| The arm as data: flat URDF, meshes, SRDF, limits with sources, scene | `piper_assets` (pip wheel) | **no** |
| Run planner trajectories in MuJoCo | `piper_mujoco` (pip wheel) | **no** |
| The real-arm tools as commands (e-stop, info, session, CAN up), also on a box | `piper_tools` (pip wheel, needs `piper_core` and the vendor SDK pyAgxArm) | **no** |
| Run planner trajectories in Isaac Sim | `piper_isaac` (pip wheel; Isaac Sim's own environment and the GPU on the same machine) | **no** |
| Direct control of the real arm without ROS | `piper_core.RealArm` (tested on a fake SDK only; the first real run needs the user) | no |

## 0. Python packages (no ROS): the way for planners such as cuRobo

**The wrist camera is on by default (0.8.0, breaking; the camera names became the ZED wrapper's in 0.9.0, breaking).** Every description, URDF, SRDF, simulator model and wheel carries the Stereolabs **ZED X Mini on the user's own clamp bracket**
(`camera` = `zedxm`) unless you ask for the robot only: `camera:=none` (xacro, launch, `export_urdf.py --camera none`), `camera=None` (`piper_assets.urdf_path`, `srdf_path`), `camera=False`
(`PiperSim`, `PiperIsaacSim`), `build-usd no_camera`. The camera's frames are Stereolabs' own (`piper_zed_camera_link` = the camera's mounting hole, `piper_zed_camera_center`, `piper_zed_left_camera_frame` and `piper_zed_left_camera_frame_optical` = the **left lens**, z forward, x right, y down, in which colour, depth and camera_info are published; `piper_zed_right_camera_frame(_optical)`), so the ZED ROS 2 wrapper started with `camera_name:=piper_zed` and its TF switched off (`ros2 launch piper_control zed_camera.launch.py`) fits this description. The images are rectified pinhole images; size, field of view and depth range come from
one file, `piper_description/config/camera_models/zedxm.yaml` (default: wide 2.2 mm lens, 960x600, 110 deg horizontal, 0.1 to 8 m, 30 Hz; `camera_lens:=narrow`, `camera_width`, `camera_height` override).
The bracket and its pose are versioned (`config/camera_mounts/zedxm_bracket_v1..v3.yaml`, default the newest; `camera_calibration:=zedxm_bracket_v2` or a yaml path with a measured pose).
**Not calibrated:** the pose is nominal, from the user's CAD (`measured: null`); hand-eye calibration on the real arm is open. A planner that wants the robot only passes `camera=None` explicitly
(a call that already did so, like `piper_assets.urdf_path(0.07, None)`, is unchanged).

Four wheels, one version, built into `dist/` from the tagged commit by `scripts/build_wheels.sh` (needs the ROS environment once, to generate the data). A downstream project installs them into its own venv (Python 3.10 or newer) and never reads this repo's `install/`:

```bash
uv pip install --no-deps piper_core-0.10.1-py3-none-any.whl piper_assets-0.10.1-py3-none-any.whl piper_mujoco-0.10.1-py3-none-any.whl piper_isaac-0.10.1-py3-none-any.whl
```

| Package | Gives | Needs |
|---|---|---|
| `piper_core` | `Trajectory`, `ExecResult`, `ArmBackend` (`piper_core.backend`), `check_trajectory`, the guard, `zero_pose`, `RealArm`, `Kinematics` (`piper_core.kinematics`: link, TCP and camera poses from the URDF), `resolve_canbus` (`piper_core.canbus`: the USB-CAN adapter's interface, by USB not by name) | numpy |
| `piper_assets` | `urdf_path(gripper_width, camera=True)`, `srdf_path(camera=True)`, `camera()` (K, size, field of view, depth range, optical frame, its pose in gripper_base / the TCP / link6), `camera_pose(frame)`, `limits()` (hard, planning, velocity, acceleration, jerk, effort, each with its source), `zero_pose()`, `scene()` (placeholder table), `manifest()` | numpy, `piper_core`, **the cuRobo model** `curobo_model(camera)`, `curobo_config(camera)`, `curobo_provenance(camera)`, `curobo_model_is_current(camera)`, `curobo_planner_defaults()` (collision spheres, limits, ignore list; tuned and regenerated here: `docs/CUROBO.md`) | numpy (pyyaml for `curobo_config`) |
| `piper_mujoco` | `PiperSim(scene="empty"|"table", gripper_width=0.07, camera=True)`: an `ArmBackend` on MuJoCo; `camera_image(depth=False)` renders the wrist camera (RGB, or RGB and metric depth), `camera_pose()` its optical frame in the base frame, `camera_info()` | `mujoco`, the two above |
| `piper_isaac` | `PiperIsaacSim(scene="empty"|"table", gripper_width=0.07)`: an `ArmBackend` on Isaac Sim; starts `isaac/run_server.py` in Isaac's environment through `gpu-run` (`ISAAC_ENV_SH`), the runner and the USDs (with the camera, and the robot only) are inside the wheel; `camera=True` by default: `camera_image(depth)`, `camera_pose()`, `camera_info()` as in `PiperSim`; `render=True` and `record_start(path)` record a video; `gui=True` opens the full editor, also available as `python -m piper_isaac --gui` | the two above; Isaac Sim 6.x and a GPU at run time, not to install |

**Between releases (development path).** A release (tag, `dist/`) is for milestones. To try a change before it is released, the maintainer runs `scripts/build_wheels.sh --dev`, which builds a snapshot of the working tree into `dist-dev/` (same version numbers; the source commit inside gets `-dirty` when uncommitted), and the downstream project installs it with `uv pip install --no-deps --reinstall <repo>/dist-dev/*.whl`. No commit, push or tag is needed. For live changes of the Python code, a downstream project can instead install the source folders editable (`uv pip install --no-deps -e <repo>/src/piper_core -e .../piper_assets ...`): the generated data (URDFs, MJCF, USD) sits git-ignored inside the folders and is refreshed by the same script. `piper_core.__source_commit__` and each wheel's `manifest.json` say which commit a snapshot is.

The contract for a planner: plan inside `limits().planning` (the hard limits less the 0.01 rad guard margin), start and end at `zero_pose()` (joints 2 and 3
are 0.01 rad inside their 0 limit; exact zeros are refused), and hand a `Trajectory` to any backend. A trajectory is executed unchanged or refused whole
before anything moves (`guard:limit`, `guard:margin`, `guard:step`, `guard:velocity`, `start_too_far`, `bad_trajectory`); nothing is clamped or rescaled.
`Trajectory.to_json()` (schema `piper.trajectory/1`) is the hand-off to another Python environment. "Same trajectory" means the same setpoints on every
backend: the real arm's tracking and `speed_percent` mapping are unmeasured, and the MuJoCo actuator model is provisional (README of `piper_mujoco`).
On the ROS stack the same file runs on any backend: `piper execute plan.json --result result.json` (alias `piper-ros-execute`) against a running stack, or `python tools/run_trajectory.py --backend mock|gazebo|mujoco|isaac --trajectory plan.json --result result.json` which starts and stops the stack itself (Isaac through `gpu-run`). The result is `piper.exec_result/1` JSON (`ExecResult.from_json`).
What stays downstream: planner configuration, collision spheres, the scene around the arm, calibration data, experiments.

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

  <!-- the wrist camera: the ZED X Mini on the own bracket (leave these four lines out for the robot only) -->
  <xacro:include filename="$(find piper_description)/urdf/cameras/zedxm.xacro"/>
  <xacro:include filename="$(find piper_description)/urdf/mounts/zedxm_bracket.xacro"/>
  <xacro:include filename="$(find piper_description)/urdf/piper_camera.xacro"/>
  <xacro:piper_camera prefix="piper_" parent="piper_gripper_base" hardware="mock" width="960" height="600" hfov_deg="110.0" update_rate="30"/>
</robot>
```

- Root: the macro attaches `${prefix}base_link` to `parent` with a fixed joint `${prefix}mount_joint`.
- ros2_control: `piper_ros2_control` (`urdf/piper.ros2_control.xacro`) adds the hardware block; `hardware` is `mock | real | gazebo | mujoco | isaac`. For **your own
  simulator**, write your hardware plugin into that block (or your own `<ros2_control>`) with the same joint names and one `position` command interface per joint.
- A complete example, tested: `src/piper_examples` (carrier robot with its own controller manager, prefix `arm_`, mounted at an offset, with the camera). Its launch shows the three steps: expand your own description, run your own controller manager with `piper_control.launch_lib.controllers_file(prefix)`, include `piper_control control.launch.py backend:=external prefix:=arm_ robot_name:=<your robot name> robot_description_file:=<your URDF> camera:=zedxm` (MoveIt needs the URDF of your whole robot, its name in the SRDF, and `camera:=` to know the camera links; `camera:=none` if your robot has none).
- Standalone (the arm at the world origin): `piper_description/urdf/piper.urdf.xacro`, or `ros2 launch piper_description view.launch.py`.
- Another camera or bracket: add `urdf/cameras/<model>.xacro` defining `piper_camera_model(prefix, parent, hardware, width, height, hfov_deg, update_rate, near, far, *origin)` and a mount file defining `piper_camera_mount(prefix, parent)` with the properties `piper_mount_camera_xyz/rpy`, then include them as in the snippet above.
- A new bracket version or a measured camera pose: copy `config/camera_mounts/zedxm_bracket_v3.yaml` to `_v4` (a measured pose goes in the same file, `measured:` filled in), put its mesh in `meshes/` (`tools/build_zedxm_bracket_mesh.py`), and pass `camera_calibration:=<name or path>` (xacro, launch, `export_urdf.py --camera-calibration`). It replaces the default pose on every backend.
- For planners (cuRobo): `ros2 run piper_sim export_urdf.py --gripper-width 0.07 --camera zedxm --prefix "" --out X.urdf` writes a flat URDF with the gripper held at that opening; `piper_core.RealArm` (`start`, `execute(trajectory)`, `stop`) drives the real arm without ROS (tested against a fake SDK only).

## 2. The ROS interface (what `piper_control` + `piper_py` give you)

Names below are for the default prefix; with another prefix, joints and frames follow it. Controllers, actions and topics do not.

| Item | Name | Notes |
|---|---|---|
| Arm joints | `piper_joint1..6` | radians; limits come from the URDF (`piper_core.limits_from_urdf`) |
| Gripper | `piper_gripper` | opening **width in metres**, 0 to 0.07 (the firmware-reported stroke); the fingers (`piper_gripper_joint1/2`) are mimic joints |
| Frames | `world` (caller's), `piper_base_link`, `piper_link1..6`, `piper_tcp_link`, camera: `piper_zed_camera_link`, `piper_zed_camera_center`, `piper_zed_left_camera_frame_optical` | the TCP is the fingertip centre, `piper_gripper_base` + 0.138 m z |
| State | `/joint_states` | position, velocity and effort per joint (the real arm and Isaac fill effort: motor torque estimate, N·m); `Piper.joint_state()`, `joint_velocities()`, `joint_efforts()`, `piper state --full` |
| Trajectories | action `/arm_controller/follow_joint_trajectory` | `joint_trajectory_controller` |
| Streaming targets | topic `/arm_position_controller/commands` | `position_controllers/JointGroupPositionController`; teleop, policies, Servo |
| Gripper | action `/gripper_controller/gripper_cmd` (`ParallelGripperCommand`) | width in metres |
| Planning | MoveIt `move_group`, group `arm` (end effector group `gripper`) | OMPL and Pilz (LIN, PTP, CIRC) |
| Servo | MoveIt Servo `~/delta_twist_cmds`, `~/pose_target_cmds`, `~/delta_joint_cmds` | publishes to the streaming controller |
| Named poses | SRDF group states `zero`, `ready` (and the internal `park`) | `zero`: start and end pose of every session. `park` is a Piper Studio internal used by `stop()` |
| Driver health (real arm) | `/feedback/arm_status`, `/feedback/motor_status`, `/feedback/gripper_status` | `diagnostic_msgs/DiagnosticArray`, 2 Hz, published by `piper_real_driver` in the vendor layout; `Piper.status()`, `Piper.faults()`, `piper status`; `activate()` refuses (`ARM_FAULT`) while one is reported. Simulators publish none. |
| Camera | `/piper_zed/zed_node/rgb/color/rect/image`, `/piper_zed/zed_node/rgb/color/rect/camera_info`, `/piper_zed/zed_node/depth/depth_registered` | the ZED ROS 2 wrapper's names for `camera_name:=<prefix>zed` (`piper_description.camera_topics(prefix)`, `Piper.camera_topics()`, `piper_assets.camera_topics(prefix)`), frame `piper_zed_left_camera_frame_optical` (the left lens): published by Gazebo, MuJoCo and Isaac, and by the real ZED X Mini through the wrapper (`ros2 launch piper_control zed_camera.launch.py`, `docs/real_arm/ZED_BOX.md`; **not tried on the box yet**). Colour is `rgb8` in the simulators and `bgra8` from the ZED (`Piper.image()` returns RGB for both); depth is 32FC1 metres; camera_info is valid for the depth too. Breaking change of 0.9.0: the RealSense-style names `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info` and the frames `camera_link`, `camera_color_optical_frame`, `camera_depth_optical_frame` no longer exist. |

**One command owner:** exactly one of `arm_controller` and `arm_position_controller` is active; switch explicitly (`Piper.use_streaming()` / `use_trajectories()`).
`controllers.yaml`, the MoveIt yaml and the MuJoCo inputs are templates with a `${prefix}` token; fill them with `piper_description.render_config(path, prefix)`.

Python client (`piper_py`): `Piper(prefix="piper_")`, then `joint_state()`, `move_joints()`, `stream_joints()`, `gripper()`, `start()` / `stop()` (session begin and end), `move_named()`, `move_pose()`,
`servo_twist()`, `image()`, `tcp_pose()`. Every call returns a `Result` and **success means executed**, not planned.

## 3. Without ROS: `piper_core`

Plain Python, nothing to install but the package (`pip install src/piper_core`).

```python
from piper_core import GuardConfig, check_command, limits_from_urdf

limits = limits_from_urdf(open("piper.urdf").read())               # from the generated URDF
cfg = GuardConfig(arm_joints=tuple(f"joint{i}" for i in range(1, 7)))
verdict = check_command(names, positions, limits, measured, cfg)   # verdict.ok, verdict.reason, verdict.positions (clamped)
```

Apply the guard to every point of a planner's joint trajectory before it goes to the arm. It is the same function the ROS `command_guard` uses.

### The real arm without ROS: `RealArm` (`pip install piper_core[arm]`, needs `pyAgxArm`)

```python
from piper_core import RealArm
arm = RealArm.connect("can0")            # fails clearly if the CAN bus is down or another program drives the arm
arm.start()                              # the only way in: clears an e-stop, motors on, to zero. Safe to repeat
arm.status()                             # started? enabled? e_stop? faults? feedback age?
arm.execute(trajectory)                  # ExecResult(ok, reason, detail, ...) ; refused whole or executed; nothing outside the bounds is ever sent
arm.set_gripper(0.0, effort=1.0)         # blocks; returns the width; arm.last_gripper.reason: reached | stalled (an object) | no_motion | timeout
arm.stop()                               # the only way out: zero, park, e-stop, clear. From any state. Then start() works at once
```

Everything else follows the same `ArmBackend` contract as the simulators (`piper_core.backend`). Rules: the arm is only commanded inside its bounds and only after `start()`; a refused call sends nothing; a fault, stale feedback,
`abort()` (any thread) or Ctrl-C while running stops and holds the pose; if the process dies the arm keeps holding and the operator stops it (`piper-arm`, key `e`). One program drives the arm at a time (a second `connect` fails with the holder's pid).
Gripper effort defaults to 1.0 N and is capped at 3.0 N (`arm.gripper_max_effort`). Hand-off note with the details and what is verified on the arm: `docs/FINDINGS.md`. Operator tools: `piper-arm` (cockpit), `docs/real_arm/REAL_ARM.md`.

## 4. Real arm rules (do not weaken in a wrapper)

Motors are not enabled at start. The arm speed is set explicitly. Command controllers start inactive and are activated only after fresh, consistent feedback
(`piper activate`). Commands pass the guard. **The ROS path on the real arm is experimental (parked, CLEANUP C14): use `RealArm` (section 3) for the real arm.** Every session starts with `piper start` (`Piper.start()`: clear an e-stop, motors on, one checked move to zero, controllers active) and ends with `piper stop` (zero, park, electronic e-stop, clear). `piper estop` is the emergency stop. The real driver is `piper_real_driver` (our node on `RealArm`); one program drives the arm at a time. Nothing is ever commanded from outside the limits.
Anything that can move the real arm needs the user present and approving.

## 5. Versioning

This file is the contract. Renaming a controller, topic, frame, macro parameter or macro file is a breaking change: record it in `docs/CLEANUP.md` and tell the projects that use the contract. Adding a macro parameter with a default, a camera or a mount is not. The names changed once, on 2026-11-08 (the `piper_` prefix).
