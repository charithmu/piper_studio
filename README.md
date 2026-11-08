# Piper Studio

One workspace to run the AgileX **Piper** arm (standard Piper, AgileX gripper) at every control
level, on the real arm and in simulation, built on the official AgileX stack. Designed so other
systems (VLA policies, agents, teleoperation) and other platforms (e.g. a Unitree Go2 carrying the
arm) can use it later.

The legacy implementation is preserved at tag `legacy-final-20261008`.

## Layout

```
external/agx_arm_ros      official AgileX ROS 2 stack (driver, description, MoveIt config, msgs), pinned, unmodified
src/piper_description     the robot model as xacro macros (`piper_arm` generated from the official URDF, TCP frame, camera = model + mount,
                          ros2_control backend switch), meshes, model audit and macro validation tools
src/piper_bringup         one launch file for every backend; shared controllers; MoveIt config; command guard (real arm)
src/piper_py              Python API + `piper` CLI: state, joint/trajectory/streaming/gripper/MoveIt motion, demo recorder
isaac/                    Isaac Sim runner and USD conversion (own environment; see isaac/README.md)
tools/real_arm            direct real-arm scripts (no ROS): info, zero, joints, stop, enable test
tools/demo                demo page builder, site pages, step analysis
tools/capture             camera stills, video recording, MuJoCo replay renderer
env/                      pinned Python environment (uv)
scripts/                  bootstrap.sh (rebuild from clean clone), env.sh (source before anything), session.sh (env + display + domain per terminal)
docs/ARCHITECTURE.md      layers, interfaces (ROS and non-ROS), mounting and prefix, migration plan
comms/                    machine-wide agent protocol files (status, inbox)
for_reference/            (not in git, ~3 GB) cloned vendor and community repos used for comparison: AgileX agx_arm_sim and piper_isaac_sim, realsense-ros
docs/description/         model audit and description decisions
docs/real_arm/            real-arm knowledge (REAL_ARM.md: vendor docs, firmware gates, modes, safety) and per-arm files (arms/<arm-id>.md)
```

Gazebo and MuJoCo run from `piper_bringup`; their models are generated from `piper_description`
(each `ROS_DOMAIN_ID` gets its own `GZ_PARTITION`; the Gazebo server runs headless with EGL rendering so camera
sensors work without a display). Isaac Sim runs as its own process (see `isaac/README.md`). See
[docs/ROADMAP.md](docs/ROADMAP.md).

## Setup

```bash
git clone --recursive git@github.com:charithmu/piper_studio.git && cd piper_studio
scripts/bootstrap.sh          # submodules, .venv (uv), rosdep check, colcon build
source scripts/env.sh         # ROS Jazzy + .venv + overlay; do this in every shell
```

`bootstrap.sh` installs only inside the workspace. If system packages are missing it prints the
`rosdep install` command for you to run.

## Run

```bash
ros2 launch piper_bringup piper.launch.py                 # mock hardware + MoveIt (headless)
ros2 launch piper_bringup piper.launch.py rviz:=true      # with RViz
ros2 launch piper_bringup piper.launch.py backend:=gazebo [gui:=true]   # Gazebo Harmonic
ros2 launch piper_bringup piper.launch.py backend:=mujoco [gui:=true]   # MuJoCo (model generated at launch)
ros2 launch piper_bringup piper.launch.py backend:=isaac    # Isaac Sim, own process on the GPU; see isaac/README.md
ros2 launch piper_description view.launch.py              # model only, joint sliders; same description options as piper.launch.py
ros2 launch piper_description view.launch.py camera:=d435 camera_mount:=custom camera_xyz:="0.05 0 0.03"   # try a camera pose
ros2 launch piper_bringup piper.launch.py backend:=gazebo camera:=d435   # simulated wrist D435 on /camera/color/image_raw etc. (also mujoco, isaac)
```

```bash
ros2 run piper_py piper state
ros2 run piper_py piper named ready
ros2 run piper_py piper pose 0.25 0 0.10                  # tool pointing down, TCP at fingertips
ros2 run piper_py piper pose 0.25 0 0.06 --linear         # straight-line move
ros2 run piper_py piper joints 0 0.8 -0.8 0 0.5 0         # direct trajectory, no MoveIt
ros2 run piper_py piper gripper 0.05                      # opening width in metres
```

```python
from piper_py import Piper
with Piper() as arm:
    arm.move_named("ready")
    r = arm.move_pose([0.25, 0.0, 0.10], [0, 1, 0, 0])
    print(r.success, r.code)        # success only if execution succeeded
```

### Names, prefix and the camera

Every link, joint and frame carries a prefix, `piper_` by default (`piper_link1`, `piper_joint1`, `piper_gripper`, `piper_tcp_link`,
`piper_camera_color_optical_frame`); `prefix:=""` gives the official AgileX names. `piper_py.Piper(prefix=...)` and `piper --prefix` follow it.
The wrist camera is a model (`urdf/cameras/<name>.xacro`: d435) on a mount (`urdf/mounts/<name>.xacro`: `agilex_mid_stand`, `custom`);
select with `camera:=` and `camera_mount:=`. To add a camera or a mount (a CAD-exported bracket), add one file with the same macro
interface (see `urdf/piper_camera.xacro`).

`urdf/piper_arm.xacro` and `config/moveit/piper_official.srdf.xacro` are generated from the pinned official AgileX files and committed;
`python src/piper_description/tools/piper_arm_macro.py check` validates them (it runs in the tests), `generate` rewrites them after a
submodule bump. See docs/ARCHITECTURE.md.

### Demo

`scripts/run_demo.sh BACKEND OUT/` runs one scripted sequence (planning, straight-line moves, gripper, direct trajectory,
50 Hz streaming) on a backend and records it; `tools/demo/build_demo_page.py OUT/ SITE/` builds the comparison page
(videos, step results, cross-backend agreement, plus status, control-level, architecture and camera pages).
On atlas it is served at `https://atlas.buri-fence.ts.net:8700/piper-studio/demo/` (tailnet only).

### Control levels

| Level | Interface | Use |
|---|---|---|
| State | `/joint_states`, `Piper.joint_state()` | observation |
| Streaming joint targets | `arm_position_controller` via `Piper.use_streaming()` + `stream_joints()` | teleop, policies, servoing |
| Trajectories | `arm_controller` (FollowJointTrajectory) via `Piper.move_joints()` | scripted motion, no planner |
| Planning | MoveIt `move_group` via `Piper.move_pose()/move_named()/move_to_joints()` | collision-aware, Cartesian (Pilz LIN) |
| Cartesian servoing | MoveIt Servo via `Piper.servo_twist()/servo_to_pose()` | TCP jogging and tracking (through the streaming controller) |
| Gripper | `gripper_controller` (ParallelGripperCommand), width in m | all |
| Vendor-native | `agx_arm_ctrl` topics (`control/move_*`, MIT) or pyAgxArm directly | firmware features, impedance research (not wrapped by `piper_py` yet; see the control levels page of the demo site) |

Exactly one controller owns the arm at a time (`arm_controller` or `arm_position_controller`).

### Real arm

```bash
ros2 launch piper_bringup piper.launch.py backend:=real canbus:=can0 speed_percent:=20   # CAN up first: scripts/can_up.sh
```

Safety defaults: motors are not enabled at start, the arm speed is set explicitly (`speed_percent`, default 20 %), command controllers start inactive, and
`command_guard` drops NaN, jump and far-out-of-limit commands and keeps arm joints 0.01 rad inside their limits (a Piper rests slightly past them). Enable and
activate deliberately, with someone at the arm:

```bash
ros2 run piper_py piper enable       # motors on
ros2 run piper_py piper activate     # refuses unless driver feedback is fresh and consistent
ros2 run piper_py piper zero         # all joints 0: the start and end pose of every simulation and of every real session
ros2 run piper_py piper rest         # zero, then joint 5 lowered 16.8 deg: park pose, disabling the motors here drops the arm only a few mm
ros2 run piper_py piper disable      # controllers off, 5 s countdown to get a hand on the arm, motors off
```

Without ROS, `tools/real_arm/arm_info.py` (summary), `arm_zero.py`, `arm_joints.py` (live joints) and `arm_stop.py` talk to the arm directly; see
[docs/real_arm/REAL_ARM.md](docs/real_arm/REAL_ARM.md) and the per-arm file in `docs/real_arm/arms/`.

### Windows on a headless machine

RViz, the MuJoCo viewer and the MuJoCo camera need a display with GLX. A TurboVNC desktop has none (use `vglrun -d egl`, `scripts/session.sh` defines `gl`);
a plain virtual display works for checks and screenshots: `Xvfb :96 -screen 0 2560x1440x24 +extension GLX &`, then `DISPLAY=:96 ros2 launch ...` and
`DISPLAY=:96 import -window root shot.png`. The Gazebo GUI does not come up there yet (docs/CLEANUP.md C3); Gazebo's and Isaac's cameras need no display.

## Test

```bash
python -m pytest src/piper_description src/piper_bringup -q                         # offline: description, generated macros vs official, MuJoCo model, poses
PIPER_TEST_BACKENDS=mock,gazebo,mujoco python -m pytest src/piper_py/test -q        # launch tests per backend (add isaac: GPU, needs scripts/isaac.sh build-usd d435)
python src/piper_description/tools/piper_arm_macro.py check                         # generated macros still equal the pinned official files
```

## Status

See [VERSIONS.md](VERSIONS.md) for the qualified combination and what has been tested.
