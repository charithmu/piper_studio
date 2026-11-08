# Piper Studio

The AgileX **Piper** arm (standard Piper, AgileX gripper) at every control level, on the real arm and in simulation (mock, Gazebo, MuJoCo, Isaac Sim),
built on the official AgileX stack. It is meant to be used two ways: standalone, to drive the arm in simulation or for real, and as a set of
importable parts for other projects (an ML endpoint with its own simulator, a Go2 carrying the arm, a planner with no ROS at all).

| I want to... | Start here |
|---|---|
| run the arm in a simulator or on the real arm | [Quick start](#quick-start) |
| use the Piper from another repo, with my own simulator or without ROS | [docs/INTEGRATION.md](docs/INTEGRATION.md) |
| understand the design | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| start a new development session | [docs/STATE.md](docs/STATE.md) (orientation, gotchas), then [docs/CLEANUP.md](docs/CLEANUP.md) (to-do and state) |

## Layout

```
src/piper_description   the robot as xacro macros: arm (generated from the official model), tool frame, camera = model + mount, ros2_control; meshes
src/piper_core          rules without ROS: joint limits, command guard, named poses
src/piper_control       controllers (one config for every backend), MoveIt, Servo, command guard node; control.launch.py (mock | real | external)
src/piper_py            Python client and `piper` CLI: state, trajectories, streaming, gripper, MoveIt, Servo, camera
src/piper_examples      example of using the Piper from another project: a carrier robot with its own controller manager (tested)
src/piper_sim           simulators (Gazebo, MuJoCo), scenes, and piper.launch.py: one launch file for every backend (internal to this repo)
isaac/                  Isaac Sim runner and USD conversion (own environment, see isaac/README.md)
tools/                  real_arm/ (direct scripts, no ROS), demo/ (demo page), capture/ (stills and video)
external/agx_arm_ros    official AgileX ROS 2 stack, pinned submodule, unmodified
docs/                   ARCHITECTURE, INTEGRATION, CLEANUP (tracker), VERSIONS, ROADMAP, description/ (decisions), real_arm/, archive/ (historical)
scripts/                bootstrap.sh, session.sh (environment for each terminal), env.sh, isaac.sh, run_demo.sh, can_up.sh
```

`for_reference/` (not in git, about 5 GB) holds cloned vendor and community repositories for comparison; `scripts/reference_repos.sh` lists and restores them ([docs/REFERENCE_REPOS.md](docs/REFERENCE_REPOS.md)).

## Quick start

```bash
git clone --recursive git@github.com:charithmu/piper_studio.git && cd piper_studio
scripts/bootstrap.sh            # submodules, .venv (uv), rosdep check, colcon build
source scripts/session.sh 101   # every terminal: ROS + .venv + overlay + ROS_DOMAIN_ID (+ display)
```

`bootstrap.sh` installs only inside the workspace; if system packages are missing it prints the `rosdep install` command for you to run.

```bash
ros2 launch piper_sim piper.launch.py                                   # mock hardware + MoveIt (headless)
ros2 launch piper_sim piper.launch.py rviz:=true                        # with RViz
ros2 launch piper_sim piper.launch.py backend:=gazebo [gui:=true]       # Gazebo Harmonic
ros2 launch piper_sim piper.launch.py backend:=mujoco [gui:=true]       # MuJoCo (model generated at launch)
ros2 launch piper_sim piper.launch.py backend:=isaac                    # Isaac Sim on the GPU, own process (isaac/README.md)
ros2 launch piper_sim piper.launch.py backend:=gazebo camera:=d435      # simulated wrist D435 on /camera/color/image_raw etc. (also mujoco, isaac)
ros2 launch piper_control control.launch.py                             # the control stack alone: mock, real, or on your own controller manager
ros2 launch piper_description view.launch.py camera:=d435               # the model only, joint sliders; same description options as the launches
```

```bash
ros2 run piper_py piper state
ros2 run piper_py piper named ready
ros2 run piper_py piper pose 0.25 0 0.10                  # tool pointing down, TCP at the fingertips
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

## Names, prefix and the camera

Every link, joint and frame carries a prefix, `piper_` by default (`piper_link1`, `piper_joint1`, `piper_gripper`, `piper_tcp_link`,
`piper_camera_color_optical_frame`); `prefix:=none` gives the official AgileX names (`""` in the Python API and xacro). `Piper(prefix=...)` and `piper --prefix` follow it.
The wrist camera is a model (`urdf/cameras/<name>.xacro`: d435) on a mount (`urdf/mounts/<name>.xacro`: `agilex_mid_stand`, `custom`), chosen with
`camera:=` and `camera_mount:=`. A new camera or a CAD-exported bracket is one more file with the same macro interface (`urdf/piper_camera.xacro`).

`urdf/piper_arm.xacro` and `config/moveit/piper_official.srdf.xacro` are generated from the pinned official files and committed.
`python src/piper_description/tools/piper_arm_macro.py check` validates them (it is part of the tests); `generate` rewrites them after a submodule bump.

## Control levels

| Level | Interface | Use |
|---|---|---|
| State | `/joint_states`, `Piper.joint_state()` | observation |
| Streaming joint targets | `arm_position_controller` via `use_streaming()` + `stream_joints()` | teleop, policies, servoing |
| Trajectories | `arm_controller` (FollowJointTrajectory) via `move_joints()` | scripted motion, no planner |
| Planning | MoveIt `move_group` via `move_pose()`, `move_named()`, `move_to_joints()` | collision-aware, Cartesian (Pilz LIN) |
| Cartesian servoing | MoveIt Servo via `servo_twist()`, `servo_to_pose()` | TCP jogging and tracking (through the streaming controller) |
| Gripper | `gripper_controller` (ParallelGripperCommand), width in m | all |
| Vendor-native | `agx_arm_ctrl` topics or pyAgxArm directly | firmware features (not wrapped by `piper_py`) |

Exactly one controller owns the arm at a time (`arm_controller` or `arm_position_controller`).

## Real arm

```bash
ros2 launch piper_sim piper.launch.py backend:=real canbus:=can0 speed_percent:=20    # CAN up first: scripts/can_up.sh
```

Safety defaults: motors are not enabled at start, the speed is set explicitly (default 20 %), command controllers start inactive, and `command_guard` drops NaN,
jump and out-of-limit commands. Enable and activate deliberately, with someone at the arm:

```bash
ros2 run piper_py piper enable       # motors on
ros2 run piper_py piper activate     # refuses unless driver feedback is fresh and consistent
ros2 run piper_py piper zero         # all joints 0: the start and end pose of every simulation and every real session
ros2 run piper_py piper rest         # zero, then joint 5 lowered 16.8 deg: park pose, disabling the motors here drops the arm only a few mm
ros2 run piper_py piper disable      # controllers off, 5 s countdown to get a hand on the arm, motors off
```

Without ROS, the scripts in `tools/real_arm/` (info, zero, joints, stop) talk to the arm directly. Read [docs/real_arm/REAL_ARM.md](docs/real_arm/REAL_ARM.md) and the per-arm file in
`docs/real_arm/arms/` before touching it.

## Tests

```bash
scripts/test_all.sh                       # offline tests + generated-macro check
scripts/test_all.sh mock gazebo mujoco    # + launch tests per backend, the real-arm plumbing and the carrier example (add isaac: GPU)
python -m pytest src/piper_description src/piper_control src/piper_core src/piper_sim -q       # offline
PIPER_TEST_BACKENDS=mock,gazebo,mujoco python -m pytest src/piper_py/test src/piper_examples -q   # launch tests per backend, and the carrier example (add isaac: GPU, needs scripts/isaac.sh build-usd d435)
python src/piper_description/tools/piper_arm_macro.py check                                     # generated macros still equal the pinned official files
```

## Demo and visual checks

`scripts/run_demo.sh BACKEND OUT/` runs one scripted sequence (planning, straight lines, gripper, direct trajectory, 50 Hz streaming) and records it;
`tools/demo/build_demo_page.py OUT/ SITE/` builds the comparison page. On atlas it is served at `https://atlas.buri-fence.ts.net:8700/piper-studio/demo/` (tailnet only).

Servo step responses (the provisional actuator model): `scripts/run_demo.sh BACKEND OUT/ DOMAIN stepresp`, then `python tools/demo/step_analysis.py OUT/BACKEND.step.json`.
One wrist-camera still on a running backend: `python tools/capture/capture_stills.py out.png`.

RViz, the MuJoCo viewer and the MuJoCo camera need a display with GLX, which a TurboVNC desktop does not have (`scripts/session.sh` defines `gl` for `vglrun -d egl`).
For checks and screenshots use a private virtual display: `Xvfb :96 -screen 0 2560x1440x24 +extension GLX &`, then `DISPLAY=:96 ros2 launch ...` and `DISPLAY=:96 import -window root shot.png`.
The Gazebo GUI also comes up there (its default view is far from the arm); the Gazebo and Isaac cameras need no display.

## Status

[docs/VERSIONS.md](docs/VERSIONS.md) records the qualified software combination and what has been tested; [docs/ROADMAP.md](docs/ROADMAP.md) the milestones.
