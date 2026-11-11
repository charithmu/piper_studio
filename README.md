# Piper Studio

The AgileX **Piper** arm (standard Piper, AgileX gripper) at every control level, on the real arm and in simulation (mock, Gazebo, MuJoCo, Isaac Sim),
built on the official AgileX stack. It is meant to be used two ways: standalone, to drive the arm in simulation or for real, and as a set of
importable parts for other projects (an ML endpoint with its own simulator, a Go2 carrying the arm, a planner with no ROS at all).

| I want to... | Start here |
|---|---|
| run the arm in a simulator or on the real arm | [Quick start](#quick-start) |
| use the Piper from another repo, with my own simulator or without ROS | [docs/INTEGRATION.md](docs/INTEGRATION.md) |
| understand the design and who owns what (repo map, package responsibilities, how `external/` and `isaac/` fit) | [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) |
| start a new development session | [docs/STATE.md](docs/STATE.md) (orientation, gotchas), then [docs/CLEANUP.md](docs/CLEANUP.md) (to-do and state) |

## Layout

```
src/piper_description   the robot as xacro macros: arm (generated from the official model), tool frame, camera = model + mount, ros2_control; meshes
src/piper_core          rules without ROS: joint limits, command guard, named poses
src/piper_control       controllers (one config for every backend), MoveIt, Servo, command guard node; control.launch.py (mock | real | external)
src/piper_py            Python client and `piper` CLI: state, trajectories, streaming, gripper, MoveIt, Servo, camera
src/piper_examples      example of using the Piper from another project: a carrier robot with its own controller manager (tested)
src/piper_sim           simulators (Gazebo, MuJoCo, Isaac models), scenes, and piper.launch.py: one launch file for every backend (internal to this repo)
isaac/                  Isaac Sim runner and USD conversion (own environment, see isaac/README.md)
tools/                  real_arm/ (direct scripts, no ROS), demo/ (demo page), capture/ (stills and video)
external/agx_arm_ros    official AgileX ROS 2 stack, pinned submodule, unmodified: source of the geometry (our macros are generated from it) and the real-arm driver
env/                    pinned Python environment (uv)
docs/                   STATE (start here), ARCHITECTURE (repo map, responsibilities), INTEGRATION (contract), CLEANUP (tracker), VERSIONS, ROADMAP, REFERENCE_REPOS,
                        description/ (decisions), real_arm/ (real-arm knowledge), archive/ (historical)
scripts/                bootstrap.sh, session.sh (environment for each terminal), env.sh, test_all.sh, isaac.sh, run_demo.sh, can_up.sh, reference_repos.sh
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
ros2 launch piper_sim piper.launch.py backend:=isaac [gui:=true]        # Isaac Sim on the GPU, own process (isaac/README.md)
ros2 launch piper_sim piper.launch.py backend:=gazebo camera:=none      # the robot only: every launch has the ZED X Mini wrist camera (topics /piper_zed/zed_node/...) unless you say none
ros2 launch piper_control control.launch.py                             # the control stack alone: mock, real, or on your own controller manager
ros2 launch piper_description view.launch.py                              # the model only (with the camera), joint sliders; same description options as the launches
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
`piper_zed_left_camera_frame_optical`); `prefix:=none` gives the official AgileX names (`""` in the Python API and xacro). `Piper(prefix=...)` and `piper --prefix` follow it.
**The wrist camera is on by default:** the Stereolabs ZED X Mini on the own clamp bracket (`urdf/cameras/zedxm.xacro`, `urdf/mounts/zedxm_bracket.xacro`, chained by `urdf/piper_camera.xacro`),
frames as in Stereolabs' `zed-ros2-description`, optical frame `piper_zed_left_camera_frame_optical` (the left lens), topics of the ZED ROS 2 wrapper with `camera_name` `piper_zed` (`piper_description.camera_topics()`, `Piper.camera_topics()`), the same from every simulator and from the real camera (`zed_camera.launch.py`, `docs/real_arm/ZED_BOX.md`). `camera:=none` is the robot only. Lens, image size and field of view:
`config/camera_models/zedxm.yaml`; bracket versions and poses: `config/camera_mounts/zedxm_bracket_v*.yaml` (`camera_calibration:=zedxm_bracket_v2`). Another camera or bracket is one more file with the same macro interface.

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
| Vendor-native | pyAgxArm directly (the vendor `agx_arm_ctrl` node is not used for the real arm) | firmware features (not wrapped by `piper_py`) |

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
ros2 run piper_py piper start        # real arm: motors on, leave the powered-off pose, checked arrival at zero, controllers active
ros2 run piper_py piper stop         # real arm (experimental, parked: use RealArm / piper-arm for the real arm): zero, park, electronic e-stop (damped lowering), clear
ros2 run piper_py piper estop        # real arm: EMERGENCY STOP, immediately
piper-arm                           # real arm cockpit (status + one key per action; e = emergency stop). `source scripts/env.sh` first
ros2 run piper_py piper disable      # controllers off, 5 s countdown to get a hand on the arm, motors off
```

Without ROS, `piper_core.RealArm` and the tools in `tools/real_arm/` talk to the arm directly: the `piper-arm` console, `arm_session.py` (start / end a session), `arm_pose.py`, `arm_jog.py`, `arm_stop.py` (emergency stop), `arm_info.py` / `arm_joints.py` (read only), `arm_stream_test.py`, `gripper_test.py`, `realarm_smoke.py`, `dead_process_drill.py`, and the cuRobo scenario (`curobo_plan_scenario.py`, `run_scenario.py`). Measured results: `docs/real_arm/REPORT.md`. Read [docs/real_arm/REAL_ARM.md](docs/real_arm/REAL_ARM.md) and the per-arm file in
`docs/real_arm/arms/` before touching it.

## Tests

```bash
scripts/test_all.sh                       # offline tests + generated-macro check
scripts/test_all.sh mock gazebo mujoco    # + launch tests per backend, the real-arm plumbing and the carrier example (add isaac: GPU)
python -m pytest src/piper_description src/piper_control src/piper_core src/piper_sim -q       # offline
PIPER_TEST_BACKENDS=mock,gazebo,mujoco python -m pytest src/piper_py/test src/piper_examples -q   # launch tests per backend, and the carrier example (add isaac: GPU, needs scripts/isaac.sh build-usd)
python src/piper_description/tools/piper_arm_macro.py check                                     # generated macros still equal the pinned official files
```

## Demo and visual checks

**Home page of all Studio pages** (running status, how to start each, tools, documents): `https://atlas.buri-fence.ts.net:8700/piper-studio/` (always-on, static; rebuild after registering a page or tool: `python tools/home/build_home.py`).

**cuRobo model** (collision spheres for planners; tuned and regenerated here, optional environment, `docs/CUROBO.md`): `piper_assets.curobo_config(camera)`; viewer `scripts/curobo_viewer.sh start` then <http://localhost:8782/> (tailnet: `https://atlas.buri-fence.ts.net:8700/piper-studio/curobo/`).

**Model viewer** (browser, Three.js; `tools/viewer/`): `scripts/viewer.sh start` then <http://localhost:8781/> (tailnet: `https://atlas.buri-fence.ts.net:8700/piper-studio/viewer/`); `scripts/viewer.sh stop` stops it.
It builds the description live from the xacro and SRDF, so reload the page after a change. It shows visual and collision shapes (mesh and box marked apart), joint sliders with limits, the named poses (and the powered-off `slumped` pose, display only),
gripper width, camera and calibration options, link masses, collision pairs disabled in the SRDF, and the TCP and camera poses. Meshes are read in place from the workspace; the browser loads Three.js from cdn.jsdelivr.net (pinned). Visuals are light copies by default (about 1.3 MB gzip with the camera, grey; made once by the server and cached in `tools/viewer/run/cache`); the "full-detail visuals" box loads the real meshes (about 35 MB raw, 11 MB gzip). Collision shapes load only when switched on. Files are gzip-compressed and cached by ETag.

`scripts/run_demo.sh BACKEND OUT/` runs one scripted sequence (planning, straight lines, gripper, direct trajectory, 50 Hz streaming) and records it;
`tools/demo/build_demo_page.py OUT/ SITE/` builds the comparison page. On atlas it is served at `https://atlas.buri-fence.ts.net:8700/piper-studio/demo/` (tailnet only).

Servo step responses (the provisional actuator model): `scripts/run_demo.sh BACKEND OUT/ DOMAIN stepresp`, then `python tools/demo/step_analysis.py OUT/BACKEND.step.json`.
One wrist-camera still on a running backend: `python tools/capture/capture_stills.py out.png`.

RViz, the MuJoCo viewer and the MuJoCo camera need a display with GLX, which a TurboVNC desktop does not have (`scripts/session.sh` defines `gl` for `vglrun -d egl`).
For checks and screenshots use a private virtual display: `Xvfb :96 -screen 0 2560x1440x24 +extension GLX &`, then `DISPLAY=:96 ros2 launch ...` and `DISPLAY=:96 import -window root shot.png`.
The Gazebo GUI also comes up there (its default view is far from the arm); the Gazebo and Isaac cameras need no display.

## Status

[docs/VERSIONS.md](docs/VERSIONS.md) records the qualified software combination and what has been tested; [docs/ROADMAP.md](docs/ROADMAP.md) the milestones.
