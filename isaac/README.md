# Isaac Sim backend

`backend:=isaac` runs the same shared controllers and MoveIt config as every other backend. Isaac Sim itself
is a **separate process in its own environment**; the two sides exchange ROS 2 topics:

```
 ROS side (apt Jazzy, workspace .venv)                    Isaac side (isaac/run_piper.py, Isaac's own env)
 ros2_control_node + JointStateTopicSystem  --/isaac/joint_commands-->  ROS2SubscribeJointState -> articulation drives
 controllers, MoveIt, piper_py              <--/isaac/joint_states----  ROS2PublishJointState
                                            <--/clock-----------------  ROS2PublishClock
```

Nothing Piper-specific is installed into Isaac's environment, and nothing from Isaac into `.venv`. Each run is
its own process (headless by default) (not the shared live session on port 8226), so several agents can use Isaac at once;
the only shared resource is the GPU (about 3 GB for this scene).

## Use

```bash
source scripts/env.sh
scripts/isaac.sh build-usd                         # once, and after any description change
ros2 launch piper_sim piper.launch.py backend:=isaac   # starts ROS side AND the Isaac runner (via gpu-run)
scripts/isaac.sh stop                              # stop this workspace's runner (PID file; never other Isaac processes)
PIPER_TEST_BACKENDS=isaac colcon test --packages-select piper_py   # opt-in test (uses the GPU)
```

Use a unique `ROS_DOMAIN_ID` (the runner inherits it). Standalone runner options: `scripts/isaac.sh run --seconds 30
--video out.mp4 [--gui]`.

For the full Isaac editor (viewport, stage tree, properties and Isaac tools), use a display with NVIDIA graphics support:

```bash
scripts/isaac.sh run --gui                           # ROS runner; also accepts --seconds N
ros2 launch piper_sim piper.launch.py backend:=isaac gui:=true
# With the ROS-free wheels installed into your project environment:
python -m piper_isaac --gui --scene table             # interactive prompt with `sim`
```

The ROS-free Python equivalent is `PiperIsaacSim(scene="table", gui=True)`. It enables recording too. The editor stays responsive between
client calls; rendering idle frames does not advance physics. Use the client to step and control the simulation. `DISPLAY` and `XAUTHORITY`
are forwarded into Isaac's clean environment. Close the interactive session with Ctrl-D to stop its server and release the GPU.
For direct server use, add `--gui` to `isaac/run_server.py` with its required `--usd` and `--socket` arguments (always through `gpu-run`).

## What it does

1. `ros2 run piper_sim export_urdf.py --hardware isaac --physics` writes the one description as a plain
   URDF (absolute mesh paths, widened physics limits, no mimic). Same source as Gazebo and MuJoCo.
2. `convert_urdf.py` turns it into USD with Isaac's URDF importer (`isaacsim.asset.importer.urdf`). Physics is
   a USD variant set; the runner selects `physx`.
3. `run_piper.py` loads the USD, sets the provisional drive gains (same family as MuJoCo's servos), couples the
   fingers to `gripper` (`gripper_joint1 = +0.5 g`, `gripper_joint2 = -0.5 g`, as in the URDF mimic), builds the ROS 2
   OmniGraph (clock, joint state publish/subscribe, articulation controller) and paces frames to real time
   (frame rate auto: 240 Hz without rendering, 90 Hz with a camera or video; physics >= 240 Hz).

Why topics and not `isaacsim.ros2.control`: the ros2_control manager stays in the apt ROS install, identical
to the real-arm path (`JointStateTopicSystem`), so controller behavior is the same on every backend and the
Isaac-hosted controller manager (own ROS ABI, USD-synthesized URDF) is not needed. `isaacsim.ros2.control`
remains an option if in-process control is wanted later.

## ROS-free server (`run_server.py`, the `piper_isaac` package)

For planners that have no ROS (cuRobo, ML endpoints): `isaac/run_server.py` is a second front end on the same Isaac world. `isaac/world.py` owns what
every front end needs (stage and USD, the `physx` variant, drive gains from `servo_gains.py`, gravity compensation, finger coupling, the scene);
`run_server.py` adds a unix socket (JSON lines, one client, under the client's run directory, removed on exit) and no ROS. The client
`piper_isaac.PiperIsaacSim` (pure Python, an `ArmBackend`) starts it through `gpu-run`, applies the `piper_core` rules and resampling, and sends the
setpoints in chunks of 25 control steps; the server holds no rules. Differences from `run_piper.py`, by design:

| | `run_piper.py` (ROS) | `run_server.py` (no ROS) |
|---|---|---|
| Time | paced to wall clock, `/clock` | simulation time only: the world advances only while a command runs; as fast as the GPU allows; physics steps directly through PhysX, independently of rendering |
| Rates | frame rate 240 Hz (90 Hz with a camera), physics >= 240 Hz | control 250 Hz, physics 250 Hz (each PhysX step costs about 2 ms whatever its size, so 500/1000 Hz runs at 0.3x real time and 250/250 at 1.0x; accuracy is the same, 0.021 rad on the parity plan; `ctrl_hz=500, physics_hz=1000` as `PiperSim` is still available; note the guard's 0.15 rad step limit is per control step) |
| Scene objects | static colliders | free bodies, mass 0.05 kg, friction 1.0 (larger of the two bodies wins), like the MuJoCo `table` scene; table top at z = 0, ground plane 0.5 m lower |
| Output | joint states on a topic | measured joints per control step and arm contacts (penetrations of an arm link with the table, an object or another link) |

`world.py` is not yet used by `run_piper.py`: moving the ROS runner onto it is open (CLEANUP G13, phase 2) and needs the Isaac launch test and `scripts/isaac.sh check`.
Recording (`render=True`, or `gui=True`) renders at the requested video fps, with H.264 encoding on a bounded worker queue. Merely enabling
headless recording adds no per-step rendering cost. `python isaac/bench_step.py --seconds 4` benchmarks plain, render-enabled and recording modes
from the workspace environment with the source packages on `PYTHONPATH` (or the wheels installed).

Contacts come from the PhysX contact report; the schema has to be applied before the simulation starts (applying it to a running body invalidates PhysX's
tensor views). Self-collision is off in the USD (`convert_urdf.py`: `allow_self_collision=False`), so self-contacts are not reported, unlike `PiperSim`.
The servo gains are the provisional ones, equal to MuJoCo's: tracking numbers from this server do not describe the real arm.

## Wrist camera and scene objects

`scripts/isaac.sh build-usd` builds the USD with the ZED X Mini on its bracket (the default; `build-usd no_camera` builds the robot-only USD) and writes
`camera_pose.json`: the left (colour) optical frame in `link6`, turned 180 deg about x because USD cameras look along -Z with +Y up, plus the image size,
horizontal field of view, near/far and intrinsics of `piper_description/config/camera_models/zedxm.yaml` (960x600, 110 deg wide lens, 0.1 to 8 m). The runner
creates a USD camera there and publishes it through Isaac's ROS 2 camera graph on the contract's topics (`/camera/color/image_raw`,
`/camera/aligned_depth_to_color/image_raw`, `/camera/color/camera_info`, frame `piper_camera_color_optical_frame`); `camera:=none` runs the robot-only USD.
The static objects of `piper_sim/config/scene_objects.yaml` are passed as JSON (`--scene`) so all
simulators see the same scene. Cost: each rendered view is about 8 ms per frame (real-time factor ~0.6 with the wrist camera at 90 Hz frames).

## Things learned (each cost time)

- The importer output has the physics in a variant set; select `physx` or the stage has no articulation.
- Isaac needs `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `LD_LIBRARY_PATH=$ISAAC_SIM_DIR/exts/isaacsim.ros2.core/jazzy/lib`
  **before** it starts to use its bundled ROS; `scripts/isaac.sh` sets both and starts from a clean environment
  (`env -i`: no system ROS, no `~/.local` NumPy 2 mix-ups).
- Isaac's joint-state publisher always sends efforts, so `piper.ros2_control.xacro` declares an `effort` state
  interface for `hardware:=isaac`. Fingers are not declared in ros2_control (names/positions arrays must match).
- With sim time, ros2_control must not start before `/clock` ticks (Isaac takes ~30 s): the launch gates
  `ros2_control_node` and the spawners on `wait_for_clock.py`.
- `gpu-run` does not forward signals; the runner writes `run.pid` and `scripts/isaac.sh stop` signals it.
- A bare PhysX PD drive sags under gravity (about 9 mm at the TCP, which defeated Servo's 3 mm tolerance); the runner turns gravity off
  for the arm's rigid bodies, as MuJoCo's gravcomp and the real arm's firmware do.
- Hand-authored PhysX mimic joints did not couple the fingers; the runner applies the ratio per frame instead.

## Recreating this on another machine

Requirements: Isaac Sim 6.x as a pip install (Python 3.12) with its `isaacsim.ros2.*` extensions, NVIDIA GPU, a
launcher that activates that environment and defines `ISAAC_SIM_DIR` (here `~/projects/sim/robosim/env.sh`).

```bash
export ISAAC_ENV_SH=/path/to/your/isaac-env.sh      # sources the Isaac Sim 6.x env, sets ISAAC_SIM_DIR
export PIPER_ISAAC_DATA=/path/for/generated/files   # URDF + USD (default ~/data/ml/isaac/piper_studio)
# If `gpu-run` (this machine's GPU scheduler) does not exist, replace it in scripts/isaac.sh `in_isaac()` by `exec "$@"`.
scripts/bootstrap.sh && source scripts/env.sh
scripts/isaac.sh build-usd && ros2 launch piper_sim piper.launch.py backend:=isaac
```

Verified with: Isaac Sim 6.1.0.0 (Kit 110.3), `isaacsim.ros2.core` 1.11.0, `isaacsim.asset.importer.urdf` (urdf-usd-converter 0.3.2),
RTX 4090. Domain/ports: ROS 2 DDS only (no listening sockets of its own).

## Names and the prefix

Every joint and link in the USD carries the description's prefix (default `piper_`: `piper_joint1`, `piper_gripper`, ...), because the USD is
converted from the prefixed URDF. `run_piper.py --prefix` and `check_model.py ... PREFIX` must match the prefix the USD was built with (the launch
passes it; `servo_gains.py` keeps the official names and the runner adds the prefix). After changing the description or the prefix, rebuild:
`scripts/isaac.sh build-usd` and `scripts/isaac.sh build-usd d435`.
