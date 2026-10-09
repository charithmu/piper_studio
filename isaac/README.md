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
its own headless process (not the shared live session on port 8226), so several agents can use Isaac at once;
the only shared resource is the GPU (about 3 GB for this scene).

## Use

```bash
source scripts/env.sh
scripts/isaac.sh build-usd                         # once, and after any description change
ros2 launch piper_bringup piper.launch.py backend:=isaac   # starts ROS side AND the Isaac runner (via gpu-run)
scripts/isaac.sh stop                              # stop this workspace's runner (PID file; never other Isaac processes)
PIPER_TEST_BACKENDS=isaac colcon test --packages-select piper_py   # opt-in test (uses the GPU)
```

Use a unique `ROS_DOMAIN_ID` (the runner inherits it). Standalone runner options: `scripts/isaac.sh run --seconds 30
--video out.mp4 [--gui]`.

## What it does

1. `ros2 run piper_description export_urdf.py --hardware isaac --physics` writes the one description as a plain
   URDF (absolute mesh paths, widened physics limits, no mimic). Same source as Gazebo and MuJoCo.
2. `convert_urdf.py` turns it into USD with Isaac's URDF importer (`isaacsim.asset.importer.urdf`). Physics is
   a USD variant set; the runner selects `physx`.
3. `run_piper.py` loads the USD, sets the provisional drive gains (same family as MuJoCo's servos), couples the
   fingers to `gripper` (`gripper_joint1 = +0.5 g`, `gripper_joint2 = -0.5 g`, as in the URDF mimic), builds the ROS 2
   OmniGraph (clock, joint state publish/subscribe, articulation controller) and paces frames to real time
   (60 Hz frames, 240 Hz physics).

Why topics and not `isaacsim.ros2.control`: the ros2_control manager stays in the apt ROS install, identical
to the real-arm path (`JointStateTopicSystem`), so controller behavior is the same on every backend and the
Isaac-hosted controller manager (own ROS ABI, USD-synthesized URDF) is not needed. `isaacsim.ros2.control`
remains an option if in-process control is wanted later.

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
- Hand-authored PhysX mimic joints did not couple the fingers; the runner applies the ratio per frame instead.

## Recreating this on another machine

Requirements: Isaac Sim 6.x as a pip install (Python 3.12) with its `isaacsim.ros2.*` extensions, NVIDIA GPU, a
launcher that activates that environment and defines `ISAAC_SIM_DIR` (here `~/projects/sim/robosim/env.sh`).

```bash
export ISAAC_ENV_SH=/path/to/your/isaac-env.sh      # sources the Isaac Sim 6.x env, sets ISAAC_SIM_DIR
export PIPER_ISAAC_DATA=/path/for/generated/files   # URDF + USD (default ~/data/ml/isaac/piper_studio)
# If `gpu-run` (this machine's GPU scheduler) does not exist, replace it in scripts/isaac.sh `in_isaac()` by `exec "$@"`.
scripts/bootstrap.sh && source scripts/env.sh
scripts/isaac.sh build-usd && ros2 launch piper_bringup piper.launch.py backend:=isaac
```

Verified with: Isaac Sim 6.1.0.0 (Kit 110.3), `isaacsim.ros2.core` 1.11.0, `isaacsim.asset.importer.urdf` (urdf-usd-converter 0.3.2),
RTX 4090. Domain/ports: ROS 2 DDS only (no listening sockets of its own).
