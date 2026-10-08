# Piper Studio

One workspace to run the AgileX **Piper** arm (standard Piper, AgileX gripper) at every control
level, on the real arm and in simulation, built on the official AgileX stack. Designed so other
systems (VLA policies, agents, teleoperation) and other platforms (e.g. a Unitree Go2 carrying the
arm) can use it later.

The legacy implementation is preserved at tag `legacy-final-20261008`.

## Layout

```
external/agx_arm_ros      official AgileX ROS 2 stack (driver, description, MoveIt config, msgs), pinned, unmodified
src/piper_description     our robot model: official URDF + TCP/mount frames + ros2_control backend switch; model audit tool
src/piper_bringup         one launch file for every backend; shared controllers; MoveIt config; command guard (real arm)
src/piper_py              Python API + `piper` CLI: state, joint/trajectory/streaming/gripper/MoveIt motion
env/                      pinned Python environment (uv)
scripts/                  bootstrap.sh (rebuild from clean clone), env.sh (source before anything)
docs/description/         model audit and description decisions
```

Gazebo and MuJoCo run from `piper_bringup`; their models are generated from `piper_description`
(each `ROS_DOMAIN_ID` gets its own `GZ_PARTITION`). Planned: Isaac and `piper_perception` (wrist
camera). See
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
ros2 launch piper_description view.launch.py              # model only, joint sliders
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

### Control levels

| Level | Interface | Use |
|---|---|---|
| State | `/joint_states`, `Piper.joint_state()` | observation |
| Streaming joint targets | `arm_position_controller` via `Piper.use_streaming()` + `stream_joints()` | teleop, policies, servoing |
| Trajectories | `arm_controller` (FollowJointTrajectory) via `Piper.move_joints()` | scripted motion, no planner |
| Planning | MoveIt `move_group` via `Piper.move_pose()/move_named()/move_to_joints()` | collision-aware, Cartesian (Pilz LIN) |
| Gripper | `gripper_controller` (ParallelGripperCommand), width in m | all |
| Vendor-native | `agx_arm_ctrl` topics (`control/move_*`, MIT) or pyAgxArm directly | firmware features, impedance research |

Exactly one controller owns the arm at a time (`arm_controller` or `arm_position_controller`).

### Real arm

```bash
ros2 launch piper_bringup piper.launch.py backend:=real can_port:=can0
```

Safety defaults: motors are not enabled at start, command controllers start inactive, and
`command_guard` drops NaN, out-of-limit and jump commands before they reach the driver. Enable and
activate deliberately, with someone at the arm:

```bash
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 run piper_py piper activate     # refuses unless driver feedback is fresh and consistent
```

## Test

```bash
colcon test --packages-select piper_description piper_py && colcon test-result --verbose
```

## Status

See [VERSIONS.md](VERSIONS.md) for the qualified combination and what has been tested.
