# piper_studio

`piper_studio` is a ROS 2 workspace for AgileX Piper-family arms that keeps the upstream hardware and MoveIt stacks in-tree, adds simulation and higher-level motion packages on top, and is intended to grow into a full sim-to-real manipulation workspace.

The 2026-10-08 legacy snapshot and earlier plans are preserved on separate archive/planning branches. The proposed official-stack migration, control hierarchy, optional planning and learning profiles, and validation roadmap are in [docs/overhaul/README.md](docs/overhaul/README.md). The archived workflows below have not been revalidated; see the audit and preservation record before using them.

The current development strategy is:

- keep `pyAgxArm` as the authoritative low-level SDK,
- keep `agx_arm_ros` as the authoritative ROS 2 bridge and base description stack,
- build and validate new features in simulation first,
- then port those features onto the real robot with the same frames, controller names, and package boundaries.

## Workspace Layout

- `src/pyAgxArm`: standalone Python SDK for CAN transport, firmware-aware APIs, and end-effector support; intentionally excluded from `colcon` with `COLCON_IGNORE`
- `src/agx_arm_ros`: upstream-frozen ROS 2 driver, descriptions, messages, and MoveIt config
- `src/agx_arm_gzsim`: workspace-owned Gazebo Harmonic simulation overlay for Piper + gripper (we may extend it as needed)
- `src/agx_arm_motion`: MoveItPy-based motion convenience layer, currently simulation-first
- `src/agx_arm_wristcam`: scaffolded camera adapter and eye-in-hand calibration package
- `src/agx_arm_detect`: scaffolded camera-agnostic detection package, starting with AprilTags
- `src/agx_arm_graspgen`: scaffolded grasp-candidate generation package
- `src/agx_arm_manipulation`: scaffolded task-level manipulation package
- `scripts/piper_studio.sh`: root workflow launcher for common build, visualization, planning, and CAN tasks
- `DESIGN.md`: workspace roadmap and package design
- `TODO.md`: human-editable implementation backlog
- `ORCHESTRATION.md`: multi-agent deployment plan (parallel vs sequential, per-agent briefs)

## Setup

### 1. Activate the workspace venv

```bash
source .venv/bin/activate
```

### 2. Install or refresh the SDK into the venv

```bash
pip install -e src/pyAgxArm
```

### 3. Build the ROS 2 packages you need

```bash
colcon build --packages-select \
  agx_arm_ctrl \
  agx_arm_description \
  agx_arm_moveit \
  agx_arm_msgs \
  agx_arm_gzsim \
  agx_arm_motion \
  agx_arm_wristcam \
  agx_arm_detect \
  agx_arm_graspgen \
  agx_arm_manipulation
```

### 4. Source the workspace overlay

```bash
source install/setup.bash
```

The root launcher script automatically activates `.venv` and sources `install/setup.bash` when those files exist.

## Common Workflows

The simplest way to switch between simulation and hardware is to keep the same command and change `--backend sim` to `--backend real`.

### Plain RViz model visualization

Use this when you want URDF-only visualization without Gazebo or hardware.

```bash
./scripts/piper_studio.sh model-viz --arm-type piper --effector-type agx_gripper
```

### Real arm driver only

Use this when you want the hardware bridge without RViz or MoveIt.

```bash
./scripts/piper_studio.sh driver --can-port can0 --arm-type piper --effector-type agx_gripper
```

### Visualization ladder

Simulation visualization:

```bash
./scripts/piper_studio.sh viz --backend sim
```

Real arm with RViz follow:

```bash
./scripts/piper_studio.sh viz --backend real --can-port can0 --arm-type piper --effector-type agx_gripper
```

### Planning ladder

Simulation with Gazebo + MoveIt + RViz:

```bash
./scripts/piper_studio.sh moveit --backend sim
```

Real arm with driver + MoveIt + RViz:

```bash
./scripts/piper_studio.sh moveit --backend real --can-port can0 --arm-type piper --effector-type agx_gripper
```

MoveIt demo only, without Gazebo or hardware:

```bash
./scripts/piper_studio.sh moveit-demo --arm-type piper --effector-type agx_gripper
```

### Motion convenience layer

`agx_arm_motion` currently loads `agx_arm_gzsim` assets and sets `use_sim_time:=true`, so it should be treated as simulation-first until it is refactored to share both sim and real MoveIt configurations.

One-shot pose goal:

```bash
./scripts/piper_studio.sh motion-once --x 0.30 --y 0.0 --z 0.25 --pitch 1.5708
```

Persistent pose goal server:

```bash
./scripts/piper_studio.sh motion-server
```

### CAN helpers

List detected CAN interfaces and USB bus addresses:

```bash
./scripts/piper_studio.sh scan-can
```

Activate a single CAN module through the upstream helper:

```bash
./scripts/piper_studio.sh setup-can --can-name can0 --bitrate 1000000
```

If you have multiple CAN adapters connected, pass `--usb-address` as well.

## Workflow Matrix

| Scenario | Command | Status | Notes |
|---|---|---|---|
| URDF model in RViz | `./scripts/piper_studio.sh model-viz ...` | Available now | No Gazebo, no hardware |
| Real driver only | `./scripts/piper_studio.sh driver ...` | Available now | Hardware bridge only |
| Sim visualization | `./scripts/piper_studio.sh viz --backend sim` | Available now | Piper + gripper Gazebo overlay |
| Real arm + RViz | `./scripts/piper_studio.sh viz --backend real ...` | Available now | Uses `start_single_agx_arm_rviz.launch.py` |
| Sim + MoveIt | `./scripts/piper_studio.sh moveit --backend sim` | Available now | Gazebo + move_group + RViz |
| Real arm + MoveIt | `./scripts/piper_studio.sh moveit --backend real ...` | Available now | Driver + MoveIt + RViz |
| MoveIt demo only | `./scripts/piper_studio.sh moveit-demo ...` | Available now | No Gazebo, optional real follow |
| One-shot TCP move | `./scripts/piper_studio.sh motion-once ...` | Available now, sim-first | `agx_arm_motion` still tied to sim assets |
| Persistent pose server | `./scripts/piper_studio.sh motion-server` | Available now, sim-first | Same limitation as above |
| `agx_arm_wristcam` | Scaffolded | Interfaces and goals documented | Camera adapter and calibration implementation pending |
| `agx_arm_detect` | Scaffolded | Interfaces and goals documented | AprilTag pipeline implementation pending |
| `agx_arm_graspgen` | Scaffolded | Interfaces and goals documented | Candidate-generation implementation pending |
| `agx_arm_manipulation` | Scaffolded | Interfaces and goals documented | Task-level behaviors implementation pending |

## Recommended Development Flow

1. Start with `./scripts/piper_studio.sh moveit --backend sim` and validate new behavior in simulation.
2. Add or refine the feature in the high-level package without bypassing `pyAgxArm` or `agx_arm_ctrl`.
3. Move the workflow onto hardware with `./scripts/piper_studio.sh moveit --backend real ...`.
4. Keep frames, controller names, planning groups, and topic semantics aligned between both backends.

## References

- `src/agx_arm_gzsim/README.md`
- `src/agx_arm_motion/README.md`
- `src/agx_arm_ros/README.md`
- `src/agx_arm_ros/src/agx_arm_moveit/README.md`
- `src/pyAgxArm/README.md`
- `src/agx_arm_ros/docs/CAN_USER.md`
- `src/agx_arm_ros/docs/tcp_offset/TCP_OFFSET.md`
- `DESIGN.md`
- `TODO.md`
