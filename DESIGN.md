# Piper Studio Design

This document is the human-editable design reference for the workspace. It should capture the intended architecture, package boundaries, and rollout sequence for new features. Agents should follow it, but human edits are authoritative.

## Goals

- Build a sim-first workspace for AgileX Piper-family arms that can be ported onto real hardware with minimal divergence.
- Preserve one-to-one correspondence between simulation and hardware for frames, controllers, planning groups, and topic semantics.
- Keep the low-level control stack authoritative in `pyAgxArm` and `agx_arm_ros`, while adding higher-level behavior in supplemental packages.
- Standardize perception so camera selection, calibration, tag detection, and manipulation features can compose cleanly.

## Current Baseline

- `pyAgxArm` the SDK to interface with the hardware, given in-tree for any low-level interaction testing, managed in the workspace venv, and intentionally excluded from `colcon build`.
- `agx_arm_ros` already provides the hardware bridge, URDFs, messages, and MoveIt config for multiple arm and effector variants.
- `agx_arm_gzsim` provides the current Gazebo Harmonic simulation overlay for Piper + gripper. It is workspace-owned (not upstream-frozen like `agx_arm_ros` and `pyAgxArm`) and may be improved when needed to keep sim/real parity or host sim-side launch glue.
- `agx_arm_motion` provides MoveItPy-based motion convenience commands, but its launch files are currently tied to `agx_arm_gzsim` and `use_sim_time:=true`, need to change to support both sim and real, and have not yet been validated on real hardware.
- `agx_arm_eyes`, `agx_arm_detect`, `agx_arm_graspgen`, and `agx_arm_manipulation` are scaffolded with package-local goals and agent guidance, but their runtime implementation is still pending.

## Design Principles

- Fix shared behavior at the lowest sensible layer. If CAN behavior, firmware behavior, or end-effector behavior is wrong, fix `pyAgxArm` or `agx_arm_ctrl` first.
- Keep higher-level packages thin and parameter-driven.
- Prefer standard ROS interfaces over package-private message types whenever a standard type is sufficient.
- Keep simulation first, but do not let simulation invent incompatible frame names or controller semantics.
- Separate perception adapters from perception algorithms from task-level manipulation logic.

## Proposed Package Roadmap

### 1. `agx_arm_eyes`

Purpose:

- Provide a single entry point for supported wrist cameras.
- Load the vendor-specific camera driver and publish a standard camera interface.
- Handle mount selection and calibration profile loading.

Initial supported cameras:

- Intel RealSense D435
- Orbbec Astra U3
- Luxonis OAK-D Max

Expected responsibilities:

- Common launch API: `camera_model`, `mount_name`, `namespace`, `parent_frame`, `calibration_profile`, `align_depth`, `publish_pointcloud`
- Vendor-specific driver wrappers under one package surface
- Standard topic contract such as:
  - `/wrist_camera/color/image_raw`
  - `/wrist_camera/color/camera_info`
  - `/wrist_camera/depth/image_rect_raw`
  - `/wrist_camera/depth/camera_info`
  - `/wrist_camera/points`
- Stable TF chain `gripper_base -> camera_mount_link -> camera_link -> optical frames` (parents off the real URDF link from the standard AgileX gripper xacros, not the MoveIt-synthesized `tcp_link`)
- Mount metadata and static transforms stored in package config files
- Calibration assets stored by arm, camera, mount, and profile

Suggested file layout:

- `launch/bringup.launch.py`
- `launch/realsense.launch.py`
- `launch/orbbec.launch.py`
- `launch/oakd.launch.py`
- `config/mounts/*.yaml`
- `config/calibration/<arm>/<camera>/<mount>/<profile>.yaml`
- `agx_arm_eyes/*.py`

### 2. Eye-in-hand calibration support

This can live inside `agx_arm_eyes`.

Responsibilities:

- Launch MoveIt calibration tools with the selected wrist camera and robot configuration
- Save calibration outputs in a predictable package-owned location
- Support multiple calibration profiles per camera and mount
- Provide a simple way to select the active calibration at launch time

Saved calibration convention:

- Treat raw calibration results as data, not code
- Store them under package config directories, not scattered in home directories
- Track profile metadata including arm type, effector type, camera model, mount name, date, and operator notes

### 3. `agx_arm_detect`

Purpose:

- Consume any camera stream that conforms to the camera adapter contract
- Do various detection tasks such as AprilTag detection, object detection, or pose estimation
- Initially only detect AprilTags and publish detections and TF
- Avoid binding tag detection to a specific camera vendor package

Expected responsibilities:

- Input topics configurable, but defaulting to the standard wrist camera interface
- Publish detections as TF and a ROS topic suitable for downstream task logic
- Package predefined tag families, sizes, and workspace-specific tag layouts
- Support both wrist-mounted cameras and future fixed workspace cameras

### 4. `agx_arm_graspgen`

Purpose:

- Turn perception outputs into grasp candidates
- Start simple with deterministic heuristics before introducing heavier grasp planners

Planned progression:

- Phase 1: tag-aligned approach poses and top-down grasp templates
- Phase 2: object-frame grasp offsets for known objects
- Phase 3: richer grasp candidate generation from geometry or learned models if needed

Outputs:

- Candidate grasp poses
- Pre-grasp and retreat offsets
- Quality ranking or rule-based selection metadata

### 5. `agx_arm_manipulation`

Purpose:

- Package application-level behaviors such as pick-and-place while depending on `agx_arm_motion` for primitives and sequencing.

Expected responsibilities:

- Convenience actions such as move to detected tag pose
- Pre-grasp, grasp, retreat, place, and recovery sequences
- Frame conversion glue between perception outputs and motion goals
- Policy for when to use Cartesian moves versus joint-space planning

## Sim-To-Real Strategy

### Phase A: simulation-first development

- Use Gazebo and synthetic or replayed camera data
- Keep the same frame names intended for hardware
- Validate motion, TF composition, and perception integration before touching hardware

### Phase B: hardware parity

- Swap the backend from sim to real without changing the task-level interfaces
- Load wrist camera drivers and calibration profiles through the adapter package
- Reuse the same tag detection and manipulation APIs from simulation

### Phase C: production hardening

- Add calibration refresh workflows
- Add logging, diagnostics, and health checks
- Add validation scenarios for repeated pick-and-place tasks

## Interfaces To Preserve

- `pyAgxArm` remains the authoritative SDK layer
- `agx_arm_ctrl` remains the authoritative ROS hardware bridge
- `agx_arm_moveit` remains the base MoveIt config package for arm and effector variants
- `agx_arm_motion` should evolve into a backend-agnostic convenience layer rather than staying simulation-bound

## Pinned Cross-Package Contract

These decisions are locked so parallel agent development cannot drift.

- **Perception scope (first milestone): AprilTags only.** `agx_arm_detect` and `agx_arm_graspgen` implement the AprilTag path end-to-end before any other detection mode is added. Other modes attach additively under `/detections/<mode>` rather than reshaping the contract.
- **Detection messages:** `apriltag_msgs/AprilTagDetectionArray` on `/detections/apriltag`, `geometry_msgs/PoseStamped` on `/detections/tag_pose`, plus TF frames under the `tag` prefix.
- **Grasp messages:** `geometry_msgs/PoseArray` on `/grasp/candidates` and `visualization_msgs/MarkerArray` on `/grasp/debug_markers`.
- **Camera mount parent frame:** `gripper_base` (the real URDF link from the standard AgileX gripper xacros). Do not use `tcp_link`; it is only synthesized inside the MoveIt SRDF from `tcp_offset` and is not present in plain URDF or model-viz launches.
- **Sequencing:** `agx_arm_motion` must be generalized to backend-agnostic config (sim and real) before `agx_arm_manipulation` implementation starts. Manipulation depends on motion primitives; duplicating MoveIt glue in manipulation is not allowed.

## Immediate Design Gaps

- `agx_arm_motion` needs to stop hardcoding `agx_arm_gzsim` assets if it is expected to serve both simulation and real hardware.
- A standard camera topic and TF contract does not exist yet at the workspace level.
- Wrist camera mount metadata and calibration storage conventions are not yet defined in code.
- The new perception and manipulation packages are scaffolded, but their runtime implementation is not started yet.

## Acceptance Criteria For The Next Stage

- A single camera adapter package can bring up RealSense D435, Astra U3, and OAK-D Max through one launch surface.
- Calibration profiles are stored in-version and can be selected explicitly.
- AprilTag detection can consume the standard camera interface without camera-specific code changes.
- A high-level package can command `move to detected tag pose` in simulation first, then on hardware.
- A basic pick-and-place demo exists with a clear sim-to-real story.