# Piper Studio TODO

This file is intended to be edited directly as the roadmap changes.

## Phase 0: workspace foundation

- [x] Add workspace-level agent guidance
- [x] Add a root README that explains the meta-workspace and common workflows
- [x] Add a unified launcher script with a simple sim-versus-real switch
- [ ] Add a root helper for selective package builds and test shortcuts if the current launcher proves insufficient
- [ ] **Generalize `agx_arm_motion` so it can target both sim and real MoveIt configurations** (gates `agx_arm_manipulation`; see `ORCHESTRATION.md`)

## Phase 1: camera adapter stack

- [x] Create `agx_arm_wristcam`
- [ ] Define the standard wrist camera topic contract
- [ ] Define the standard wrist camera TF contract
- [ ] Add support for RealSense D435
- [ ] Add support for Orbbec Astra U3
- [ ] Add support for OAK-D Max
- [ ] Define mount configuration files and naming conventions
- [ ] Add launch-time camera model and mount switching

## Phase 2: eye-in-hand calibration

- [x] Keep calibration support inside `agx_arm_wristcam` for now
- [ ] Add MoveIt calibration workflow launch files
- [ ] Define where calibration profiles are stored in-version
- [ ] Define profile metadata fields
- [ ] Add a way to select the active calibration profile at launch time
- [ ] Validate the calibration workflow on at least one wrist camera in simulation or replay first

## Phase 3: AprilTag perception

Scope locked to AprilTag-only for the first milestone. Other detection modes
are out of scope until the AprilTag path is end-to-end stable; new modes
attach additively under `/detections/<mode>`.

- [x] Create `agx_arm_detect`
- [x] Pin detection message contract (`apriltag_msgs/AprilTagDetectionArray`, `geometry_msgs/PoseStamped`)
- [ ] Choose the base detector implementation and ROS wrapper strategy
- [ ] Publish detections in a camera-agnostic format
- [ ] Publish TF for detected tags
- [ ] Add workspace configuration for common tag families and sizes
- [ ] Validate wrist-camera tag detection in simulation

## Phase 4: grasp generation

- [x] Create `agx_arm_graspgen`
- [ ] Implement tag-aligned approach and grasp pose generation
- [ ] Add pre-grasp and retreat pose generation
- [ ] Define grasp candidate ranking or selection rules
- [ ] Add known-object grasp templates

## Phase 5: manipulation applications

- [x] Create `agx_arm_manipulation`
- [ ] Expose `move_to_detected_tag_pose`
- [ ] Expose a basic pick sequence
- [ ] Expose a basic place sequence
- [ ] Add a first complete pick-and-place demo in simulation
- [ ] Port the pick-and-place demo to real hardware

## Cross-cutting validation

- [ ] Keep sim and real frame names aligned
- [ ] Keep controller names and planning groups aligned
- [ ] Add focused tests or demo checks for each new package
- [ ] Document every new launch surface in the package README and in the root README when it becomes user-facing