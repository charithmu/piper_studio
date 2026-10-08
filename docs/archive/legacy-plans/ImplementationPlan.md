# NOTE: THIS IS A BRAINSTORM, NOT A FINAL SPEC. The agent should refine this plan as it works through the implementation.

# Implementation Plan Brainstorm
## Workspace-Level MoveIt Setup for Arm + Gripper + Cameras + Environment

## Goal

Build a maintainable ROS 2 / MoveIt 2 setup where:

```text
vendor arm + vendor gripper
+ virtual TCP
+ optional cameras
+ optional workspace/environment objects
+ improved MoveIt config
+ safe planning façade
```

can be launched consistently for:

```text
simulation
real robot
different camera attachments
future tools/sensors
```

The vendor packages must remain untouched.

---

# 1. Target package architecture

Create these packages:

```text
agx_workspace_description
agx_workspace_moveit_config
agx_workspace_bringup
agx_motion_facade
```

Optional later:

```text
agx_perception_config
agx_calibration_tools
agx_workspace_tests
```

## Package responsibilities

### `agx_workspace_description`

Owns the complete robot/workcell model.

Contains:

```text
arm + gripper + TCP + camera mount + camera collision + optional environment frames
```

This package should compose vendor descriptions using Xacro.

### `agx_workspace_moveit_config`

Owns the MoveIt configuration for the full composed robot.

Contains:

```text
SRDF
joint_limits.yaml
kinematics.yaml
ompl_planning.yaml
moveit_controllers.yaml
planning_scene_monitor.yaml
initial_positions.yaml
```

This should be derived from the vendor MoveIt config, but improved.

### `agx_workspace_bringup`

Owns launch files.

It selects:

```text
real/sim/fake mode
camera model
environment setup
controller config
MoveIt launch
RViz launch
```

### `agx_motion_facade`

Owns the safe interface for other applications.

External apps should talk to this, not directly to raw MoveIt.

---

# 2. Description package implementation

## 2.1 Create top-level Xacro

File:

```text
agx_workspace_description/urdf/piper_workcell.urdf.xacro
```

It should accept launch-time arguments:

```xml
<xacro:arg name="prefix" default=""/>
<xacro:arg name="gripper" default="piper_gripper"/>
<xacro:arg name="camera" default="none"/>
<xacro:arg name="environment" default="none"/>
<xacro:arg name="use_fake_hardware" default="false"/>
```

It should include:

```text
vendor arm xacro
vendor gripper xacro
workspace TCP xacro
optional camera xacro
optional mount xacro
optional static environment xacro
```

## 2.2 Add virtual TCP

Create:

```text
agx_workspace_description/urdf/tools/piper_gripper_tcp.xacro
```

Example structure:

```xml
<link name="${prefix}gripper_tcp"/>

<joint name="${prefix}gripper_tcp_joint" type="fixed">
  <parent link="${prefix}gripper_base_link"/>
  <child link="${prefix}gripper_tcp"/>
  <origin xyz="0 0 0.12" rpy="0 0 0"/>
</joint>
```

The TCP must be attached to a rigid gripper body, not to a moving finger.

Acceptance check:

```bash
ros2 run tf2_ros tf2_echo base_link gripper_tcp
```

must work in both sim and real.

## 2.3 Add camera modules

Create one Xacro per camera/mount setup:

```text
urdf/sensors/astra_s_u3_mount.xacro
urdf/sensors/realsense_d435_mount.xacro
urdf/sensors/no_camera.xacro
```

Each camera module should define:

```text
camera_link
camera_optical_frame
mount_link
fixed joint from gripper/wrist to camera mount
simple collision geometry
visual geometry if available
```

Do not use complex meshes for collision. Use boxes/cylinders.

## 2.4 Camera selection pattern

In top-level Xacro:

```xml
<xacro:if value="${camera == 'astra_s_u3'}">
  <xacro:include filename="$(find agx_workspace_description)/urdf/sensors/astra_s_u3_mount.xacro"/>
  <xacro:astra_s_u3_mount prefix="${prefix}" parent="${prefix}gripper_base_link"/>
</xacro:if>

<xacro:if value="${camera == 'realsense_d435'}">
  <xacro:include filename="$(find agx_workspace_description)/urdf/sensors/realsense_d435_mount.xacro"/>
  <xacro:realsense_d435_mount prefix="${prefix}" parent="${prefix}gripper_base_link"/>
</xacro:if>
```

Launch usage:

```bash
ros2 launch agx_workspace_bringup moveit.launch.py camera:=astra_s_u3
```

---

# 3. MoveIt config implementation

## 3.1 Start from vendor MoveIt config

Copy or generate your own config:

```text
agx_workspace_moveit_config
```

Do not modify vendor config directly.

Use vendor config as reference for:

```text
planning groups
controllers
kinematics plugin
joint limits
default OMPL settings
```

## 3.2 Update SRDF

The main planning group should represent only the arm joints:

```text
arm:
  joint1
  joint2
  joint3
  joint4
  joint5
  joint6
```

The end-effector should reference the gripper/TCP correctly.

Preferred planning TCP:

```text
gripper_tcp
```

Check with code:

```cpp
move_group.getEndEffectorLink()
```

Expected:

```text
gripper_tcp
```

## 3.3 Add safe named states

Do not use joint values exactly on hard limits.

Bad:

```yaml
joint2: 0.0
```

Good:

```yaml
joint2: 0.01
```

Create named states:

```text
ready
home_safe
inspect
park
calibration
```

Example:

```yaml
ready:
  joint1: 0.0
  joint2: 0.05
  joint3: 0.05
  joint4: 0.0
  joint5: 0.05
  joint6: 0.0
```

## 3.4 Fix joint limits

Ensure consistency across:

```text
URDF
joint_limits.yaml
ros2_control
simulator
real driver
MoveIt
```

The agent must search for conflicting limits:

```bash
grep -R "joint2" src/ -n
grep -R "lower=\"0\"" src/ -n
grep -R "min_position: 0" src/ -n
```

If a joint is wrongly limited to `[0, pi]`, correct it to the actual manufacturer limit.

## 3.5 Planning scene configuration

Use the MoveIt planning scene for:

```text
table
wall
fixture
safety zone
forbidden zone
workpiece
```

Do not put all environment objects permanently into robot URDF unless they are physically attached to the robot.

Create:

```text
agx_workspace_bringup/config/scenes/desk_setup.yaml
agx_workspace_bringup/config/scenes/empty.yaml
agx_workspace_bringup/config/scenes/calibration_setup.yaml
```

---

# 4. Bringup package implementation

## 4.1 Main launch arguments

File:

```text
agx_workspace_bringup/launch/moveit.launch.py
```

Arguments:

```text
mode:=fake | sim | real
camera:=none | astra_s_u3 | realsense_d435
environment:=none | desk | calibration
rviz:=true | false
use_sim_time:=true | false
```

Example:

```bash
ros2 launch agx_workspace_bringup moveit.launch.py \
  mode:=sim \
  camera:=astra_s_u3 \
  environment:=desk \
  rviz:=true
```

Real robot:

```bash
ros2 launch agx_workspace_bringup moveit.launch.py \
  mode:=real \
  camera:=astra_s_u3 \
  environment:=desk \
  rviz:=true
```

## 4.2 Use `MoveItConfigsBuilder`

The launch file should construct MoveIt config dynamically using Xacro mappings.

Concept:

```python
moveit_config = (
    MoveItConfigsBuilder("piper", package_name="agx_workspace_moveit_config")
    .robot_description(
        file_path="config/piper_workcell.urdf.xacro",
        mappings={
            "camera": camera,
            "environment": environment,
            "mode": mode,
        },
    )
    .robot_description_semantic(file_path="config/piper.srdf")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .planning_pipelines(pipelines=["ompl"])
    .to_moveit_configs()
)
```

## 4.3 Launch validation

The launch should print:

```text
selected mode
selected camera
selected environment
planning frame
end-effector link
controller mode
```

This helps debugging.

---

# 5. Motion façade implementation

Create a ROS 2 node:

```text
agx_motion_facade
```

Purpose:

```text
External apps → agx_motion_facade → MoveIt → controllers
```

External apps should not directly call MoveIt.

## 5.1 Façade services/actions

Implement these APIs:

```text
/get_robot_state
/validate_joint_target
/validate_pose_target
/plan_to_joint_target
/plan_to_pose_target
/execute_trajectory
/plan_and_execute
/stop_motion
/add_collision_object
/remove_collision_object
/attach_object
/detach_object
```

## 5.2 Validation before planning

Every request must pass:

```text
joint names valid
target frame known
target transform available
numbers finite
state timestamp fresh
start state inside bounds
start state not colliding
goal state inside bounds
IK solvable if Cartesian target
goal state not colliding
planner available
controller available
robot not faulted
```

## 5.3 Separate validate, plan, execute

Do not collapse everything into direct movement.

Correct lifecycle:

```text
validate target
plan trajectory
validate trajectory
execute trajectory
monitor execution
report result
```

## 5.4 Handle small numerical bound errors

For tiny errors like:

```text
-1e-14
```

the façade may sanitize the state.

Rule:

```text
if violation < 1e-6 rad:
    clamp to limit
else:
    reject as real bounds violation
```

Never silently correct large violations.

## 5.5 Execution monitoring

During execution, monitor:

```text
joint state freshness
actual vs expected position error
controller status
emergency stop/fault flag
trajectory timeout
collision scene updates
```

If unsafe:

```text
stop trajectory
report error
do not retry automatically
```

---

# 6. Reachability and workspace tools

Implement a utility node:

```text
agx_motion_facade/reachability_sampler
```

It should sample:

```text
x, y, z
orientation set
IK result
collision result
planning result
```

Output:

```text
RViz MarkerArray
CSV/JSON reachability map
```

Color convention:

```text
green = IK + planning valid
yellow = IK valid but planning failed
red = IK failed
gray = collision
```

This helps visualize usable workspace.

---

# 7. Calibration and camera management

## 7.1 Static nominal camera transform

Each camera mount Xacro defines nominal transform:

```text
gripper_base_link → camera_link
```

## 7.2 Calibrated transform

Later calibration should produce a refined transform.

Store calibration here:

```text
agx_workspace_description/config/calibration/astra_s_u3_extrinsics.yaml
```

or:

```text
agx_perception_config/config/extrinsics/astra_s_u3.yaml
```

Format:

```yaml
parent_frame: gripper_base_link
child_frame: camera_link
xyz: [0.03, 0.00, 0.08]
rpy: [0.0, 0.2, 0.0]
source: moveit_camera_calibration
date: 2026-05-15
```

## 7.3 Launch-time calibrated transform

The Xacro or launch file should allow:

```bash
camera_extrinsics:=calibrated
```

or:

```bash
camera_extrinsics_file:=...
```

But keep one canonical path for stable production use.

---

# 8. Testing plan

## 8.1 Description tests

Check generated URDF:

```bash
ros2 run xacro xacro \
  src/agx_workspace_description/urdf/piper_workcell.urdf.xacro \
  camera:=astra_s_u3 > /tmp/piper_workcell.urdf
```

Then:

```bash
check_urdf /tmp/piper_workcell.urdf
```

Check TF:

```bash
ros2 run tf2_tools view_frames
```

Required frames:

```text
base_link
gripper_tcp
camera_link
camera_optical_frame
```

## 8.2 MoveIt tests

Test:

```text
current state valid
ready pose valid
TCP pose available
planning to named state works
planning to simple pose works
collision object blocks motion
collision object removal restores motion
```

## 8.3 Sim/real parity tests

For both sim and real:

```bash
ros2 run tf2_ros tf2_echo base_link gripper_tcp
ros2 topic echo /joint_states --once
ros2 service call /check_state_validity ...
```

Expected:

```text
same frame names
same TCP frame
same planning group
same MoveIt config
same façade API
different controller backend only
```

---

# 9. Agent work strategy

The coding agent should work in phases, not randomly edit files.

## Phase 1: Inspect

Agent should inspect:

```text
existing packages
vendor URDF
vendor SRDF
joint limits
controller configs
launch files
current TF tree
current /joint_states
MoveIt planning group names
end-effector link
```

Deliverable:

```text
short report of current structure and detected problems
```

## Phase 2: Create workspace description

Agent creates:

```text
agx_workspace_description
top-level Xacro
TCP link
camera module structure
```

Acceptance:

```text
URDF builds
TF tree contains gripper_tcp
RViz RobotModel displays robot
```

## Phase 3: Create improved MoveIt config

Agent creates:

```text
agx_workspace_moveit_config
```

Acceptance:

```text
MoveIt launches
planning group exists
EEF link is gripper_tcp
ready named state is valid
no start-state bounds errors
```

## Phase 4: Create bringup launch

Agent creates:

```text
agx_workspace_bringup
```

Acceptance:

```bash
ros2 launch agx_workspace_bringup moveit.launch.py camera:=none
ros2 launch agx_workspace_bringup moveit.launch.py camera:=astra_s_u3
```

Both must launch.

## Phase 5: Create motion façade

Agent creates:

```text
agx_motion_facade
```

Acceptance:

```text
get state works
validate target works
plan target works
execute trajectory is separated
invalid target is rejected with clear reason
```

## Phase 6: Add reachability tools

Agent adds:

```text
reachability sampler
RViz MarkerArray output
CSV/JSON export
```

Acceptance:

```text
reachable workspace visible in RViz
sampled points classify correctly
```

---

# 10. Rules for “think and improve”

Give the agent these operating rules.

## Before editing

The agent must answer internally:

```text
What package owns this responsibility?
Is this robot-attached or workspace-attached?
Is this URDF, SRDF, launch, controller, or planning-scene concern?
Will this affect real robot and sim consistently?
Can this break vendor packages?
```

## After each change

The agent must run the smallest relevant validation:

```text
Xacro change → generate URDF and check frames
MoveIt config change → launch MoveIt and plan to ready
Controller change → check controller list
Façade change → run service call test
```

## Improvement loop

For each failed test:

```text
observe error
identify layer
fix only that layer
rerun test
document reason
```

Do not shotgun-edit multiple layers at once.

## Layer diagnosis rule

Use this mapping:

```text
TF missing → URDF/Xacro or robot_state_publisher
EEF wrong → SRDF or MoveGroup config
bounds error → joint limits or initial state
collision false positive → collision geometry or ACM
controller failure → ros2_control or controller manager
planning failure → IK, limits, collision, planner config, or target
execution failure → controller/action interface
```

---

# 11. Important design decisions

## Do not regenerate MoveIt config for every camera

Use one parameterized config unless the camera changes:

```text
planning groups
joint limits
kinematics
controllers
end-effector semantics
```

Different camera model alone should be a Xacro/launch argument.

## Keep vendor packages clean

Never edit:

```text
vendor_arm_description
vendor_moveit_config
```

unless absolutely necessary.

Override or compose them from your workspace packages.

## Use URDF for attached hardware

Put these in URDF/Xacro:

```text
gripper
TCP
camera
mount
tool
attached sensor
```

## Use planning scene for environment

Put these in planning scene:

```text
table
wall
fixture
box
workpiece
forbidden zone
```

## Use façade for external applications

External applications should not know MoveIt internals.

They should call:

```text
validate
plan
execute
stop
get_state
```

---

# 12. Final desired command examples

No camera:

```bash
ros2 launch agx_workspace_bringup moveit.launch.py \
  mode:=sim \
  camera:=none \
  environment:=none
```

Astra camera:

```bash
ros2 launch agx_workspace_bringup moveit.launch.py \
  mode:=sim \
  camera:=astra_s_u3 \
  environment:=desk
```

Real robot:

```bash
ros2 launch agx_workspace_bringup moveit.launch.py \
  mode:=real \
  camera:=astra_s_u3 \
  environment:=desk
```

Validate target:

```bash
ros2 service call /agx_motion_facade/validate_pose_target ...
```

Plan only:

```bash
ros2 action send_goal /agx_motion_facade/plan_to_pose ...
```

Execute approved trajectory:

```bash
ros2 action send_goal /agx_motion_facade/execute_trajectory ...
```

---

# Summary

The correct production-grade setup is:

```text
vendor packages untouched
workspace-level composed URDF
parameterized camera/tool selection
custom MoveIt config for full workcell
planning scene for environment
safe façade above MoveIt
validate-plan-execute separation
continuous tests after each change
```

This gives you a setup that can evolve from:

```text
arm + gripper
```

to:

```text
arm + gripper + TCP + camera + calibrated extrinsics + workspace constraints + application API
```

without rebuilding the whole system every time you change a camera or add a fixture.
