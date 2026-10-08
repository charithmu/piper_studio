# Piper Studio — Implementation Plan

This document is the authoritative actionable plan for the next build-out
phase. It supersedes the brainstorm in `ImplementationPlan.md`.

Each chunk ends with build verification, smoke tests, acceptance criteria,
and the git steps needed before moving on.

---

## Architecture decisions locked in this plan

| Decision | Choice | Rationale |
|---|---|---|
| Camera package name | `agx_arm_wristcam` (renamed from `agx_arm_eyes`) | Naming clarity |
| Workspace MoveIt config | Inside `agx_arm_workspace` as `agx_arm_workspace_moveit` inner package | Keeps workspace self-contained |
| `agx_arm_workspace` layout | Multi-package container (`agx_arm_workspace_description`, `agx_arm_workspace_moveit`, `agx_arm_workspace_bringup`) | Separates concerns cleanly |
| MoveIt config generation | Human runs MoveIt Setup Assistant when URDF is ready | MoveIt SA GUI is the right tool |
| Scene objects | Planning-scene-only yamls, not baked into URDF | Simpler, extend later |
| Adding new cameras | Developer writes 4 files; not a single-line switch | Explicit wiring enforces interface contract |

---

## Submodule map

| Package dir | Type | Remote |
|---|---|---|
| `src/agx_arm_eyes` | git submodule | github.com/charithmu/agx_arm_eyes |
| `src/agx_arm_motion` | git submodule | github.com/charithmu/agx_arm_motion |
| `src/agx_arm_ros` | git submodule (frozen) | github.com/charithmu/agx_arm_ros |
| `src/agx_arm_gzsim` | git submodule | github.com/charithmu/agx_arm_gzsim |
| `src/agx_arm_workspace` | regular dir in superrepo | (part of piper_studio) |
| `src/agx_arm_mjsim` | standalone git repo in src/ | (not registered as submodule) |

**Submodule commit discipline:** commit inside the submodule first, then update
the pointer in the superrepo. Never edit `agx_arm_ros` or `pyAgxArm`.

---

## Chunk dependency order

```
Chunk 1 (rename eyes→wristcam)
    └─▶ Chunk 2 (wristcam xacro + config schemas)
            ├─▶ Chunk 3 (restructure agx_arm_workspace into 3 packages)
            │       └─▶ Chunk 4 (wristcam driver bringup)
            │               └─▶ Chunk 5 (calibration tooling)
            └─▶ Chunk 6 (workspace_bringup unified launcher)
                    └─▶ Chunk 7a (prep for MoveIt SA)
                            └─▶ [human: 7b run MoveIt Setup Assistant]
                                    └─▶ Chunk 7c (retarget agx_arm_motion)
                                            └─▶ Chunk 8 (docs tidy)
```

---

## Chunk 1 — Rename `agx_arm_eyes` → `agx_arm_wristcam`

**Goal:** pure rename, no behavior change. Make the package name match project
intent before adding any code.

### Current broken state to be aware of

`agx_arm_workspace/launch/view_workspace.launch.py` and `real_workspace.launch.py`
already import `agx_arm_eyes.config_io` — a module that does not exist yet.
These launches are currently broken. Chunk 2 will create `config_io.py` as part
of the wristcam implementation.

### Developer steps

#### 1.1 Inside the `src/agx_arm_eyes` submodule

Files to rename / update:

| Action | Path |
|---|---|
| Rename dir | `agx_arm_eyes/` → `agx_arm_wristcam/` |
| Update | `package.xml` — `<name>agx_arm_wristcam</name>` |
| Update | `setup.py` — `package_name = "agx_arm_wristcam"` + all `glob()` paths |
| Update | `setup.cfg` — `[ros2run]` script prefix |
| Rename | `resource/agx_arm_eyes` → `resource/agx_arm_wristcam` |
| Update | `agx_arm_wristcam/__init__.py` — module docstring |
| Update | `agx_arm_wristcam/contracts.py` — module docstring only (constants unchanged) |
| Update | `AGENTS.md` if it references the old package name |
| Update | `README.md` — package name references |

Commit message: `refactor: rename package agx_arm_eyes → agx_arm_wristcam`

#### 1.2 In the superrepo (`piper_studio`)

Update `.gitmodules`:
```
[submodule "src/agx_arm_wristcam"]
    path = src/agx_arm_wristcam
    url = git@github.com:charithmu/agx_arm_wristcam.git
```

Move the submodule directory pointer:
```bash
git submodule deinit src/agx_arm_eyes
git rm src/agx_arm_eyes
# add the renamed submodule back once the GitHub repo is renamed (human step 1.3)
git submodule add git@github.com:charithmu/agx_arm_wristcam.git src/agx_arm_wristcam
```

Update `src/agx_arm_workspace/urdf/piper_workspace.urdf.xacro`:
- Change `$(find agx_arm_eyes)` → `$(find agx_arm_wristcam)` (two occurrences)

Update `src/agx_arm_workspace/package.xml`:
- Change `<exec_depend>agx_arm_eyes</exec_depend>` → `<exec_depend>agx_arm_wristcam</exec_depend>`

Update `src/agx_arm_workspace/launch/*.launch.py`:
- Change all `agx_arm_eyes` imports and `get_package_share_directory("agx_arm_eyes")` →
  `agx_arm_wristcam`

Superrepo commit: `refactor: update submodule pointer and references to agx_arm_wristcam`

### Human steps

1. **On GitHub:** rename repo `charithmu/agx_arm_eyes` → `charithmu/agx_arm_wristcam`.
   (Settings → General → Repository name)
2. After rename, push the submodule's `main` branch:
   ```bash
   cd src/agx_arm_wristcam
   git push origin main
   ```
3. Push superrepo changes:
   ```bash
   cd /path/to/piper_studio
   git push origin main
   ```

### Acceptance

```bash
# From workspace root with sourced env:
ros2 pkg prefix agx_arm_wristcam   # must resolve
python3 -c "import agx_arm_wristcam; print('ok')"
```

---

## Chunk 2 — `agx_arm_wristcam`: camera xacro contract + RealSense URDF

**Goal:** make `piper_workspace.urdf.xacro` actually build; define the camera
extension interface that future cameras follow.

### Files to create inside `src/agx_arm_wristcam`

#### Config schemas

**`config/realsense_d435.yaml`** — camera descriptor:
```yaml
model: realsense_d435
mount_xacro: realsense_d435.xacro   # file in urdf/models/
optical_frames:
  - realsense_d435_color_optical_frame
  - realsense_d435_depth_optical_frame
default_namespace: wrist_camera
```

**`config/extrinsics/realsense_d435__factory.yaml`** — nominal factory transform:
```yaml
# Nominal transform from gripper_base to camera_mount_link.
# Replace with calibrated values from calibration workflow (Chunk 5).
parent_frame: gripper_base
child_frame: camera_mount_link
xyz: [0.0, 0.0, 0.05]
rpy: [0.0, 0.0, 0.0]
source: factory
date: 2026-05-16
notes: placeholder nominal — replace after eye-in-hand calibration
```

Schema contract (all extrinsic files must have these keys):
- `parent_frame`, `child_frame`, `xyz` (list[3]), `rpy` (list[3]), `source`, `date`, `notes`

#### URDF / Xacro files

**`urdf/wrist_camera.xacro`** — top-level dispatcher macro:
```
Macro: agx_wrist_camera(parent_link, camera_config_file, extrinsic_file)
  - Reads model name from camera_config_file via xacro property
  - Dispatches to the appropriate model xacro via xacro:if chain
  - Accepts: realsense_d435 | astra_u3 | oakd_max | none
```

**`urdf/models/realsense_d435.xacro`** — concrete model:
```
Links:
  camera_mount_link       (box 80×30×20 mm collision + visual)
  realsense_d435_link     (box 90×25×25 mm, visual mesh if available)
  realsense_d435_color_optical_frame  (pure rotation, no mass)
  realsense_d435_depth_optical_frame  (pure rotation, no mass)

Joints:
  camera_mount_joint      fixed,  parent_link → camera_mount_link
                          xyz/rpy from extrinsic_file argument
  realsense_d435_joint    fixed,  camera_mount_link → realsense_d435_link
                          nominal offset [0, 0, 0.02]
  color_optical_joint     fixed,  standard RealSense optical convention
  depth_optical_joint     fixed,  standard RealSense optical convention
```

**`urdf/models/_template.xacro`** — documented template for adding new cameras.
A developer adding Astra U3 copies this, fills in the macro name, link names,
dimensions, and optical frame offsets.

**`urdf/models/no_camera.xacro`** — empty model for `camera:=none`:
```xml
<!-- intentionally empty — no links or joints added -->
```

#### Python module

**`agx_arm_wristcam/config_io.py`** — runtime config resolver:
```python
def resolve_camera_setup(camera_model, calibration_name, namespace) -> dict:
    """
    Resolves camera_config_file and extrinsic_file absolute paths for
    use in launch files. Fails loudly if files are missing.
    Returns: {"camera_model", "calibration_name", "namespace",
              "camera_config_file", "extrinsic_file"}
    """
```
This is the module already imported by the existing `agx_arm_workspace` launch files.
Creating it here unblocks `view_workspace.launch.py` and `real_workspace.launch.py`.

#### `setup.py` additions

Add glob entries for:
- `config/*.yaml`
- `config/extrinsics/*.yaml`
- `urdf/*.xacro`
- `urdf/models/*.xacro`

### Commit

Inside `src/agx_arm_wristcam`:
`feat(wristcam): add camera xacro contract, realsense_d435 URDF, and config_io`

### Human steps

None. All developer work.

### Build & smoke test

```bash
# From piper_studio root with sourced env:
colcon build --packages-select agx_arm_wristcam agx_arm_workspace
source install/setup.bash

# Expand workspace URDF — must succeed with no xacro errors:
ros2 run xacro xacro \
  src/agx_arm_workspace/urdf/piper_workspace.urdf.xacro \
  > /tmp/piper_workspace.urdf

# Validate URDF structure:
check_urdf /tmp/piper_workspace.urdf

# Check TF tree contains required frames:
ros2 launch agx_arm_workspace view_workspace.launch.py &
sleep 5
ros2 run tf2_tools view_frames   # output in /tmp/frames.pdf
# Required: base_link, link6, gripper_base, camera_mount_link,
#           realsense_d435_link, realsense_d435_color_optical_frame
kill %1
```

### Acceptance

- `check_urdf` exits 0 with no errors
- TF tree contains all required frames listed above
- `view_workspace.launch.py` opens RViz showing arm + gripper + camera box

---

## Chunk 3 — Restructure `agx_arm_workspace` into 3 inner packages

**Goal:** split the single flat `agx_arm_workspace` into
`agx_arm_workspace_description`, `agx_arm_workspace_moveit` (skeleton only),
`agx_arm_workspace_bringup`. No logic changes in this chunk — pure file moves.

### New layout

```
src/agx_arm_workspace/
├── README.md
└── src/
    ├── agx_arm_workspace_description/
    │   ├── package.xml          (ament_cmake, depends on agx_arm_description + agx_arm_wristcam)
    │   ├── CMakeLists.txt
    │   ├── urdf/
    │   │   └── piper_workspace.urdf.xacro   (moved from flat layout)
    │   └── config/
    │       └── scenes/
    │           ├── empty.yaml
    │           ├── desk.yaml
    │           └── calibration.yaml
    │
    ├── agx_arm_workspace_moveit/
    │   ├── package.xml          (ament_cmake, skeleton only)
    │   ├── CMakeLists.txt
    │   └── config/              (empty — filled by MoveIt Setup Assistant in 7b)
    │
    └── agx_arm_workspace_bringup/
        ├── package.xml          (ament_python, depends on _description + _moveit + wristcam)
        ├── setup.py
        ├── setup.cfg
        ├── resource/agx_arm_workspace_bringup
        ├── agx_arm_workspace_bringup/
        │   └── __init__.py
        └── launch/
            ├── view_workspace.launch.py    (moved + updated imports)
            ├── real_workspace.launch.py    (moved + updated imports)
            └── piper_gripper_moveit.launch.py  (moved unchanged)
```

The old top-level `agx_arm_workspace` Python package becomes obsolete.
Its `package.xml` and `setup.py` are replaced by the three inner `package.xml`s.

### Why `agx_arm_workspace_description` uses `ament_cmake`

The description package only installs URDF/Xacro and YAML files — no Python.
`ament_cmake` with `install(DIRECTORY ...)` is idiomatic for pure-data ROS 2
packages. `agx_arm_workspace_bringup` keeps `ament_python` because it has
Python launch files.

### Scene yaml schema (planning-scene-only, not URDF)

`config/scenes/empty.yaml` — no objects:
```yaml
scene_name: empty
objects: []
```

`config/scenes/desk.yaml` — example planning-scene collision boxes:
```yaml
scene_name: desk
objects:
  - id: desk_surface
    primitive: box
    dimensions: [1.2, 0.8, 0.02]
    pose:
      frame_id: base_link
      xyz: [0.4, 0.0, -0.01]
      rpy: [0.0, 0.0, 0.0]
  - id: desk_leg_front_left
    primitive: box
    dimensions: [0.05, 0.05, 0.72]
    pose:
      frame_id: base_link
      xyz: [0.55, 0.35, -0.37]
      rpy: [0.0, 0.0, 0.0]
```

`config/scenes/calibration.yaml` — calibration board stand:
```yaml
scene_name: calibration
objects:
  - id: calibration_board
    primitive: box
    dimensions: [0.25, 0.01, 0.18]
    pose:
      frame_id: base_link
      xyz: [0.35, 0.0, 0.10]
      rpy: [0.0, 0.0, 0.0]
```

Dimensions and poses are approximate placeholders — update before first real use.

### Key package.xml dependencies

`agx_arm_workspace_description/package.xml`:
```xml
<depend>agx_arm_description</depend>
<depend>agx_arm_wristcam</depend>
<depend>xacro</depend>
```

`agx_arm_workspace_bringup/package.xml`:
```xml
<depend>agx_arm_workspace_description</depend>
<depend>agx_arm_workspace_moveit</depend>
<depend>agx_arm_wristcam</depend>
<depend>agx_arm_ctrl</depend>
<depend>robot_state_publisher</depend>
<depend>joint_state_publisher</depend>
<depend>rviz2</depend>
```

### Superrepo note

`src/agx_arm_workspace` is a regular directory in the superrepo (not a
submodule). All changes go directly into a superrepo commit.

### Commit (superrepo)

`refactor(workspace): split into _description, _moveit skeleton, _bringup`

### Human steps

None.

### Build & smoke test

```bash
colcon build --packages-select \
  agx_arm_workspace_description \
  agx_arm_workspace_moveit \
  agx_arm_workspace_bringup \
  agx_arm_wristcam
source install/setup.bash

# Should still work after the move:
ros2 launch agx_arm_workspace_bringup view_workspace.launch.py
```

### Acceptance

- All three inner packages build cleanly
- `view_workspace.launch.py` still works (resolves `$(find agx_arm_workspace_description)`)
- Old `agx_arm_workspace` package is removed from `colcon build` without errors

---

## Chunk 4 — `agx_arm_wristcam` driver bringup + `/wrist_camera` topic contract

**Goal:** one-command camera bringup that publishes the standard topic set;
sim-bypass mode for use without real hardware.

### New files inside `src/agx_arm_wristcam`

#### Launch files

**`launch/wrist_camera.launch.py`** — top-level entry point:

Arguments:
| Arg | Default | Description |
|---|---|---|
| `camera_model` | `realsense_d435` | Model key from `SUPPORTED_CAMERAS` |
| `mount_name` | `default` | Mount config label (unused in v1, reserved) |
| `calibration_profile` | `factory` | Extrinsic profile name (maps to `<model>__<profile>.yaml`) |
| `namespace` | `wrist_camera` | ROS namespace root for all topics |
| `align_depth` | `true` | Align depth to color frame |
| `publish_pointcloud` | `false` | Enable `/wrist_camera/points` |
| `use_sim` | `false` | Skip driver; expect topics already present |
| `publish_static_tf` | `true` | Publish static TF from extrinsic yaml |

Logic:
- If `use_sim:=false`, include `_realsense.launch.py` (or the appropriate model's launch)
- Always publish the static TF from the extrinsic file (sim and real both need it)
- Remap vendor topics onto the standard `/wrist_camera/...` contract

**`launch/_realsense.launch.py`** — private RealSense wrapper:
- Includes `realsense2_camera/launch/rs_launch.py` with remaps
- Maps:
  - `color/image_raw` → `/{namespace}/color/image_raw`
  - `color/camera_info` → `/{namespace}/color/camera_info`
  - `depth/image_rect_raw` → `/{namespace}/depth/image_rect_raw`
  - `depth/camera_info` → `/{namespace}/depth/camera_info`
  - `color/image_rect_color` (post-rectify node) → `/{namespace}/color/image_rect`

**`launch/calibration.launch.py`** — eye-in-hand calibration entry point
(implemented in Chunk 5; just a stub placeholder here).

#### Python additions

**`agx_arm_wristcam/static_tf_publisher.py`** — reads an extrinsic yaml and
publishes a static transform:
```python
def publish_from_file(extrinsic_yaml_path: str) -> Node:
    """Return a StaticTransformBroadcaster node configured from an extrinsic yaml."""
```
Used by `wrist_camera.launch.py` so calibration profiles are live without
recompiling.

#### `package.xml` additions

```xml
<exec_depend>realsense2_camera</exec_depend>
<exec_depend>image_proc</exec_depend>    <!-- for rectification node -->
<exec_depend>tf2_ros</exec_depend>
```

### Standard topic contract (from `contracts.py`, restated)

```
/{namespace}/color/image_raw
/{namespace}/color/image_rect          (rectified, consumed by AprilTag detector)
/{namespace}/color/camera_info
/{namespace}/depth/image_rect_raw
/{namespace}/depth/camera_info
/{namespace}/points                    (only if publish_pointcloud:=true)
```

### TF chain contract

```
gripper_base
  └─ camera_mount_link    (from extrinsic yaml via static TF publisher)
       └─ realsense_d435_link
            ├─ realsense_d435_color_optical_frame
            └─ realsense_d435_depth_optical_frame
```

### Commit (inside `src/agx_arm_wristcam`)

`feat(wristcam): add realsense driver bringup and standard topic contract`

### Human steps

1. Ensure `realsense2_camera` ROS 2 package is installed in the ROS environment:
   ```bash
   sudo apt install ros-jazzy-realsense2-camera
   # or from source if vendor version needed
   ```
2. For hardware testing: connect RealSense D435 and confirm it appears as
   `/dev/video*` or via `rs-enumerate-devices`.

### Build & smoke test

```bash
colcon build --packages-select agx_arm_wristcam
source install/setup.bash

# Test with sim-bypass (no physical camera needed):
ros2 launch agx_arm_wristcam wrist_camera.launch.py use_sim:=true
# Verify: static TF is published
ros2 run tf2_ros tf2_echo gripper_base camera_mount_link

# Test with real camera (hardware required):
ros2 launch agx_arm_wristcam wrist_camera.launch.py
ros2 topic hz /wrist_camera/color/image_raw   # expect ~30 Hz
ros2 topic echo /wrist_camera/color/camera_info --once
```

### Acceptance

- `use_sim:=true` launch starts without errors; static TF is published
- (With hardware) all standard topics publish at expected rates
- `ros2 topic list | grep wrist_camera` shows only the contracted topic names

---

## Chunk 5 — Eye-in-hand calibration tooling

**Goal:** developer can run calibration, save results, and switch profiles at
launch time without touching code.

### New files inside `src/agx_arm_wristcam`

#### Launch

**`launch/calibration.launch.py`**:

Arguments:
| Arg | Default | Description |
|---|---|---|
| `camera_model` | `realsense_d435` | Camera to calibrate |
| `mount_name` | `default` | Mount label for output filename |
| `output_profile` | `workshop_YYYYMMDD` | Output extrinsic filename prefix |
| `tag_type` | `charuco` | Calibration target: `charuco` or `apriltag` |
| `robot_base_frame` | `base_link` | Fixed frame |
| `ee_frame` | `tcp_link` | End-effector frame |

Logic:
1. Include `wrist_camera.launch.py use_sim:=false publish_static_tf:=false`
   (real camera, no static TF yet — calibration determines the TF)
2. Launch `moveit_calibration` handeye GUI node configured with:
   - sensor topic: `/{namespace}/color/image_rect`
   - camera_info topic: `/{namespace}/color/camera_info`
   - robot base frame: `base_link`
   - EE frame: `tcp_link`

**`scripts/save_calibration_result.py`** — converts MoveIt handeye output
into the extrinsic yaml schema:

```
Usage:
  python3 scripts/save_calibration_result.py \
    --moveit-result /path/to/handeye_output.yaml \
    --camera-model realsense_d435 \
    --profile workshop_20260516 \
    --notes "calibrated on desk setup, 15 poses"

Output:
  config/extrinsics/realsense_d435__workshop_20260516.yaml
```

The script copies the transform fields into our schema and adds metadata.

#### `config_io.py` update

`resolve_camera_setup()` already resolves the extrinsic file path from
`<model>__<profile>.yaml` convention. Ensure it raises `FileNotFoundError`
with a clear message if the requested profile does not exist:
```
FileNotFoundError: No extrinsic file for camera=realsense_d435, profile=missing_profile.
Expected: /path/to/agx_arm_wristcam/config/extrinsics/realsense_d435__missing_profile.yaml
Available profiles: factory, workshop_20260516
```

#### Documentation

Add `config/extrinsics/README.md` explaining:
- File naming convention: `<camera_model>__<profile_name>.yaml`
- When to use `factory` vs a dated workshop profile
- How to run calibration and save a new profile
- How to select a profile at launch time

### Commit (inside `src/agx_arm_wristcam`)

`feat(wristcam): add eye-in-hand calibration launch and save script`

### Human steps

1. Install `moveit_calibration`:
   ```bash
   sudo apt install ros-jazzy-moveit-calibration
   ```
2. To run a calibration session:
   ```bash
   # Start the robot (sim or real) with MoveIt
   ros2 launch agx_arm_workspace_bringup workspace_bringup.launch.py mode:=real

   # In a second terminal, start calibration:
   ros2 launch agx_arm_wristcam calibration.launch.py \
     output_profile:=workshop_20260516

   # Follow the MoveIt Handeye Calibration GUI:
   #  1. Select "Eye-in-Hand" configuration
   #  2. Verify sensor and robot frame parameters match the launch args
   #  3. Move the arm to 10–15 varied poses; click "Take Sample" at each
   #  4. Click "Calibrate" — inspect residual error
   #  5. Save result to /tmp/handeye_output.yaml
   ```
3. Convert and save the result:
   ```bash
   cd src/agx_arm_wristcam
   python3 scripts/save_calibration_result.py \
     --moveit-result /tmp/handeye_output.yaml \
     --camera-model realsense_d435 \
     --profile workshop_20260516 \
     --notes "desk setup, 15 poses"
   ```
4. Commit the new extrinsic file:
   ```bash
   git add config/extrinsics/realsense_d435__workshop_20260516.yaml
   git commit -m "chore(wristcam): add realsense_d435 workshop calibration profile"
   git push origin main
   ```
5. Test the new profile:
   ```bash
   ros2 launch agx_arm_wristcam wrist_camera.launch.py \
     calibration_profile:=workshop_20260516
   ros2 run tf2_ros tf2_echo gripper_base camera_mount_link
   # Verify the transform matches saved values
   ```

### Acceptance

- `calibration.launch.py` starts MoveIt handeye GUI connected to wrist camera topics
- `save_calibration_result.py` produces a valid extrinsic yaml
- `wrist_camera.launch.py calibration_profile:=<new>` applies the new transform
- `calibration_profile:=missing` raises the clear FileNotFoundError message

---

## Chunk 6 — `agx_arm_workspace_bringup` unified launcher

**Goal:** single top-level launch surface that composes the full system for
any mode/camera/scene combination.

### New file

**`src/agx_arm_workspace/src/agx_arm_workspace_bringup/launch/workspace_bringup.launch.py`**

Arguments:
| Arg | Default | Choices | Description |
|---|---|---|---|
| `mode` | `sim` | `sim`, `real` | Backend: Gazebo or real hardware |
| `camera` | `none` | `none`, `realsense_d435` | Wrist camera model |
| `calibration_profile` | `factory` | any saved profile | Extrinsic profile for camera TF |
| `scene` | `none` | `none`, `desk`, `calibration` | Planning-scene collision objects |
| `rviz` | `true` | `true`, `false` | Launch RViz |
| `use_sim_time` | (auto) | — | Inferred from `mode`; do not pass manually |

#### Composition logic

`mode:=sim`:
- Include `agx_arm_gzsim/launch/piper_with_gripper_moveit_gzsim.launch.py`
- `use_sim_time:=true`

`mode:=real`:
- Include `agx_arm_workspace_bringup/launch/piper_gripper_moveit.launch.py`
  (which wraps `agx_arm_ctrl`)
- `use_sim_time:=false`

`camera:=realsense_d435` (any value except `none`):
- Include `agx_arm_wristcam/launch/wrist_camera.launch.py` with
  `camera_model:=<value>`, `calibration_profile:=<value>`,
  `use_sim:=(mode == sim)`

`scene:=desk` (any value except `none`):
- Launch a `scene_loader` node that reads
  `agx_arm_workspace_description/config/scenes/<value>.yaml` and
  publishes collision objects to MoveIt's planning scene monitor

`rviz:=true`:
- Launch RViz2 with MoveIt RViz plugin config

#### Scene loader node

**`agx_arm_workspace_bringup/agx_arm_workspace_bringup/scene_loader.py`**:

```python
"""
Node that reads a scene yaml and adds collision objects to the MoveIt
planning scene monitor on startup. Exits after all objects are added.
"""
```

Reads the schema from Chunk 3 `config/scenes/*.yaml`, constructs
`moveit_msgs/CollisionObject` messages, and publishes to `/collision_object`.

#### Startup diagnostic log

The launcher prints on startup:
```
[workspace_bringup] mode=sim  camera=realsense_d435  scene=desk  rviz=true
[workspace_bringup] calibration_profile=factory
[workspace_bringup] use_sim_time=true
```

### Commit (superrepo, `src/agx_arm_workspace`)

`feat(workspace_bringup): add unified workspace_bringup launcher and scene loader`

### Human steps

None.

### Build & smoke test

```bash
colcon build --packages-select agx_arm_workspace_bringup agx_arm_workspace_description
source install/setup.bash

# Sim, no camera:
ros2 launch agx_arm_workspace_bringup workspace_bringup.launch.py \
  mode:=sim camera:=none scene:=none rviz:=true

# Sim, with camera, with desk scene:
ros2 launch agx_arm_workspace_bringup workspace_bringup.launch.py \
  mode:=sim camera:=realsense_d435 scene:=desk rviz:=true

# Verify planning scene has desk objects:
ros2 topic echo /planning_scene --once | grep desk_surface
```

### Acceptance

- Both smoke test commands launch cleanly with MoveIt reporting
  "You can start planning now!"
- RViz shows the robot model
- With `scene:=desk`, the desk box appears in the RViz planning scene
- With `camera:=realsense_d435 mode:=sim`, static TF is published

---

## Chunk 7a — Prep for MoveIt Setup Assistant

**Goal:** produce a clean, flat URDF from the workspace xacro; document
exactly what the human needs to configure in the MoveIt Setup Assistant;
create the empty `agx_arm_workspace_moveit` package skeleton.

### Developer steps

#### URDF generation script

**`src/agx_arm_workspace/src/agx_arm_workspace_description/scripts/generate_workspace_urdf.sh`**:
```bash
#!/usr/bin/env bash
# Expands piper_workspace.urdf.xacro with default args suitable for
# MoveIt Setup Assistant. Output written to /tmp/agx_workspace.urdf.
set -euo pipefail
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../../../../../install/setup.bash" 2>/dev/null || true

xacro "$(ros2 pkg prefix --share agx_arm_workspace_description)/urdf/piper_workspace.urdf.xacro" \
  > /tmp/agx_workspace.urdf

check_urdf /tmp/agx_workspace.urdf
echo "URDF written to /tmp/agx_workspace.urdf — ready for MoveIt Setup Assistant"
```

#### `agx_arm_workspace_moveit` skeleton

```
src/agx_arm_workspace_moveit/
├── package.xml           (ament_cmake, version 0.1.0)
├── CMakeLists.txt        (minimal, install(DIRECTORY config DESTINATION ...) stub)
└── config/               (empty — filled by MoveIt SA in step 7b)
```

`package.xml` build/exec deps:
```xml
<depend>moveit_ros_planning_interface</depend>
<depend>agx_arm_workspace_description</depend>
```

#### MoveIt Setup Assistant guidance (for the human)

Create `src/agx_arm_workspace/src/agx_arm_workspace_moveit/SETUP_ASSISTANT.md`:

```markdown
# Running MoveIt Setup Assistant for agx_arm_workspace_moveit

## Prerequisites
- Workspace built: `colcon build` from piper_studio root
- URDF generated: run `scripts/generate_workspace_urdf.sh`

## Steps

### 1. Launch Setup Assistant
    ros2 launch moveit_setup_assistant setup_assistant.launch.py

### 2. Load URDF
    Click "Edit Existing MoveIt Configuration Package" if updating,
    or "Create New MoveIt Configuration Package" for first run.
    Load URDF from: /tmp/agx_workspace.urdf

### 3. Self-Collision (Auto)
    Click "Generate Collision Matrix".

### 4. Virtual Joints
    None needed — arm base is fixed to world via base_link.

### 5. Planning Groups
    Keep the existing "arm" group unchanged from vendor config:
      Chain: base_link → tcp_link
    Keep the existing "gripper" group unchanged:
      Links: gripper_base, gripper_link1, gripper_link2
      Joints: gripper_joint1, gripper_joint2

    DO NOT add camera links to any planning group.

### 6. Robot Poses (Named States)
    Add these in addition to any vendor defaults:
      ready:
        joint1: 0.0
        joint2: 0.3
        joint3: -0.5
        joint4: 0.0
        joint5: 0.5
        joint6: 0.0
      park:
        joint1: 0.0
        joint2: 1.0
        joint3: -1.2
        joint4: 0.0
        joint5: 0.8
        joint6: 0.0
    Note: avoid exact zero values on joints with [0, max] limits.

### 7. End Effectors
    Name: gripper_eef
    Group: gripper
    Parent link: tcp_link
    Parent group: arm

### 8. Passive Joints
    None.

### 9. Controllers
    Skip — controller config is managed per-profile in launch_utils.py.

### 10. Simulation
    Skip.

### 11. 3D Perception
    Skip — perception is handled outside MoveIt config.

### 12. Author Information
    Fill in name and email.

### 13. Generate Configuration Files
    Output path: src/agx_arm_workspace/src/agx_arm_workspace_moveit/
    Click Generate.

### Post-generation
    Remove any auto-generated launch/ directory — launch is owned by
    agx_arm_workspace_bringup.
    Commit: git add config/ && git commit -m "feat(workspace_moveit): add MoveIt config from SA"
```

### Commit (superrepo)

`feat(workspace): add moveit skeleton and URDF generation script`

### Human steps

None yet — this chunk is pure developer work. Human work is in 7b.

### Acceptance

```bash
bash src/agx_arm_workspace/src/agx_arm_workspace_description/scripts/generate_workspace_urdf.sh
# Must print: "URDF written to /tmp/agx_workspace.urdf — ready for MoveIt Setup Assistant"
check_urdf /tmp/agx_workspace.urdf  # exit 0
```

---

## Chunk 7b — [HUMAN] Run MoveIt Setup Assistant

**This entire chunk is human-performed. Developer (AI) waits.**

### Steps

1. Build the workspace:
   ```bash
   cd /path/to/piper_studio
   source /opt/ros/jazzy/setup.bash
   source .venv/bin/activate
   colcon build
   source install/setup.bash
   ```

2. Generate the flat URDF:
   ```bash
   bash src/agx_arm_workspace/src/agx_arm_workspace_description/scripts/generate_workspace_urdf.sh
   ```

3. Run the MoveIt Setup Assistant and follow `SETUP_ASSISTANT.md` instructions.
   Set output directory to:
   ```
   src/agx_arm_workspace/src/agx_arm_workspace_moveit/
   ```

4. After generation, delete the auto-generated `launch/` subdirectory if present.

5. Commit inside `src/agx_arm_workspace` (not a submodule — direct superrepo commit):
   ```bash
   git add src/agx_arm_workspace/src/agx_arm_workspace_moveit/config/
   git commit -m "feat(workspace_moveit): add MoveIt config from Setup Assistant"
   ```

6. Signal to AI: "7b done, proceed with 7c".

---

## Chunk 7c — Retarget `agx_arm_motion` to workspace MoveIt config

**Goal:** `agx_arm_motion/launch_utils.py` builds the `MoveItConfigs` object
from `agx_arm_workspace_moveit` instead of vendor `agx_arm_moveit`. All
existing executables and `MotionFacade` API stay identical.

### Changes inside `src/agx_arm_motion` submodule

#### `agx_arm_motion/launch_utils.py`

Replace the `MoveItConfigsBuilder` calls in both `sim` and `real` branches:

Before (both profiles):
```python
MoveItConfigsBuilder("agx_arm", package_name="agx_arm_moveit")
.robot_description(file_path="config/agx_arm.urdf.xacro", mappings=_URDF_MAPPINGS)
.robot_description_semantic(file_path="config/agx_arm.srdf.xacro", mappings=_SRDF_MAPPINGS)
```

After (both profiles):
```python
MoveItConfigsBuilder("agx_arm", package_name="agx_arm_workspace_moveit")
.robot_description(
    file_path="config/agx_workspace.urdf.xacro",   # generated by SA from piper_workspace
    mappings={},   # SA-generated URDF has args baked in; no mappings needed
)
.robot_description_semantic(file_path="config/agx_workspace.srdf")
```

Trajectory execution files remain profile-specific:
- `sim` → `agx_arm_gzsim/config/moveit_controllers.yaml` (unchanged)
- `real` → `agx_arm_workspace_moveit/config/moveit_controllers.yaml`
  (generated by SA, or copied from vendor + adjusted)

Remove `_URDF_MAPPINGS`, `_SRDF_MAPPINGS` module-level constants if no longer used.

#### `package.xml`

Add: `<exec_depend>agx_arm_workspace_moveit</exec_depend>`

The `agx_arm_moveit` dependency can be removed once the switch is validated.
The `agx_arm_gzsim` dependency stays (needed for sim controller yaml path).

**Note:** the exact URDF filename and SRDF filename are determined after 7b.
This section will be updated with correct filenames when 7b is complete.

### Validation sequence

```bash
colcon build --packages-select agx_arm_motion agx_arm_workspace_moveit
source install/setup.bash

# Sim smoke test:
# Terminal 1:
ros2 launch agx_arm_workspace_bringup workspace_bringup.launch.py mode:=sim
# Terminal 2 (after "You can start planning now!"):
ros2 launch agx_arm_motion move_to_named_pose.launch.py pose_name:=ready profile:=sim
ros2 launch agx_arm_motion move_to_pose.launch.py x:=0.30 y:=0.0 z:=0.25 pitch:=1.5708 profile:=sim
```

### Commit (inside `src/agx_arm_motion`)

`feat(motion): retarget MoveItConfigsBuilder to agx_arm_workspace_moveit`

### Human steps

1. Push the `agx_arm_motion` submodule:
   ```bash
   cd src/agx_arm_motion
   git push origin main
   ```
2. Update superrepo submodule pointer:
   ```bash
   cd /path/to/piper_studio
   git add src/agx_arm_motion
   git commit -m "chore: bump agx_arm_motion submodule to workspace moveit config"
   git push origin main
   ```

### Acceptance

- `move_to_named_pose pose_name:=ready` moves arm to the `ready` pose defined in the SA config
- `move_to_pose x:=0.30 z:=0.25 pitch:=1.5708` plans and executes successfully
- No `agx_arm_moveit` package references remain in `launch_utils.py`
- Camera collision geometry appears in the MoveIt planning scene (arm
  cannot plan through the camera box)

---

## Chunk 8 — Documentation tidy

**Goal:** all docs reflect the current state; nothing refers to the old layout
or the brainstorm phase.

### Files to update

| File | Action |
|---|---|
| `DESIGN.md` | Update "Current Baseline" section; update package roadmap; update "Pinned Cross-Package Contract" to add wristcam TF and topic contract |
| `TODO.md` | Mark Phase 0 motion generalization done; mark Phase 1 camera adapter items done; update Phase 2 calibration items |
| `AGENTS.md` | Update package ownership list to include `agx_arm_wristcam` and the three `agx_arm_workspace_*` packages |
| `README.md` | Update quick-start and package overview |
| `ImplementationPlan.md` | Move to `for_reference/ImplementationPlan_brainstorm.md` |
| `PLAN.md` (this file) | Note at top: "Implemented — see git log for details" |

### Commit (superrepo)

`docs: update DESIGN, TODO, AGENTS, README after wristcam and workspace restructure`

### Human steps

Push superrepo:
```bash
git push origin main
```

---

## Appendix A — Adding a new camera (developer guide)

When Astra U3 or OAK-D Max needs to be added, a developer follows these steps.
No existing code changes; purely additive.

1. **`src/agx_arm_wristcam/urdf/models/astra_u3.xacro`**
   Copy `_template.xacro`, fill in link names, dimensions, optical frame offsets.

2. **`src/agx_arm_wristcam/config/astra_u3.yaml`**
   Fill in `model`, `mount_xacro`, `optical_frames`, `default_namespace`.

3. **`src/agx_arm_wristcam/config/extrinsics/astra_u3__factory.yaml`**
   Fill in nominal transform from gripper_base to camera_mount_link.

4. **`src/agx_arm_wristcam/launch/_astra_u3.launch.py`**
   Wrap the Astra/Orbbec ROS driver with the same topic remaps as `_realsense.launch.py`.

5. **`urdf/wrist_camera.xacro`** — add one `<xacro:if>` branch for `astra_u3`.

6. **`launch/wrist_camera.launch.py`** — add one `<IfCondition>` block to include
   `_astra_u3.launch.py` when `camera_model:=astra_u3`.

7. **`agx_arm_wristcam/contracts.py`** — add `"astra_u3"` to `SUPPORTED_CAMERAS`.

8. Run Chunk 4 smoke tests with `camera_model:=astra_u3`.

No other files change.

---

## Appendix B — Git workflow summary

```
After Chunk 1:
  push: src/agx_arm_wristcam (main) → github.com/charithmu/agx_arm_wristcam
  push: piper_studio (main) — .gitmodules update + agx_arm_workspace xacro fix

After Chunk 2:
  push: src/agx_arm_wristcam (main) — xacro + config_io
  push: piper_studio (main) — submodule pointer bump

After Chunk 3:
  push: piper_studio (main) — workspace restructure (regular dir, no submodule push needed)

After Chunk 4:
  push: src/agx_arm_wristcam (main) — driver bringup launch files
  push: piper_studio (main) — submodule pointer bump

After Chunk 5:
  push: src/agx_arm_wristcam (main) — calibration launch + save script
  push: piper_studio (main) — submodule pointer bump

After Chunk 6:
  push: piper_studio (main) — workspace_bringup launcher (regular dir)

After Chunk 7a:
  push: piper_studio (main) — generate script + moveit skeleton

After Chunk 7b (human):
  push: piper_studio (main) — SA-generated moveit config files

After Chunk 7c:
  push: src/agx_arm_motion (main) — retargeted launch_utils.py
  push: piper_studio (main) — submodule pointer bump

After Chunk 8:
  push: piper_studio (main) — docs
```
