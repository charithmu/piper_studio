# Robot description decisions

Status 2026-11-08 (D1-D10 from 2026-10-08; D11-D13 added). Evidence: [MODEL_AUDIT.md](MODEL_AUDIT.md) (all sources compared with `model_audit.py`).
**Rule (user, 2026-11-08): the arm is on the latest firmware it supports and is calibrated, so the values its firmware reports win.** Next come the latest vendor URDF values
(`agx_arm_urdf`). Where the two differ, the firmware value is used (`piper_description/config/firmware_values.yaml`, applied by the generator, `piper_arm_macro.py report` lists
the differences) and the difference is told to the user. Older firmware values and old URDF values are not kept. Every value used by Piper Studio has a stated provenance. **Provisional** values must not be used
as physical ratings (for safety limits, torque budgets or actuator identification).

## Single source of truth

`piper_description/urdf/piper_arm.xacro` is the official `agilexrobotics/agx_arm_urdf` model (pinned through
`external/agx_arm_ros`) as a mountable xacro macro with a name prefix. It is **generated** from the unmodified official files by
`tools/piper_arm_macro.py`, committed with provenance (source commits and file hashes in its header), and `check` (run by
`test_description.py`) fails if it differs from the official model in any link, joint, limit, inertial, mesh or mimic. The same tool
generates the prefixed official SRDF in `piper_control`. `piper.urdf.xacro` assembles the standalone robot from the macros and adds only:

- `${prefix}tcp_link` (fingertip centre by default; `urdf/piper.tool.xacro`),
- the wrist camera (a model in `urdf/cameras/` on a mount in `urdf/mounts/`, D11/D13),
- the ros2_control block that selects the backend.

Names carry the prefix `piper_` by default (D12); the tests also compare the unprefixed expansion with the official model. MuJoCo, Gazebo and
Isaac models are generated from this description (`piper_sim`), not maintained by hand. Names in the tables below are the official ones; the
robot's are `piper_` + these.

## Decisions

| # | Item | Decision | Provenance / status |
|---|---|---|---|
| D1 | Kinematics | Official agx_arm_urdf@680fa92 | Vendor CAD export. Isaac description agrees within 0.05 mm. Stable since the previous pin (3080af4). |
| D2 | Joint position limits | J1 ±2.618, J2 [0, 3.1416], J3 [−2.967, 0], J4 ±1.745, J5 ±1.222, J6 ±2.967 rad (±150/180/170/100/70/170°) | **Firmware and vendor URDF agree** (read from the arm, firmware S-V1.9-1, 2026-10-14). The vendor URDF said J6 ±120° until 2026-10-13, when AgileX changed it to ±170°. |
| D3 | Velocity and acceleration limits | **3.0 rad/s** and **5.0 rad/s²** on all six joints (MoveIt scaling 0.1 by default) | **Firmware** (`max_joint_spd`, `max_joint_acc`). **Differs from the vendor URDF** (5 rad/s; no acceleration). Jerk: no source. |
| D4 | Effort limits | **±16 N·m (joints 1-3), ±13.012 N·m (joints 4-6)**; gripper 10 (vendor) | The SDK's `move_mit` feed-forward torque range at firmware >= S-V1.8-8 (pyAgxArm `firmware_reference.md`). **Not firmware-reported, not a motor rating**: an upper bound on commandable torque, arm-specific, used instead of the vendor placeholder 100 (`config/firmware_values.yaml`). Static holding of joint 2 needs about 12.5 N·m (CAD masses), so the range is plausible. Simulators use their own actuator limits (MuJoCo ±100, Isaac drive gains) and are not tied to it. |
| D5 | Masses/inertias | Vendor URDF (total 4.71 kg arm + gripper; link6 corrected from CAD on 2026-10-16: 16.6 g) | Vendor CAD estimate, not in the firmware, unmeasured. Per-link table with sources: [MASSES.md](MASSES.md). |
| D6 | Gripper model | One `gripper` joint = opening width in metres, **0–0.07**; fingers are mimic joints (±0.035 each) | **Firmware** `max_range_config` 0.07 m (confirmed by the user, about 70 mm). **Differs from the vendor URDF** (0.1 m, ±0.05). |
| D7 | TCP | `tcp_link` = gripper_base + 0.138 m z (fingertip plane centre) = flange + 0.1425 m | Derived from the finger meshes (tips end at z = 0.138 m in gripper_base). Overridable via `tcp_xyz`. |
| D8 | MuJoCo Menagerie / legacy `agx_arm_mjsim` model | **Not used** | Legacy MJCF is an exact copy of Menagerie. link6 FK differs by up to 11.3 mm (configuration dependent, so a real kinematic difference). Total mass 2.37 kg. J3/J4/J6 limits differ (−2.697 looks like a digit swap of −2.967). Independent 35 mm fingers. Kept as a contact-parameter reference only. |
| D9 | AgileX Isaac description (Dec 2025) | Reference only; regenerate USD from D1 | J1 upper 2.168 is a digit swap of 2.618. J6 velocity 3. Old two-finger gripper. |
| D10 | Piper-L material (IIT) | Not applicable | Different link lengths (joint3→4 0.338 vs 0.285 m). |
| D11 | Wrist camera mount and meshes (see the 4.5 mm question in CLEANUP C5) | D435 on AgileX's `realsense_mid_stand` at AgileX's pose from agx_arm_sim `piper_girpper_D435.urdf` (MIT): stand at gripper_base (−0.034, −0.001, 0.022), camera on the stand (−0.031, −0.028, 0.0018), rpy 0 −1.22 0. Coloured D435 mesh from MuJoCo Menagerie `realsense_d435i` (Apache-2.0), built by `tools/build_d435_mesh.py` | The old pose was AgileX's older piper_isaac_sim mount (link6 frame rotated 90° about z), which put the camera in line with one finger. Verified numerically (camera_link in link6 equals AgileX's) and visually (Gazebo wrist image, RViz meshes and colours). **Decided 2026-11-08:** AgileX's numbers are used relative to `gripper_base`, which the official model puts 4.5 mm out from link6 (the stand is clamped to the gripper body); **open:** stand collision box vs fingers in MoveIt not checked; MuJoCo drops mesh colours; Isaac USD needs `isaac.sh build-usd d435`. |
| D12 | Prefix and mount for other platforms | A `piper_arm` xacro macro generated once from the unmodified official URDF and committed (option A), default prefix `piper_`. B (fork the URDF) and C (no prefix) are recorded in `docs/ARCHITECTURE.md` | **Done 2026-11-08.** The macro is committed with provenance and validated by `tools/piper_arm_macro.py check` (also in the tests). Default prefix `piper_`. For the Go2 mount (M9). |
| D13 | Camera: model and mount | Two separate xacro macros: `urdf/cameras/<model>.xacro` (frames, body mesh, simulated sensor; d435) and `urdf/mounts/<mount>.xacro` (bracket mesh, collision, poses; `agilex_mid_stand`, `custom`), chained by `piper_camera`; selected with `camera:=` and `camera_mount:=` | New cameras (ZED, OAK-D Pro) and CAD-exported brackets are added as one file each. Real-arm calibration will override the nominal pose (planned: `config/camera_mounts/`). |

## Dynamic limits: what each source says, and starting values for downstream work (2026-11-08)

| Quantity | Firmware (S-V1.9-1) | Vendor files | Other | Used / suggested start |
|---|---|---|---|---|
| Velocity | 3.0 rad/s, all joints | URDF 5; vendor MoveIt yaml 5; vendor Isaac description 10 (J6 3) | flange (Cartesian moves only): 1.0 m/s, 0.25 rad/s | 3.0 is the maximum. Start planners at 50 % (1.5 rad/s); Studio MoveIt default scaling is 10 % |
| Acceleration | 5.0 rad/s², all joints (the drive's CPV acceleration and deceleration are also 5.0) | URDF none; vendor MoveIt yaml 5 | flange 1.5 m/s², 0.3 rad/s² | 5.0 is the maximum. Start at 50 % (2.5 rad/s²) |
| Jerk | not reported | none anywhere in the vendor repos (checked agx_arm_ros, agx_arm_urdf, agx_arm_sim, pyAgxArm, piper_sdk) | cuRobo's bundled Franka uses 500 rad/s³ (not Piper data) | no source. Derive: reach the acceleration limit in about 0.2 s, 25 rad/s³ at the maximum (12.5 at 50 %). Our value; tune with `stepresp` |
| Torque | not reported | vendor URDF 100 N·m on every joint (placeholder, same in the vendor Isaac and MuJoCo models; gripper 10) | SDK `move_mit` feed-forward range at firmware >= S-V1.8-8: ±16 N·m (joints 1-3), ±13.0 N·m (joints 4-6), a command encoding range, not a rating | **now used in the description** (D4). No rating exists. Static holding torque from the CAD masses (arm + gripper, worst pose, no payload; our estimate): J1 0, **J2 12.5**, J3 4.9, J4 0.9, J5 1.0, J6 0.02 N·m, so a planning limit must exceed these. Start with torque-limited planning off, or use the SDK range as an upper bound; the driver publishes motor torque estimates (`feedback/joint_states` effort) to log during real runs |

Safe numbers are starting points for the first runs; none is a rating.

## Simulator models

Simulators get the same description plus generated, tested transforms (`piper_sim`):

- `robot_description(<simulator>)` (ROS side: robot_state_publisher, MoveIt, ros2_control): gives the
  virtual `gripper_link` a 10 g placeholder inertial, because physics engines drop massless moving
  links (SDFormat removed the `gripper` joint). Limits and kinematics stay official.
- `physics_model()` (what the physics engine loads): no URDF `<mimic>` constraints, because
  gz_ros2_control drives the mimic fingers in software, and every joint limit widened by 0.01 rad /
  2 mm. Gazebo/DART velocity control cannot move a joint off a limit it rests on: a gripper closed to
  0 never reopened. Commanded targets stay within the official limits enforced by ros2_control,
  MoveIt and piper_py, so they never touch a physics stop.
- MuJoCo: `mujoco_model()` runs mujoco_ros2_control's URDF converter on the simulator description
  (collision STLs reused as visuals, because the converter keys meshes by file stem and mixed up the
  DAE visuals with the collision meshes) plus `piper_sim/config/mujoco/inputs.xml`: position
  actuators named like the ros2_control joints, finger equality constraints (±0.5 × gripper),
  contact exclusions (world–link1 because the fused base is not filtered as a parent; finger–finger
  as in the official SRDF). FK equals the official URDF to 1e-6 m; mass is conserved (fixed links
  fused into link6). The **actuator model is provisional**: critically damped position servos
  (kp 400/400/400/100/100/50, gripper 400 with dampratio 2) with ideal gravity compensation,
  standing in for the unidentified firmware loop.
- Gazebo position control is kinematic (velocity commands proportional to position error, joint
  ground truth matches commands to ~1e-13). It validates interfaces, kinematics and planning, not
  actuator dynamics.

## Consequences found while qualifying

- **Top-down reach.** With the TCP at the fingertips, joint 5 (±70°) allows a vertical tool-down
  approach only for TCP heights up to ~0.10–0.12 m above the base plane, about 0.17–0.30 m forward.
  Higher top-down targets are infeasible (the legacy example poses at z = 0.2–0.3 m were).
- **Direct trajectories do not check limits.** `joint_trajectory_controller` does not enforce URDF
  limits, so `piper_py` validates direct joint targets against the URDF before sending them.

## Open items needing the real arm (read-only, user present)

1. (Done: firmware S-V1.9-1, limits and gripper stroke 0.07 m read from the arm, see `docs/real_arm/arms/piper-01.md`.) Still open: a caliper measurement of the opening.
3. Real-arm confirmation of the camera model, the stand and its pose (nominal from AgileX, D11) and the TCP offset; calibration override (`config/camera_mounts/`, CLEANUP C9).
