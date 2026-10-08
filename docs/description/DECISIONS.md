# Robot description decisions

Status 2026-10-08. Evidence: [MODEL_AUDIT.md](MODEL_AUDIT.md) (all sources compared with `model_audit.py`).
Every value used by Piper Studio has a stated provenance. **Provisional** values must not be used
as physical ratings (for safety limits, torque budgets or actuator identification).

## Single source of truth

`piper_description/urdf/piper.urdf.xacro` includes the official
`agilexrobotics/agx_arm_urdf` model (pinned through `external/agx_arm_ros`) **unmodified** and adds only:

- `tcp_link` (fingertip centre by default),
- an optional world anchor,
- the ros2_control block that selects the backend.

`test_description.py` fails if the expanded model differs from the official one in any joint limit,
mass or link6 forward kinematics. MuJoCo, Gazebo and Isaac models are to be generated from this
description, not maintained by hand.

## Decisions

| # | Item | Decision | Provenance / status |
|---|---|---|---|
| D1 | Kinematics | Official agx_arm_urdf@983788b | Vendor CAD export. Isaac description agrees within 0.05 mm. Stable since the previous pin (3080af4). |
| D2 | Joint position limits | Official: J1 ±2.618, J2 [0, 3.1416], J3 [−2.967, 0], J4 ±1.745, J5 ±1.222, J6 ±2.094 rad | Vendor. Consistent with the published ±150/180/170/100/70/120°. To confirm: firmware-enforced limits (read-only query on the real arm). |
| D3 | Velocity limits | 5 rad/s all joints (MoveIt scaling 0.1 by default) | **Provisional** (vendor placeholder; Isaac uses 3 rad/s for J6). |
| D4 | Effort limits | 100 N·m in the URDF | **Provisional placeholder.** Not a rating. Simulators use position actuators. |
| D5 | Masses/inertias | Official (total 4.70 kg incl. gripper; end-link fix of 2026-10-08) | Vendor CAD estimate. Unmeasured. |
| D6 | Gripper model | One `gripper` joint = opening width in metres, 0–0.1; fingers are mimic joints (±0.05 each) | Vendor (June 2026 model). **Open:** the real stroke is a gripper firmware setting (`max_range_config` 0.07 or 0.1 m). Read it from the arm before relying on widths > 0.07 m. |
| D7 | TCP | `tcp_link` = gripper_base + 0.138 m z (fingertip plane centre) = flange + 0.1425 m | Derived from the finger meshes (tips end at z = 0.138 m in gripper_base). Overridable via `tcp_xyz`. |
| D8 | MuJoCo Menagerie / legacy `agx_arm_mjsim` model | **Not used** | Legacy MJCF is an exact copy of Menagerie. link6 FK differs by up to 11.3 mm (configuration dependent, so a real kinematic difference). Total mass 2.37 kg. J3/J4/J6 limits differ (−2.697 looks like a digit swap of −2.967). Independent 35 mm fingers. Kept as a contact-parameter reference only. |
| D9 | AgileX Isaac description (Dec 2025) | Reference only; regenerate USD from D1 | J1 upper 2.168 is a digit swap of 2.618. J6 velocity 3. Old two-finger gripper. |
| D10 | Piper-L material (IIT) | Not applicable | Different link lengths (joint3→4 0.338 vs 0.285 m). |

## Simulator models

Simulators get the same description plus two generated, tested transforms (`piper_description`):

- `robot_description(<simulator>)` (ROS side: robot_state_publisher, MoveIt, ros2_control): gives the
  virtual `gripper_link` a 10 g placeholder inertial, because physics engines drop massless moving
  links (SDFormat removed the `gripper` joint). Limits and kinematics stay official.
- `physics_model()` (what the physics engine loads): no URDF `<mimic>` constraints, because
  gz_ros2_control drives the mimic fingers in software, and every joint limit widened by 0.01 rad /
  2 mm. Gazebo/DART velocity control cannot move a joint off a limit it rests on: a gripper closed to
  0 never reopened. Commanded targets stay within the official limits enforced by ros2_control,
  MoveIt and piper_py, so they never touch a physics stop.
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

1. Firmware version and the joint limits the firmware enforces.
2. Gripper stroke setting (0.07 or 0.1 m) and measured opening.
3. Camera model, mount and TCP/camera offsets.
