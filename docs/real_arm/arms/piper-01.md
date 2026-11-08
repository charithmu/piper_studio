# piper-01: the arm on the `atlas` workstation

`piper-01` is a placeholder label: rename it when you choose a real one. Raw read-out: `piper-01.readout.json` (`tools/real_arm/arm_info.py`, 2026-10-14, nothing moved).

**This arm runs the latest firmware it supports (S-V1.9-1, upgraded 2026-10-14) and has been calibrated by the user.** The firmware-reported values below are the
reference for this arm; where the vendor URDF differs, the firmware value is used and the difference is listed in "Firmware vs vendor URDF". Older firmware
values are not kept.

| Item | Value |
|---|---|
| Label serial | **to record by hand from the arm's label** |
| Model / gripper | Piper standard / AgileX gripper, firmware-reported stroke **70 mm** (`max_range_config` 0.07 m; teaching range 100, friction 1) |
| Mounting | normal flat position (installation setting: flat). Payload setting: none (empty). Neither can be read back from the arm; both were set by the user. |
| USB-CAN adapter | candleLight (gs_usb), serial `002000295631511920313857`, appears as `can0` (1 Mbit/s) |
| Firmware / hardware | **S-V1.9-1** (latest supported) / H-V1.2-1, motor ratio/batch 10, node ARM_MC #15, date field 261014 (the day of the upgrade), unparsed 4-char field `30AH` |
| SDK profile | `v189` |

## Firmware vs vendor URDF (vendor URDF = `agx_arm_urdf` @ 680fa92, 2026-10-16)

| Value | Firmware (used) | Vendor URDF | Status |
|---|---|---|---|
| Joint 1-6 position limits | same as the URDF; j6 ±170° | j6 ±170° (changed from ±120° on 2026-10-13) | agree since the vendor update |
| Joint max speed | **3.0 rad/s** | 5 rad/s | **differ**: we use 3.0 (`config/firmware_values.yaml`) |
| Joint max acceleration | **5.0 rad/s^2** | not in the URDF | MoveIt uses 5.0 |
| Gripper stroke | **0.07 m** | 0.1 m (fingers 0.05 each) | **differ**: we use 0.07 (fingers 0.035 each) |
| Joint effort | not reported | 100 N.m (placeholder) | **differ**: we use the SDK's MIT torque range, ±16 N.m (joints 1-3) / ±13.012 (joints 4-6): an upper bound, not a rating |
| Link masses/inertias | not reported | CAD estimates | unmeasured, see `docs/description/MASSES.md` |

## Current state (2026-10-14, after the upgrade and the calibration)

- Role slave/feedback, CAN control, MOVE_J, teach off, error 0, all joints disabled, `motion_status` = "reach target failed" (0x1) with no error code (leftover flag).
- Rest pose (deg) 0.10, -2.25, 2.21, -0.43, 22.72, 0.00; gripper width 0.14 mm. Drivers 23-24 V, 31-34 C, no fault bits.
- Stored limits: joint angle limits equal the vendor URDF (joint 6 +-170 deg); speed 3.0 rad/s, acceleration 5.0 rad/s^2 per joint;
  flange 1.0 m/s, 0.25 rad/s, 1.5 m/s^2, 0.3 rad/s^2.
- Crash protection **0 on every joint (no collision detection)**; assistance rating 0 on every joint.
- Bus: status/pose/joints/gripper 200 Hz, motor 100 Hz, driver 20 Hz; extra frames 0x1C0-0x1C3 (20-40 Hz; the SDK's Revo2 hand ids) and 0x212 (1 Hz, undocumented).
  Whether a Revo2 hand is attached or the firmware emits them is not established.
- Joints 2 and 3 rest about -2 deg and +2 deg, outside their stored ranges, also after calibration: the arm sags a little when the motors go off (the user's explanation, 2026-11-08).
  The vendor `piper_ros` README also says the DH parameters after S-V1.6-3 offset those two joint frames by 2 degrees (not confirmed with AgileX). Either way it only matters for leaving the resting pose: see
  "Start-up and shut-down sequence" below.
- Enable test on S-V1.9-1 (2026-10-14): all six joints enabled, max drift 0.08 deg over 2.5 s, currents <= 0.08 A, status normal, clean disable. No motion has been commanded on the real arm yet.
- **First motion (2026-10-14, `tools/real_arm/arm_zero.py`, 50 % speed):** from the resting pose (0.1, -2.2, 2.3, 0.0, 22.0, 0.0 deg) to the zero pose in 0.6 s (joint 5 moved 22 deg), final error 0.06 deg,
  final pose 0.0, +0.48, -0.44, 0.0, 0.0, 0.0 deg (joints 2/3 are aimed 0.5 deg inside their stored limit), no fault, clean disable. The path used was the SDK directly: CAN control + MOVE_J read back, speed 50 %, one `move_j`.
  Our ROS stack (`backend:=real`, command_guard, controllers) has not driven the arm yet.

- CPV drive parameters (read 2026-10-14 with `arm_info.py -c`): acceleration 5.0 and deceleration 5.0 rad/s^2 on every joint, profile velocity 0, position gain 5.0, velocity-loop kp 0.7 (joints 1-3) / 0.8 (joints 4-6), ki 40 (joints 1-3) / 80 (joints 4-6). Reading them put the arm in MOVE_CPV motion mode
  (the first `-c` version did not restore it; the fixed tool sets MOVE_J on the next `-c`, and any `move_j` switches it back).

## Start-up and shut-down sequence (user rule, 2026-11-08)

The resting pose is slightly outside the stored limits of joints 2 and 3 (about -2.2 and +2.3 deg against j2 >= 0, j3 <= 0). This is a quirk of the arm resting on its stops, and it only matters for the first move:

1. **Start:** the arm sits at the resting pose with the motors off. Enable it, then leave the resting pose with one known `move_j` to the `zero` pose, aimed 0.01 rad inside the joint 2/3 limits (done 2026-10-14, `arm_zero.py`, 0.6 s, no fault). The firmware accepts this.
   MoveIt would accept the same start state: its default allowance for a start state outside the limits is 0.05 rad (2.9 deg), more than the 2.3 deg here (default value, not yet seen on the real arm).
2. **Work:** from `zero` onwards everything stays inside the limits. Nothing is ever commanded outside them (the guard keeps commands `limit_margin` = 0.01 rad inside and clamps tiny overshoots to that range; planners and cuRobo get the same limits).
3. **Stop:** always return to `zero` first (`piper rest` goes zero, then `rest`, joint 5 lowered), then disable the motors. The arm drops to its resting pose on its own, which is fine and expected.

Never command a target past a stored limit to "reach" the resting pose; use the sequence above both ways.

## Park pose (recorded by hand 2026-10-14)

Pose recorded with the motors off, wrist a few mm above the support brackets: joints (deg) 0.23, -2.19, 2.45, -0.42, 16.82, 0.0 (`arm_poses.log`). Against zero only joint 5 is lowered
(16.82 deg); joints 2/3 are at the stops (-2.19 / +2.45), which are outside the stored limits (j2 >= 0, j3 <= 0), so the pose cannot be commanded as recorded.
Flange height from the SDK's forward kinematics: resting on the stops (j5 22 deg) 167.0 mm, recorded pose 174.3 mm, the same pose with joints 2/3 clamped inside their limits 189.2 mm, all joints 0: 213.3 mm.
The SRDF `rest` therefore is zero with joint 5 = 0.2936 rad (16.82 deg), about 22 mm above resting; disabling there drops the wrist a little more than the recorded pose would.
Verified 2026-10-14: the firmware accepts *leaving* this resting pose (joints 2/3 at -2.2 / +2.3 deg, outside the stored range) with one `move_j` to a target 0.01 rad inside the limits. Not tested, and not needed: commanding a target past a stored limit (the vendor status code 4 "target angle out of range" is the likely reaction). `rest` therefore stays at joints 2/3 = 0.01 rad inside.

Joint ranges and where each pose sits: ![joint ranges](piper-01.joint_ranges.svg). Limit sources: the arm's stored limits (`get_joint_angle_vel_limits`, saved in `piper-01.readout.json`),
the official URDF `external/agx_arm_ros/.../piper/urdf/piper_description.urdf` (all six joints identical since the vendor update of 2026-10-13). Joint 5 has room to ±70°, so the park pose is not a limit: the wrist drops
from 16.8° to its resting position at 22.7° when the motors go off.

## Decisions pending (nothing applied to the code base)

1. `speed_percent` for `backend:=real` is a launch argument (default 20); the value used for real runs is the user's to set. What 100 % corresponds to in rad/s is not documented.
2. Crash-protection level is 0 (no detection); changing it is a write to the arm, the user's decision.
3. Whether the arm has a Revo2 hand or another device on the bus (extra frames 0x1C0-0x1C3 are seen).
4. Joint torque rating: not reported by the firmware. The description uses the SDK's `move_mit` range (+-16 / +-13.012 N.m) as an upper bound; the real rating needs the vendor or a measurement.
