# piper-01: the arm on the `atlas` workstation

`piper-01` is a placeholder label: rename it when you choose a real one. Raw read-out: `piper-01.readout.json` (`tools/arm_info.py`, 2026-10-14, nothing moved).

| Item | Value |
|---|---|
| Label serial | **to record by hand from the arm's label** |
| Model / gripper | Piper standard / AgileX gripper, stored stroke 70 mm (teaching range 100, friction 1) |
| Mounting | not recorded |
| USB-CAN adapter | candleLight (gs_usb), serial `002000295631511920313857`, appears as `can0` (1 Mbit/s) |
| Firmware / hardware | **S-V1.9-1** / H-V1.2-1, motor ratio/batch 10, node ARM_MC #15, date field 261014 (reads the day of the upgrade), unparsed 4-char field `30AH` |
| SDK profile | `v189` |

## Current state (2026-10-14, after upgrade and calibration)

- Role slave/feedback, CAN control, MOVE_J, teach off, error 0, all joints disabled, `motion_status` = "reach target failed" (0x1) with no error code (leftover flag).
- Rest pose (deg) 0.10, -2.25, 2.21, -0.43, 22.72, 0.00; gripper width 0.14 mm. Drivers 23-24 V, 31-34 C, no fault bits.
- Stored limits: joints 1-5 angle limits equal the official URDF; joint 6 stores +-170 deg (URDF +-120); speed 3.0 rad/s, acceleration 5.0 rad/s^2 per joint;
  flange 1.0 m/s, 0.25 rad/s, 1.5 m/s^2, 0.3 rad/s^2.
- Crash protection **0 on every joint (no collision detection)**; assistance rating 0 on every joint.
- Bus: status/pose/joints/gripper 200 Hz, motor 100 Hz, driver 20 Hz; extra frames 0x1C0-0x1C3 (20-40 Hz; the SDK's Revo2 hand ids) and 0x212 (1 Hz, undocumented).
  Whether a Revo2 hand is attached or the firmware emits them is not established.
- Joints 2 and 3 rest about -2 deg and +2 deg, outside their stored ranges, also after calibration. The vendor `piper_ros` README says the DH parameters after S-V1.6-3 offset those
  two joint frames by 2 degrees, so this is plausibly by design (not confirmed with AgileX).
- Enable test on S-V1.9-1 (2026-10-14): all six joints enabled, max drift 0.08 deg over 2.5 s, currents <= 0.08 A, status normal, clean disable. No motion has been commanded on the real arm yet.
- **First motion (2026-10-14, `tools/arm_zero.py`, 50 % speed):** from the resting pose (0.1, -2.2, 2.3, 0.0, 22.0, 0.0 deg) to the zero pose in 0.6 s (joint 5 moved 22 deg), final error 0.06 deg,
  final pose 0.0, +0.48, -0.44, 0.0, 0.0, 0.0 deg (joints 2/3 are aimed 0.5 deg inside their stored limit), no fault, clean disable. The path used was the SDK directly: CAN control + MOVE_J read back, speed 50 %, one `move_j`.
  Our ROS stack (`backend:=real`, command_guard, controllers) has not driven the arm yet.

- CPV drive parameters (read 2026-10-14 with `arm_info.py -c`): acceleration 5.0 and deceleration 5.0 rad/s^2 on every joint, profile velocity 0, position gain 5.0, velocity-loop kp 0.7 (joints 1-3) / 0.8 (joints 4-6), ki 40 (joints 1-3) / 80 (joints 4-6). Reading them put the arm in MOVE_CPV motion mode
  (the first `-c` version did not restore it; the fixed tool sets MOVE_J on the next `-c`, and any `move_j` switches it back).

## Park pose (recorded by hand 2026-10-14)

Pose recorded with the motors off, wrist a few mm above the support brackets: joints (deg) 0.23, -2.19, 2.45, -0.42, 16.82, 0.0 (`arm_poses.log`). Against zero only joint 5 is lowered
(16.82 deg); joints 2/3 are at the stops (-2.19 / +2.45), which are outside the stored limits (j2 >= 0, j3 <= 0), so the pose cannot be commanded as recorded.
Flange height from the SDK's forward kinematics: resting on the stops (j5 22 deg) 167.0 mm, recorded pose 174.3 mm, the same pose with joints 2/3 clamped inside their limits 189.2 mm, all joints 0: 213.3 mm.
The SRDF `rest` therefore is zero with joint 5 = 0.2936 rad (16.82 deg), about 22 mm above resting; disabling there drops the wrist a little more than the recorded pose would.
To check on the arm: whether the firmware accepts a slightly negative joint 2 / positive joint 3 target (then `rest` could use the recorded values and the guard margin could shrink).

## Decisions pending (nothing applied to the code base)

1. MoveIt/controller max joint speed is 5 rad/s, the arm's limit is 3.0; acceleration matches (5.0).
2. `speed_percent` for `backend:=real` is now a launch argument (default 20); the value used for real runs is yours to set.
3. Crash-protection level (a write to the arm, user's decision).
4. Whether the arm has a Revo2 hand or another device on the bus.

## History

- 2026-10-12: first sessions on S-V1.7-3 (hardware H-V1.2-1, date 250809): standby, no faults, speed 2.5 rad/s, accel 3.5, joint 6 +-180, crash protection 1, assistance rating unsupported,
  gripper -4.27 mm, rest pose 3.06 / -1.85 / 2.32 / -4.69 / 16.11 / 0.48 deg. Enable/disable test clean. The arm was later found in teach playback (flashing light) and the adapter dropped off USB.
- 2026-10-14: firmware upgraded to S-V1.9-1 and calibrated by the user (procedure and tool not recorded).
