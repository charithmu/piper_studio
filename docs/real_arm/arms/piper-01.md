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
- Enable test (old firmware, same arm): all six joints enabled and held within 0.08 deg. Not repeated on S-V1.9-1 yet. No motion has been commanded on the real arm.

## Decisions pending (nothing applied to the code base)

1. MoveIt/controller max joint speed is 5 rad/s, the arm's limit is 3.0; acceleration matches (5.0).
2. Explicit `speed_percent` for `backend:=real` (10-20 % for the first moves).
3. Crash-protection level (a write to the arm, user's decision).
4. Whether the arm has a Revo2 hand or another device on the bus.

## History

- 2026-10-12: first sessions on S-V1.7-3 (hardware H-V1.2-1, date 250809): standby, no faults, speed 2.5 rad/s, accel 3.5, joint 6 +-180, crash protection 1, assistance rating unsupported,
  gripper -4.27 mm, rest pose 3.06 / -1.85 / 2.32 / -4.69 / 16.11 / 0.48 deg. Enable/disable test clean. The arm was later found in teach playback (flashing light) and the adapter dropped off USB.
- 2026-10-14: firmware upgraded to S-V1.9-1 and calibrated by the user (procedure and tool not recorded).
