# Real Piper: what we know, what we measured, what is still open

Status date: 2026-10-12. Source: the vendor documentation read in full (pyAgxArm `piper_api.md`, `firmware_reference.md`,
AgxGripper API, `agx_arm_ros` README and node code, `Piper_sdk_ui`, `piper_sdk` changelog, the quick-start manual) plus the first
read-only and enable-only sessions on our own arm.

## Rules for using this page

- **Every number measured on this arm is provisional.** The arm runs old firmware (S-V1.7-3) that has not been upgraded or fully tested.
  Do **not** change simulator, MoveIt, controller or guard parameters from the values below. Re-read everything after the firmware upgrade
  and decide then (section 9).
- The documentation knowledge (sections 4-7) is independent of this arm and can be relied on, with the firmware gates noted.
- Real-arm motion still needs the user present and approving each step (see `AGENTS.md`). Nothing here moved the arm.

## 1. Setup used

- USB-CAN adapter: gs_usb (`1d50:606f`, "Geschwister Schneider CAN adapter"), appears as `can0` once the kernel driver loads.
- Bring-up (needs sudo, done by the user): `sudo ip link set can0 up type can bitrate 1000000` (1 Mbit/s).
- Listen only: `candump can0` (nothing is transmitted).
- SDK: pyAgxArm 1.0.0 (in `.venv`), profile `PiperFW.DEFAULT` for firmware <= S-V1.8-2.
- Only one program may talk to the bus at a time (a second SDK process, or the vendor GUI, both transmit and both read).

## 2. Arm identity and first baseline (provisional, see rule above)

| Item | Value |
|---|---|
| Firmware (software) | **S-V1.7-3** |
| Hardware | H-V1.2-1, motor ratio/batch 10, node ARM_MC, node number 15 |
| Production date | 2025-08-09 |
| Gripper | AgileX gripper, stored max range 0.07 m (70 mm stroke), teaching range 100, teaching friction 1 |
| State at rest | standby, status normal, error 0, mode MOVE_P, teach off, all joints disabled |
| Drivers | 23-24 V, driver 27-29 C, motor 23-24 C, no fault bits, no bus current |
| Rest pose (deg) | 3.06, -1.85, 2.32, -4.69, 16.11, 0.48 (j2 and j3 sit about 2 deg outside their stored ranges: possible zero offsets) |
| Gripper reading | -4.27 mm on frame 0x2A8, outside 0-70 mm: zero offset suspected |
| Stored limits | joints 1-5 angle limits equal the official URDF; joint 6 stores +-180 deg (URDF +-120); speed 2.5 rad/s and acceleration 3.5 rad/s^2 per joint; flange 0.75 m/s, 0.18 rad/s, 1.0 m/s^2, 0.25 rad/s^2 |
| Crash protection | rating 1 on every joint (scale 0 = off ... 8 = most sensitive) |
| Assistance rating | no answer (needs firmware >= S-V1.8-1) |
| Enable test | all six joints enable, hold 2.5 s, max drift 0.08 deg, currents < 0.15 A, disable clean |

Raw outputs: `baseline_S-V1.7-3/` (probe, limits, extra reads, enable test).

## 3. What we saw on the bus (arm to host, listen only, 4 s)

| CAN id | Content | Rate seen |
|---|---|---|
| 0x2A1 | arm status: ctrl mode, arm status, mode feedback, teach state, motion status, trajectory number, error code | 200 Hz |
| 0x2A2-0x2A4 | flange pose | 200 Hz |
| 0x2A5-0x2A7 | joint angles, 0.001 deg units, two joints per frame | 200 Hz |
| 0x2A8 | gripper: width int32 (x1e-6 m), force int16 (x1e-3 N), status byte, mode byte | 200 Hz |
| 0x251-0x256 | per-motor speed (0.001 rad/s), current (0.001 A), position | 200 Hz |
| 0x261-0x266 | per-driver voltage (0.1 V), driver temp, motor temp, status bits, bus current | 40 Hz |

Push feedback is already on (no web/GUI toggle needed). No host-to-arm frames were present during the capture. A gripper control frame
(0x159, 1.0 N, status code 1) appeared later, during a session where the vendor GUI may still have been connected; its source is not confirmed.

## 4. Firmware gates (from the vendor firmware reference)

| Firmware | SDK profile | What changes |
|---|---|---|
| <= S-V1.8-2 | `default` | baseline; **our arm (S-V1.7-3)** |
| S-V1.8-3 .. 1.8-7 | `v183` | MIT torque encoding |
| >= S-V1.8-5 | (any) | seamless mode switching (see 5) |
| S-V1.8-8 | `v188` | `get_ik_joint_angles` (after `move_p` only), MIT 12-bit torque without CRC, MIT mode code 0x06, status message types change |
| >= S-V1.8-9 | `v189` | only `piper_x` MIT joint 4/5 sign fix |
| >= S-V1.8-1 | - | joint assistance rating query |
| >= S-V1.7-4 | - | firmware return-to-zero (0x191, the vendor node's `move_home`) |
| < S-V1.6-3 | - | older DH parameters (use the old URDF); we are past this |

The SDK profile must match the arm: a wrong profile mis-decodes MIT and status frames. `get_firmware()` plus `resolve_firmware_profile()` picks it.

## 5. Control modes and what they imply

| Mode | Behaviour | Our stance |
|---|---|---|
| `move_j` (+ `set_speed_percent`) | position-velocity mode, the firmware smooths and interpolates; each command overwrites the previous target, so it can stream | the path we use; safe default |
| `move_p` / `move_l` / `move_c` | firmware Cartesian point / line / arc to a flange pose; `move_l` re-plans on every point, so it cannot stream | not used (MoveIt does Cartesian) |
| `move_js` | follower / MIT pass-through, no smoothing or planning, lower stiffness, "extremely high risk" | not used on this firmware |
| `move_mit` | single-joint MIT: T = kp(p_des-p) + kd(v_des-v) + t_ff; start with small gains; torque encoding depends on firmware | not used |
| CPV | per-joint position/velocity commands and drive gains; `set_cpv_*` writes motor flash (limited write life); the SDK flips signs on some joints | not used |

Mode switching: below S-V1.8-5 there is no seamless switching. After `move_js` / MIT, going back to `move_j` needs `reset()`, which powers the arm
off and drops it if raised. From S-V1.8-5 the firmware switches by itself.

The vendor ROS node `agx_arm_ctrl` (parameters `fast_mode`, `speed_percent`, `auto_enable`, `tcp_offset`, `gripper_default_effort`, `fw_version`):
- `fast_mode` routes `control/joint_states` through `move_js`.
- Its `emergency_stop` service, on firmware without seamless switching, calls `move_js(current angles)`, which leaves the arm in follower mode. Do not use it
  on S-V1.7-3. **This arm has no physical e-stop button**: the hard stop is cutting power (drops a raised arm, so keep motion tests near the folded rest pose); the protocol also has an electronic emergency stop (CAN 0x150, SDK `electronic_emergency_stop()`).
- `speed_percent` default 0 means "do not set"; the SDK function default is 100. **Our launch does not set it.** Set it explicitly low before any motion.

## 6. Safety semantics from the docs

- `disable()` and `reset()` drop a raised arm immediately. `electronic_emergency_stop()` lowers it slowly with constant damping, then `reset()` is needed. Only enable/disable with the arm resting folded.
- SDK software joint-limit clamping is **off** by default (`get_joint_limits_enabled()`); the `joint_limits` config argument is not applied to control. Our `command_guard` and the arm's stored limits are the protection for `move_j`.
- Crash-protection rating 0-8 (0 = none), assistance rating 0-10.
- MIT and `move_js` can cause impact and oscillation; there is no e-stop button on this arm, so these stay off.

## 7. API catalogue (reads are safe, writes are not)

- **Reads used so far** (queries only, no state change): `get_firmware`, `get_arm_status`, `get_joint_angles`, motor/driver states, enable flags, `get_tcp_pose`, `get_joint_angle_vel_limits`, `get_joint_acc_limits`, `get_flange_vel_acc_limits`, `get_crash_protection_rating`, gripper status / control echo / range parameters, `get_joint_assistance_rating`. The enquiry frame is CAN 0x477 with every setting field 0, which cannot write.
- **Writes that exist, none used on the real arm yet**: `enable`/`disable`, `reset`, all `move_*`, `set_speed_percent`, `set_motion_mode`, `set_payload`, `set_installation_pos` (no read-back for the last two), `set_*_limits`, `set_joint_angle_vel_acc_limits_to_default` (the GUI's "Config Init"), `set_crash_protection_rating`, `set_joint_assistance_rating`, `calibrate_joint` (disable the joint, move it to zero by hand), `calibrate_gripper`, `set_gripper_teaching_pendant_param`, leader/follower and teach-mode calls, `clear_joint_error`.
- Windows needs the `python-can-agx-cando` plugin (`interface="agx_cando"`); macOS uses `slcan`. CAN frames 0x481-0x486 are deprecated (all zeros).

## 8. Tools and where firmware/software come from

- **Our tools** (all in `tools/`, read-only unless stated): `arm_probe.py` (firmware, status, joints, drivers, TCP; `--limits` adds stored limits), `arm_probe_extra.py` (gripper, crash/assistance ratings), `arm_enable_test.py` (**enables and disables the motors, sends no motion**; user present, arm folded).
- **Vendor GUI** `Piper_sdk_ui` (Linux, PyQt5; last commit 2025-07-10, older than the SDK): asks for the sudo password and pipes it through `sudo -S` for its CAN buttons. Its Reset, Go Zero, Joint Ctrl (Center/Random/Set Zero), Config Init, Gripper Zero, Master/Slave, Installation Position and Gripper Stroke buttons move the arm or write parameters. No firmware upgrade function is listed.
- **Windows host software** ArmRobotUA (Windows 7+): Google Drive link in the quick-start manual (an older manual copy has a Baidu link); the text we could extract lists no firmware flashing.
- **Firmware**: there is no public download. The maintainer says to request it through the pyAgxArm README or `support@agilex.ai` (the README lists that address for "firmware, host software, tool packages, after-sales"; Discord is also listed). The thread says S-V1.8-9 is the fix for old-firmware bugs (S-V1.8-2: corrupted limits after standalone use, flash data corruption). pyAgxArm issue #70 asks for an official recovery procedure and was unanswered when read.

## 9. Open items (decide after the firmware upgrade, not before)

1. Gripper status read returned `None` in one run although frame 0x2A8 is on the bus; the arm status line also did not print in that run. Likely a read-too-early timing issue in `arm_probe_extra.py` (now waits up to 3 s); rerun to confirm.
2. Source of the 0x159 gripper control frame (vendor GUI left open?). Confirm with a listen-only capture with every other program closed.
3. j2/j3 and gripper zero offsets: understand before any calibration write; `calibrate_*` are user decisions.
4. Real-arm limit profile for MoveIt / guard (speed <= the arm's stored speed, acceleration <= its stored acceleration): **not applied**, the stored values may change with firmware.
5. Explicit `speed_percent` for `backend:=real` (10-20 for first motion): not applied.
6. Whether the firmware flash needs the Windows tool or the SDK; request release notes and the flashing/recovery procedure from AgileX support in writing before flashing.
7. After the upgrade: rerun `arm_probe.py --limits`, `arm_probe_extra.py`, `arm_enable_test.py`, diff against `baseline_S-V1.7-3/`, then plan supervised motion (smallest `move_j` first) through `piper activate` with the guard.

## 10. Teach (drag) mode and its light (vendor quick-start manual, section 2.2)

The button light between joints 5 and 6 shows the drag-teach state: **off** = stopped (normal); **solid green** = recording a trajectory
(drag the arm by hand); **flashing green (500 ms)** = **playing back** the last recorded trajectory (the arm moves by itself; stay clear).
Single click = start/stop recording; double click = play back the last recording. A new recording erases the previous one; max 3 minutes.
Before using host/CAN control the light must be off; switch to standby from the host, then to CAN control mode, and when coming back from
teach or link mode the arm must first sit at the zero pose. The protocol has teach controls on CAN 0x150 byte 2 (1 record, 2 end record,
3 execute, 4 pause, 5 resume, 6 terminate, 7 move to the start point), plus an electronic emergency stop on byte 0 (1 activate, 2 resume).
The manual does not say how to stop a playback with the button. Session note 2026-10-12: after the first sessions the arm showed flashing green
lights and sent no CAN frames; the vendor GUI could still enable/disable but not reach the zero pose. Cause not established yet.
