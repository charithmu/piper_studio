# Real Piper: vendor knowledge and how we work with it

Arm-independent knowledge. Facts about one physical arm (firmware, limits, quirks, history) live in `arms/<arm-id>.md`; nothing in the code base is tied to
one arm. Source: the vendor documentation read in full (pyAgxArm `piper_api.md`, `firmware_reference.md`, AgxGripper API, `agx_arm_ros` README and node code,
`piper_sdk` changelog, the quick-start manual, `Piper_sdk_ui`), checked against sessions on a real arm.

Rules: anything that can move or write to a real arm needs the user present and approving that step (`AGENTS.md`). Read first, write only on request.
Never change simulator, MoveIt, controller or guard parameters from one arm's read-out without a decision recorded in the arm file.

## 1. Bring-up

- USB-CAN adapter: gs_usb ("candleLight / Geschwister Schneider", `1d50:606f`) shows up as `can0`. Bring-up needs sudo (the user runs it):
  `scripts/can_up.sh` (auto-detects the interface, 1 Mbit/s, `--dry-run` shows the commands; plain form: `sudo ip link set can0 up type can bitrate 1000000`). Listening (`candump can0`) sends nothing.
- The "CAN bus" of our tools is the name of that Linux CAN interface (`can0`), not something the arm reports. The tools auto-detect it (the only CAN bus, else the only one that is up); `-b/--canbus` names one.
- SDK: pyAgxArm (in `.venv`). Only one program may use the bus at a time (a second SDK process or the vendor GUI both transmit).
- No CAN interface means the adapter is not seen (check `lsusb`); an interface with a flat RX counter means the arm is silent (power, plug, teach playback).
- Windows: `python-can-agx-cando` plugin (`interface="agx_cando"`); macOS: `slcan`.

## 2. Bus map (arm to host)

| CAN id | Content | Rate |
|---|---|---|
| 0x2A1 | status: ctrl mode, arm status, mode feedback, teach state, motion status, trajectory no., error code (+ per-joint limit/comm flags) | 200 Hz |
| 0x2A2-0x2A4 | flange pose | 200 Hz |
| 0x2A5-0x2A7 | joint angles, 0.001 deg, two joints per frame | 200 Hz |
| 0x2A8 | gripper: width int32 (1e-6 m), force int16 (1e-3 N), status, mode | 200 Hz |
| 0x251-0x256 | per-motor speed (0.001 rad/s), current (0.001 A), position | 100-200 Hz (firmware dependent) |
| 0x261-0x266 | per-driver voltage (0.1 V), temperatures, status bits, bus current | 20-40 Hz (firmware dependent) |
| 0x4AF | firmware info (11 frames, answer to our query) | on request |
| 0x155-0x157, 0x159, 0x151 | control frames: sent by an arm in **master/leader** mode (and by whatever commands the arm); a leader sends only these | |
| 0x1C0-0x1C3 | Revo2 dexterous-hand feedback ids in the SDK | seen on one arm, source unknown |

Slave (follower) is the normal role for us: it sends all feedback and accepts commands.

## 3. Firmware gates (vendor firmware reference)

| Firmware | SDK profile | What changes |
|---|---|---|
| <= S-V1.8-2 | `default` | baseline |
| S-V1.8-3 .. 1.8-7 | `v183` | MIT torque encoding |
| >= S-V1.8-5 | (any) | seamless mode switching (section 4) |
| S-V1.8-8 | `v188` | `get_ik_joint_angles` (after `move_p`), MIT 12-bit torque without CRC, MIT mode code 0x06, status message types change |
| >= S-V1.8-9 | `v189` | `piper_x` only: MIT joint 4/5 sign fix |
| >= S-V1.8-1 | - | joint assistance rating query |
| >= S-V1.7-4 | - | firmware return-to-zero (0x191, the vendor node's `move_home`) |
| < S-V1.6-3 | - | older DH parameters (use the old URDF) |

The SDK profile must match the arm or status/MIT frames mis-decode. `tools/real_arm/arm_common.open_arm()` reads the version and picks it (`resolve_firmware_profile`).

## 4. Control modes

| Mode | Behaviour | Our stance |
|---|---|---|
| `move_j` (+ `set_speed_percent`) | position-velocity mode, firmware smooths and interpolates; each command overwrites the target, so it can stream | the path we use |
| `move_p` / `move_l` / `move_c` | firmware Cartesian point / line / arc; `move_l` re-plans on every point, so it cannot stream | not used (MoveIt does Cartesian) |
| `move_js` | follower / MIT pass-through, no smoothing, lower stiffness, "extremely high risk" | not used |
| `move_mit` | single-joint MIT: T = kp(p_des-p) + kd(v_des-v) + t_ff; small gains first; torque encoding depends on firmware | not used |
| CPV | per-joint position/velocity commands and drive gains; `set_cpv_*` writes motor flash (limited life); the SDK flips signs on some joints. The SDK's `get_cpv_*` reads first switch the motion mode to CPV | not used |

Mode switching: below S-V1.8-5 there is no seamless switching. After `move_js` / MIT, returning to `move_j` needs `reset()` (powers the arm off, drops it if raised).

Vendor ROS node `agx_arm_ctrl`: `fast_mode` routes `control/joint_states` through `move_js`; `speed_percent` default 0 means "do not set" (the SDK function default is
100; **our launch does not set it**, so set it explicitly low before motion); on firmware without seamless switching its `emergency_stop` service calls
`move_js(current angles)` and leaves the arm in follower mode (do not use it there); `fw_version` auto-detects.

## 5. Safety semantics

- `disable()` and `reset()` drop a raised arm immediately. `electronic_emergency_stop()` (CAN 0x150 byte 0) lowers it slowly with constant damping; `reset()` is needed afterwards.
- A Piper has no emergency-stop button (the host software shows an on-screen one). The hard stop is cutting power, which drops a raised arm, so keep tests near the folded rest pose
  and keep a hand on the power switch or plug.
- SDK joint-limit clamping (`set_joint_limits_enabled(True)`) clamps every command into the model limits. A Piper rests with joints 2 and 3 about 2 deg outside their stored range, so a "hold" command built from the current pose is clamped and moves the arm; do not turn it on for commands derived from the current pose.
- SDK software joint-limit clamping is **off** by default (`get_joint_limits_enabled()`), and the `joint_limits` config argument is not applied to control. Only our `command_guard`
  and the arm's stored limits protect `move_j`.
- Crash protection rating 0-8 (0 = no detection), assistance rating 0-10; both are stored in the arm and can change with firmware.
- MIT and `move_js` can cause impact and oscillation.

## 6. API catalogue

- **Reads (queries only):** `get_firmware`, `get_arm_status`, `get_joint_angles`, motor/driver states, enable flags, flange/TCP pose, `get_joint_angle_vel_limits`,
  `get_joint_acc_limits`, `get_flange_vel_acc_limits`, crash/assistance ratings, gripper status / control echo / range parameters. The enquiry frame is CAN 0x477 with every
  setting field 0, which cannot write.
- **Writes, none used by our tools except enable/disable/move_j:** `enable`/`disable`, `reset`, all `move_*`, `set_speed_percent`, `set_motion_mode`, `set_payload` and
  `set_installation_pos` (no read-back), `set_*_limits`, `set_joint_angle_vel_acc_limits_to_default` (the GUI's "Config Init"), `set_crash_protection_rating`,
  `set_joint_assistance_rating`, `calibrate_joint` (disable the joint, move it to zero by hand), `calibrate_gripper`, `set_gripper_teaching_pendant_param`,
  leader/follower and teach calls, `clear_joint_error`.
- CAN frames 0x481-0x486 are deprecated (all zeros).

## 6b. Named poses and the routine

Words: `zero` (the one valid start and end pose: all joints 0, joints 2/3 held 0.01 rad inside their limits), `park` (zero with joint 5 lowered 0.3840 rad (22 deg), Piper Studio internal) and the
slumped pose (what a powered-off arm sags into: joints 2/3 about 0.04 rad outside their limits). Definitions and reasons: `arms/piper-01.md`, "Poses and the start / stop sequence".
Routine on the real arm: `piper start` (enable, leave the slumped pose by one checked move to zero, activate), work inside the limits, return to `zero`, `piper stop` (zero, wait, park, wait, motors off;
someone at the arm). `tools/real_arm/arm_zero.py [--park]` is the same sequence at SDK level. The guard rejects every command while the arm is outside its limits and every target outside them.

## 6c. Joint limits and the zero point

Limits come from four places that must agree: the arm's stored limits (`get_joint_angle_vel_limits`), the official URDF (`piper_description.urdf`, used by MoveIt and all simulators), the SDK's model preset
(`pyAgxArm/api/constants.py`, used only if `set_joint_limits_enabled(True)`) and our `command_guard` (URDF minus 0.01 rad). Piper values (deg): j1 ±150, j2 0..180, j3 -170..0, j4 ±100, j5 ±70 in all three;
j6 is ±170 in the arm (S-V1.9-1), the vendor URDF (since 2026-10-13) and the SDK preset. Speed: the arm stores 3.0 rad/s, which we use everywhere (the vendor URDF says 5; see `config/firmware_values.yaml`).

The zero point lives in the joint motors, not on the host. `calibrate_joint(n)` (CAN 0x475 byte 1 = 0xAE) stores the joint's present angle as its zero; for one joint the joint must be disabled and moved by hand to its
hardware zero mark first (index 255 = all joints "with offset", the SDK docstring refers to an official tutorial for the pre-calibration poses, not found in the sources we hold). The GUI `Piper_sdk_ui` has Set Zero /
All Set Zero buttons (commented out in its code), the Windows app ArmRobotUA does the same. Since S-V1.6-3 the zero of joints 2 and 3 is offset 2 deg from the stops, which is why they rest 2 deg past their stored range.
Calibration is a write that changes where every pose is: do it only deliberately, then re-read `arm_info.py` and the arm's park pose.

## 6d. Vendor ROS driver `agx_arm_ctrl` (agx_arm_ros 286633e, read 2026-11-08)

Topics (namespace as launched). Feedback: `feedback/joint_states` (200 Hz; position, **velocity** and **effort** = motor torque estimate in N.m, current x k x b x c, per joint plus `gripper`),
`feedback/tcp_pose`, and three **diagnostics** at 2 Hz (`diagnostic_msgs/DiagnosticArray`, values are strings): `feedback/arm_status` (keys `ctrl_mode`, `arm_status`, `mode_feedback`, `teach_status`,
`motion_status`, `trajectory_num`, `err_status`, `joint_angle_limit[]`, `communication_status_joint[]`; level ERROR for arm_status 1-10, 14, 15 or a nonzero error code), `feedback/motor_status` (one entry per joint:
`enabled`, `bus_voltage_v`, `bus_current`, `driver_temp_deg`, `motor_temp_deg`, `status_raw`, fault flags `undervoltage`, `motor_overtemp`, `driver_overcurrent`, `driver_overtemp`, `collision_tripped`,
`driver_error`, `stall_tripped`; STALE when feedback is older than 2 s) and `feedback/gripper_status`. These replaced the custom `AgxArmStatus`/`GripperStatus`/`HandStatus` messages (removed upstream 2026-10-21).
`piper_py` reads them: `Piper.status()`, `Piper.faults()`, `piper status`; `activate()` refuses while one is reported (tested with the fake driver, not yet on the arm).
Codes: arm_status 0 normal, 1 e-stop, 2 no solution, 3 singularity, **4 target angle out of range**, 5 joint communication, 6 brake not released, 7 collision, 8 teach drag overspeed, 9 joint status, 10 other, 11-13 teaching, 14-15 overtemperature.
Services: `enable_agx_arm` (SetBool), **`control_enable`** (SetBool: an external gate; closed, the driver ignores all control topics without disabling the motors, a safe way to hold off commands), `move_home` (move_j to zeros),
`emergency_stop` (holds the current pose with `move_j`, or `move_js` on firmware without seamless switching; not a power cut), `exit_teach_mode` (only offered on firmware without seamless switching, so not on this arm).
Commands: `control/joint_states`, `move_j`, `move_js`, `move_p`, `move_l`, `move_c`, `move_cpv`, `move_mit` (not used by us). Parameters: `auto_enable` (default true: our launch turns it off), `speed_percent` (0 = do not set), `fast_mode`, `fw_version`, `tcp_offset`, `gripper_default_effort`, `control_enabled`.
The driver ignores control commands until a valid joint stream exists. Commented-out code only hides an unused `feedback/flange_pose` publisher; nothing else useful was found in comments.

## 7. Teach (drag) mode

The button light between joints 5 and 6: **off** = stopped (normal); **solid green** = recording (drag the arm by hand); **flashing green** = **playing back** the last recording
(the arm moves by itself, stay clear). Single click starts/stops recording, double click plays back; a new recording erases the previous one; max 3 minutes.
Before host/CAN control the light must be off; switch to standby from the host, then to CAN control, and when coming back from teach or link mode the arm must first sit at
the zero pose. Protocol: teach controls on CAN 0x150 byte 2 (1 record, 2 end, 3 execute, 4 pause, 5 resume, 6 terminate, 7 move to start). The manual does not say how to
stop a playback with the button.

Master/leader mode (`set_leader_mode` / GUI "Master"): zero-force drag, the motors do not hold, only control frames are sent, and returning to position control needs a `reset()`
with the arm at the zero pose. We do not use it.

## 8. Our tools (`tools/`, all use `arm_common.py`)

| Tool | What it does | Sends |
|---|---|---|
| `arm_info.py [-r] [-c] [-b CANBUS] [--arm-id ID]` | coloured summary (default; top rows: role master/slave, motion mode, motors enabled/disabled, software e-stop, teach mode, faults) or the long raw form (`-r`); fixed 1.5 s passive listen first (stops if the arm is silent), then firmware, status, joints, drivers, limits, ratings, gripper; always saves JSON; `-c` also reads CPV parameters (switches the arm to CPV mode, then restores the previous mode and checks it) | the firmware query and enquiry frames |
| `arm_enable_test.py` | enable, hold a few seconds, disable; no motion command; aborts on drift | enable/disable |
| `arm_zero.py [--dry-run] [--speed 50] [--exact] [--park] [--stay]` | preflight (slave, status normal, teach off, limits readable), known state (speed first, CAN control + MOVE_J read back), enable, one `move_j` to the zero pose at the chosen speed (default 50 %, no time limit), stops only on a real fault (status, error code, driver bits) and then holds; asks before disabling. Replaces `arm_move_rest.py` | enable, mode frames, move_j, disable |
| `arm_session.py status\|start\|stop [--speed 10]` | the session routine. `start`: preflight, operator Enter, speed + CAN control + MOVE_J read back, enable, one move to zero (the way out of the slumped pose). `stop`: zero, park, operator Enter, e-stop (damped lowering), operator Enter, reset (`--plain-disable`: disable instead). Run on the arm 2026-11-10 (start, stop with e-stop + reset) | e-stop, reset, mode frames, move_j |
| `arm_pose.py zero\|park\|ready [--speed 10] [--dry-run]` | move an already ENABLED, valid arm between the hard-coded poses (table `POSES`, equal to the SRDF, tested); never enables, disables or leaves the slumped pose; holds on a fault. Run on the arm 2026-11-10 (zero, park, ready) | mode frames, move_j |
| `arm_jog.py [--joint 5] [--speed 3] [--step 0.5]` | one joint in typed steps (1 cm park search 2026-11-10); never disables | enable, mode frames, move_j |
| `arm_stop.py [--disable \| --reset]` | EMERGENCY STOP, no prompts: electronic e-stop by default (damped lowering), `--disable` plain motors off, `--reset` e-stop + reset (clears the e-stop, motors off, arm stays powered). Drilled 2026-11-10 | e-stop / disable / reset |
| **`piper-arm`** (`piper_core.arm_cli`) | **the cockpit: one terminal console with live status and one key per action** (start, zero, park, ready, jog, end session, disable, clear e-stop, speed; gripper `o` open, `l` close, `g` 5 mm steps, `f` force 0.5-3 N). Every motion waits for Enter; `e` = emergency stop and SPACE = hold work at once, also mid-move. Built on `RealArm` reads + the same SDK moves as the tools below. Installed by `scripts/bootstrap.sh` (`uv pip install -e src/piper_core` into `.venv`); `source scripts/env.sh` then `piper-arm`. Tested against a fake SDK and a pty; **first run on the arm (arm keys) done 2026-11-10; the gripper keys are still to do on the arm** | queries, mode frames, enable, move_j, e-stop, reset |
| `arm_joints.py [--rad] [--once] [-b CANBUS]` | live joint table (j1-j6 and gripper) updated in place; passive (listens to 0x2A5-0x2A8, sends nothing, can run beside any other program); Enter saves the pose as a copy-paste line (radians) and appends it to `~/data/ml/isaac/piper_studio/arm_poses.log`; q quits | nothing |

Run each after any firmware change; diff the saved `arm_info` JSON against the arm's file in `arms/`.

## 9. Vendor GUI, host software and firmware

- `Piper_sdk_ui` (Linux, PyQt5; last commit 2025-07-10, older than the SDK) asks for the sudo password and pipes it through `sudo -S` for its CAN buttons. Its Reset, Go Zero,
  Joint Ctrl, Config Init, Gripper Zero, Master/Slave, Installation Position and Gripper Stroke buttons move the arm or write parameters. Its CAN port rename can change `can0`.
- Windows host software ArmRobotUA: Google Drive link in the quick-start manual (an older copy has a Baidu link).
- Firmware has no public download: request it through the pyAgxArm README / `support@agilex.ai` (also listed: Discord). Ask for release notes, the flashing procedure and the
  recovery path in writing; pyAgxArm issue #70 asks for an official recovery procedure.

## 10. Identifying a physical arm

The arm does not report a serial number over CAN. `get_firmware()` returns hardware version, software version, production date, motor ratio/batch, node type and node number,
which describe the model and batch, not one unit (the raw 88-byte block also holds an unparsed 4-character field). So identify an arm by a label you choose (e.g. `piper-01`),
the serial printed on the arm's label (record it by hand), and the read-out fingerprint. The USB-CAN adapter has its own serial (`lsusb -v`), which identifies the adapter, not the arm.
To register an arm: copy `arms/_template.md` to `arms/<arm-id>.md`, run `tools/real_arm/arm_info.py --arm-id <arm-id>` and fill the table from the JSON.

## 11. Gripper (RealArm, piper-arm)

`RealArm.set_gripper(width_m, effort)` blocks until the gripper reached the width, stopped short of it ("stalled": an object or the end stop) or timed out, and returns the measured width; `arm.last_gripper` holds
the reason, width and force. `gripper_command` is the non-blocking, guarded send; `gripper_opening()` and `gripper_force()` read CAN 0x2A8; `state()` carries `gripper_width` and `gripper_force`.
Guard: width in [0, 0.07 m] (firmware stroke), effort in (0, `gripper_max_effort`], nothing while a fault or e-stop is reported. Default effort 1.0 N (the vendor's `gripper_default_effort`); our cap is 3.0 N, not a vendor figure
(none is documented): raise it deliberately with `arm.gripper_max_effort = 5.0`. The SDK call is `move_gripper_m(value, force)` (1e-6 m, 1e-3 N). A close that ends "stalled" at a width above the empty-closed width is how a held object shows.
Not yet run on the real arm (2026-11-10): stroke, closing time, width and force with objects, whether a command works with the arm motors off, whether the arm's e-stop also stops the gripper.
