# Real-arm checklist: the ladder, one rung at a time

Rules for every rung: **the user is at the arm, hand on the power switch, approving each step.** Nothing starts from a background job or a test run. Speed starts at 10 % and is raised one step at a time
(10 -> 20 -> 30 -> 50, the user's ceiling for this task). One program on the CAN bus at a time. `e` in `piper-arm` is the emergency stop (electronic e-stop); Space holds; the power switch is the hard stop.
Record results in `arms/piper-01.md` (hardware log) and the report `docs/real_arm/REPORT.md` (numbers, method, confidence). Do not change guard limits, firmware values or calibration from one read-out.

Tools: `piper-arm` (cockpit: status, keys, gripper), `tools/real_arm/arm_session.py`, `arm_pose.py`, `arm_jog.py`, `arm_stream_test.py` (see `REAL_ARM.md` section 8). Start of day: `scripts/can_up.sh` (sudo, once per boot), `source scripts/env.sh`.

## R0 read-only (done 2026-11-10)

- [x] `scripts/can_up.sh`; `arm_joints.py` shows frames; `arm_info.py --arm-id piper-01` equals `piper-01.readout.json` (firmware S-V1.9-1, slave, MOVE_J, teach off, no faults, limits as stored).
- [x] State at rest recorded (powered-off pose, gripper 0 mm, drivers 26-28 C).
- [ ] Gripper range on the arm: open stroke and empty-closed width (R2 below measures both).

## R1 trajectories through RealArm

Status 2026-11-10: R1.5 passed (the cuRobo scenario, all steps ok, 0.33 deg worst joint error, back at zero); R1.00 passed; R1.1 and R1.2 done on joint 1 at 6 % (`lookahead=2`: lag 0 ms, rms 0.04 deg; `fixed` and `per_point`: lag 45 ms, rms 0.25 deg; numbers in `arms/piper-01.md`); R1.6 done at zero and `ready`. R1.3 done at low speed on joints 2, 3, 5; R1.4 done on joint 1 up to 44 %; R2.0-R2.4 done (numbers in `REPORT.md`; the gripper does not move with the motors off and the e-stop does not release a grip). R1.7 (dead-process drill) done, user reported all working (no figures). The post-review regression passed (user report). The ROS rungs are parked.

Pre-conditions: `piper-arm` `s` (enabled, at zero), no object near. Each step is approved by the user, then run by the user.

| Step | Command | Record | Pass |
|---|---|---|---|
| R1.00 | `python tools/real_arm/realarm_smoke.py` (stages 0-8, an Enter before each) | status, start, state, guard refusals, hold + 5 deg move, 10 deg cosine, gripper, abort, stop | every stage as printed; nothing moves in stage 3 |
| R1.0 | `python tools/real_arm/arm_stream_test.py --offline` | plan, speed needed | numbers printed |
| R1.1 | `arm_stream_test.py` (joint 1, 10 deg cosine, 4 s, 10 %) | lag, rms, final error, timing | final error < 0.5 deg, no guard event |
| R1.2 | same with `--mode per_point`, then `--lookahead 2`, `--repeat 3` | lag, rms, repeatability per mode | compare the three: smaller lag/rms at the same plan wins |
| R1.3 | joints 2, 3, 5 one at a time (`--joint 2 --amplitude 8`, ...) | per-joint lag, gravity effect (j2/j3/j5) | within 1 deg rms at 10 % |
| R1.4 | speed ladder 20 -> 30 -> 50 % on the best mode (`--period` shorter, `--max-percent` raised by the user) | `peak speed` vs planned vs nominal (3.0 rad/s x percent) | measured speed within 10 % of nominal, no fault |
| R1.5 | the cuRobo scenario on the arm: `python tools/real_arm/run_scenario.py tools/real_arm/fixtures/scenario_pick_25_0_05.json --backend real` (approach, grasp, close, lift, open, home; 0.3 rad/s; an Enter per step; clear the workspace under (0.25, 0, 0.05) m) | per step ok, tracking error, back at zero | all steps ok, end error < 0.5 deg, back at zero |
| R1.6 | `python tools/real_arm/fk_compare.py --arm` at zero, park and a pose from `arm_pose.py ready` | the arm-reported flange pose vs URDF vs SDK DH (offline: 0.09 mm over 300 poses) | within 1 mm |
| R1.7 | dead-process drill: in `realarm_smoke.py` after stage 1, kill the process (Ctrl-\\ or `kill` its pid) while the arm holds at zero | does the arm keep holding with the motors on? (expected: yes); then `piper-arm` `e` | the arm holds; the operator e-stops |

Measure per joint (from `stream_metrics`): tracking error, lag, overshoot, repeatability (`--repeat`), `speed_percent` vs real speed, behaviour at 25 ms setpoints (timing late/spacing). Stop and ask if any run ends with a guard event, a fault, or a final error above 1 deg.

## R2 gripper (nothing in the way; `tools/real_arm/gripper_test.py` does A-D with an Enter per stage and saves the numbers)

Pre-condition: `piper-arm`; the gripper panel reads width and force. Commands: `o` open, `l` close, `g` steps, `f` force (0.5-3 N). Keep fingers out of the closing path.

| Step | What | Record |
|---|---|---|
| R2.0 | empty: `o`, `l` x5 at 1.0 N | open stroke (mm), **empty-closed width** (mm, the value `piper_pick` uses to detect a miss), repeatability (std over 5), closing time (stopwatch or video until "reached") |
| R2.1 | closing time vs force: `l` at 0.5, 1, 1.5, 2, 3 N | time to reach, force read-back (sign negative while gripping) |
| R2.2 | objects: the 10 mm object (done: held at 9.4 mm, -2.05 N at 3 N), the white plastic box, foam brick / soap / banana if available | stalled width, force read-back, whether it holds |
| R2.3 | slip: hold the object at 1.0 N, then move the arm (zero <-> park, then `arm_pose.py ready`) at 10 % | slip yes/no, width change during the move |
| R2.4 | no-motor case: gripper `o` / `l` with the arm motors OFF; and `e` (e-stop) with the gripper closed on an object | does it move; does the e-stop release the grip |

Then compare with the simulated gripper (Isaac, MuJoCo): stroke, closing time, held width, force (the simulators have no force read-back and ignore effort) and write the gap table in the report.

## R3 the ROS path on the real arm: PARKED (experimental, 2026-11-10; first run in `arms/piper-01.md`; redesign CLEANUP C14). Kept for when it is picked up again

(`piper_real_driver`; one program on the bus: close `piper-arm` first)

Terminal 1: `ros2 launch piper_sim piper.launch.py backend:=real canbus:=can0 speed_percent:=10` (moves and enables nothing). Terminal 2 (`source scripts/session.sh 71`, a domain in 70-161): each line is a step the user approves.

| Step | Command | Pass |
|---|---|---|
| R3.0 | `piper status`, `piper state` | arm status normal, joints as `piper-arm` shows, no faults |
| R3.1 | `piper start` | motors on, at zero, controllers active; a second `piper start` is harmless |
| R3.2 | `piper joints 0.1 0.5 -0.5 0 0.3 0 --duration 4` then `piper zero` | reaches both within 0.5 deg; no guard rejection in terminal 1 |
| R3.3 | `piper named ready` (big move, clear the space), `piper zero` | reaches; MoveIt plan executed |
| R3.4 | `piper gripper 0.0`, `piper gripper 0.07`, with the 10 mm object `piper gripper 0.0` | GRASPED when it holds the object; width and force plausible |
| R3.5 | `piper stop` | zero, park, e-stop (damped lowering), clear; then `piper start` works at once |
| R3.6 | `piper estop` during a slow `piper joints ... --duration 8`; then `piper clear-estop`, `piper start` | the arm lowers under damping within about a second; recovery works |
| R3.7 | kill terminal 1 (Ctrl-C) while holding at zero | the arm keeps holding; `piper-arm` `e` stops it |
| R3.9 | the same scenario through ROS: `python tools/real_arm/run_scenario.py tools/real_arm/fixtures/scenario_pick_25_0_05.json --backend ros` (stack from terminal 1, speed_percent at least 15) | all steps ok, back at zero, gripper closes and opens |
| R3.8 | a small MoveIt plan with the camera/table scene, `piper pose ...` (tool-down reach limits in INTEGRATION.md) | executed, TCP within 5 mm of the request |
| R3.10 | the cuRobo scenario with the camera model, after R3.0-R3.5: `gpu-run <study venv>/bin/python tools/real_arm/curobo_plan_scenario.py --camera zedxm`, then `python tools/real_arm/run_scenario.py <scenario.json> --backend real --camera-mounted` (planned for the arm with the bracket and camera; refused without `--camera-mounted`; the robot-only scenario of R1.5 is `--camera none`, the default) | steps ok, worst joint error, back at zero, the wrist and camera clear of the table | as R1.5 (all steps ok, below 1 deg worst joint error, at zero); the model's bracket spheres cover only 70 % of the bracket (worst 10.7 mm outside, `curobo_piper_study` docs/04 1b): keep 2 cm more clearance than the plan to anything near the wrist. Simulated 2026-11-11: MuJoCo, all steps ok. |

## What must be true before `piper_pick` runs on the arm

R1.1-R1.3 passed at 10 %; R2.0 and R2.2 recorded; `docs/FINDINGS.md` read by the `piper_pick` agent; the user has decided the speed percent for pick-and-place.

## R3 wrist camera: the ZED X Mini on the own bracket (added 2026-11-11, NOTHING DONE ON THE ARM YET)

The simulators, the description and the wheels carry the camera by default (D14); the arm has never carried it. Same rules as every rung: the user is at the arm, approves each step, speed starts at 10 %.

| Step | What | Record | Pass |
|---|---|---|---|
| R3.0 | offline, no arm: `python tools/camera_clearance.py` (bracket version via `--calibration zedxm_bracket_vN`) | pairs in collision, closest gaps | no contact with link3..6 or the fingers. 2026-11-11 (v3): closest gap 15 mm to the fingers, 18 mm to link5; contacts with link1/link2/base only in extreme poses |
| R3.1 | arm powered off: fit the bracket, cable routing, turn joints 5 and 6 and open/close the gripper by hand through their ranges | gaps by eye and with a gauge, the printed bore vs the 57 mm gripper body (the design bore is 57.0 mm: no clearance) | nothing rubs or catches; clamp holds; cable has slack |
| R3.2 | weigh the bracket and the camera with its cable; put the values into `mounts/zedxm_bracket.xacro` (0.098 kg PLA estimate) and `cameras/zedxm.xacro` (0.150 kg datasheet) | masses | replaced the estimates; `docs/description/MASSES.md` regenerated |
| R3.3 | `piper-arm`: start (zero), `ready`, back to zero at 10 %, gripper 0 / 35 / 70 mm, the camera mounted | clearance by eye at each; state, efforts at rest | no contact; the arm holds zero (the payload grew by about 0.25 kg at the wrist: do not change firmware payload or crash-protection settings without explicit approval) |
| R3.4 | repeat R1.3 (`arm_stream_test.py` joints 2, 3, 5 at 10 %) with the camera on | lag, rms, repeatability vs the table without the camera | within 1 deg rms as before; any sag at zero noted |
| R3.5 | the ZED Box driver (docs/real_arm/ZED_BOX.md): topics, rate, frames, TF off | `ros2 topic hz`, `ros2 launch piper_control zed_camera.launch.py`, `tf2_echo piper_zed_left_camera_frame_optical piper_gripper_base` | frames equal the description (position 0.1 mm, orientation 0.1 deg); image 30 Hz |
| R3.6 | hand-eye check: a flat target (checkerboard or ArUco) at a measured place on the table, the arm at 3 poses, the ZED's measured target pose through the forward kinematics | target position error per pose | below 5 mm and 1 deg; else fit the bracket pose into `config/camera_mounts/zedxm_bracket_v4.yaml` (`measured:` filled in) and pass `camera_calibration:=zedxm_bracket_v4` |
| R3.7 | simulation vs real: the same pose (`ready`) in MuJoCo (`PiperSim.camera_image()`) and from the ZED | fingertips, horizon and table edge in both images | the fingers and the field of view agree; the lens (wide or narrow) is confirmed |
