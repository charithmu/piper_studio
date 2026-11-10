# Real-arm checklist: the ladder, one rung at a time

Rules for every rung: **the user is at the arm, hand on the power switch, approving each step.** Nothing starts from a background job or a test run. Speed starts at 10 % and is raised one step at a time
(10 -> 20 -> 30 -> 50, the user's ceiling for this task). One program on the CAN bus at a time. `e` in `piper-arm` is the emergency stop (electronic e-stop); Space holds; the power switch is the hard stop.
Record results in `arms/piper-01.md` (hardware log) and the report `docs/real_arm/REPORT.md` (numbers, method, confidence). Do not change guard limits, firmware values or calibration from one read-out.

Tools: `piper-arm` (cockpit: status, keys, gripper), `tools/real_arm/arm_session.py`, `arm_pose.py`, `arm_jog.py`, `arm_stream_test.py` (see `REAL_ARM.md` section 8). Start of day: `scripts/can_up.sh` (sudo, once per boot), `source scripts/env.sh`.

## R0 read-only (done 2026-11-10)

- [x] `scripts/can_up.sh`; `arm_joints.py` shows frames; `arm_info.py --arm-id piper-01` equals `piper-01.readout.json` (firmware S-V1.9-1, slave, MOVE_J, teach off, no faults, limits as stored).
- [x] State at rest recorded (slumped pose, gripper 0 mm, drivers 26-28 C).
- [ ] Gripper range on the arm: open stroke and empty-closed width (R2 below measures both).

## R1 trajectories through RealArm (software ready, hardware to do)

Pre-conditions: `piper-arm` `s` (enabled, at zero), no object near. Each step is approved by the user, then run by the user.

| Step | Command | Record | Pass |
|---|---|---|---|
| R1.0 | `python tools/real_arm/arm_stream_test.py --offline` | plan, speed needed | numbers printed |
| R1.1 | `arm_stream_test.py` (joint 1, 10 deg cosine, 4 s, 10 %) | lag, rms, final error, timing | final error < 0.5 deg, no guard event |
| R1.2 | same with `--mode per_point`, then `--lookahead 2`, `--repeat 3` | lag, rms, repeatability per mode | compare the three: smaller lag/rms at the same plan wins |
| R1.3 | joints 2, 3, 5 one at a time (`--joint 2 --amplitude 8`, ...) | per-joint lag, gravity effect (j2/j3/j5) | within 1 deg rms at 10 % |
| R1.4 | speed ladder 20 -> 30 -> 50 % on the best mode (`--period` shorter, `--max-percent` raised by the user) | `peak speed` vs planned vs nominal (3.0 rad/s x percent) | measured speed within 10 % of nominal, no fault |
| R1.5 | `RealArm.start()/execute()/stop()` from Python (a 10-line script written with the user), then a cuRobo Cartesian path planned offline and executed at 10 %, 20 % | tracking error of the end effector (FK), joint errors | no guard event; tracking as in R1.3 |

Measure per joint (from `stream_metrics`): tracking error, lag, overshoot, repeatability (`--repeat`), `speed_percent` vs real speed, behaviour at 25 ms setpoints (timing late/spacing). Stop and ask if any run ends with a guard event, a fault, or a final error above 1 deg.

## R2 gripper (arm parked or at zero, nothing in the way)

Pre-condition: `piper-arm`; the gripper panel reads width and force. Commands: `o` open, `l` close, `g` steps, `f` force (0.5-3 N). Keep fingers out of the closing path.

| Step | What | Record |
|---|---|---|
| R2.0 | empty: `o`, `l` x5 at 1.0 N | open stroke (mm), **empty-closed width** (mm, the value `piper_pick` uses to detect a miss), repeatability (std over 5), closing time (stopwatch or video until "reached") |
| R2.1 | closing time vs force: `l` at 0.5, 1, 1.5, 2, 3 N | time to reach, force read-back (sign negative while gripping) |
| R2.2 | objects: the 10 mm object (done: held at 9.4 mm, -2.05 N at 3 N), the white plastic box, foam brick / soap / banana if available | stalled width, force read-back, whether it holds |
| R2.3 | slip: hold the object at 1.0 N, then move the arm (zero <-> park, then `arm_pose.py ready`) at 10 % | slip yes/no, width change during the move |
| R2.4 | no-motor case: gripper `o` / `l` with the arm motors OFF; and `e` (e-stop) with the gripper closed on an object | does it move; does the e-stop release the grip |

Then compare with the simulated gripper (Isaac, MuJoCo): stroke, closing time, held width, force (the simulators have no force read-back and ignore effort) and write the gap table in the report.

## What must be true before `piper_pick` runs on the arm

R1.1-R1.3 passed at 10 %; R2.0 and R2.2 recorded; `docs/FINDINGS.md` read by the `piper_pick` agent; the user has decided the speed percent for pick-and-place.
