# Real-arm report: piper-01, 2026-11-10

What was measured on the real arm, how, how sure we are, and what is not known. Arm: standard Piper, AgileX gripper, firmware S-V1.9-1. Every motion was run with the user present, hand on the power switch, speed raised one step at a time
(10 % first, 44 % the highest so far). Raw data: `~/data/ml/isaac/piper_studio/{stream,gripper,scenario}/`; the hardware log with the sequence of events is `arms/piper-01.md`; the ladder is `CHECKLIST.md`.
Interface under test: `piper_core.RealArm` (the supported real-arm interface), the `piper-arm` console and the SDK tools in `tools/real_arm/`. The ROS path on the real arm is parked (`docs/CLEANUP.md` C14).

## 1. Arm and session

| Item | Result | Method, confidence |
|---|---|---|
| Read-only state vs stored read-out | identical (firmware, limits, no faults) | `arm_info.py`, high |
| start (power-off pose to zero) | 2.4-2.7 s at 10 %, end error 0.05-0.07 deg, every time | about 10 runs, high |
| stop (zero, park, e-stop, clear), then start again at once | works; the e-stop lowers the arm under damping; it settles about 5 deg deeper on joint 5 than a plain disable (27 vs 22-24 deg), below the park pose | 4 runs, high |
| Park pose | joint 5 = 22.0 deg (zero with joint 5 lowered), about 1 cm above the brackets | found with `arm_jog.py`, high |
| Speed percent vs real speed | 100 % = 3.0 rad/s: measured peak equals planned/nominal at 6, 10 and 44 % (0.18 / 0.30 / 1.1 rad/s) | about 15 runs, high |
| Kinematics | the pose the arm reports equals the SDK modified-DH to 0.00 mm (zero, ready); our URDF is within 0.09 mm (300 random poses) | high |
| Guard refusals | out-of-limit and bad trajectories refused, nothing moved (0.000 deg) | smoke test, high |
| abort() | stops a running trajectory and holds | 1 run, medium |
| Stale-feedback check | no false alarm on real data (age 0.05 ms at rest) | high |
| Bus lock | refused a second program twice, named the holder's pid | high |
| Dead-process drill (the arm keeps holding if the program dies) | not run | open |

## 2. Streaming 25 ms setpoints through `RealArm.execute`

Setpoint timing: late 0.3 ms on average (max 1.9 ms), spacing 25.0 +- 0.1 ms. The end pose is exact (0.00 deg) in every run; no overshoot, no fault; final-pose repeatability < 0.01 deg over 3 runs. The arm trails a streamed setpoint by about
**35 ms + 73 ms per rad/s** (joint 1; four speeds). `lookahead="auto"` (now the default) sends points ahead by that lag.

| Joint, plan | Speed % | Mode | Lag | rms | max |
|---|---|---|---|---|---|
| j1, 10 deg / 4 s | 6 | fixed | 45 ms | 0.25 deg | 0.43 |
| j1, same | 6 | per_point | 45 ms | 0.26 | 0.44 |
| j1, same | 6 | lookahead 2 (10 runs) | 0 ms | 0.04 | 0.09-0.12 |
| j2, 8 deg / 4 s | 5 | lookahead 2 | 0 ms | 0.05 | 0.11 |
| j3, -8 deg / 4 s | 5 | lookahead 2 | 0 ms | 0.04 | 0.11 |
| j5, 15 deg / 4 s | 9 | lookahead 2 | 5 ms | 0.07 | 0.35 |
| j1, 20 deg / 2 s | 22 | lookahead 2 / **auto** | 25 / 5 ms | 0.62 / **0.10** | 1.06 / 0.28 |
| j1, 30 deg / 2 s | 33 | lookahead 2 / **auto** | 45 / 10 ms | 1.50 / **0.37** | 2.61 / 0.75 |
| j1, 40 deg / 2 s | 44 | lookahead 2 / **auto** | 65 / 20 ms | 2.96 / **1.03** | 4.72 / 2.24 |

Confidence: high at low speed (several runs per case); one run per case at 22, 33, 44 % and only joint 1 at speed; the lag model is fitted on four points. The guard's 0.15 rad step from the measured pose limits full compensation to about 1.2 rad/s
(`auto` backs off there). `per_point` speed percent gives no gain. Not measured: speeds above 44 %, joints 2, 3, 5 at speed, a payload.

## 3. cuRobo scenario (approach, grasp, close, lift, open, home) on the real arm

TCP (0.25, 0, 0.05) m tool down, planned by cuRobo (study planner, table plane at the base only), slowed to 0.3 rad/s, 49.8 s, all steps ok, back at zero. Worst joint error 0.33 deg (approach), end errors 0.05-0.09 deg. The same plan passes in MuJoCo and through
the ROS client on the mock backend. Confidence: one run on the arm, no object in the path.

## 4. Gripper (nothing in it, then a plastic box of about 54 mm)

| Item | Result | n |
|---|---|---|
| Open width for a 70 mm command | **69.44 mm**, spread 0.00 mm | 5 + 10 |
| Empty-closed width for a 0 mm command | **0.14 mm**, spread 0.00 mm (a held object shows as a stalled close far above this) | 5 + 10 |
| Closing time to within 1 mm | 0.28 s (0.26-0.34); settles in 0.34 s | 15 |
| Opening time to within 1 mm | 0.26-0.36 s | 15 |
| Time vs force (0.5 to 3 N) | 0.34 s at 0.5 N, 0.26-0.28 s from 1 N up: barely depends on force | 2 each |
| Force read-back, empty, at rest | open +0.18 N, closed -0.08 N (noise about +-0.2 N); impact peak while closing empty -0.4 N (0.5 N) to -1.2 N (2-3 N) | 15 |
| Box, stalled width at 1 / 2 / 3 N | 54.27 / 53.25 / 53.55 mm (spread 0.03-0.17 mm) | 3 each |
| Box, steady force read-back at 1 / 2 / 3 N | -1.04 / -2.09 / **-2.09 N**: the steady grip **plateaus at about 2.1 N** although 3 N is commanded; impact peaks -2.9 / -5.0 / -6.2 N | 3 each |
| Box, relaxation while held (3 s) | width grows +0.54 / +0.96 / +1.07 mm | 3 each |
| Box, held while joint 1 moves 20 deg in 2 s (0.55 rad/s), 2 N | no loss of the object; width 53.55 to 54.74 mm (+1.2 mm, mostly the relaxation above), force dips from -2.08 to -0.47 N during the move | 2 |
| 10 mm object (earlier) | held at 9.4 mm, -2.05 N at 3 N | 1 |

Behaviour found by the user: **the gripper does not move while the arm's motors are off**, also after `stop()` or an e-stop clear, until `start()`; **the e-stop does not release the grip**. `RealArm.set_gripper` now refuses at once with that reason
instead of ending as `no_motion`. Open the gripper before `stop()` if it should be released. Confidence: high for empty widths and times, medium for the box (one object, three repeats), low for slip (two runs at 0.55 rad/s, one force). Not measured: fast arm
motion with an object, heavier objects, other objects (foam, soap, banana), the force below the 2.1 N plateau for soft objects.

## 5. Simulator gaps and recommendation (the simulators were NOT changed)

| Item | Real arm | MuJoCo (`PiperSim`) | Isaac (`PiperIsaacSim`) |
|---|---|---|---|
| Stroke | 69.44 mm | 70.0 mm | 70 mm commanded; physical opening not measured here |
| Empty-closed width | 0.14 mm | 0.00 mm | not measured |
| Closing time (70 to 0 mm) | 0.28 s | **0.074 s** | not measured |
| Opening time | 0.26-0.36 s | 0.074 s | not measured |
| Grip force | steady plateau about 2.1 N (3 N commanded), read back as negative | actuator +-10 N (kp 400), no read-back (NaN), `effort` ignored | no read-back, `effort` ignored |
| Held object | relaxes +0.5 to 1 mm in 3 s | rigid contact (not measured) | not measured |
| Arm servo | trails a streamed setpoint 35 ms + 73 ms per rad/s | ideal servos (not measured here) | ideal servos (not measured here) |

Recommended simulator parameters (for the owners to apply and re-run their tests): limit the finger speed so a full close or open takes about 0.28 s (about 250 mm/s) instead of 0.074 s; limit the steady grip force to about 2 N (the real plateau) instead of +-10 N; keep
the stroke at 69.5 mm; add an optional arm-servo delay of about 45 ms at low speed rising to about 115 ms at 1.1 rad/s if tracking realism matters. Isaac needs the same gripper numbers measured on the GPU (not done).

## 6. What is not tested, and open risks

- The ROS path on the real arm (parked, `docs/CLEANUP.md` C14). Speeds above 44 %; joints 2, 3, 5 at speed; a payload (camera, object) and the arm's payload setting (none set); crash protection is level 0 (no collision detection), left as it is until tried.
- The cuRobo scene is a table plane only: the real workspace must match. Hand-eye calibration, camera pose and the TCP offset (fingertip centre 0.138 m from the gripper base) are not measured.
- The dead-process drill; the e-stop with an object gripped was observed once by the user (no release).
- The Isaac edits of 0.7.0 were only run against the fake-server client tests, not on the GPU.

## 7. The API `piper_pick` should call

`RealArm.connect("can0")` then `start()`; `status()`; `execute(traj)` (25 ms setpoints, `lookahead="auto"` default, `ExecResult`); `set_gripper(width, effort)` (blocks; `last_gripper.reason` is `reached`, `stalled` for a held object, `no_motion` or `timeout`; needs the arm started); `hold()`, `abort()`;
`stop()` (zero, park, e-stop, clear; the grip is not released). Details and the exact guarantees: `docs/FINDINGS.md`, `docs/INTEGRATION.md` section 3.
