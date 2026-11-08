# Cleanup and optimisation tracker

The living to-do list for making this repo clean, organised and importable by other projects. It is the current view of the repo:
trust this over older notes (`HANDOFF.md`, `docs/archive/`, `comms/`). Each pass appends a section to the log at the bottom and
ticks items here. An item is ticked only when it is done *and* verified; say how in the log.

Conventions: `[x]` done, `[ ]` open, `[~]` partly done or blocked (reason on the line). IDs stay stable across passes.
Design background: `docs/ARCHITECTURE.md` (layers, interfaces, prefix) and `docs/description/DECISIONS.md` (D1..D13).

## Target shape (from ARCHITECTURE.md)

Exported (others import): `piper_description`, `piper_core` (no ROS), `piper_control`, `piper_py`.
Internal (this repo only): `piper_sim` (Gazebo, MuJoCo, Isaac models, scenes, `piper.launch.py backend:=`), `tools/`, `isaac/`.

## A. Hygiene: stale or misplaced files

- [x] A1 `HANDOFF.md` (generated 2026-10-30, stale) moved to `docs/archive/legacy-plans/HANDOFF_T1_20261030.md` on the user's decision (pass 2); RULES.md still names a root HANDOFF.md as the machine-wide hand-off convention, which this repo no longer follows: this tracker replaces it.
- [x] A2 Stray `MUJOCO_LOG.TXT` files deleted (root and `src/piper_py`) and `ucode.elf` (written by the NVIDIA driver when Isaac runs in the repo directory); `.gitignore` ignores both. `piper_studio.code-workspace` appeared at the root during pass 1 and is not ours: left alone.
- [x] A3 `docs/description/MODEL_AUDIT.md` is a dated snapshot (2026-10-08, compares historical candidates): marked as such instead of regenerated; `model_audit.py` expands our description unprefixed on purpose.
- [x] A4 `docs/VERSIONS.md` (moved from the root in pass 2) and `docs/ROADMAP.md` updated (qualification table, M8 camera, M9 composition, date).
- [x] A5 README re-checked against the code (layout, prefix, camera, tools layout, test commands).
- [x] A6 `isaac/README.md`: prefix and USD rebuild note.
- [x] A7 `comms/` (agent protocol files, 25 KB) moved whole to `docs/archive/comms-20261108/` on the user's decision (pass 2); its 2026-11-08 STATUS entry is the last one. State now lives in this tracker and git history.
- [x] A8 `docs/archive/` (312 KB, three historical sets) kept, with a README saying it is historical and not to be followed.
- [x] A9 `.claude/settings.json` removed (pass 3, user decision): it forced `acceptEdits` and an allow list that conflicted with the machine rules (auto-allowed push, pip, apt, pkill); Claude behaviour now comes from `~/.claude/settings.json` and `~/projects/RULES.md`.

## B. Organisation

- [x] B1 `tools/` regrouped by purpose: `tools/real_arm/` (arm_* scripts and `arm_common.py`), `tools/demo/` (demo page, site pages, recording, step analysis, FK reference), `tools/capture/` (stills, video). `make_fk_reference.py` moved to `isaac/` (only used by `isaac.sh check`). References updated (README, docs, `artifacts.yaml`, scripts, tests). Verified: unit tests, `make_fk_reference`, py_compile of all moved scripts. Downstream callers by path (the bake-off overlay uses `tools/record_video.py`) must follow: B7.
- [x] B2 `piper_sim` split out: simulator model helpers (`mujoco_model`, `physics_model`, ...), `scene.py`, `config/mujoco`, `scene_objects.yaml`, `export_urdf.py`, `gz_server.sh`, `wait_for_clock.py` and `piper.launch.py backend:=...` live there; `piper_description` keeps only xacro, meshes, `robot_description()`, `render_config` and the validation tools. `isaac/` stays at the repo root (own environment). Verified: 30 offline tests, launch tests mock, Gazebo, MuJoCo, Isaac (USD rebuilt, check 0.001 mm), real plumbing + resting, `run_demo.sh mock` 11/11.
- [~] B3 `piper_core`, pure Python without ROS. Done: package `src/piper_core` with `limits_from_urdf`, `check_command` (the guard as one function, now used by `command_guard.py`) and `named_poses_from_srdf`; 8 tests; imports with no ROS; real plumbing and resting launch tests pass with the refactored guard. `RealArm` written (`piper_core/real_arm.py`: preflight, `start`, `move_to`, `execute(trajectory)` through the guard, `stop`), tested against a fake SDK (`test_real_arm.py`, 7 tests); **not yet run on the real arm** (needs the user: `execute()` streams one `move_j` per point, never tried with 25 ms points; the speed percent assumes 100 % = 3.0 rad/s). Open: that first run; let `piper_py` use `piper_core` limits.
- [x] B4 `piper_control` (renamed from `piper_bringup`): controllers, MoveIt, Servo, guard, `piper_control.launch_lib` (builders for other launch files) and `control.launch.py backend:=mock|real|external`. Verified: mock end to end (named pose, planned pose), `external` starts spawners, MoveIt and RViz and waits for a controller manager. Not yet verified: `external` against a real downstream simulator (B5 example).
- [x] B5 `docs/INTEGRATION.md` (contract: macros, names, units, controllers, frames, no-ROS use, real-arm rules, versioning); its xacro snippet is a test, and `src/piper_examples` (a carrier robot with its own controller manager, non-default prefix `arm_`, mount transform, camera; `control.launch.py backend:=external`) is a launch test that drives the arm through `piper_py` and checks the TCP lands where the mount puts it. Building the example found and fixed a real bug: MoveIt ignores an SRDF whose `<robot name>` differs from the URDF's, so `control.launch.py` now has `robot_name` and `robot_description_file`. The no-ROS use is covered by the `piper_core` tests and the doc snippet; a separate no-ROS example script is not needed.
- [x] B6 `piper.repos` (vcstool import list) pins `v0.2.0`; the tag is pushed (2026-11-08, user approved).
- [x] B7 The downstream overlay `~/projects/ml_tests/.../00_studio_overlay` copies `src/` and uses the old unprefixed names and tool paths: note left in `FOR_OTHERS.md` (now `docs/archive/comms-20261108/`; not ours to edit).

## C. Functional follow-ups

- [x] C1 Stand collision box vs fingers in MoveIt: start state valid at zero and ready for the then gripper range (`GetStateValidity`, 2026-11-08; the stroke is now 0 to 0.07 m and the test uses 0, 0.035, 0.07). Not yet a test (see C2).
- [x] C2 C1 is now a launch test, `test_10_camera_stand_does_not_collide` (two poses x three gripper widths via `GetStateValidity`; runs on the simulator backends with the camera; verified on MuJoCo).
- [x] C3 Gazebo GUI on a virtual display (Xvfb 1600x900 with GLX) now brings the arm up: the controller spawners gave up after the default 10 s service timeout while the software-rendered GUI loaded the machine; `piper_control.launch_lib.SPAWNER_TIMEOUTS` raises `--service-call-timeout` and `--switch-timeout` to 30 s. Verified: Gazebo GUI + `piper named ready` succeeds; mock, Gazebo and real-plumbing launch tests still pass. Left over, cosmetic: the GUI opens with a camera far from the arm (`13_gazebo_gui_ready.png`); a `<gui><camera>` pose in the world SDF is ignored by Gazebo, so it needs a GUI config file (not done).
- [ ] C4 (cosmetic, not fixed) Gazebo shows the camera stand white (the mesh's dark material is not applied). MuJoCo drops mesh colours (collision meshes are used as visuals); the arm is white there.
- [x] C5 4.5 mm (2026-11-08, user decision after comparing renders): the stand is clamped to the gripper body, so it follows `gripper_base` (4.5 mm out from link6 in the official model). `piper_camera` now takes `gripper_base` as its parent; AgileX's numbers are kept relative to it. Camera optical pose in link6 moved +4.5 mm in z. Test `test_wrist_camera_mount` checks both frames; offline, launch tests and Isaac USDs rebuilt. Still to confirm with a caliper on the real mount (stand pose, C9).
- [x] C6 (mitigated) `piper_py` exit noise: rclpy's `Executor.shutdown()` destroys its guard condition while a callback may still trigger it, which killed the spin thread with `InvalidHandle` (found after ~57 runs of `piper stop` on mock). The spin thread now ignores errors raised while `close()` runs (`Piper._run_executor`); the traceback is gone. A one-line "exception was never retrieved" can still appear in about 1 of 50 CLI runs at exit; exit code and result are unaffected.
- [ ] C7 (looked at, not fixed: upstream noise) Log noise: `servo_node` logger registered twice, octomap "No 3D sensor plugin", Gazebo `gz_frame_id` not in SDF, controller period 0.005 s vs Gazebo 0.001 s.
- [x] C8 `ros2 launch` itself rejects `prefix:=` (empty): all launches accept `prefix:=none` for the official names (`piper_description.normalize_prefix`); `view.launch.py` now expands through the Python API. Verified below.
- [x] C9 Camera calibration override: `config/camera_mounts/agilex_mid_stand.yaml` is now the single source of the nominal stand and camera poses; `camera_calibration:=<name or yaml path>` (xacro, launch files, `export_urdf.py --camera-calibration`) replaces them on every backend (test `test_camera_calibration_replaces_the_nominal_pose`). Still to do with the arm: measure the pose or run a hand-eye calibration and add the file.
- [ ] C10 Real arm through the ROS stack (needs the user at the arm): `backend:=real` with the prefix, driver names translated by `command_guard`. Only the fake-driver plumbing is tested so far.
- [ ] C11 Isaac launch test fails when the machine is busy: `test_8_servo_tcp_motion[isaac]` timed out (30 s, 3 mm tolerance) in 3 runs at 16:xx-17:xx on 2026-11-08, with the committed model and with the old box collisions alike, while load average was ~50 and another agent's Isaac testbed used the GPU at 78 %; it passed 5 times earlier in the day on a quiet machine. Likely cause: the simulator runs slower than real time under load. Rerun when `uptime` and `gpu-run --status` are quiet; not a model problem.
- [x] C12 `feature/piper-macros` merged into `main` (2026-11-08, user approved). `feature/piper-sim-split` (B2, B4, root cleanup) merged too, after all checks were green. Pushing stays the user's call.


## D. Root cleanup (pass 2, user decisions)

- [x] D1 Root now holds only `README.md`, `AGENTS.md`, `CLAUDE.md`, `artifacts.yaml`, `piper.repos` and the folders: `HANDOFF.md`, `comms/` and `VERSIONS.md` moved into `docs/` (archive or `docs/VERSIONS.md`).
- [x] D2 `AGENTS.md` rewritten (current layout, design rules, safety, process hygiene) and `README.md` rewritten for humans (what, quick start, layout, control levels, real arm, tests).
- [x] D3 Dangling references checked (user approved the edits): go2_studio's `comms/` references point at its own `comms/`, which still exists, so nothing to change there. In `~/projects/sim/robosim/handoff/` three places named piper_studio's `HANDOFF.md` or `comms/`: `START_PROMPTS.md`, `AGENT_AGREEMENT.md` and `build_handoffs.py` (which would have regenerated `piper_studio/HANDOFF.md`); updated and committed there (robosim `8312964`).

## E. Tooling

- [x] E1 `scripts/test_all.sh [backends...]`: one command for the offline tests, the generated-macro check, launch tests per backend, the real-arm plumbing and the carrier example, each in its own ROS domain with the daemon stopped afterwards. Verified with `mock` (all PASS).

## F. Repo-wide inventory (pass 3, 2026-11-08)

Every root entry and folder was mapped. Kept as they are: `src/` (7 packages), `docs/`, `scripts/`, `tools/`, `isaac/`, `env/`, `external/agx_arm_ros` (submodule), `artifacts.yaml`, `piper.repos`, `.gitmodules`, `.gitignore`, `log/COLCON_IGNORE`.
Generated, ignored: `build/`, `install/`, `log/` (older colcon logs deleted), `.venv/` (425 MB, `scripts/bootstrap.sh` rebuilds it), `.vscode/` (user's editor settings, paths updated for the new packages), `for_reference/`.

- [x] F0 Deleted (user approved): `.venv-legacy/` (237 MB, legacy venv; code is at tag `legacy-final-20261008`), `.vscode/browse.vc.db` (1.4 GB VS Code index), `.pytest_cache/`, old `log/build_*`; `.claude/settings.json` removed (A9); `*.code-workspace` git-ignored (the user's `piper_studio.code-workspace` stays on disk).
- [ ] F1 Prune `for_reference/` (5.3 GB; about 2.5 GB is study material nothing refers to). Kept for now at the user's request. Restore script and pinned commits: `scripts/reference_repos.sh`, `docs/REFERENCE_REPOS.md`.
- [x] F2 Stale text fixed: DECISIONS (single source of truth is now the generated macro, simulator transforms live in `piper_sim`), `meshes/README.md`, `isaac/README.md`, `.vscode/settings.json` paths.
- [x] F4 Per-file reference check of every script and module: all are used or documented; `tools/capture/capture_stills.py` and `tools/demo/step_analysis.py` are manual tools with no caller, now described in the README. Nothing dead found.
- [x] F3 `docs/STATE.md` written for fresh sessions (orientation, verified state, gotchas, where data lives); `AGENTS.md` and `README.md` point to it.

## G. Firmware-first bump (pass 4, 2026-11-08, branch `feature/firmware-priority-bump`)

The arm runs the latest firmware it supports (S-V1.9-1) and is calibrated; its reported values win over the vendor URDF (`docs/description/DECISIONS.md`).

- [x] G1 Submodule `agx_arm_ros` bumped `e4ccec1` to `286633e` (nested `agx_arm_urdf` `680fa92`), `pyAgxArm` `841a625` to `d90e234`; reference clones fast-forwarded. Vendor changes: joint 6 limit ±120° to ±170°, link6 inertia from CAD (6.1 g to 16.6 g), driver status topics (`arm_status`, `gripper_status`) are now `diagnostic_msgs/DiagnosticArray` plus new `feedback/motor_status` (we consume none of them).
- [x] G2 `config/firmware_values.yaml` + generator v2: arm joint velocity 3.0 rad/s and gripper stroke 0.07 m (fingers ±0.035, SRDF open/half scaled) follow the firmware; `piper_arm_macro.py report` lists the differences from the vendor URDF; a test compares the description with `piper-01.readout.json`.
- [x] G3 MoveIt limits 3.0 rad/s / 5.0 rad/s² (firmware); demo and tests use widths up to 60-70 mm; INTEGRATION gripper range 0 to 0.07 (**contract change**, noted here and in INTEGRATION).
- [x] G4 Stale values removed: S-V1.7-3 history and its numbers, J6 ±120°, 5 rad/s, 0.1 m gripper, "stroke unmeasured". Docs for ARCHITECTURE step 6 / Isaac and B7 pointer fixed. `docs/description/MASSES.md` (per-link masses with sources) and `tools/mass_table.py` added.
- [x] G5 Verified: 31 offline tests, `check`, launch tests mock, Gazebo, MuJoCo, Isaac (USD rebuilt with and without camera, `isaac.sh check` 0.001 mm, mass 4.7196 = URDF), real plumbing, carrier example. Not on the real arm.
- [x] G7 Driver status consumed (read-only): `Piper.status()`, `Piper.faults()`, `piper status`, `activate()` refuses on a reported fault; the fake driver publishes the same diagnostics; test `test_2b`. Vendor interfaces and codes documented in `REAL_ARM.md` 6d. Joint 2/3 start from the resting pose verified (2026-10-14 first motion). Limits table with sources and starting values in `DECISIONS.md`.
- [ ] G6 Open (needs the user): **weigh the arm, gripper, camera and stand and replace the masses in `MASSES.md` (to-do)**; joint torque rating and jerk (not in the firmware); camera and stand; the 4.5 mm is decided (stand on `gripper_base`, see C5; confirm with a caliper).
- [x] G9 Poses and start/stop (2026-11-08, user rules): one word per thing (zero / park / slumped / vendor zero, see `arms/piper-01.md`). `rest` renamed `park` and hidden from downstream (**breaking**: SRDF pose, `piper rest`, `Piper.move_rest`, `arm_zero.py --rest` are gone); new `Piper.start()` / `stop()` and `piper start` / `piper stop`; `command_guard` service `go_zero` is the only way out of the slumped pose; the guard is strict (no `limit_slack`; rejects targets outside the limits and every command while the arm is outside them); `piper_core.zero_pose` / `classify_state`; `activate()` returns `NOT_STARTED` on a slumped arm; fake driver can sag (`slump_positions`). Tests: `test_real_start_stop.py` (replaces `test_real_resting.py`), `test_core.py`, `test_named_poses.py`. Not yet on the real arm (needs the user): `start` / `stop` through the ROS stack, and the real drop from zero / park.
- [x] G10 Planner export (2026-11-08, request of the cuRobo study): `piper_description.fixed_gripper(urdf, width, prefix)` and `export_urdf.py --gripper-width W` give a flat URDF with the gripper and both finger joints fixed at W (test `test_fixed_gripper_puts_the_fingers_where_the_joints_would`). Example outputs: `~/data/ml/isaac/piper_studio/export/` (camera d435, widths 0.07 and 0, prefix `piper_` and none).
- [x] G8 `command_guard` drops every command while the driver reports an ERROR (`feedback/arm_status`, `feedback/motor_status`, newer than `fault_max_age` 3 s; test `test_2c`). The vendor `control_enable` gate is deliberately not used: the guard is the only writer to the driver, so the gate adds nothing and would need a reopen path.

## Verification record

### Pass 1, 2026-11-08 (prefix and macro migration)

Screenshots and video: `~/data/ml/isaac/piper_studio/verification/2026-11-08/` (viewed on my own Xvfb display `:96`, 2560x1440 with GLX;
Isaac and Gazebo cameras from their own sensors).

| Check | Result |
|---|---|
| Unit tests: description, bringup (macro-vs-official check included) | 21 pass |
| Mock + MoveIt + piper_py (launch test) | pass |
| Gazebo (headless server cameras) | pass; wrist depth 96 % valid, 30 Hz |
| MuJoCo with viewer and camera on the virtual display | pass; camera publishes (it could not on the VNC desktop); depth 100 %, 13 Hz wall clock |
| Isaac Sim 6.1 (GPU): USD rebuilt, runner, camera, video | pass; wrist depth 100 %, 31 Hz; `isaac.sh check`: fingertip error 0.001 mm, mass equal |
| Isaac backend launch test | pass on rerun (one flake, C11) |
| Real-arm plumbing with the fake driver; start/stop from the slumped pose | pass |
| RViz on the virtual display (model view, MoveIt) | renders; screenshots 01-06 |
| Gazebo GUI on the virtual display | GUI renders, controllers fail to come up (C3) |
| Real arm | not run |

Screenshots: 01-06 RViz (model, MoveIt, zero and ready pose, wrist close-up), 07-08 MuJoCo, 09-10 Gazebo, 11-12 Isaac (+ `11_isaac_run.mp4`).

## Log

- 2026-11-08 pass 4: firmware-first bump (section G).
- 2026-11-08 pass 2 (later): `piper_examples` carrier example (tested), `control.launch.py` `robot_name`/`robot_description_file`, `prefix:=none`, spawner timeouts (Gazebo GUI works on a virtual display, C3), `scripts/test_all.sh`; main pushed and tagged `v0.2.0` (user approved); robosim handoff references fixed (D3). Open and why: B3 `RealArm` and C10 (need the user at the arm), C5 (measure on the mount), C9 (needs real calibration data), C4, C6, C7, C11 (small, intermittent or cosmetic; see each).

- 2026-11-08 pass 2: merged the macros branch; root cleanup (D1, D2); `piper_bringup` replaced by `piper_control` + `piper_sim` (B2, B4); `piper_control.launch_lib` and `control.launch.py backend:=mock|real|external`; `piper_description` is now pure xacro/meshes/`robot_description()`. Re-verified: 30 offline tests, launch tests mock, Gazebo, MuJoCo, Isaac, real plumbing and resting, `run_demo.sh mock` 11/11, `isaac.sh build-usd`/`check`. Launch entry changed: `ros2 launch piper_sim piper.launch.py` (was `piper_bringup`).
- 2026-11-08 pass 1 (later): A1-A9, B1, B7, C1, C2 done; B3 (guard, limits, poses), B5 (INTEGRATION.md, snippet is a test), B6 (`piper.repos`) partly done. Tests: 31 offline (`piper_description`, `piper_bringup`, `piper_core`), launch tests per backend. Next pass: B2 (`piper_sim` split) and B4 (`piper_control`), then `RealArm` (B3) with the user at the arm, C3-C5, C7-C11, merge (C12).

- 2026-11-08 pass 1: created this tracker; verified the prefix migration on all backends (above); surveyed the repo (see A and B).
