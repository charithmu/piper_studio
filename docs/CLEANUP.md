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
- [x] A9 `.claude/settings.json` (project agent permissions, shared by whoever works here) stays tracked; decided to keep.

## B. Organisation

- [x] B1 `tools/` regrouped by purpose: `tools/real_arm/` (arm_* scripts and `arm_common.py`), `tools/demo/` (demo page, site pages, recording, step analysis, FK reference), `tools/capture/` (stills, video). `make_fk_reference.py` moved to `isaac/` (only used by `isaac.sh check`). References updated (README, docs, `artifacts.yaml`, scripts, tests). Verified: unit tests, `make_fk_reference`, py_compile of all moved scripts. Downstream callers by path (the bake-off overlay uses `tools/record_video.py`) must follow: B7.
- [x] B2 `piper_sim` split out: simulator model helpers (`mujoco_model`, `physics_model`, ...), `scene.py`, `config/mujoco`, `scene_objects.yaml`, `export_urdf.py`, `gz_server.sh`, `wait_for_clock.py` and `piper.launch.py backend:=...` live there; `piper_description` keeps only xacro, meshes, `robot_description()`, `render_config` and the validation tools. `isaac/` stays at the repo root (own environment). Verified: 30 offline tests, launch tests mock, Gazebo, MuJoCo, Isaac (USD rebuilt, check 0.001 mm), real plumbing + resting, `run_demo.sh mock` 11/11.
- [~] B3 `piper_core`, pure Python without ROS. Done: package `src/piper_core` with `limits_from_urdf`, `check_command` (the guard as one function, now used by `command_guard.py`) and `named_poses_from_srdf`; 8 tests; imports with no ROS; real plumbing and resting launch tests pass with the refactored guard. Open: `RealArm` (direct pyAgxArm driver, firmware-profile handling from `tools/real_arm/arm_common.py`, `execute(trajectory)` for planners such as cuRobo). It moves the real arm, so it is written with the user present and tested on the arm; also let `piper_py` use `piper_core` limits.
- [x] B4 `piper_control` (renamed from `piper_bringup`): controllers, MoveIt, Servo, guard, `piper_control.launch_lib` (builders for other launch files) and `control.launch.py backend:=mock|real|external`. Verified: mock end to end (named pose, planned pose), `external` starts spawners, MoveIt and RViz and waits for a controller manager. Not yet verified: `external` against a real downstream simulator (B5 example).
- [x] B5 `docs/INTEGRATION.md` (contract: macros, names, units, controllers, frames, no-ROS use, real-arm rules, versioning); its xacro snippet is a test, and `src/piper_examples` (a carrier robot with its own controller manager, non-default prefix `arm_`, mount transform, camera; `control.launch.py backend:=external`) is a launch test that drives the arm through `piper_py` and checks the TCP lands where the mount puts it. Building the example found and fixed a real bug: MoveIt ignores an SRDF whose `<robot name>` differs from the URDF's, so `control.launch.py` now has `robot_name` and `robot_description_file`. The no-ROS use is covered by the `piper_core` tests and the doc snippet; a separate no-ROS example script is not needed.
- [~] B6 `piper.repos` written. Open: a tag (after the merge, C12).
- [x] B7 The downstream overlay `~/projects/ml_tests/.../00_studio_overlay` copies `src/` and uses the old unprefixed names and tool paths: note left in `comms/FOR_OTHERS.md` (not ours to edit).

## D. Root cleanup (pass 2, user decisions)

- [x] D1 Root now holds only `README.md`, `AGENTS.md`, `CLAUDE.md`, `artifacts.yaml`, `piper.repos` and the folders: `HANDOFF.md`, `comms/` and `VERSIONS.md` moved into `docs/` (archive or `docs/VERSIONS.md`).
- [x] D2 `AGENTS.md` rewritten (current layout, design rules, safety, process hygiene) and `README.md` rewritten for humans (what, quick start, layout, control levels, real arm, tests).
- [ ] D3 Not ours, dangling now: `~/projects/ros/go2_studio/AGENTS.md` and `demo/pages.py`, and `~/projects/sim/robosim/handoff/` (AGENT_AGREEMENT.md, build_handoffs.py, START_PROMPTS.md, SESSION_LOG.md) still point at `piper_studio/comms/` and `HANDOFF.md`. The user decides whether those projects are updated.

## C. Functional follow-ups

- [x] C1 Stand collision box vs fingers in MoveIt: start state valid at zero and ready for gripper 0 to 0.1 m (`GetStateValidity`, 2026-11-08). Not yet a test (see C2).
- [x] C2 C1 is now a launch test, `test_10_camera_stand_does_not_collide` (two poses x three gripper widths via `GetStateValidity`; runs on the simulator backends with the camera; verified on MuJoCo).
- [ ] C3 Gazebo GUI on a virtual display (Xvfb with GLX): the GUI renders but the controller spawner fails (`switch_controller` after 3 attempts), so the arm never comes up. Headless server cameras work and are what the tests use.
- [ ] C4 Gazebo shows the camera stand white (the mesh's dark material is not applied). MuJoCo drops mesh colours (collision meshes are used as visuals); the arm is white there.
- [ ] C5 4.5 mm ambiguity: AgileX's flat URDF puts `gripper_base` at link6, the official model 4.5 mm further out; the stand is mounted relative to link6. Measure on the real mount.
- [ ] C6 `piper_py` sometimes exits with a harmless "cannot use Destroyable because destruction was requested" message (seen once after `pose` + `gripper`, reported by the bake-off as `InvalidHandle`). Not reproduced in 7 CLI runs (state, named, pose, joints, gripper x3): intermittent; likely a pending future when the node is destroyed in `Piper.close()`.
- [ ] C7 Log noise: `servo_node` logger registered twice, octomap "No 3D sensor plugin", Gazebo `gz_frame_id` not in SDF, controller period 0.005 s vs Gazebo 0.001 s.
- [x] C8 `ros2 launch` itself rejects `prefix:=` (empty): all launches accept `prefix:=none` for the official names (`piper_description.normalize_prefix`); `view.launch.py` now expands through the Python API. Verified below.
- [ ] C9 Camera calibration override: `config/camera_mounts/<name>.yaml` (measured pose replaces the nominal one on the real arm).
- [ ] C10 Real arm through the ROS stack (needs the user at the arm): `backend:=real` with the prefix, driver names translated by `command_guard`. Only the fake-driver plumbing is tested so far.
- [ ] C11 Isaac backend test failed once right after `scripts/isaac.sh check` and passed on rerun (1 of 2 runs): find out whether a leftover process from the check run (`omni.telemetry`) or the pid file is the cause.
- [x] C12 `feature/piper-macros` merged into `main` (2026-11-08, user approved). `feature/piper-sim-split` (B2, B4, root cleanup) merged too, after all checks were green. Pushing stays the user's call.

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
| Real-arm plumbing with the fake driver; resting pose | pass |
| RViz on the virtual display (model view, MoveIt) | renders; screenshots 01-06 |
| Gazebo GUI on the virtual display | GUI renders, controllers fail to come up (C3) |
| Real arm | not run |

Screenshots: 01-06 RViz (model, MoveIt, zero and ready pose, wrist close-up), 07-08 MuJoCo, 09-10 Gazebo, 11-12 Isaac (+ `11_isaac_run.mp4`).

## Log

- 2026-11-08 pass 2: merged the macros branch; root cleanup (D1, D2); `piper_bringup` replaced by `piper_control` + `piper_sim` (B2, B4); `piper_control.launch_lib` and `control.launch.py backend:=mock|real|external`; `piper_description` is now pure xacro/meshes/`robot_description()`. Re-verified: 30 offline tests, launch tests mock, Gazebo, MuJoCo, Isaac, real plumbing and resting, `run_demo.sh mock` 11/11, `isaac.sh build-usd`/`check`. Launch entry changed: `ros2 launch piper_sim piper.launch.py` (was `piper_bringup`).
- 2026-11-08 pass 1 (later): A1-A9, B1, B7, C1, C2 done; B3 (guard, limits, poses), B5 (INTEGRATION.md, snippet is a test), B6 (`piper.repos`) partly done. Tests: 31 offline (`piper_description`, `piper_bringup`, `piper_core`), launch tests per backend. Next pass: B2 (`piper_sim` split) and B4 (`piper_control`), then `RealArm` (B3) with the user at the arm, C3-C5, C7-C11, merge (C12).

- 2026-11-08 pass 1: created this tracker; verified the prefix migration on all backends (above); surveyed the repo (see A and B).
