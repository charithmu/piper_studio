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

- [x] A1 `HANDOFF.md` (generated 2026-10-30 from robosim/handoff, tells agents to ask questions that were answered) stays where it is, because RULES.md names it as the hand-off file and it is regenerated from elsewhere; `AGENTS.md` now points to this tracker as the current state.
- [x] A2 Stray `MUJOCO_LOG.TXT` files deleted (root and `src/piper_py`) and `ucode.elf` (written by the NVIDIA driver when Isaac runs in the repo directory); `.gitignore` ignores both. `piper_studio.code-workspace` appeared at the root during pass 1 and is not ours: left alone.
- [x] A3 `docs/description/MODEL_AUDIT.md` is a dated snapshot (2026-10-08, compares historical candidates): marked as such instead of regenerated; `model_audit.py` expands our description unprefixed on purpose.
- [x] A4 `VERSIONS.md` and `docs/ROADMAP.md` updated (qualification table, M8 camera, M9 composition, date).
- [x] A5 README re-checked against the code (layout, prefix, camera, tools layout, test commands).
- [x] A6 `isaac/README.md`: prefix and USD rebuild note.
- [x] A7 `comms/STATUS.md` milestone entry appended (2026-11-08); `comms/` stays: AGENTS.md and the other threads rely on it.
- [x] A8 `docs/archive/` (312 KB, three historical sets) kept, with a README saying it is historical and not to be followed.
- [x] A9 `.claude/settings.json` (project agent permissions, shared by whoever works here) stays tracked; decided to keep.

## B. Organisation

- [x] B1 `tools/` regrouped by purpose: `tools/real_arm/` (arm_* scripts and `arm_common.py`), `tools/demo/` (demo page, site pages, recording, step analysis, FK reference), `tools/capture/` (stills, video). `make_fk_reference.py` moved to `isaac/` (only used by `isaac.sh check`). References updated (README, docs, `artifacts.yaml`, scripts, tests). Verified: unit tests, `make_fk_reference`, py_compile of all moved scripts. Downstream callers by path (the bake-off overlay uses `tools/record_video.py`) must follow: B7.
- [ ] B2 Split `piper_sim` out of `piper_description` and `piper_bringup`: simulator model helpers (`mujoco_model`, `physics_model`, `scene.py`, Gazebo world), `config/mujoco`, `scene_objects.yaml`, `isaac/`, and the `backend:=` launch. `piper_description` keeps only xacro, meshes and `robot_description()`.
- [ ] B3 `piper_core`, pure Python without ROS: limits parsed from the URDF, the command guard as one function (used by `command_guard.py` and the direct driver), named poses, and `RealArm` (direct pyAgxArm driver, firmware-profile handling from `tools/real_arm/arm_common.py`, `execute(trajectory)` for planners such as cuRobo).
- [ ] B4 `piper_control`: controllers, MoveIt, Servo, guard and `control.launch.py` that run on any controller manager (the real driver, mock, or a downstream simulator). `piper.launch.py backend:=` stays in `piper_sim`.
- [ ] B5 `docs/INTEGRATION.md`: the contract for other repos (names, units, controllers, frames, versioning) plus a small example project that uses its own simulator and one that uses no ROS.
- [ ] B6 `piper.repos` (vcstool import list) and a tag, so downstream projects pin a version.
- [x] B7 The downstream overlay `~/projects/ml_tests/.../00_studio_overlay` copies `src/` and uses the old unprefixed names and tool paths: note left in `comms/FOR_OTHERS.md` (not ours to edit).

## C. Functional follow-ups

- [x] C1 Stand collision box vs fingers in MoveIt: start state valid at zero and ready for gripper 0 to 0.1 m (`GetStateValidity`, 2026-11-08). Not yet a test (see C2).
- [ ] C2 Turn C1 into a launch test (cheap: one service call per pose).
- [ ] C3 Gazebo GUI on a virtual display (Xvfb with GLX): the GUI renders but the controller spawner fails (`switch_controller` after 3 attempts), so the arm never comes up. Headless server cameras work and are what the tests use.
- [ ] C4 Gazebo shows the camera stand white (the mesh's dark material is not applied). MuJoCo drops mesh colours (collision meshes are used as visuals); the arm is white there.
- [ ] C5 4.5 mm ambiguity: AgileX's flat URDF puts `gripper_base` at link6, the official model 4.5 mm further out; the stand is mounted relative to link6. Measure on the real mount.
- [ ] C6 `piper_py` exits with a harmless `InvalidHandle` traceback (found by the bake-off).
- [ ] C7 Log noise: `servo_node` logger registered twice, octomap "No 3D sensor plugin", Gazebo `gz_frame_id` not in SDF, controller period 0.005 s vs Gazebo 0.001 s.
- [ ] C8 `view.launch.py prefix:=""` falls back to `piper_` (xacro ignores an empty command-line value); the Python API handles it.
- [ ] C9 Camera calibration override: `config/camera_mounts/<name>.yaml` (measured pose replaces the nominal one on the real arm).
- [ ] C10 Real arm through the ROS stack (needs the user at the arm): `backend:=real` with the prefix, driver names translated by `command_guard`. Only the fake-driver plumbing is tested so far.
- [ ] C11 Isaac backend test failed once right after `scripts/isaac.sh check` and passed on rerun (1 of 2 runs): find out whether a leftover process from the check run (`omni.telemetry`) or the pid file is the cause.
- [ ] C12 Merge `feature/piper-macros` into `main` (after the user has seen the screenshots); push on the user's say-so.

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

- 2026-11-08 pass 1: created this tracker; verified the prefix migration on all backends (above); surveyed the repo (see A and B).
