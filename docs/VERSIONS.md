# Versions (last qualified 2026-11-08)

The tested combination. Source revisions are authoritative in git (submodule pins and
`env/requirements.txt`); this file records what the qualification ran against.

## Sources

| Component | Revision | Notes |
|---|---|---|
| agilexrobotics/agx_arm_ros | `286633e` (ros2, 2026-10-21) | submodule `external/agx_arm_ros`, unmodified |
| agilexrobotics/agx_arm_urdf | `680fa92` (main, 2026-10-16) | nested in agx_arm_ros |
| agilexrobotics/pyAgxArm | `d90e234` (master, 2026-10-15) | installed into `.venv` from git |

## System

| Item | Version |
|---|---|
| OS | Ubuntu 24.04.5 LTS |
| ROS 2 | Jazzy (`/opt/ros/jazzy`) |
| MoveIt | 2.12.4 |
| ros2_control / ros2_controllers | 4.48.0 / 4.42.1 |
| joint_state_topic_hardware_interface | 1.1.0 |
| mujoco_ros2_control | 0.1.1 (libmujoco 3.12.0) |
| gz_ros2_control / Gazebo Sim | 1.2.20 / 8.15.0 (Harmonic) |
| Isaac Sim | 6.1.0.0 (Kit 110.3), RTX 4090 |
| Python | 3.12.3; NumPy 1.26.4 (ROS ABI); MuJoCo 3.12.0; trimesh 4.11.4; obj2mjcf 0.0.25; pytest 7.4.4 |

## Qualification status

| Profile | Status | Evidence |
|---|---|---|
| Description (official model unmodified, TCP) | pass | `colcon test --packages-select piper_description` |
| mock + MoveIt + piper_py | pass | `piper_py/test/test_bringup.py` [mock] |
| Gazebo Harmonic + MoveIt + piper_py | pass (kinematic position control) | `piper_py/test/test_bringup.py` [gazebo]; finger mimic verified against Gazebo ground truth |
| real driver node `piper_real_driver` (topics, services, setpoints, safety rules) | pass against a fake SDK, **no arm**; the fake-driver plumbing tests were removed 2026-11-10 | `piper_control/test/test_real_driver.py` |
| scripted demo (11 steps, every control level) | pass on mock, Gazebo, MuJoCo, Isaac | `scripts/run_demo.sh`; end states agree with mock within 0.035 rad / 4.3 mm |
| MoveIt Servo (TCP twist / pose) | pass on mock, Gazebo, MuJoCo, Isaac | `test_8_servo_tcp_motion` |
| simulated D435 wrist camera (colour, aligned depth, camera_info on the RealSense topics) | pass on Gazebo and MuJoCo (30 Hz), Isaac (see status page) | `test_9_wrist_camera`; Gazebo needs `gz sim --headless-rendering` and the sensor at the lens plane |
| named poses zero/park, strict guard, `piper start/stop/zero`, `go_zero`, launch `canbus`/`speed_percent` | pass on mock, MuJoCo, Gazebo, Isaac (end-to-end), real plumbing with a fake driver started on the stops (`test_real_start_stop.py`), pose consistency (`test_named_poses.py`) | not yet on the real arm |
| prefixed description (`piper_` default), generated `piper_arm` macro and SRDF, camera model + mount | pass on mock, Gazebo, MuJoCo (viewer and camera), Isaac (USD rebuilt, `isaac.sh check` 0.001 mm), real plumbing with the fake driver | `piper_arm_macro.py check`, unit tests (21), launch tests per backend; screenshots in `~/data/ml/isaac/piper_studio/verification/2026-11-08/`, see docs/CLEANUP.md |
| real arm | S-V1.9-1: read-out, enable test and a first supervised `move_j` to the zero pose done 2026-10-14 (SDK tools); not yet driven through the ROS stack | `docs/real_arm/arms/piper-01.md` |
| MuJoCo 3.12 + MoveIt + piper_py | pass (provisional actuator model) | `piper_py/test/test_bringup.py` [mujoco]; `piper_sim/test/test_mujoco_model.py` (FK = official to 1e-6 m, finger coupling) |
| Isaac Sim 6.1 + MoveIt + piper_py | pass (opt-in: uses the GPU) | `PIPER_TEST_BACKENDS=isaac`; fingers verified against Isaac ground truth (±0.5 × gripper) |

## v0.7.0 (2026-11-10; 0.6.0 was built locally and never released)

All four Python wheels use 0.7.0 (built from a clean commit, `dist/SHA256SUMS`; `piper_mujoco` and `piper_isaac` need `piper_core>=0.7.0`). New: gripper in `ArmBackend` (`gripper_opening`, `gripper_force`, `set_gripper(width, effort, timeout) -> width`) and in
`ArmState`; `RealArm` (guarded gripper, `stop()` ending with e-stop + reset, `abort()`, motors-enabled check, streaming options `mode` / `lookahead`); `piper_core.stream_metrics`, `stream_plan`; the `piper-arm` console (extra `arm`: needs `pyAgxArm`). Park joint 5 = 22.0 deg.
Verified: offline suite 148 passed (6 GPU tests skipped), MuJoCo wheel tests and the Isaac client protocol tests (fake server) pass; the GPU tests were not rerun for 0.6.0. Real arm (SDK tools and the console, user present): session, poses, jog, e-stop + reset, gripper close on a 10 mm object.
Not verified on the arm: `RealArm` motion methods, 25 ms streaming, speeds above 10 %. The fake-driver ROS plumbing tests are flaky on their own (see CLEANUP log). Hand-off note: `docs/FINDINGS.md`.

## v0.5.0 (2026-11-10)

All four Python wheels use 0.5.0. The Isaac client defaults to 250/250 Hz control/physics; headless recording renders at video fps with threaded encoding.
`PiperIsaacSim(gui=True)` / `python -m piper_isaac --gui` and `scripts/isaac.sh run --gui` select the full editor. Headless remains the default.

Verification (2026-11-10): offline suite 99 passed, 6 GPU tests skipped by default; the six GPU tests passed separately (parity 0.0210 rad vs MuJoCo 0.0212). Isaac ROS launch test and carrier example passed. Both fake-driver plumbing tests failed in the combined run, then passed on fresh domain 157 (48.76 s); this was a transient failure, not a fully green first run. Both full GUI runners opened and exited cleanly; the ROS runner stops the timeline before stage teardown to avoid the installed full-editor debug-draw crash. GUI idle time did not advance physics. Four-second recording: exactly 120 frames at 30 fps, no stale pairs; 0.88x real time (plain 1.29x, render-enabled 1.28x), identical final joints. USD checks passed with and without D435 (fingertip max 0.001 mm). Registry check: 0 failures, 6 existing machine warnings. No new network exposure.
