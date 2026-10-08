# Versions (last qualified 2026-10-08)

The tested combination. Source revisions are authoritative in git (submodule pins and
`env/requirements.txt`); this file records what the qualification ran against.

## Sources

| Component | Revision | Notes |
|---|---|---|
| agilexrobotics/agx_arm_ros | `e4ccec1` (ros2) | submodule `external/agx_arm_ros`, unmodified |
| agilexrobotics/agx_arm_urdf | `983788b` (main) | nested in agx_arm_ros |
| agilexrobotics/pyAgxArm | `841a625` (master) | installed into `.venv` from git |

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
| Python | 3.12.3; NumPy 1.26.4 (ROS ABI); MuJoCo 3.12.0; trimesh 4.11.4; obj2mjcf 0.0.25; pytest 7.4.4 |

## Qualification status

| Profile | Status | Evidence |
|---|---|---|
| Description (official model unmodified, TCP) | pass | `colcon test --packages-select piper_description` |
| mock + MoveIt + piper_py | pass | `piper_py/test/test_bringup.py` [mock] |
| Gazebo Harmonic + MoveIt + piper_py | pass (kinematic position control) | `piper_py/test/test_bringup.py` [gazebo]; finger mimic verified against Gazebo ground truth |
| real plumbing (ros2_control <-> driver topics, command guard) | pass, **no arm** | `piper_py/test/test_real_plumbing.py` with `fake_driver` |
| scripted demo (11 steps, every control level) | pass on mock, Gazebo, MuJoCo, Isaac | `scripts/run_demo.sh`; end states agree with mock within 0.035 rad / 4.3 mm |
| real arm | not run | needs the user present |
| MuJoCo 3.12 + MoveIt + piper_py | pass (provisional actuator model) | `piper_py/test/test_bringup.py` [mujoco]; `piper_bringup/test/test_mujoco_model.py` (FK = official to 1e-6 m, finger coupling) |
| Isaac Sim 6.1 + MoveIt + piper_py | pass (opt-in: uses the GPU) | `PIPER_TEST_BACKENDS=isaac`; fingers verified against Isaac ground truth (±0.5 × gripper) |
