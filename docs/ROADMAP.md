# Roadmap

Each milestone ends with passing tests and an updated VERSIONS.md.

| # | Milestone | Status |
|---|---|---|
| M1 | Official stack pinned; `piper_description`, `piper_bringup`, `piper_py`; mock + MoveIt end to end; real-arm plumbing with command guard (tested without arm) | done 2026-10-08 |
| M2 | Real arm, read-only: driver connects, firmware version, enforced joint limits, gripper stroke; then supervised low-speed motion through `activate` | needs the user at the arm |
| M3 | Gazebo Harmonic: same controllers via gz_ros2_control; gripper mimic handling; same piper_py tests pass | done 2026-10-08 (in piper_bringup; no separate package needed yet) |
| M4 | MuJoCo: MJCF generated from the URDF (mujoco_ros2_control); FK parity check vs URDF; same tests pass | done 2026-10-08 |
| M5 | `piper_sim` Isaac Sim 6.1: USD generated from the URDF; ROS 2 control path (topic-based or isaacsim.ros2.control); same tests pass | GPU slot |
| M6 | Streaming/servo: MoveIt Servo on `arm_position_controller`; teleop input | |
| M7 | Recording and policy interface: synchronized episodes (state, commands, camera), LeRobot export, policy runner using streaming targets | |
| M8 | `piper_perception`: actual wrist camera driver, calibration, mount frames | needs camera model and mount |
| M9 | Platform composition (Go2 + Piper): frame prefix/namespace, mount transform | |

Cross-cutting references (not dependencies): IIT piper-ros2-dls (gain scaling, Piper-L), Renesas
and piper_cpp native drivers, MuJoCo Menagerie (contact parameters), AgileX Isaac assets and
IsaacLab examples, LeRobot Piper plugin. See `docs/archive/chatgpt-review-20261008/UPSTREAMS.md`.
