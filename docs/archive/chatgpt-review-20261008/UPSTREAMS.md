# Official candidates and selective reuse

Verified 2026-10-08 with `git ls-remote`. These are candidate revisions for an integration branch, not an approved working lock.

| Source | Branch | Candidate commit |
|---|---|---|
| [pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) | master | `841a625f5f4920e776f20b934eb13048b747e6d0` |
| [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2) | ros2 | `e4ccec1999279d063c2962030b9437b78b0d0db6` |
| [agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf) | main | `983788b58d7eae76511177f768d85877534917fe` |

## Migration concerns

- The ROS/description gripper model changed to one commanded aperture joint with passive mimic fingers. Driver, URDF, SRDF, controller, simulator, and application mappings must migrate together. [ROS history](https://github.com/agilexrobotics/agx_arm_ros/commits/ros2).
- Description updates introduce flange and mounting changes. Recheck TCP/camera transforms rather than preserving old offsets by name alone. [Description history](https://github.com/agilexrobotics/agx_arm_urdf/commits/main).
- The SDK has firmware-specific behavior and revised MIT torque scaling; ROS changes depend on SDK firmware-resolution APIs. [Torque conversion change](https://github.com/agilexrobotics/pyAgxArm/commit/a13cd89fe17347dcced1f8c1d26ab2ed0c8a0d1b).
- The old [piper_sdk](https://github.com/agilexrobotics/piper_sdk) directs users toward pyAgxArm; keep legacy-dependent examples optional.
- Latest source is not automatically qualified against the user's firmware. Record model/firmware/controller support and exact tested revisions.

## Components worth reusing

| Component | Proposed treatment |
|---|---|
| [Topic-based hardware interfaces](https://github.com/ros-controls/topic_based_hardware_interfaces) | Evaluate measured-state ros2_control integration without replacing the official SDK; Jazzy is listed as supported |
| [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control) | Reuse maintained engine/control integration |
| [MuJoCo Menagerie Piper](https://github.com/google-deepmind/mujoco_menagerie/tree/main/agilex_piper) | Model/contact reference with pinned provenance; not physical dynamics ground truth |
| [Isaac native ROS control](https://docs.isaacsim.omniverse.nvidia.com/latest/ros2_tutorials/robot_control/tutorial_ros2_control.html) | Preferred Isaac ROS candidate; controller manager is hosted in-process; qualify Piper and USD interface mapping |
| [Official Piper Isaac assets](https://github.com/agilexrobotics/piper_isaac_sim) | Reference assets; compare with the canonical latest description before reuse |
| [AgileX IsaacLab examples](https://github.com/agilexrobotics/Agilex-College/tree/master/isaac_sim/agx_arm_IsaacLab) | Select task/teleoperation/data examples; verify version compatibility |
| [MoveIt Task Constructor](https://github.com/moveit/moveit_task_constructor/tree/ros2) | Evaluate for multistage manipulation; select a Jazzy-compatible revision |
| [MoveIt Servo](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html) | Optional ROS Cartesian-streaming adapter; separate from finite planning |

## Alternatives requiring isolated evaluation

- [Renesas native Piper hardware](https://github.com/renesas-rdk/agilex_piper_ros2_control) and [MuJoCo workspace](https://github.com/renesas-rdk/agilex_piper_mujoco): C++ protocol/control alternatives; older gripper conventions and activation behavior require review.
- [piper_cpp](https://github.com/justagist/piper_cpp): native SDK/plugin alternative. [ROS documentation](https://github.com/justagist/piper_cpp/blob/main/piper_cpp_ros/README.md) lists position commands and defaults to a zero move on activation; this is not a drop-in firmware-qualified replacement.
- [IIT-DLS](https://github.com/iit-DLSLab/piper-ros2-dls): useful impedance research; Piper-L defaults/gains do not establish standard-Piper behavior.
- [LeRobot Piper plugin](https://github.com/AgRoboticsResearch/lerobot_robot_piper): optional integration using legacy piper_sdk; adapt semantics rather than replacing the core dependency stack.
- [PiperPilot](https://github.com/tomakeIT/PiperPilot): experimental teleoperation/recording example using pyAgxArm; independently review control modes, defaults, and dataset format.

Retain links and compatibility notes for references. Add a repository to the source dependency set only when a supported profile actually requires it. Pin adopted components and preserve their licence/provenance records.
