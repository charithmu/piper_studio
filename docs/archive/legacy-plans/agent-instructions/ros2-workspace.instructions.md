---
description: "Use when editing ROS 2 packages, launch files, URDF or Xacro files, MoveIt configs, Gazebo configs, or package metadata under agx_arm_gzsim, agx_arm_motion, or agx_arm_ros. Covers colcon workflow, package boundaries, and real-vs-sim parity."
applyTo: "src/agx_arm_gzsim/**,src/agx_arm_motion/**,src/agx_arm_ros/**,src/agx_arm_wristcam/**,src/agx_arm_detect/**,src/agx_arm_graspgen/**,src/agx_arm_manipulation/**"
---

# ROS 2 Workspace Instructions

## Environment Setup — Required Before Any Command

Always source these three layers in order before building, testing, or launching:

```bash
source /opt/ros/jazzy/setup.bash
source <workspace_root>/.venv/bin/activate
source <workspace_root>/install/setup.bash   # only after first build
```

ROS distro is **jazzy**. The workspace venv is `.venv/` at the workspace root. `install/setup.bash` only exists after the first `colcon build`.

- Build from the workspace root with selective `colcon build --packages-select ...` commands before widening to a full workspace build.
- Keep the package boundaries intact: `agx_arm_ctrl` bridges ROS 2 to `pyAgxArm`, `agx_arm_description` owns URDF assets, `agx_arm_moveit` owns reusable MoveIt config, `agx_arm_gzsim` layers simulation on top, and `agx_arm_motion` is a higher-level client layer.
- Respect the ownership split: `agx_arm_ros` (including `agx_arm_ctrl`, `agx_arm_description`, `agx_arm_moveit`, `agx_arm_msgs`) is upstream-frozen — do not edit without coordination. `agx_arm_gzsim`, `agx_arm_motion`, `agx_arm_wristcam`, `agx_arm_detect`, `agx_arm_graspgen`, and `agx_arm_manipulation` are workspace-owned and may be improved when needed; in particular `agx_arm_gzsim` is ours and is the right home for sim-side launch glue.
- Prefer parameterized launch and config changes over model-specific branching. The main exception is code that is already explicitly Piper-only, such as the current Gazebo package.
- Preserve one-to-one correspondence between real and simulated behavior wherever the package intent is parity: frames, controller names, joint names, planning groups, and topic semantics should stay aligned unless a simulator constraint requires a documented divergence.
- When editing `package.xml`, `setup.py`, launch files, or YAML configs, keep README examples in sync if user-visible behavior changes.
- Validate ROS changes with the narrowest relevant command: package-select build, targeted launch, or focused test before broader workspace checks.