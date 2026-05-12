---
description: "Use when creating or editing agx_arm_eyes, agx_arm_detect, agx_arm_graspgen, or agx_arm_manipulation in piper_studio. Covers the planned package boundaries, sim-first rollout, and standard perception interfaces."
applyTo: "src/agx_arm_eyes/**,src/agx_arm_detect/**,src/agx_arm_graspgen/**,src/agx_arm_manipulation/**"
---

# Perception And Manipulation Instructions

- Read `DESIGN.md` and `TODO.md` before changing the planned perception or manipulation stack. Those files are the human-editable source of truth for roadmap intent.
- Keep the layers separate: `agx_arm_eyes` loads camera drivers and manages calibration assets, `agx_arm_detect` consumes standardized camera data, `agx_arm_graspgen` produces candidate poses, and `agx_arm_manipulation` orchestrates motion primitives.
- Do not couple task-level logic to a camera vendor package. Downstream packages should depend on the standardized camera interface, not on RealSense, Orbbec, or OAK-specific topics.
- Preserve the sim-first workflow. New perception or manipulation features should have a simulation or replay path before relying on live hardware.
- Reuse `agx_arm_motion` for motion primitives where practical, but fix missing backend abstraction in `agx_arm_motion` instead of duplicating MoveIt glue in every new package.
- Keep calibration assets, mount configs, and vendor configuration in package-owned `config/` directories with predictable naming.
- Prefer standard ROS messages and TF over custom message types unless a custom type clearly improves interoperability.