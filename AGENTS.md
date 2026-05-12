# Piper Studio Agent Guide

## Workspace

- This repository is a ROS 2 workspace rooted at `piper_studio` with source packages under `src/`.
- `src/agx_arm_ros` and `src/pyAgxArm` are maintained forks of AgileX upstream repositories.
- `src/agx_arm_motion`, `src/agx_arm_gzsim`, and future sibling packages are supplemental workspace packages that add motion-planning, simulation, and higher-level tooling.
- The workspace Python environment lives in `.venv/`. Activate it before Python tooling so package installs and tests stay local to this workspace.

## Deployment Discipline

- Multi-agent rollout is governed by `ORCHESTRATION.md`. Each wave ends in
  a **human checkpoint** that must pass before the next wave dispatches.
  Outcomes are logged in `log/checkpoints.md`.
- Agents must keep their work inside the package(s) named in their brief
  and must not edit `agx_arm_ros` or `pyAgxArm` (upstream-frozen).

## Ownership

- **Upstream-frozen, do not edit without coordination:** `src/pyAgxArm`,
  `src/agx_arm_ros` (including `agx_arm_ctrl`, `agx_arm_description`,
  `agx_arm_moveit`, `agx_arm_msgs`).
- **Workspace-owned, free to improve:** `src/agx_arm_gzsim`,
  `src/agx_arm_motion`, `src/agx_arm_eyes`, `src/agx_arm_detect`,
  `src/agx_arm_graspgen`, `src/agx_arm_manipulation`. `agx_arm_gzsim` is
  ours — change it when needed to keep sim/real parity or to host
  sim-side launch glue.

## Architecture

- Treat `src/pyAgxArm` as the authoritative low-level SDK for CAN transport, firmware-specific behavior, and arm/end-effector APIs.
- Treat `src/agx_arm_ros/src/agx_arm_ctrl` as the ROS 2 bridge onto the SDK. Prefer fixing shared hardware behavior in the SDK or bridge layer rather than duplicating logic in higher-level packages.
- Treat `src/agx_arm_ros/src/agx_arm_description` and `src/agx_arm_ros/src/agx_arm_moveit` as the base robot description and MoveIt configuration owned by the upstream stack.
- Treat `src/agx_arm_gzsim` as the simulation overlay. Preserve one-to-one correspondence with the real robot stack whenever practical so simulated launches, frames, controllers, and planning behavior mirror hardware behavior.
- Treat `src/agx_arm_motion` as a higher-level MoveIt client layer. Keep it thin and parameter-driven.

## Build And Test

- `src/pyAgxArm` intentionally contains `COLCON_IGNORE`; keep it excluded from `colcon build` unless the workspace strategy changes.
- For SDK work, use the workspace venv and install or refresh the SDK there instead of trying to build it with colcon.
- Prefer narrow package builds from the workspace root, for example `colcon build --packages-select agx_arm_motion` or `colcon build --packages-select agx_arm_gzsim`.
- After ROS package changes, source `install/setup.bash` before running launches or ROS 2 nodes.
- For SDK changes, prefer targeted `pytest` runs under `src/pyAgxArm/tests`.

## Conventions

- Keep real-arm and simulation interfaces aligned where the package intent is parity, especially joint names, frames, controller names, and planning groups.
- Preserve parameterized support for `arm_type`, `effector_type`, namespaces, and firmware variants. Do not hardcode a single model unless the package is explicitly Piper-only.
- When adding new workspace packages, follow the existing package layout: `README.md`, `package.xml` or `setup.py`, plus `launch/`, `config/`, and package source directories as needed.
- Prefer linking to the existing package READMEs and docs instead of copying large blocks of setup text into new files.

## Primary References

- `README.md`
- `DESIGN.md`
- `TODO.md`
- `ORCHESTRATION.md`
- `src/agx_arm_gzsim/README.md`
- `src/agx_arm_motion/README.md`
- `src/agx_arm_ros/README.md`
- `src/agx_arm_ros/src/agx_arm_moveit/README.md`
- `src/pyAgxArm/README.md`