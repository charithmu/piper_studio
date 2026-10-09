> **Folder layout changed on 2026-10-09.** Read `~/projects/RULES.md` first (storage, GPU, exposure, installs; it wins over anything older in this repo) and the machine map in `~/projects/AGENTS.md`. Use the new paths (`~/projects/{platform,ml_tests,sim,ros,core}/...`); the old `~/projects/dev/...` symlinks disappear around 2026-10-16. Keep your `artifacts.yaml` current (`python3 ~/projects/platform/registry/registry.py check`).

# Piper Studio agent guide

Read README.md, docs/ROADMAP.md, VERSIONS.md and docs/description/DECISIONS.md before changing anything.

## Environment

- ROS 2 Jazzy, Ubuntu 24.04, workstation `atlas`. Always `source scripts/env.sh` first (ROS, then
  `.venv`, then `install/`). It sets `PYTHONNOUSERSITE=1`: `~/.local` contains NumPy 2, which breaks
  ROS Jazzy's compiled Python packages.
- Build with `python -m colcon build --symlink-install` (colcon from the venv, so entry points use
  the venv Python that has pyAgxArm).
- Python deps: edit `env/requirements.in`, then
  `uv pip compile env/requirements.in -o env/requirements.txt --python .venv/bin/python` and
  `uv pip sync env/requirements.txt --python .venv/bin/python`. NumPy stays at the ROS ABI (1.26.x),
  pytest at 7.x (launch_testing), MuJoCo equal to the `mujoco_vendor` libmujoco version.
- No passwordless sudo: give the user exact `apt`/`rosdep` commands; never install system-wide yourself.
- Use a unique `ROS_DOMAIN_ID` for launches and tests so parallel sessions do not cross-talk.

## Ownership

- `external/agx_arm_ros` is the official AgileX stack, **unmodified**. Do not edit it. If a vendor
  bug blocks us, prefer a workaround in our packages; if a patch is unavoidable, propose forking to the
  user first and record the patch in VERSIONS.md.
- Workspace packages (`src/piper_*`) are ours. Keep the number of packages small; add a package only
  for a separate dependency set (simulators, cameras).

## Design rules

- One robot description: `piper_description` includes the official URDF unmodified. Never hand-edit
  copies of geometry/limits; generate simulator models from it. `test_description.py` guards this.
- One controller configuration (`piper_bringup/config/controllers.yaml`) for every backend; backends
  differ only in the ros2_control hardware plugin.
- One command owner per resource; switch controllers explicitly.
- Success means executed (controller/MoveIt result), not planned.
- Real arm: motors not auto-enabled, command controllers start inactive, `command_guard` filters
  commands. Do not weaken these defaults.

## Safety and approvals

- Anything that can move the real arm requires the user physically present and approving that step.
  Start read-only. No firmware changes or parameter writes to the arm without explicit approval.
- Ask before: pushes, installs outside the workspace, deleting user data, network exposure.
- GPU (Isaac) work: check `nvidia-smi` and the machine status boards; use `gpu-run`.

## Working style

- One bounded milestone at a time; report results with evidence (commands, test output).
- Update VERSIONS.md qualification status and `comms/STATUS.md` (append-only, machine-wide
  agent protocol) when a milestone completes.
- Keep docs short; the code and tests are the documentation of record.
