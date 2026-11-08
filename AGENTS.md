# Piper Studio agent guide

Machine rules come first: `~/projects/RULES.md` (they win). This file adds only what is specific to this repo.

**Read before changing anything:** `README.md`, `docs/ARCHITECTURE.md` (layers, interfaces), `docs/CLEANUP.md` (the current state and
to-do list: trust it over `docs/archive/`), `docs/INTEGRATION.md` (the contract other repos rely on), `docs/description/DECISIONS.md`.
Before touching the real arm also read `docs/real_arm/REAL_ARM.md` and the arm's file in `docs/real_arm/arms/`.

## Environment

- ROS 2 Jazzy, Ubuntu 24.04, workstation `atlas`. In every shell: `source scripts/session.sh [ROS_DOMAIN_ID]` (ROS, `.venv`, `install/`,
  domain, display; `scripts/env.sh` alone does the first three). `PYTHONNOUSERSITE=1` is set because `~/.local` has NumPy 2, which breaks ROS Jazzy.
- Build: `python -m colcon build --symlink-install` (colcon from the venv, so entry points use the venv Python that has pyAgxArm).
- Python deps: edit `env/requirements.in`, then `uv pip compile env/requirements.in -o env/requirements.txt --python .venv/bin/python` and
  `uv pip sync env/requirements.txt --python .venv/bin/python`. NumPy stays at the ROS ABI (1.26.x), pytest at 7.x, MuJoCo equal to the `mujoco_vendor` libmujoco.
- No passwordless sudo, no installs without approval (RULES.md section 6): root steps go into `sudo-steps.sh` for the user to run.
- Unique `ROS_DOMAIN_ID` per launch or test (Piper range 70-161); Gazebo gets its own partition per domain.
- Windows (RViz, MuJoCo viewer) need a display with GLX: the TurboVNC desktop has none. For checks and screenshots use a private virtual display
  (`Xvfb :96 -screen 0 2560x1440x24 +extension GLX`, `DISPLAY=:96 ... import -window root shot.png`). Isaac and Gazebo cameras need no display.

## Layout and ownership

| Path | What | Who may import it |
|---|---|---|
| `external/agx_arm_ros` | official AgileX stack (submodule), **unmodified** | |
| `src/piper_description` | xacro macros (`piper_arm` generated from the official model, tool frames, camera model + mount, ros2_control), meshes, `robot_description()` | everyone |
| `src/piper_core` | rules without ROS: limits, command guard, named poses | everyone |
| `src/piper_control`, `src/piper_py` | controllers, MoveIt, Servo, guard node; Python client and CLI | everyone |
| `src/piper_sim`, `src/piper_examples`, `isaac/`, `tools/` | simulators, scenes, the integration example, demos, real-arm scripts | this repo only |

Do not edit `external/agx_arm_ros`. If a vendor bug blocks us, work around it in our packages; if a patch is unavoidable, propose forking to the user first and record it in `docs/VERSIONS.md`.

## Design rules

- One robot description. The official URDF/SRDF are never hand-edited: `piper_arm.xacro` and `piper_official.srdf.xacro` are generated and committed;
  after a submodule bump run `python src/piper_description/tools/piper_arm_macro.py generate`, review the diff, and `check` must pass (it is a test).
- Every link, joint and frame carries the prefix (default `piper_`). Code that names joints takes the prefix; the real driver keeps the official names and `command_guard` is the only translator.
- One controller configuration for every backend (`controllers.yaml`, a `${prefix}` template); backends differ only in the ros2_control hardware plugin.
- One command owner per resource; switch controllers explicitly. One implementation of each rule (`piper_core`); interfaces are thin wrappers.
- Success means executed (controller or MoveIt result), not planned. Every session starts and ends at the named pose `zero`; the real arm is parked at `rest` before its motors are disabled.
- A change to a name in the contract (controller, topic, frame, macro parameter) is breaking: record it in `docs/CLEANUP.md` and `docs/INTEGRATION.md`.

## Safety and approvals

- Anything that can move the real arm needs the user physically present and approving that step. Start read-only. No firmware changes or parameter writes without explicit approval.
- Real arm defaults must not be weakened: motors not auto-enabled, explicit speed, command controllers start inactive, `command_guard` filters commands.
- Ask before: pushes, installs outside the workspace, deleting user data, network exposure.
- GPU (Isaac): check `nvidia-smi`, use `gpu-run`, never touch another agent's Isaac process.

## Working style

- One bounded step at a time, with evidence (commands, test output). Run the offline tests after every change
  (`scripts/test_all.sh`) and the launch tests of the backends you touched (`scripts/test_all.sh mock gazebo mujoco`).
- Start long processes with a PID file and stop them by PID (never `pkill -f`): background jobs start with SIGINT ignored, so launch them via
  `python3 -c 'signal.signal(SIGINT, SIG_DFL); os.execvp(...)'` or send SIGTERM, and check for orphans (`ros2 launch` children survive a killed parent).
- Keep `docs/CLEANUP.md` current: tick items only when done and verified, add a log line per pass. Update `docs/VERSIONS.md` when a qualification changes.
- Commit your own files by explicit path (never `git add -A`), on a branch for anything bigger than a fix. Keep docs short; code and tests are the documentation of record.
