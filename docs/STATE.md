# Current state: read this first when starting a new session

Orientation from 2026-11-08, updated for v0.7.0 on 2026-11-10 (branch `main`, remote `origin` = github.com/charithmu/piper_studio).
Facts that change are in `CLEANUP.md` (to-do, verification log) and `VERSIONS.md` (qualified versions); this file is the orientation and the hard-won gotchas.

## What this is
The AgileX Piper arm (standard Piper, AgileX gripper) as importable parts plus a standalone setup: description macros, control stack, a ROS-free rules package, a Python
client, simulators (mock, Gazebo, MuJoCo, Isaac Sim) and the real arm. Layout and design: `README.md`, `ARCHITECTURE.md`; contract for other repos: `INTEGRATION.md`.

## Start in five lines
```bash
cd ~/projects/ros/piper_studio && source scripts/session.sh 101     # pick a free ROS_DOMAIN_ID (Piper range 70-161)
scripts/test_all.sh                                                  # offline tests + generated-macro check, ~5 s
scripts/test_all.sh mock gazebo mujoco                               # + launch tests and the carrier example, ~2 min
ros2 launch piper_sim piper.launch.py backend:=mujoco camera:=d435   # run something
git log --oneline | head; sed -n 1,60p docs/CLEANUP.md              # what was done and what is open
```

## What works (verified 2026-11-10, details in CLEANUP "Verification record" and the hardware log in `docs/real_arm/arms/piper-01.md`)
**Simulators:** mock, Gazebo (also its GUI on a virtual display), MuJoCo (viewer and camera), Isaac Sim (GPU, camera, video) with the same controllers and MoveIt; simulated D435 wrist camera on the RealSense topic names; prefix `piper_` on every name; carrier example (own controller manager, prefix `arm_`).
**Real arm, on the arm with the user present:** `RealArm` (`piper_core`: start, stop, execute with 25 ms setpoints, abort, gripper), the `piper-arm` console, the SDK tools in `tools/real_arm/`, a cuRobo pick-like scenario in simulation; firmware kinematics equal our model (0.09 mm).
**Real arm, ROS path: experimental, parked (user decision 2026-11-10).** `piper_real_driver` (our node on `RealArm`) starts, stops and does small moves on the arm, but large MoveIt moves failed and were jerky and the gripper action misreports a stall (hardware log in `arms/piper-01.md`). Use `RealArm`. The redesign for later is CLEANUP C14; the scenario through ROS runs on the mock backend only.

## Using the real arm (details: `docs/real_arm/REAL_ARM.md`, `docs/FINDINGS.md`, `docs/INTEGRATION.md`)
- Operator: `scripts/can_up.sh` (sudo), `source scripts/env.sh`, `piper-arm` (status and one key per action; `e` is the emergency stop). One program drives the arm at a time (bus lock).
- Code: `from piper_core import RealArm` (`start()`, `execute(traj)`, `set_gripper()`, `status()`, `stop()`); `execute` defaults to `lookahead="auto"` (the arm trails a streamed setpoint by about 35 ms + 73 ms per rad/s). The off pose a powered-off arm rests in is internal to this layer: never exposed, never commanded.
- Scenario test (cuRobo plan, any backend): `gpu-run <cuRobo study venv>/bin/python tools/real_arm/curobo_plan_scenario.py`, then `python tools/real_arm/run_scenario.py <scenario.json> --backend mujoco|real|ros`; the stored plan and the dry run are tested without GPU or arm.

## Packages for other repos
`scripts/build_wheels.sh` builds `piper_core`, `piper_assets`, `piper_mujoco` and `piper_isaac` wheels (no ROS to install them; `piper_isaac` needs the USD of `scripts/isaac.sh build-usd`); contract and usage: `INTEGRATION.md` section 0, log: `CLEANUP.md` G12.

## Open work, grouped by what it needs (full list: CLEANUP.md)
- Needs the user at the arm: the gripper timing and slip (R2, `tools/real_arm/gripper_test.py`), speeds above 44 % and a planned Cartesian path on the arm (R1.4-R1.5), measuring the stand mount (C5), camera calibration (C9), crash protection and payload settings.
- Can be done alone: small issues C4 (stand white in Gazebo, MuJoCo drops colours), C6, C7, C11; pruning `for_reference/` (F1); the next development (ML endpoints: pick and place, Go2 mount).

## Gotchas that cost time
- **Firmware first (2026-11-08):** the arm runs the latest supported firmware (S-V1.9-1), calibrated. Firmware-reported values (joint speed 3.0 rad/s, acceleration 5.0, gripper stroke 70 mm) win over the vendor URDF; the differences are in `docs/real_arm/arms/piper-01.md` and `piper_arm_macro.py report`. Masses: `docs/description/MASSES.md`.
- **Processes:** `ros2 launch` children survive a killed parent, and background jobs start with SIGINT ignored. Start with a PID file via `python3 -c 'signal.signal(SIGINT, SIG_DFL); os.execvp(...)'`, stop by PID, then look for orphans of your ROS_DOMAIN_ID. `ros2 daemon stop` per domain afterwards. Never `pkill -f`.
- **Display:** TurboVNC (`:2`) has no GLX. For RViz, the MuJoCo viewer/camera and Gazebo GUI use `Xvfb :96 -screen 0 1600x900x24 +extension GLX` and `DISPLAY=:96 import -window root x.png`. Isaac and Gazebo cameras need no display.
- **GPU:** one RTX 4090 shared with other agents (an Isaac testbed may be running): Isaac only through `gpu-run` / `scripts/isaac.sh`, never touch another agent's process. Running Isaac in the repo directory drops `ucode.elf` there (git-ignored, delete it). The USD must be rebuilt after any description or prefix change: `scripts/isaac.sh build-usd [d435]`.
- **Names:** every name has the prefix; the real driver keeps the official ones and only `command_guard` translates. `ros2 launch` rejects empty values and xacro CLI ignores them: `prefix:=none` means no prefix in launch files, `""` in Python/xacro mappings.
- **Generated files:** `piper_arm.xacro` and `piper_official.srdf.xacro` are generated; never hand-edit, run `piper_arm_macro.py generate` after a submodule bump and review the diff. Config yaml/xml with a `${prefix}` token are templates (`render_config`).
- **MoveIt:** the SRDF `<robot name>` must equal the URDF's name (`robot_name:=`), and the SRDF needs `camera:=` to know the camera links, or MoveIt silently applies no collision matrix (start state "in collision").
- **Tests:** a launch test shows as one pytest test; run with `-s` to see the inner results. Each uses its own ROS domain (`test_all.sh` does this).
- **Git:** commit by explicit path (a hook blocks `git add -A`); work on a branch for anything bigger than a fix; push and tags only with the user's approval.
- **Machine rules** (`~/projects/RULES.md`) win: storage (bulk data in `~/data/ml`), GPU, exposure, no installs without approval. Claude settings come from `~/.claude/settings.json` (this repo has none).

## Where things live outside the repo
`~/data/ml/isaac/piper_studio/`: generated URDF/USD, demo runs, `verification/2026-11-08/` (screenshots and Isaac video of the last full check). `for_reference/` (git-ignored): `docs/REFERENCE_REPOS.md`.
