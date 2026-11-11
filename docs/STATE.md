# Current state: read this first when starting a new session

Written 2026-11-11 for release **v0.10.1** (branch `main`, remote `origin` = github.com/charithmu/piper_studio). Facts that change are in `CLEANUP.md` (to-do, verification log) and `VERSIONS.md`;
this file is the orientation: where things are, what is verified, how to check you did not break anything, what is open, and the gotchas that cost time.

## What this is
The AgileX Piper arm (standard Piper, AgileX gripper, **ZED X Mini on the user's own bracket, mounted on the real arm since 2026-11-11 and the default everywhere**) as importable parts plus a standalone setup:
description macros, control stack, a ROS-free rules package (`piper_core`), a Python client, simulators (mock, Gazebo, MuJoCo, Isaac Sim), the real arm, and the **cuRobo collision-sphere model of the arm (owned here)**.
It is the stable repo for the Piper: other projects (`piper_pick`, `curobo_piper_study`) are clients of its wheels. Layout and design: `README.md`, `ARCHITECTURE.md`; the contract for other repos: `INTEGRATION.md`.

## Your first ten minutes
1. Read, in this order: this file, `AGENTS.md` (rules for this repo) and `~/projects/RULES.md` (machine rules, they win), `README.md`, `docs/CLEANUP.md` (top: what is open; bottom: the log, newest last), then what your task touches:
   `docs/INTEGRATION.md` (the contract), `docs/CUROBO.md` (the cuRobo model), `docs/real_arm/REAL_ARM.md` + `CHECKLIST.md` + `arms/piper-01.md` (before the real arm), `docs/description/DECISIONS.md` (why things are as they are), `HANDOFF.md` (notes for downstream).
2. `git status` and `git log --oneline | head`: whatever is uncommitted is not yours. `gpu-run --status` and `nvidia-smi`: other agents share the one GPU.
3. Set up a shell and prove the tree is healthy:
```bash
cd ~/projects/ros/piper_studio && source scripts/session.sh 101     # pick a free ROS_DOMAIN_ID (Piper range 70-161); never reuse another agent's
scripts/smoke.sh                  # ~30 s: offline tests, registry, live pages, the cuRobo study (CPU)
scripts/smoke.sh --sims           # ~2 min: + launch tests on mock and MuJoCo and the carrier example
scripts/smoke.sh --all            # ~5 min, needs a quiet GPU (gpu-run --status first): + Gazebo, Isaac Sim, a cuRobo scenario planned and run on MuJoCo, the study's GPU tests
```
All green on 2026-11-11 (`ALL OK`). If a launch hangs, see "Fast DDS shared memory" below before debugging code.

## Where things are
| What | Where |
|---|---|
| description (xacro, meshes, cameras, brackets, the generated `piper_arm`) | `src/piper_description` (`config/camera_mounts/`, `config/curobo/` = the generated cuRobo models) |
| rules without ROS: guard, limits, `RealArm`, `Trajectory`, `piper-arm` cockpit | `src/piper_core` |
| controllers, MoveIt, Servo, the guard node, the real driver (ROS path, parked) | `src/piper_control`, launch in `src/piper_sim` |
| Python client and `piper` CLI | `src/piper_py` |
| wheels for other repos (`piper_core`, `piper_assets`, `piper_mujoco`, `piper_isaac`) | built by `scripts/build_wheels.sh` into `dist/`; assets data by `tools/release/build_assets.py` |
| cuRobo: tuned policy, pipeline, models, optional env | `tools/curobo/` (`policy.yaml`, `build_model.py`), `scripts/curobo_env.sh`, `docs/CUROBO.md` |
| real-arm tools (SDK scripts, no ROS), scenario planner and runner, fixtures | `tools/real_arm/`; **what is mounted on the arm: `docs/real_arm/arms/piper-01.yaml`** |
| web pages | home `tools/home/`, demo `tools/demo/`, model viewer `tools/viewer/`, cuRobo viewer `tools/curobo_viewer/`; registered in `artifacts.yaml` |
| Isaac Sim side (own Python env, outside colcon) | `isaac/`, `scripts/isaac.sh` |
| bulk data (USD, demo runs, scenarios, reports, cuRobo work files) | `~/data/ml/isaac/piper_studio/` (never in the repo) |

**Pages (tailnet only):** home `https://atlas.buri-fence.ts.net:8700/piper-studio/` lists all of them with a live status; demo `/piper-studio/demo/`, model viewer `/piper-studio/viewer/` (`scripts/viewer.sh start`),
cuRobo viewer `/piper-studio/curobo/` (`scripts/curobo_viewer.sh start`). To add a page: entry in `artifacts.yaml`, `python3 ~/projects/platform/registry/registry.py apply --project piper_studio`, `web-pages install`, `web-pages gateway`,
`python tools/home/build_home.py`. Rebuild the demo page from recorded runs: `python tools/demo/build_demo_page.py <demo-dir> ~/data/ml/isaac/piper_studio/site` (runs: `scripts/run_demo.sh`).

## What is verified (2026-11-11; evidence in `CLEANUP.md` log and `arms/piper-01.md`)
- **Simulators:** mock, Gazebo, MuJoCo, Isaac Sim with the same controllers and MoveIt, the simulated ZED X Mini (left lens, 960x600, 110 deg) on the **ZED ROS 2 wrapper's names**: topics `/<prefix>zed/zed_node/rgb/color/rect/image`, `.../rgb/color/rect/camera_info`, `.../depth/depth_registered`,
  frame `<prefix>zed_left_camera_frame_optical` (default prefix `piper_`); ask the library (`Piper.camera_topics()`, `piper_description.camera_topics()`), do not hard-code. The RealSense-style names no longer exist.
- **Real arm, user present:** `RealArm` (start, stop, execute with 25 ms setpoints, abort, gripper), the `piper-arm` cockpit, the SDK tools; a cuRobo scenario planned **with the camera model** ran through the guard on the real arm with the camera mounted: all 7 steps ok, worst joint error 0.32 deg, back at zero;
  a stream test with the camera on joint 1: rms 0.05 deg. Firmware kinematics equal our model (0.09 mm). Crash protection is 0 on every joint (no collision detection in the firmware; the user's decision).
- **cuRobo model:** both models (with the camera 468 spheres, robot only 327) validated by planning 100 random tool poses from zero in a table scene: 92 of 100 each, no limit exceeded, goal error under 5 mm, no self-overlap at zero. Planned with cuRobo and executed in MuJoCo and Isaac Sim (sweep, picks, scenario).
- **Not verified / open facts:** the ZED Box and the real camera driver have never been tried (`docs/real_arm/ZED_BOX.md` is written, not run); the camera pose is nominal (bracket v3), not hand-eye calibrated; bracket and camera masses are estimates; the bracket is covered by only about 71 % by its cuRobo spheres (up to 10 mm outside, `tools/curobo/policy.yaml`, `docs/CUROBO.md`): keep 2 cm extra clearance near the wrist.
- **Real arm, ROS path: experimental, parked (user decision 2026-11-10).** Use `RealArm`. The redesign for later is CLEANUP C14.

## The real arm: rules and the way to run something on it
- **An agent never moves the real arm.** Everything that can move it needs the user present and approving each step. Prepare commands, plans, checklists and analysis; the user runs the hardware steps. Start read-only. No firmware or parameter writes (crash protection, payload) without explicit approval.
- Operator: `scripts/can_up.sh` (sudo), `source scripts/env.sh`, `piper-arm` (keys: `s` start, `z` zero, `r` ready, `o`/`l`/`g` gripper, `e` e-stop, Space hold, `t` end session; each move waits for Enter). One program on the CAN bus at a time.
  **E-stop terminal first, command typed and not run: `python tools/real_arm/arm_stop.py`** (Enter = e-stop; `--reset` clears it). A raised arm lowers slowly under damping.
- cuRobo scenario: `gpu-run <study venv>/bin/python tools/real_arm/curobo_plan_scenario.py` (plans for what `arms/piper-01.yaml` says is mounted), then `python tools/real_arm/run_scenario.py <scenario.json> --dry-run`, then `--backend real`.
  `run_scenario.py` **refuses a scenario planned for other hardware** (with or without the bracket, both directions, also in a dry run). **When the hardware changes, edit `arms/piper-01.yaml`.** The scenario that passed on the arm is stored: `tools/real_arm/fixtures/scenario_pick_25_0_05_zedxm.json`.
- The off pose a powered-off arm rests in ("slumped") is internal to `piper_core`: never command it, never expose it downstream.

## What each kind of change needs
| You changed | Do |
|---|---|
| anything in `src/` | `scripts/smoke.sh` (and `--sims` if launch files, controllers or backends) |
| the description (xacro, meshes, camera, bracket) | `python src/piper_description/tools/piper_arm_macro.py check`; `python tools/release/build_assets.py`; `scripts/isaac.sh build-usd` and `build-usd no_camera`, `scripts/isaac.sh check`; **regenerate the cuRobo model** (`docs/CUROBO.md`: the staleness test fails until you do); `scripts/smoke.sh --all` |
| `tools/curobo/policy.yaml` or the fit | `scripts/curobo_env.sh run tools/curobo/build_model.py` (optional env, GPU), `build_assets.py`, `python -m pytest src/piper_assets` |
| a name in the contract (controller, topic, frame, macro parameter) | breaking: record in `CLEANUP.md` and `INTEGRATION.md`, tell `HANDOFF.md` readers, bump the version |
| real-arm tools | `python -m pytest src/piper_core/test/test_scenario_tools.py`, then a `--dry-run`; the user runs hardware |
| the arm's hardware (camera, bracket, gripper) | `docs/real_arm/arms/piper-01.yaml`, the masses (`docs/description/MASSES.md`), the cuRobo model, `arms/piper-01.md` |

**Release** (the unit downstream projects pin): commit everything; bump the version in the four packages (`src/piper_core/setup.py`, `src/piper_{assets,mujoco,isaac}/pyproject.toml`) and the wheel names in `INTEGRATION.md` and `HANDOFF.md`;
`scripts/build_wheels.sh` from a clean tree (stamps the commit into the wheels); reinstall them in `curobo_piper_study` (its `docs/07_SETUP.md`); `scripts/smoke.sh --all`; then tag `vX.Y.Z` on that commit and push **with the user's approval**. Update the notes for downstream readers.

## Downstream clients
- **`~/projects/ml_tests/experiments/curobo_piper_study`**: a client now (`PiperPlanner.from_assets(camera)` reads `piper_assets`; refuses a stale model). Its venv is separate: run its tools with a clean environment (`env -i HOME=$HOME PATH=... .venv/bin/...`), the ROS environment breaks its pytest. `smoke.sh` covers it.
- **`~/projects/core/piper_pick`**: pinned to the 0.5.0 wheels, has its own planner and cuRobo env. Everything it needs to adopt the Studio model and the camera defaults is in `~/projects/core/piper_pick/handoff/STUDIO_CUROBO_MODEL.md`: its real-arm path must use the camera model, the Isaac `World`, USD and client must all say `camera=False` together while it simulates the robot only. Its agent owns the migration; do not edit that repo without the user.
- The pick bake-off keeps its own stale snapshot of the model: history, leave it.

## Open work, grouped by what it needs (full list: `CLEANUP.md`)
- **Needs the user at the arm:** weigh the bracket and camera with cable (R3.2: 98 g and 150 g are estimates) and put the masses in; the stream test with the camera on joints 2, 3, 5 (R3.4, joint 1 done); hand-eye calibration (R3.6, `docs/real_arm/ZED_BOX.md`, CLEANUP C9);
  simulation vs real image (R3.7); speeds above 44 % (R1.4); the ZED Box driver on the box (R3.5).
- **Needs a decision from the user:** crash protection and payload settings in the arm (0 today; writes to the arm); when `piper_pick` adopts the model; whether to unpark the ROS path on the real arm (C14); pruning `for_reference/` (F1, 5.3 GB); whether the home page should render the documents.
- **Can be done alone:** small cosmetic issues C4 (Gazebo stand/camera colour, MuJoCo drops mesh colours), C6, C7, C11; the cuRobo bracket coverage (a refit with a tighter overhang and more spheres is a trade-off against the 15 mm gap to the fingers: `policy.yaml`, measure with `quality_*.json`);
  help `piper_pick` adopt the model if its agent asks; the next development (ML endpoints: pick and place, Go2 mount, CLEANUP M-items in `ROADMAP.md`).

## Gotchas that cost time
- **Firmware first:** the arm runs the latest supported firmware (S-V1.9-1), calibrated. Firmware-reported values (joint speed 3.0 rad/s, acceleration 5.0, gripper stroke 70 mm, j6 +-170 deg) win over the vendor URDF; differences: `arms/piper-01.md`, `piper_arm_macro.py report`. Masses: `docs/description/MASSES.md`.
- **Processes:** `ros2 launch` children survive a killed parent, and background jobs start with SIGINT ignored. Start with a PID file via `python3 -c 'signal.signal(SIGINT, SIG_DFL); os.execvp(...)'`, stop by PID, then look for orphans of your ROS_DOMAIN_ID. `ros2 daemon stop` per domain afterwards. Never `pkill -f`.
  A waiter such as `until ! pgrep -f "<text>"` matches itself if its own command line contains the text.
- **Fast DDS shared memory:** killed ROS or Isaac processes leave `/dev/shm/fastrtps*` files behind (1400 found on 2026-11-11); stale ones can make a launch hang (the Isaac launch test timed out at 90 s on ROS domain 154 three times, then passed on all backends after cleaning). `scripts/clean_shm.sh` reports, `--yes` removes only what no running process holds. Cause not proven, the fix worked.
- **Shell scripts:** under `set -o pipefail`, `cmd | grep -q` kills `cmd` with SIGPIPE and the pipeline reports failure; capture the output first.
- **GPU:** one RTX 4090 shared with other agents: everything through `gpu-run` / `scripts/isaac.sh`, check `gpu-run --status`, never touch another agent's process. Running Isaac in the repo directory drops `ucode.elf` there (git-ignored, delete it). The USD must be rebuilt after any description or prefix change: `scripts/isaac.sh build-usd [no_camera]`.
- **Environments:** `PYTHONNOUSERSITE=1` (NumPy 2 in `~/.local` breaks ROS Jazzy); the optional cuRobo env (`.venv-curobo`, 6 GB, `scripts/curobo_env.sh`) is separate and never needed to run, control or simulate the arm; the study and `piper_pick` have their own.
- **Display:** TurboVNC (`:2`) has no GLX. For RViz, the MuJoCo viewer/camera and Gazebo GUI use `Xvfb :96 -screen 0 1600x900x24 +extension GLX` and `DISPLAY=:96 import -window root x.png`. Isaac and Gazebo cameras need no display. `scripts/session.sh` sets the VNC display only on Atlas (portable for the ZED Box).
- **Names:** every name has the prefix; the real driver keeps the official ones and only `command_guard` translates. `ros2 launch` rejects empty values and xacro CLI ignores them: `prefix:=none` means no prefix in launch files, `""` in Python/xacro mappings.
- **Generated files:** `piper_arm.xacro` and `piper_official.srdf.xacro` are generated; never hand-edit, run `piper_arm_macro.py generate` after a submodule bump and review the diff. Config yaml/xml with a `${prefix}` token are templates (`render_config`). The wheel data and `config/curobo/` models are generated too (tests compare them with a fresh build).
- **MoveIt:** the SRDF `<robot name>` must equal the URDF's name (`robot_name:=`), and the SRDF needs `camera:=` to know the camera links, or MoveIt silently applies no collision matrix (start state "in collision").
- **Tests:** a launch test shows as one pytest test; run with `-s` to see the inner results. Each uses its own ROS domain (`test_all.sh` does this).
- **Git:** commit by explicit path (a hook blocks `git add -A`); work on a branch for anything bigger than a fix; push and tags only with the user's approval; do not edit `external/agx_arm_ros` (propose a fork first).
- **Machine rules** (`~/projects/RULES.md`) win: storage (bulk data in `~/data/ml`), GPU, exposure (anything beyond a `:8700` gateway route or loopback needs the user), no installs without approval. Claude settings come from `~/.claude/settings.json` (this repo has none).

## Where things live outside the repo
`~/data/ml/isaac/piper_studio/`: generated URDF/USD, demo runs (`demo-YYYYMMDD/`), the demo and home sites, scenarios and run reports (`scenario/`, `scenario/reports/`), stream tests (`stream/`), arm read-outs (`arm_info/`), cuRobo work files (`curobo/`), `verification/` (screenshots and video of full checks).
`for_reference/` (git-ignored): `docs/REFERENCE_REPOS.md`. `third_party/curobo` and `.venv-curobo` (git-ignored): the optional cuRobo environment.
