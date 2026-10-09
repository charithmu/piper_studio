# HANDOFF for thread T1 Piper Studio (self-contained)

Generated 2026-10-08 from `/home/atlasdev/projects/sim/robosim/handoff/` (canonical). Project folder: `/home/atlasdev/projects/ros/piper_studio`.

## 0. READ THIS FIRST: how to use this document
This file is **self-contained**. It was generated from the canonical notes in `/home/atlasdev/projects/sim/robosim/handoff/` (written by another agent, Claude, in the session that installed Isaac Sim on this machine). You are a different agent with no memory of that session; everything you need to start is below, and the files it names can be read directly.
- Section 1 = the working agreement and machine facts (applies to you in full). Section 2 = your brief and task list.
- **Communication override:** wherever the brief mentions `STATUS.md`, `status-*.md`, `handoff/` or "other threads", use your project's `comms/` files instead (section 1.4): `comms/STATUS.md`, `comms/NEEDS_USER.md`, `comms/INBOX.md`, `comms/FOR_OTHERS.md`. Start every session by reading `comms/INBOX.md`.
- Wherever the brief says "skill": that means a Markdown guide plus helper scripts stored in files. Read it as ordinary documentation, e.g. `/home/atlasdev/data/ml/isaac/envs/isaaclab/lib/python3.12/site-packages/isaacsim/skills/<skill-name>/SKILL.md`. Ignore MCP/Claude-specific instructions.
- Paths starting with `~` mean `/home/atlasdev`. Relative paths in the brief were rewritten to absolute ones.
- Your first message to the user should say what you read, summarise your understanding of the mission in five lines, list the questions you have (also write them to `comms/NEEDS_USER.md`), and propose the first step with an estimated time. Do not start changing things before he agrees.

# 1. Working agreement and machine guide

Written for an agent that has NO prior context (any model, any tool). It does not rely on any vendor-specific skills, MCP servers or tools. If a step below needs something you do not have (for example write access outside your project folder), say so to the user instead of working around it.

## 1. The situation in six lines
- The user (name `charithmu`, Linux account `atlasdev`) works on a workstation called **atlas** (Ubuntu 24.04, RTX 4090 24 GB, 91 GB RAM, 32 cores), reachable remotely only through **Tailscale** (`atlas.buri-fence.ts.net`, tailnet IP 100.107.35.118). He talks to you in a chat; he can run commands for you in a terminal and paste the output back.
- He owns a **Unitree Go2 EDU with Jetson Orin NX** (older unit) and an **AgileX Piper arm (standard, normal gripper)**. The long-term goal is a Go2 carrying the Piper, controlled by agents, trained in simulation and run on the real robots.
- Simulation tools installed here: **Isaac Sim 6.1 + Isaac Lab 3.0** (NVIDIA), MuJoCo, Gazebo Harmonic, **ROS 2 Jazzy** (`/opt/ros/jazzy`). Everything about the Isaac installation is documented in `/home/atlasdev/projects/sim/robosim/docs/SETUP_REFERENCE.md`.
- The user is an **expert in robotics** (ROS 2, manipulators, hardware, control, CAD, 3D printing): do not explain basic robotics, talk to him as a peer and be precise and technical. He is **new only to NVIDIA Isaac Sim / Isaac Lab and to reinforcement learning / sim-to-real**: for those, briefly explain the concept the first time you use it and how it works in Isaac, with numbers and pictures.
- Several agents work on separate threads in parallel; they cannot talk to each other. They communicate through **files** and through the user (section 4).
- Work happens in steps; after every step you report to the user and wait for his direction when a decision is needed.

## 2. The machine (facts you need)
| Topic | Rule / fact |
|---|---|
| Users, sudo | You run as `atlasdev` (the user's own account). **There is no passwordless sudo.** When root is needed, give the user the exact command, say what it does and why, one command at a time, and wait for the output. Do not write scripts for him to run unless he asks. |
| Python | Use **`uv`** (`~/.local/bin/uv`): `uv venv`, `uv pip install`, `uv run`. One environment per project. Do not use bare `pip install` on the system Python. conda (`~/miniforge3`) only if a dependency is not available through uv. |
| Big files | Nothing larger than ~1 GB under `~/projects`. Big files (datasets, models, checkpoints, recordings, envs) go to `~/data/ml/<area>/` (own NVMe) or `~/extra`. Put an owner note `SOURCE.md` in every new folder under `~/data/ml/`. |
| GPU | One GPU, shared with a local LLM router that loads models on demand. Run GPU jobs with **`gpu-run <command>`** (it frees the GPU first). Check `nvidia-smi` before heavy jobs and free the GPU when done. Two GPU jobs at once slow each other: check the status board first. |
| Knowledge base | `/home/atlasdev/projects/platform/README.md` is the single source of truth for how this machine is set up; **read it**. `~/projects/RULES.md` has the machine rules. When you add, change or remove anything on the machine, add a dated line to the change log at the end of the "Change log" heading in that README. Experiments follow `/home/atlasdev/projects/ml_tests/experiments/README.md` (own folder, README, CONCLUSIONS.md when done, kept not deleted). |
| Network and security | The tailnet is trusted, the LAN is closed by the firewall (ufw: default deny, allow everything on `tailscale0`). You MAY expose web pages/services on the tailnet (default home: a route under the gateway `https://atlas.buri-fence.ts.net:8700/<experiment>/<page>/` via `/home/atlasdev/projects/platform/webpages/pages.json` and the `web-pages install` command) but you MUST report it. You MUST ask the user first (with an exact command for him to run) before: any firewall change, `tailscale funnel`, anything reachable from the LAN or bound to 0.0.0.0 without firewall cover, Tailscale/SSH/auth settings, system services, credentials. Never print, commit or upload secrets (`~/.config/nvidia/api.env` holds an API key). |
| Installs | Report every install: what, where, why. Python packages inside your project's own environment are fine. System-wide packages: ask first, then give the user the exact command. Never touch other projects' environments. |
| Real robots | **Anything that can move a robot needs the user physically present and approving that exact step.** Read-only first. Low gains and speeds. E-stop/power switch within reach (the Go2 on a harness or hanging for first tests). Never flash firmware, upgrade or reconfigure a robot or its computer without explicit approval; back up configs first. |
| Shell pitfalls here | Do not use `pkill -f <text>` / `pgrep -f <text>` where the command line itself contains the text (it kills/matches your own shell): use PID files. Avoid `sleep` chains longer than about a minute; use a bounded wait loop with a timeout. Long jobs: run in the background with a log file and poll. |
| Paths moved | Old paths such as `/home/atlasdev/projects/ros/...` are stale; real: `/home/atlasdev/projects/ros/...`. |
| Tools on PATH | `gpu-run`, `web-pages`, `add-model`, `comfyui`, `vllm-serve` (machine tools, see ml_tests README). Isaac helpers live in `/home/atlasdev/projects/sim/robosim/scripts/` (`isaacsim-live.sh`, `publish-video.sh`, ...). |

## 3. How to work with the user
1. **Step by step.** Break work into small steps with a stated estimated time. After each step report: what you did, the evidence (command output, numbers, files, screenshots), what is next, what you need from him. Do not run for hours unattended; one bounded job at a time, then stop and report.
2. **Verify before you claim.** Show output that proves a result. If you did not verify something, say "unverified". Read changelogs and commit messages before upgrading anything (many upstream repos are in Chinese and change daily).
3. **Explain only what he does not know.** Robotics, ROS 2, kinematics, control, CAD and electronics need no explanation. The first time you use an Isaac Sim / Isaac Lab or RL / sim-to-real concept (USD stages, Isaac Lab environments and managers, PPO, reward terms, domain randomization, actuator identification in simulation, ...), give two or three sentences on what it is and how it is used here.
4. **Ask when it matters, decide when it does not.** Choose sensible defaults for small things and tell him; ask for decisions that change cost, safety or direction.
5. **Do not assume your own memory is current.** Today is early October 2026; tools and repos here are newer than most model training data. Check the actual files and upstream repos.
6. **Keep the user's work safe.** Back up before changing existing repositories (uncommitted changes exist). Do not push to his GitHub forks, delete his files or overwrite his configs without asking.
7. **Show results.** Save pictures/videos; the playlist `https://atlas.buri-fence.ts.net:8700/isaac-sim/videos/` takes videos/images via `/home/atlasdev/projects/sim/robosim/scripts/publish-video.sh <file> "<title>" "<description>"`. Tell him the file path so he can open it.

## 4. Communication through files (no direct agent-to-agent channel)
Each project folder has a `comms/` directory. Use plain Markdown, append-only, newest at the bottom, UTC or local time stated.
| File | Who writes | Content |
|---|---|---|
| `comms/STATUS.md` | you | One block per finished step: `## YYYY-MM-DD HH:MM step <n>: <title>` then 3-10 lines: done / evidence (paths) / next / blockers. Update it BEFORE you tell the user a step is done. |
| `comms/NEEDS_USER.md` | you | Questions and exact commands for the user. Each item: number, the question or the command, why it is needed, what output you want pasted back. Mark answered items `[answered]` when he replies. |
| `comms/INBOX.md` | the user or another agent (copied by the user) | Messages TO you. Read it at the start of every work session and whenever the user says "check the inbox". Acknowledge each item in `STATUS.md`. |
| `comms/FOR_OTHERS.md` | you | Facts other threads need (e.g. final robot description path, measured values, decisions). The user or another agent copies them to their `INBOX.md`. |
Other threads' project folders: Piper Studio `/home/atlasdev/projects/ros/piper_studio/comms/`, Go2 `/home/atlasdev/projects/ros/go2_studio/comms/`, Isaac/integration/RL (Claude session) `/home/atlasdev/projects/sim/robosim/handoff/` (`STATUS.md`). If you can read the other folders, read their `STATUS.md`/`FOR_OTHERS.md` at the start of a session; do not edit their files except `INBOX.md` of the thread you are messaging (append only, label the sender).

## 5. Final answers to the user
Short and concrete: result first, then evidence, then the next step, then questions. Use full absolute paths for files you mention. No unexplained jargon.


---

# 2. Your brief

# Brief for the Piper Studio agent (T1)

The user, rules and coordination are in section 1 above. This brief is your whole task list.

## 1. Mission
Make **Piper Studio** (`~/projects/ros/piper_studio`) a **deterministic, up-to-date, working setup** for the user's **standard AgileX Piper with the normal gripper**, usable from **ROS 2 (Jazzy)**, **Gazebo**, **MuJoCo** and **Isaac Sim 6.1**, with one trustworthy robot description behind all of them. The user already did a lot of work there, but it may not use the best methods and may be outdated, so: audit first, then improve, and report before big changes. This arm will later be mounted on a Go2 (thread T3); your job is the arm alone.

## 2. What exists now (verified 2026-10-08)
- Workspace: `~/projects/ros/piper_studio`, ROS 2 **Jazzy** (`/opt/ros/jazzy`), workspace venv `.venv/`, launcher `scripts/piper_studio.sh`. Docs there: `AGENTS.md`, `README.md`, `DESIGN.md`, `PLAN.md`, `ImplementationPlan.md`, `ORCHESTRATION.md`, `TODO.md`, `PACKAGE_GRAPH.md`. Packages: `agx_arm_ros` (fork, "upstream-frozen"), `pyAgxArm` (fork, `COLCON_IGNORE`), `agx_arm_gzsim` (Gazebo Harmonic), `agx_arm_mjsim` (MuJoCo), `agx_arm_motion` (MoveItPy layer), `agx_arm_wristcam`, `agx_arm_detect`, `agx_arm_graspgen`, `agx_arm_manipulation`, `agx_arm_workspace`, `support`.
- It is a git superrepo with submodules (the user's forks `github.com/charithmu/...`). **The working tree is dirty** (modified `AGENTS.md`, `DESIGN.md`, `README.md`, `.gitignore`, `.gitmodules`, `.github/instructions/*`; the `agx_arm_wristcam` submodule is not initialised). **Do not clobber it:** make a backup first (`git status`, `git diff > ...`, a tarball of the working tree outside `~/projects`), and ask the user before any push/force/reset. Pushing to his GitHub forks needs his explicit approval.
- Pins: `agx_arm_ros` @ `125ebe2` (2026-05-11), `pyAgxArm` @ `cd6f878` (2026-03-16), nested `agx_arm_urdf` older than upstream. **Upstream drift measured today:** `agilexrobotics/agx_arm_ros` HEAD `e4ccec1` (2026-10-08) is **19 commits ahead** of the pin; `pyAgxArm` HEAD `841a625` (2026-09-17) is **76 commits ahead**; `agx_arm_urdf` HEAD `983788b` (2026-10-08) changes daily (recent commits are mostly about the Nero arm and shared gripper meshes). Commit messages are in Chinese.
- Known stale content: Piper Studio's `AGENTS.md` still contains the pre-move path `/home/atlasdev/projects/ros/piper_studio` (real: `~/projects/ros/piper_studio`).
- Isaac Sim side (installed by T0, see `/home/atlasdev/projects/sim/robosim/docs/SETUP_REFERENCE.md`): official AgileX USD stages for Piper work in Isaac Sim 6.1 (checked with `/home/atlasdev/projects/sim/robosim/robots/piper/check_official_usd.py`: 6 arm + 2 gripper joints reach targets within 3 mrad). Isaac Sim 6.1 ships a stock `isaacsim.ros2.control` extension (Humble/Jazzy).

## 3. URDF findings (run `python3 ~/projects/sim/robosim/robots/piper/compare_urdfs.py`)
Three sources of the Piper description exist:
| Source | Notes |
|---|---|
| A. Piper Studio's pinned nested `agx_arm_urdf` (May) | arm-only `piper_description.urdf` + gripper xacro; total link mass 4.160 kg (arm) |
| B. `agilexrobotics/agx_arm_urdf` latest (updated daily) | identical kinematics to A; only `link6` mass 0.0070 -> 0.0061 kg (total 4.159 kg); gripper in `piper_with_gripper_description.xacro`: `gripper_base` 0.45 kg + two fingers 0.025 kg each |
| C. `agilexrobotics/piper_isaac_sim` description (Dec 2025, the one behind the Isaac USDs) | includes the gripper in the URDF (total 4.67 kg); **joint1 upper limit 2.168 rad vs 2.618 rad in A/B**; joint6 velocity limit 3 vs 5 rad/s; other differences are rounding (e.g. joint2 upper 3.14 vs pi) |
Open questions for you to resolve (with the real arm, SDK and AgileX docs): which joint1 limit is right? What are the real joint velocity/effort limits (all URDFs say effort 100 Nm: a placeholder; the IIT lab uses 45.4 Nm for their identified Piper-L)? Are link masses/inertias measured or CAD estimates? Is the gripper stroke in the URDF the real one? Decide ONE source of truth (suggested: B + gripper xacro, with a documented patch list) and generate everything else from it.
Related: the **standard Piper vs Piper-L** (Piper-L has longer forearm links, joint3->4 0.338 vs 0.285 m, joint4->5 0.322 vs 0.252 m). The IIT authors' Go2+Piper policy is trained on the **L**; we have the standard arm.

## 4. Task list (in this order; report after each)
**0. Baseline and backup.** Read all docs in the workspace. Back up the dirty tree. Build and run the existing workflows (`scripts/piper_studio.sh model-viz`, Gazebo sim + MoveIt, MuJoCo sim, and with the real arm only if the user is present and agrees) and write what works/what fails in `log/` (the workspace has `log/checkpoints.md` and an orchestration protocol; follow it). Fix the stale paths in `AGENTS.md`/docs.
**1. Sync with upstream, carefully.** On a branch, merge/rebase the forks against `agilexrobotics/agx_arm_ros` (19 commits), `pyAgxArm` (76 commits) and the nested `agx_arm_urdf`. Read the changelogs (translate the Chinese messages), note firmware-related changes (pyAgxArm is firmware-aware; its latest commit fixes the acceleration unit of `set_joint_acc_limits` from firmware v1.20 on, for the Nero arm: check whether the Piper is affected), check the real arm's firmware version with the user present. The workspace says `agx_arm_ros`/`pyAgxArm` are "upstream-frozen": keep local patches minimal, documented and re-appliable; propose a patch/branch strategy to the user before merging.
**2. One robot description.** Settle the URDF questions above, produce the **standard Piper + normal gripper** description as xacro with the real limits, and **generate** from it: the ROS description for MoveIt, the MJCF for MuJoCo (MuJoCo's URDF compiler or `mujoco_menagerie`-style fixes), and the Isaac USD (Isaac Sim 6.1 URDF importer, `isaacsim.asset.importer.urdf`; compare with AgileX's official USD). Script it (no manual steps), pin the versions, and add a check that prints mass/COM/limits for each generated file.
**3. Deterministic setup.** A `bootstrap` script that rebuilds the whole workspace from a clean clone (ROS deps via rosdep, venv with pinned versions, submodule SHAs), a `VERSIONS.md` (exact SHAs, ROS/Python/package versions, Isaac Sim version), and a `selftest` script that runs every backend headless and prints PASS/FAIL. Document it in the workspace README.
**4. Backend parity.** Gazebo (`agx_arm_gzsim`), MuJoCo (`agx_arm_mjsim`), Isaac Sim (use the `isaac-sim-remote` skill and the stock ROS 2 control path; start the sim with `/home/atlasdev/projects/sim/robosim/scripts/isaacsim-live.sh headless|stream`, read `/home/atlasdev/projects/sim/robosim/docs/SETUP_REFERENCE.md`; GPU rule applies). Same controller names/topics/frames as the real arm. Parity test: the same joint trajectory on every backend and (when allowed) on the real arm; compare end-effector pose; report max error. Also make sure MoveIt plans identically in each.
**5. Review current best practice.** Search for newer information than the workspace's design (MoveIt 2 on Jazzy, ros2_control, Pinocchio/Pink or cuMotion for IK/planning, Isaac Sim 6.1 ROS 2 control, AgileX's own newer examples, `piper-ros2-dls` PD+G controller, whatever AgileX added in the 19+76 commits). Report which improvements are worth adopting and why BEFORE re-architecting.
**6. Gains and control modes.** The official HAL scales Kp/Kd internally for joints 1-3; IIT's `piper-ros2-dls` (`ros2_ws/src/dls2_piper_bridge/.../piper_hal.py`, `agx_arm_pd_g_controller`) identified and inverts that scaling so simulated and real gains match. Check the current `agx_arm_ctrl`/`pyAgxArm` behaviour, document the true mapping from commanded gains to behaviour, and expose it to the Isaac/MuJoCo models. This is required later for RL deployment (T4).
**7. Wrist camera mount (user will 3D print).** A camera is already mounted on the Piper but without a proper bracket/CAD. Use the **standard mounts shipped for Isaac Sim / AgileX**: `~/data/ml/isaac/repos/piper_isaac_sim/piper_description/meshes/dae/{realsense_mid_stand.dae, realsense_mid_stand_v2.dae, camera_v3.dae}` (also `piper_l_description`, `piper_h_description` variants) and the camera descriptions in `realsense2_description` (D435/D435i xacro with the origin at the bottom screw mount; meshes d405/d415/d435/d455). First **ask the user which camera model he has** (Piper Studio's `agx_arm_wristcam` knows RealSense D435, Orbbec Astra U3, OAK-D Max). Then: check licences, convert the chosen mount mesh to a printable **STL/STEP** (e.g. with Blender/trimesh/FreeCAD; scale-check against calipers and the camera's real dimensions; the DAE visual mesh may need cleanup), put it with README (print orientation, material, screws) under `piper_studio/.../mounts/`, wire the TF/URDF mount into `agx_arm_wristcam`, and show the user pictures of the mount on the arm in RViz and in Isaac. The user prints and installs it.
**8. Deliverables.** Updated docs, `VERSIONS.md`, bootstrap and selftest scripts, parity results, a decision list for the user, and a `status-piper.md` entry in this folder plus `STATUS.md` lines. Add a short note for T3 (integration): the final Piper xacro path, its limits, masses and the mounting frame name to attach to the Go2.

## 5. Constraints
- Real arm: user present; CAN setup (`sudo ip link ...`) needs sudo, so give the user the exact command and why; start with read-only status; slow, small, low-speed moves; know where the e-stop/power is. No firmware changes.
- Do not break the user's working workflows: keep the old ones runnable until the new ones pass the selftest.
- Isaac work happens through the shared env (`~/data/ml/isaac/envs/isaaclab`) and `robosim` scripts; do not install into that env (ask T0/the user; use a venv of your own for experiments).
- Report every new network exposure (e.g. an RViz web page) and every install.
