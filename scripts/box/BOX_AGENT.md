# Piper Studio on the ZED Box: guide for the agent on the box

You are on the ZED Box (Jetson, Ubuntu 24.04, the Piper arm on USB-CAN, the ZED X Mini on GMSL2). Piper Studio was built and tested on the workstation `atlas`; this folder is what it sent you.
Rules of the machine win: no sudo of your own (give the user the exact command), no installs without the user's yes, the arm moves only with the user at it and approving, `piper-estop` is typed by the user.

## What is where (all under ~/piper)
| Path | What |
|---|---|
| `venv/`, `bin/piper-*`, `env.sh` | the arm tools as commands (wheels `piper_core`, `piper_assets`, `piper_tools`); `source ~/piper/env.sh` |
| `docs/` | Studio's documents: read `STATE.md` first, then `INTEGRATION.md`, `ARCHITECTURE.md`, `real_arm/` (REAL_ARM, CHECKLIST, ZED_BOX, BOX_DEPLOY, `arms/`), `VERSIONS.md`, `CLEANUP.md` |
| `src/piper_studio/` | the committed source of the repo (no `external/` submodule, no generated data). Read-only reference; edit nothing here, changes belong on atlas |
| `ros/zed_camera.launch.py` | the stand-alone ZED wrapper launch (frames and topics as the Studio description names them) |
| `~/.config/piper/arms/piper-01.yaml` | what is mounted on the arm (camera, bracket) |
| `~/.local/share/piper` | read-outs, saved poses, test results |

## Facts you must not get wrong
- The camera pose is the first HAND-EYE calibration (bracket `zedxm_bracket_v4`, 2026-11-12, board spread 4.1 mm rms; the solver's own verdict is "not acceptable", provisional). The CAD pose is `zedxm_bracket_v3`.
- The ZED's own `camera_info` (factory intrinsics) is the truth; the simulators' numbers are datasheet values.
- Pick the CAN adapter by USB (`piper_core.canbus`), never by name (can0/can1 swap between boots). `piper-can-up` needs the user's sudo.
- The slumped pose is internal to `piper_core`; downstream only knows `zero`. Sessions start and end at `zero` (`piper-session start|stop`).
- `PYTHONNOUSERSITE=1` always (the box's `~/.local` has NumPy 2; ROS Jazzy needs 1.26). Piper Pick's calibration environment (OpenCV 4, NumPy 2) is separate: do not mix them.
- One program on the CAN bus at a time; the e-stop must run on this machine.

## ROS 2 workspace (only if the user wants ROS on the box)
`~/piper/ros_setup_on_box.sh` does it step by step and stops at the first thing it cannot do: needs `/opt/ros/jazzy` (apt, sudo: give the user the commands), clones `external/agx_arm_ros` at the pinned commit from GitHub, makes `~/piper/ws/.venv` (`--system-site-packages`, `env/requirements.txt`), builds with colcon. Untested on this box. Isaac Sim, cuRobo and planning stay on atlas.
For just the camera you do not need the workspace: `ros2 launch ~/piper/ros/zed_camera.launch.py` (needs the Stereolabs `zed_wrapper`).
