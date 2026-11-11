# piper_tools

The hands-on tools for the real Piper as installed commands (no ROS; needs `piper_core`, the vendor SDK `pyAgxArm` and python-can).
The same code runs from the repo (`python tools/real_arm/arm_stop.py`, thin shims) and installed on a box (`piper-estop`, ...).

| Command | Does | Moves the arm |
|---|---|---|
| `piper-estop [--disable \| --reset]` | **emergency stop**, no prompts, works beside a running tool (skips the bus lock) | stops it |
| `piper-info [-r] [-c] [--arm-id ID]` | read-out: role, mode, motors, faults, firmware, limits, gripper; saves JSON | no (queries) |
| `piper-joints [--once]` | live joint table, passive (sends nothing) | no |
| `piper-session status\|start\|stop` | clear e-stop, motors on, move to zero / zero, park, e-stop, clear | yes |
| `piper-pose zero\|park\|ready` | move an enabled arm between named poses | yes |
| `piper-jog`, `piper-zero`, `piper-enable-test`, `piper-gripper-test`, `piper-stream-test`, `piper-realarm-smoke` | see `docs/real_arm/REAL_ARM.md` section 8 | yes |
| `piper-arm` (in `piper_core`) | the cockpit console | yes |

Anything that moves the arm needs the user present and approving. Put the e-stop command typed (not run) in a second terminal on the machine the CAN adapter is plugged into.

Paths: data (read-outs, saved poses, test results) in `$PIPER_DATA_DIR`, else `~/data/ml/isaac/piper_studio` if it exists, else `~/.local/share/piper`;
arm config (`<arm_id>.yaml`) in `$PIPER_ARMS_DIR`, else the checkout's `docs/real_arm/arms`, else `~/.config/piper/arms`. See `piper_tools/paths.py`.
Deploying to a box: `docs/real_arm/BOX_DEPLOY.md`.
