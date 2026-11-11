# The arm tools on a box (ZED Box), and on the workstation

Same code, two ways to run it. Status 2026-11-11: **built and dry-run into a scratch home on Atlas; not yet installed on the box.**

| | Workstation (Atlas) | Box (CAN adapter and arm plugged in there) |
|---|---|---|
| How | the repo: `source scripts/env.sh`, `python tools/real_arm/arm_stop.py ...` (thin shims), or the same names as commands because `.venv` has `piper_tools` editable (`piper-estop`, ...) | wheels only, no repo: `~/piper/venv`, commands in `~/piper/bin` linked from `~/.local/bin` |
| Data (read-outs, saved poses, test results) | `~/data/ml/isaac/piper_studio` (unchanged) | `~/.local/share/piper` |
| Arm config (what is mounted) | `docs/real_arm/arms/` | `~/.config/piper/arms/` |
| Override | `PIPER_DATA_DIR`, `PIPER_ARMS_DIR` | the same |

The e-stop works the same everywhere: `piper-estop` (electronic e-stop, `--disable`, `--reset`), no repo, no config, no bus lock. **It must run on the machine the CAN adapter is plugged into** (for the box: an ssh terminal on the box, command typed, not run).

## Commands
`piper-can-up`, `piper-estop`, `piper-info`, `piper-joints`, `piper-session`, `piper-pose`, `piper-jog`, `piper-zero`, `piper-enable-test`, `piper-gripper-test`, `piper-stream-test`, `piper-realarm-smoke` (wheel `piper_tools`) and `piper-arm` (the cockpit, wheel `piper_core`). Table and what each sends: `src/piper_tools/README.md`, `REAL_ARM.md` section 8.

## Install on the box
1. On Atlas: `scripts/deploy_box.sh` builds `box-bundle/` (wheels `piper_core`, `piper_assets`, `piper_tools`; `pins.txt` = numpy, pyyaml, python-can and pyAgxArm at the commit pinned in `env/requirements.in`; arm config; `install_on_box.sh`). It touches nothing on the box.
2. With the user's approval: `scripts/deploy_box.sh --push` copies it to `~/piper-bundle/` on the box and runs `install_on_box.sh` there: venv with `--system-site-packages` (keeps the system `pyzed`), `PYTHONNOUSERSITE=1` in every wrapper (the box's `~/.local` has NumPy 2), pip installs from PyPI / GitHub, no sudo, nothing started, nothing sent to the arm. Re-run it to upgrade.
3. CAN: the box has two buses, the USB adapter (gs_usb, `1d50:606f`) and the Jetson's own `mttcan`, and **their names can0/can1 change from boot to boot**. Every tool therefore finds the adapter by USB (`piper_core.canbus`, sysfs path under `usbN/`), never by name; `-b NAME` / `$PIPER_CANBUS` override, `$PIPER_CAN_USB_PORT=1-2` pins one adapter if there are several. `piper-can-up` brings the right one up (1 Mbit/s, `--dry-run`, `--down`); it needs sudo, so the user types it. The gs_usb driver on the box was built by hand and is not DKMS, so a kernel update can drop it: keep `~/hold-zedbox-kernel.sh` in force.
4. First look, nothing moves: `piper-info --arm-id piper-01`, `piper-joints --once`. Then, user present, the session routine (`piper-session start` ... `stop`, or `piper-arm`) with `piper-estop` typed in a second terminal first.

## Rules that stay
- Only the user moves the real arm; an agent prepares. One program on the CAN bus at a time (`piper_core.bus_lock`; the e-stop and `piper-joints` are the exceptions by design).
- The slumped pose stays inside `piper_core`; downstream only knows `zero`.
- The arm's hardware config (`piper-01.yaml`) must be edited on both machines when the hardware changes (re-run the deploy).
- Network arm server (`piper.arm_server/1`, spec in `piper_pick/docs/ARM_SERVER_SPEC.md`): not part of this step; it will use the same venv and call `piper-estop`'s code path.
