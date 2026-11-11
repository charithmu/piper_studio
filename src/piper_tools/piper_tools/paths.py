"""Where the tools keep data and read the arm config. No arm, no SDK, no ROS.

Data (read-outs, saved poses, stream and gripper test results): `$PIPER_DATA_DIR`, else `~/data/ml/isaac/piper_studio` when that folder exists
(the workstation, as before), else `~/.local/share/piper` (a box).
Arm config (`<arm_id>.yaml`, what is mounted on the arm): `$PIPER_ARMS_DIR`, else `docs/real_arm/arms` of the checkout when the code runs from one,
else `~/.config/piper/arms`.
"""
import os
from pathlib import Path

WORKSTATION_DATA = Path.home() / "data/ml/isaac/piper_studio"


def data_dir(*parts) -> Path:
    """The data folder (not created here), with `parts` appended."""
    if os.environ.get("PIPER_DATA_DIR"):
        base = Path(os.environ["PIPER_DATA_DIR"]).expanduser()
    elif WORKSTATION_DATA.is_dir():
        base = WORKSTATION_DATA
    else:
        base = Path(os.environ.get("XDG_DATA_HOME") or Path.home() / ".local/share") / "piper"
    return base.joinpath(*parts)


def arms_dir() -> Path:
    if os.environ.get("PIPER_ARMS_DIR"):
        return Path(os.environ["PIPER_ARMS_DIR"]).expanduser()
    checkout = Path(__file__).resolve().parents[3] / "docs/real_arm/arms"
    if checkout.is_dir():
        return checkout
    return Path(os.environ.get("XDG_CONFIG_HOME") or Path.home() / ".config") / "piper" / "arms"
