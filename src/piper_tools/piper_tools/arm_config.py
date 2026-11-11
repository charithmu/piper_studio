"""What is physically on a real arm: docs/real_arm/arms/<arm_id>.yaml (the one place the scenario tools read it from). No ROS, no arm, no SDK."""
from pathlib import Path

import yaml

from piper_tools.paths import arms_dir

DEFAULT_ARM = "piper-01"


def load(arm_id: str = DEFAULT_ARM, path=None) -> dict:
    p = Path(path) if path else arms_dir() / f"{arm_id}.yaml"
    if not p.exists():
        raise SystemExit(f"no arm config {p}: it says what is mounted on the arm (camera: none | zedxm); copy docs/real_arm/arms/piper-01.yaml (on a box: ~/.config/piper/arms/ or $PIPER_ARMS_DIR)")
    cfg = yaml.safe_load(p.read_text())
    cam = cfg.get("camera")
    if cam in (None, "none", False):
        cfg["camera"] = None
    elif cam != "zedxm":
        raise SystemExit(f"{p}: camera must be none or zedxm, got {cam!r}")
    return cfg


def camera(arm_id: str = DEFAULT_ARM, path=None):
    """None (the robot only) or "zedxm" (the ZED X Mini on its bracket) as mounted on the arm."""
    return load(arm_id, path)["camera"]
