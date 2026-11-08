"""Make the ROS-free packages importable from their source folders when pytest runs from the repo root (they are wheels, not colcon packages)."""

import sys
from pathlib import Path

for name in ("piper_core", "piper_assets", "piper_mujoco", "piper_isaac"):
    path = str(Path(__file__).resolve().parent / "src" / name)
    if path not in sys.path:
        sys.path.insert(0, path)
