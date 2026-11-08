"""The Piper arm as data; see README.md. Reads the files in `data/` that tools/release/build_assets.py generated. No ROS."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from importlib import resources
from pathlib import Path

import numpy as np

try:
    from importlib.metadata import version as _version
    __version__ = _version("piper_assets")
except Exception:       # running from a source tree
    __version__ = "0.0.0+source"

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
BASE_LINK = "base_link"
TCP_LINK = "tcp_link"
GRIPPER_LINK = "gripper_base"          # the TCP frame is TCP_OFFSET along +z of this link (+z = approach direction)
TCP_OFFSET = (0.0, 0.0, 0.138)         # m, in GRIPPER_LINK
CAMERAS = (None, "d435")


def data_dir() -> Path:
    path = Path(str(resources.files("piper_assets") / "data"))
    if not (path / "manifest.json").exists():
        raise FileNotFoundError(f"{path} has no generated data: run tools/release/build_assets.py (needs the ROS environment) or install the wheel")
    return path


def manifest() -> dict:
    return json.loads((data_dir() / "manifest.json").read_text())


def _camera(camera) -> str:
    cam = "none" if camera in (None, "none") else camera
    if cam not in ("none", "d435"):
        raise ValueError(f"camera must be None or 'd435', got {camera!r}")
    return cam


def urdf_path(gripper_width: float | None = 0.07, camera: str | None = None) -> Path:
    """Flat URDF with the gripper fixed at `gripper_width` metres (0.0 closed .. 0.07 open); None keeps the moving gripper (mimic fingers).
    0.0 and 0.07 are shipped; other widths are made on first use in the user cache with absolute mesh paths."""
    cam, d = _camera(camera), data_dir()
    if gripper_width is None:
        return d / f"piper_{cam}_open.urdf"
    shipped = d / f"piper_{cam}_w{float(gripper_width)}.urdf"
    if shipped.exists():
        return shipped
    from piper_core.urdf_tools import fixed_gripper
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "piper_assets" / __version__
    out = cache / f"piper_{cam}_w{float(gripper_width):.6g}.urdf"
    if not out.exists():
        text = (d / f"piper_{cam}_open.urdf").read_text()
        text = re.sub(r'filename="(meshes/[^"]+)"', lambda m: f'filename="{d / m.group(1)}"', text)
        cache.mkdir(parents=True, exist_ok=True)
        out.write_text(fixed_gripper(text, float(gripper_width), ""))
    return out


def srdf_path(camera: str | None = None) -> Path:
    return data_dir() / f"piper_{_camera(camera)}.srdf"


@dataclass(frozen=True)
class Limits:
    joints: list
    hard: dict               # {joint: (lower, upper)} rad: the firmware / URDF limits
    planning: dict           # {joint: (lower, upper)} rad: hard shrunk by `margin`; plan inside these
    margin: float
    velocity: float          # rad/s, every arm joint
    acceleration: float      # rad/s^2
    jerk: float              # rad/s^3, DERIVED (see sources)
    effort: dict             # {joint: N.m}, an SDK command range, not a motor rating
    gripper_stroke: float    # m
    max_step: float          # rad per control step the guard accepts
    sources: dict            # {quantity: text}; flags for derived / not-a-rating values in `flags`
    flags: dict

    def as_arrays(self, which: str = "planning"):
        d = getattr(self, which)
        return np.array([d[j][0] for j in self.joints]), np.array([d[j][1] for j in self.joints])


def limits() -> Limits:
    raw = json.loads((data_dir() / "limits.json").read_text())
    pair = lambda block: {j: (v["lower"], v["upper"]) for j, v in block["values"].items()}
    return Limits(
        joints=raw["joints"], hard=pair(raw["position"]), planning=pair(raw["planning_position"]), margin=raw["margin"],
        velocity=raw["velocity"]["value"], acceleration=raw["acceleration"]["value"], jerk=raw["jerk"]["value"],
        effort=raw["effort"]["values"], gripper_stroke=raw["gripper"]["stroke"], max_step=raw["max_step"],
        sources={k: raw[k]["source"] for k in ("position", "planning_position", "velocity", "acceleration", "jerk", "effort", "gripper")},
        flags={k: raw[k]["flag"] for k in ("jerk", "effort")})


def zero_pose() -> list:
    """[joint1 .. joint6] of the pose every session starts and ends at (joints 2 and 3 are held 0.01 rad inside their 0 limit)."""
    z = json.loads((data_dir() / "limits.json").read_text())["zero_pose"]
    return [z[j] for j in JOINT_NAMES]


def scene() -> dict:
    """{'frame': 'base_link', 'objects': [...], 'table': {...}}; the table is a PLACEHOLDER (`measured: false`)."""
    return json.loads((data_dir() / "scene.json").read_text())


def camera_mount_path(name: str = "agilex_mid_stand") -> Path:
    return data_dir() / "camera_mounts" / f"{name}.yaml"
