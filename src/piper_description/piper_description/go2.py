"""Go2 carrying the Piper: the mount, the stand pose and the dog's collision cover for a planner. Pure Python (numpy, yaml), no ROS.

Nothing is written twice. The numbers that are not in the Go2's own description (stand pose, plate and arm mount) are in `config/go2/go2_setup.yaml`; the xacro reads the same file.
The dog's cover is GENERATED from the Go2's meshes by `tools/go2_cover/build_dog_cover.py` (config/go2/dog_cover.json: one box around the whole dog at the stand pose, in the Go2 `base` frame, with a report);
this module only moves it into the frame a planner uses. The mount is an ESTIMATE until the dock plate is measured (2026-11-12).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

# ---- the three setups -------------------------------------------------------------------------------------------------------------------------------
# One selector for viewers, tools and assets. `go2` is the default of the viewers, the export tool and the `setup=` argument of piper_assets; the older
# `camera=` arguments (robot only / with camera, on a table) keep their meaning, because a real arm on a table is not on a dog.
DEFAULT_SETUP = "go2"
SETUPS = {
    "arm": {"title": "Robot only", "camera": "none", "go2": False},
    "arm_cam": {"title": "Robot with the ZED X Mini", "camera": "zedxm", "go2": False},
    "go2": {"title": "Full setup on the Go2 (nominal mount)", "camera": "zedxm", "go2": True},
}


def check_setup(setup: str | None) -> str:
    setup = setup or DEFAULT_SETUP
    if setup not in SETUPS:
        raise ValueError(f"setup must be one of {sorted(SETUPS)}, got {setup!r}")
    return setup


def config_dir() -> Path:
    from piper_description import xacro_file
    return xacro_file().parent.parent / "config" / "go2"


def setup_config() -> dict:
    """config/go2/go2_setup.yaml: stand_pose, stand_source, mount (plate_xyz, plate_size, mount_rpy, nominal)."""
    return yaml.safe_load((config_dir() / "go2_setup.yaml").read_text())


def go2_stand_pose() -> dict:
    """Joint angles of the standing Go2 by the Go2 URDF's joint names (config/go2/go2_setup.yaml; the posture the cover is generated for)."""
    p = setup_config()["stand_pose"]
    return {f"{leg}_{j}_joint": p[j] for leg in ("FL", "FR", "RL", "RR") for j in ("hip", "thigh", "calf")}


def _rot(rpy) -> np.ndarray:
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _quat(R: np.ndarray) -> list:
    """wxyz of a rotation matrix."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return [w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)]
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2
    q = [0.0] * 4
    q[1 + i], q[0] = s / 4, (R[k, j] - R[j, k]) / s
    q[1 + j], q[1 + k] = (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return q


def arm_base_pose(plate_xyz=None, plate_size=None, mount_rpy=None) -> np.ndarray:
    """4x4 pose of the arm's `base_link` in the Go2 `base` frame: the top face of the plate, rotated by mount_rpy (as go2_piper.urdf.xacro). Defaults: config/go2/go2_setup.yaml."""
    m = setup_config()["mount"]
    plate_xyz, plate_size, mount_rpy = (m["plate_xyz"] if plate_xyz is None else plate_xyz, m["plate_size"] if plate_size is None else plate_size,
                                        m["mount_rpy"] if mount_rpy is None else mount_rpy)
    T = np.eye(4)
    T[:3, :3] = _rot(mount_rpy)
    T[:3, 3] = [plate_xyz[0], plate_xyz[1], plate_xyz[2] + plate_size[2] / 2]
    return T


def cover_report() -> dict:
    """Provenance and measured quality of the generated cover (Go2 description hashes, stand pose, policy, boxes, worst gap, volumes)."""
    return json.loads((config_dir() / "dog_cover_report.json").read_text())


def dog_cover(frame: str = "arm_base", plate_xyz=None, plate_size=None, mount_rpy=None) -> dict:
    """The dog as ONE collision box for a planner: {"dog": {"dims": [x, y, z], "pose": [x, y, z, qw, qx, qy, qz]}}. GENERATED from the Go2's meshes at its stand pose
    (tools/go2_cover): the axis-aligned bounding box of every point of the dog, so from the arm it looks like standing on a block. frame="base": the Go2 `base` frame as generated.
    frame="arm_base": the arm's base_link frame, the frame cuRobo plans in (the mount arguments default to config/go2/go2_setup.yaml). The plate and the arm are not part of it (see mount_boxes, go2_world)."""
    boxes = json.loads((config_dir() / "dog_cover.json").read_text())["boxes"]
    return _in_frame(boxes, frame, plate_xyz, plate_size, mount_rpy)


def mount_boxes(plate_xyz=None, plate_size=None) -> dict:
    """The things fixed on the dog, as boxes in the Go2 `base` frame: the plate (the same numbers the xacro places it with) and any `payload_boxes` of config/go2/go2_setup.yaml. Estimates until measured."""
    m = setup_config()["mount"]
    out = {"plate": {"dims": list(m["plate_size"] if plate_size is None else plate_size), "pose": [*(m["plate_xyz"] if plate_xyz is None else plate_xyz), 1.0, 0.0, 0.0, 0.0]}}
    for b in m.get("payload_boxes") or []:
        out[b["name"]] = {"dims": list(b["size"]), "pose": [*b["xyz"], *_quat(_rot(b.get("rpy", (0, 0, 0))))]}
    return out


def _in_frame(boxes: dict, frame: str, plate_xyz, plate_size, mount_rpy) -> dict:
    if frame == "base":
        return boxes
    if frame != "arm_base":
        raise ValueError("frame is 'base' or 'arm_base'")
    inv = np.linalg.inv(arm_base_pose(plate_xyz, plate_size, mount_rpy))
    out = {}
    for name, b in boxes.items():
        T = np.eye(4)
        T[:3, :3] = _quat_to_rot(b["pose"][3:])
        T[:3, 3] = b["pose"][:3]
        T = inv @ T
        out[name] = {"dims": b["dims"], "pose": [*T[:3, 3].tolist(), *_quat(T[:3, :3])]}
    return out


def go2_world(frame: str = "arm_base", plate_xyz=None, plate_size=None, mount_rpy=None) -> dict:
    """What a planner must avoid besides the arm itself: the dog (ONE generated box, `dog_cover`) and the small boxes of what is fixed on it (`mount_boxes`: the plate and any payload).
    {name: {"dims": [x, y, z], "pose": [x, y, z, qw, qx, qy, qz]}} in the Go2 `base` frame or in the arm's base_link frame (frame="arm_base", what cuRobo plans in). Nothing else: the world is empty."""
    boxes = {**dog_cover("base"), **mount_boxes(plate_xyz, plate_size)}
    return _in_frame(boxes, frame, plate_xyz, plate_size, mount_rpy)


def _quat_to_rot(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def go2_piper_curobo(camera: str | bool | None = "zedxm", gripper_width: float = 0.07, **mount) -> dict:
    """Everything a cuRobo planner needs for the arm on the dog: {"robot": <piper_assets.curobo_config>, "world": {"cuboid": {...}}, "base_in_go2": 4x4 list}.
    The arm model is the shipped one (the arm is the same robot; only its surroundings change), planned in the arm's base frame; the world is the dog's cover in that frame and nothing else.
    `mount` = plate_xyz, plate_size, mount_rpy (defaults: config/go2/go2_setup.yaml, NOMINAL until measured). Needs piper_assets (no cuRobo, no ROS)."""
    import piper_assets as pa
    return {"robot": pa.curobo_config(camera=camera, gripper_width=gripper_width), "world": {"cuboid": go2_world("arm_base", **mount)},
            "base_in_go2": arm_base_pose(**mount).tolist()}
