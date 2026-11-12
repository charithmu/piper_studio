"""Go2 carrying the Piper: the mount transform and the dog's boxes for a planner. Pure Python (numpy), no ROS.

Everything here is NOMINAL until the expansion-dock plate is measured (2026-11-12): the defaults are the placeholders of urdf/go2_piper.urdf.xacro. Keep the
two in step: `test_go2_piper.py` checks that the defaults agree with the xacro.
"""

from __future__ import annotations

import numpy as np

PLATE_XYZ = (-0.02, 0.0, 0.072)      # plate centre in the Go2 `base` frame (trunk box top is z = 0.057; the plate bottom sits on it)
PLATE_SIZE = (0.14, 0.09, 0.030)
MOUNT_RPY = (0.0, 0.0, 0.0)
STAND_HEIGHT = 0.30                  # Go2 base height above the floor, standing (nominal; the app's stand pose is about 0.27-0.32)


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


def arm_base_pose(plate_xyz=PLATE_XYZ, plate_size=PLATE_SIZE, mount_rpy=MOUNT_RPY) -> np.ndarray:
    """4x4 pose of the arm's `base_link` in the Go2 `base` frame: the top face of the plate, rotated by mount_rpy (as go2_piper.urdf.xacro)."""
    T = np.eye(4)
    T[:3, :3] = _rot(mount_rpy)
    T[:3, 3] = [plate_xyz[0], plate_xyz[1], plate_xyz[2] + plate_size[2] / 2]
    return T


def go2_dock_world(plate_xyz=PLATE_XYZ, plate_size=PLATE_SIZE, margin: float = 0.0, mount_rpy=MOUNT_RPY, frame: str = "base", ground: bool = False) -> dict:
    """The dog's obstacles as cuRobo-style cuboids: {name: {"dims": [x, y, z], "pose": [x, y, z, qw, qx, qy, qz]}}.
    frame="base": the Go2 `base` frame. frame="arm_base": the arm's base_link frame, which is the frame cuRobo plans in (give this to the planner).
    NOMINAL: trunk box = the Go2 URDF's collision box, plate from the arguments, head and radar are rough boxes (replace with measured values).
    `margin` grows every box on all sides. `ground=True` adds the floor (a thin slab STAND_HEIGHT below `base`, nominal standing).
    The plate is included: the arm's base touches it, so a planner must ignore the plate against `base_link` (cuRobo: the world is checked against spheres, and the base spheres sit at the plate)."""
    def box(dims, xyz):
        return {"dims": [d + 2 * margin for d in dims], "pose": [*xyz, 1.0, 0.0, 0.0, 0.0]}
    w = {
        "go2_trunk": box((0.3762, 0.0935, 0.114), (0.0, 0.0, 0.0)),
        "dock_plate": box(tuple(plate_size), tuple(plate_xyz)),
        "go2_head": box((0.12, 0.10, 0.09), (0.34, 0.0, 0.0)),            # rough: Head_upper at x = 0.285
        "go2_radar": box((0.08, 0.09, 0.06), (0.28945, 0.0, -0.046825)),  # rough: at the radar link origin
    }
    if ground:
        w["floor"] = {"dims": [4.0, 4.0, 0.02], "pose": [0.0, 0.0, -STAND_HEIGHT - 0.01, 1.0, 0.0, 0.0, 0.0]}
    if frame == "base":
        return w
    if frame != "arm_base":
        raise ValueError("frame is 'base' or 'arm_base'")
    inv = np.linalg.inv(arm_base_pose(plate_xyz, plate_size, mount_rpy))
    out = {}
    for name, b in w.items():
        T = np.eye(4)
        T[:3, :3] = _rot((0, 0, 0))                                        # boxes are axis-aligned in `base`
        T[:3, 3] = b["pose"][:3]
        T = inv @ T
        out[name] = {"dims": b["dims"], "pose": [*T[:3, 3].tolist(), *_quat(T[:3, :3])]}
    return out


def go2_piper_curobo(camera: str | bool | None = "zedxm", gripper_width: float = 0.07, margin: float = 0.0, ground: bool = True, **mount) -> dict:
    """Everything a cuRobo planner needs for the arm on the dog: {"robot": <piper_assets.curobo_config>, "world": {"cuboid": {...}}, "base_in_go2": 4x4 list}.
    The arm model is the shipped one (the arm is the same robot; only its surroundings change), planned in the arm's base frame, so the Go2 boxes are given in that frame.
    `mount` = plate_xyz, plate_size, mount_rpy (NOMINAL until measured). Needs piper_assets (no cuRobo, no ROS)."""
    import piper_assets as pa
    world = go2_dock_world(margin=margin, frame="arm_base", ground=ground, **mount)
    world.pop("dock_plate")      # the arm's base sits on it
    return {"robot": pa.curobo_config(camera=camera, gripper_width=gripper_width), "world": {"cuboid": world},
            "base_in_go2": arm_base_pose(mount.get("plate_xyz", PLATE_XYZ), mount.get("plate_size", PLATE_SIZE), mount.get("mount_rpy", MOUNT_RPY)).tolist()}


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


def go2_stand_pose() -> dict:
    """Joint angles of a standing Go2 (hip 0, thigh 0.8, calf -1.5 rad; the usual stand), by the Go2 URDF's joint names. NOMINAL: for pictures and planning checks."""
    return {f"{leg}_{j}_joint": v for leg in ("FL", "FR", "RL", "RR") for j, v in (("hip", 0.0), ("thigh", 0.8), ("calf", -1.5))}
