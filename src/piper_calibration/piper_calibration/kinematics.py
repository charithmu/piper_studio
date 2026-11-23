"""Forward kinematics for planning and for the solve: Studio's own `piper_core.Kinematics` on the description in piper_assets (one implementation).

fk_points(q) -> {link1..link6, tcp, camera: xyz in the base frame}; link6_pose(q); camera_optical(q, X) -> 4x4 pose of the camera's optical frame in the base frame.
`X` is T_link6_optical: omit it for the pose the description carries now (what a calibration replaces), pass a solved one to test it.
"""
from functools import lru_cache

import numpy as np


@lru_cache(maxsize=4)
def _studio(camera):
    from piper_core.kinematics import Kinematics
    return Kinematics.from_assets(None if camera == "none" else camera)


def fk_points(q, camera="zedxm"):
    points = {name: np.asarray(p, float) for name, p in _studio(camera).points([float(v) for v in q]).items()}
    if "camera_optical" in points:
        points["camera"] = points.pop("camera_optical")
    return points


def link6_pose(q, camera="zedxm"):
    """4x4 pose of link6 (the flange the camera bracket is mounted on) in the base frame."""
    return np.asarray(_studio(camera).pose([float(v) for v in q], "link6"), float)


def nominal_link6_optical(camera="zedxm"):
    """The camera's optical frame in link6 as the description carries it now (piper_assets.camera_pose("link6")): what a calibration is compared with."""
    import piper_assets
    return np.array(piper_assets.camera_pose("link6"), float)


def joint_limits(margin=0.0):
    """(lower, upper) arrays for joints 1-6: the description's hard limits (piper_assets), shrunk by `margin` rad."""
    import piper_assets
    lower, upper = piper_assets.limits().as_arrays("hard")
    return lower + margin, upper - margin


def camera_optical(q, camera="zedxm", T_link6_optical=None):
    X = nominal_link6_optical(camera) if T_link6_optical is None else np.asarray(T_link6_optical, float)
    return link6_pose(q, camera) @ X
