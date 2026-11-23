"""Where to look from: pose planning for a hand-eye run. Pure (numpy + Studio's kinematics): testable without a camera or an arm.

Candidate joint poses around the `ready` pose are kept where the whole board is predicted to be seen well (range, angle, inside the image) and every
fingertip / wrist / camera point stays above the board plane; the most different ones are chosen. Paths between poses are slow cosine-eased joint moves.
"""
import math

import numpy as np

from .transforms import inv_T
from . import kinematics as kin

READY = [0.0, 1.0, -1.0, 0.0, 1.0, 0.0]          # Studio's named pose `ready`, the observe pose of config/real.yaml
PEAK_RAD_S = 0.25                                # peak speed of the eased joint moves
MIN_RANGE_M, MAX_RANGE_M = 0.22, 0.70            # camera to board centre
MAX_VIEW_ANGLE_DEG = 55.0                        # between the board normal and the viewing ray
IMAGE_MARGIN_PX = 25                             # all four board corners at least this far inside the image
CLEARANCE_POINT_M = 0.08                         # tcp / link5 / link6 / camera above the board plane at a pose
CLEARANCE_PATH_M = 0.05                          # same, along the path between poses (sampled)


class CalibrationError(RuntimeError):
    pass


# ------------------------------------------------------------------ pose planning (pure: testable without hardware)
def board_geometry(spec):
    w, h = spec["squares_x"] * spec["square_m"], spec["squares_y"] * spec["square_m"]
    return w, h


def point_heights(q, camera="zedxm"):
    """z of the points that must stay clear of the board: fingertip centre, link5, link6 origin, camera."""
    points = kin.fk_points(q, camera)
    return min(points["tcp"][2], points["link5"][2], points["link6"][2], points["camera"][2])


def visible(q, T_base_board, spec, K, size, camera="zedxm", X=None):
    """Is the whole board predicted to be seen well from joints q? Returns (ok, why)."""
    T_base_cam = kin.camera_optical(q, camera, X)
    T_cam_board = inv_T(T_base_cam) @ T_base_board
    w, h = board_geometry(spec)
    corners = np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0], [w / 2, h / 2, 0]]) @ T_cam_board[:3, :3].T + T_cam_board[:3, 3]
    if (corners[:, 2] < 0.05).any():
        return False, "behind the camera"
    centre = corners[4]
    distance = float(np.linalg.norm(centre))
    if not MIN_RANGE_M <= distance <= MAX_RANGE_M:
        return False, f"range {distance:.2f} m"
    normal = T_cam_board[:3, 2]                  # the printed face looks away from the viewer: its normal points away from the camera
    cos = float(normal @ centre / distance)
    if cos < math.cos(math.radians(MAX_VIEW_ANGLE_DEG)):
        return False, "board seen too obliquely or from behind"
    pixels = corners[:4] @ K.T
    pixels = pixels[:, :2] / pixels[:, 2:3]
    if (pixels[:, 0] < IMAGE_MARGIN_PX).any() or (pixels[:, 0] > size[0] - IMAGE_MARGIN_PX).any() or (pixels[:, 1] < IMAGE_MARGIN_PX).any() \
            or (pixels[:, 1] > size[1] - IMAGE_MARGIN_PX).any():
        return False, "board not fully inside the image"
    return True, ""


def plan_poses(T_base_board, spec, K, size, n=14, ready=READY, seed=0, camera="zedxm", X=None, candidates=6000):
    """n joint poses around `ready`, each with the board predicted fully visible, clear of the board plane, spread in camera position and direction."""
    rng = np.random.default_rng(seed)
    lower, upper = kin.joint_limits(margin=0.10)
    ready = np.asarray(ready, float)
    spread = np.array([0.45, 0.45, 0.45, 0.8, 0.6, 1.0])
    board_z = float(T_base_board[2, 3])
    found, features = [], []
    for _ in range(candidates):
        q = ready + rng.uniform(-1, 1, 6) * spread
        if (q < lower).any() or (q > upper).any():
            continue
        if point_heights(q, camera) < board_z + CLEARANCE_POINT_M:
            continue
        ok, _ = visible(q, T_base_board, spec, K, size, camera, X)
        if not ok:
            continue
        cam = kin.camera_optical(q, camera, X)
        found.append(q)
        features.append(np.r_[cam[:3, 3] * 4.0, cam[:3, 2] * 1.0, cam[:3, 0] * 1.0])       # position (scaled), viewing axis, roll axis
    if len(found) < n:
        raise CalibrationError(f"only {len(found)} candidate poses see the whole board (need {n}): move the board nearer the arm's front, in reach of the ready pose")
    features = np.array(features)
    chosen = [int(np.argmin(np.linalg.norm(features - features.mean(axis=0), axis=1)))]
    distance = np.linalg.norm(features - features[chosen[0]], axis=1)
    while len(chosen) < n:
        chosen.append(int(np.argmax(distance)))
        distance = np.minimum(distance, np.linalg.norm(features - features[chosen[-1]], axis=1))
    return [found[i] for i in chosen], len(found)


def eased(q0, q1, dt=0.025, peak=PEAK_RAD_S):
    """Cosine-eased joint-space line from q0 to q1 at 25 ms steps: (times, joint rows)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    T = max(1.0, float(np.max(np.abs(q1 - q0))) * math.pi / 2 / peak)
    t = np.arange(0, T + dt / 2, dt)
    t[-1] = T
    s = (1 - np.cos(math.pi * t / T)) / 2
    return t, q0 + s[:, None] * (q1 - q0)


def path_clear(q0, q1, board_z, camera="zedxm"):
    _, rows = eased(q0, q1)
    lowest = min(point_heights(q, camera) for q in rows[::6])
    return lowest >= board_z + CLEARANCE_PATH_M, lowest - board_z


