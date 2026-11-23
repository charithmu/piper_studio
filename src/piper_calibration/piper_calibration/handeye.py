"""Hand-eye calibration of the wrist camera with OpenCV (ChArUco board + cv2.calibrateHandEye). No arm, no camera, no Studio here: pure numpy/OpenCV.

Unknown: X = T_link6_optical, the pose of the camera's optical frame in the link6 frame (the number that is only a CAD value today).
Measured per capture i: A_i = T_base_link6 (forward kinematics of the measured joints) and B_i = T_cam_board (ChArUco corners + solvePnP).
The board stays still, so  A_i X B_i  is the same T_base_board for every i; the spread of that product is the quality number that needs no ruler.

Libraries only: cv2.aruco.CharucoDetector, cv2.solvePnP, cv2.calibrateHandEye (Tsai, Park, Horaud, Andreff, Daniilidis), cv2.calibrateRobotWorldHandEye
(Shah, Li) as a cross-check. OpenCV 4.x with the contrib/aruco API (4.7+); OpenCV 5.0.0 headless has no calibrateHandEye.

Moved from graspwise body/piper_box (ported from piper_pick box/piper_box @ demo-reference cfc95d3) into Studio on 2026-11-23.
"""
import json
from pathlib import Path

import cv2
import numpy as np

from .transforms import inv_T, make_T, pose_difference, rotation_angle_deg  # noqa: F401  (also reachable as handeye.make_T etc.)

METHODS = {"tsai": cv2.CALIB_HAND_EYE_TSAI, "park": cv2.CALIB_HAND_EYE_PARK, "horaud": cv2.CALIB_HAND_EYE_HORAUD,
           "andreff": cv2.CALIB_HAND_EYE_ANDREFF, "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS}
DEFAULT_METHOD = "park"
MIN_CORNERS = 12                 # inner corners needed for a pose; the 6x8 board has 35
MAX_REPROJECTION_PX = 1.0        # a board pose whose own corners reproject worse than this is not used


# ------------------------------------------------------------------ the board
def load_board(spec_path):
    """The JSON next to the printed sheet (scripts/make_charuco.py). Returns (cv2 CharucoBoard, spec dict)."""
    spec = json.loads(Path(spec_path).read_text())
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    size = (spec["squares_x"], spec["squares_y"])
    first = int(spec.get("first_marker_id", 0))                       # generators let you start the marker ids elsewhere
    ids = np.arange(first, first + size[0] * size[1] // 2, dtype=np.int32) if first else None
    board = cv2.aruco.CharucoBoard(size, spec["square_m"], spec["marker_m"], dictionary, ids) if ids is not None else \
        cv2.aruco.CharucoBoard(size, spec["square_m"], spec["marker_m"], dictionary)
    return board, spec


class BoardDetector:
    """Detects the ChArUco board in an RGB frame and returns its pose in the camera's optical frame."""

    def __init__(self, board, K, distortion=None):
        self.board, self.K = board, np.asarray(K, float)
        self.dist = None if distortion is None else np.asarray(distortion, float)       # the ZED's left image is rectified: None
        params = cv2.aruco.CharucoParameters()
        params.cameraMatrix, params.distCoeffs = self.K, self.dist
        self.detector = cv2.aruco.CharucoDetector(board, params)

    def count_markers(self, rgb):
        """How many ArUco markers (not corners) are seen: markers without corners means a wrong board description, none at all means too small, blurred or not in view."""
        gray = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2GRAY) if rgb.ndim == 3 else rgb
        _, _, marker_corners, _ = self.detector.detectBoard(gray)
        return 0 if marker_corners is None else len(marker_corners)

    def detect(self, rgb):
        """dict(T_cam_board, n_corners, reprojection_px, corners, ids) or None when the board is not usable."""
        gray = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2GRAY) if rgb.ndim == 3 else rgb
        corners, ids, _, _ = self.detector.detectBoard(gray)
        if ids is None or len(ids) < MIN_CORNERS:
            return None
        obj, img = self.board.matchImagePoints(corners, ids)
        obj, img = obj.reshape(-1, 3), img.reshape(-1, 2)
        flags = cv2.SOLVEPNP_IPPE if len(obj) >= 4 else cv2.SOLVEPNP_ITERATIVE          # ChArUco corners are coplanar
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.dist, flags=flags)
        if not ok:
            return None
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.K, self.dist, rvec, tvec)
        projected, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.dist)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - img) ** 2, axis=1))))
        if error > MAX_REPROJECTION_PX:
            return None
        return {"T_cam_board": make_T(cv2.Rodrigues(rvec)[0], tvec), "n_corners": int(len(ids)), "reprojection_px": error,
                "corners": img.astype(np.float32), "ids": ids.reshape(-1)}


# ------------------------------------------------------------------ the solve
def _stack(T_list):
    return [T[:3, :3] for T in T_list], [T[:3, 3].reshape(3, 1) for T in T_list]


def solve_one(T_base_link6, T_cam_board, method=DEFAULT_METHOD):
    """T_link6_optical from one OpenCV hand-eye algorithm."""
    Rg, tg = _stack(T_base_link6)
    Rt, tt = _stack(T_cam_board)
    R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=METHODS[method])
    return make_T(R, t)


def board_in_base(T_base_link6, T_cam_board, X):
    return [A @ X @ B for A, B in zip(T_base_link6, T_cam_board)]


def board_spread(T_base_board):
    """Spread of the board pose over the captures: per-axis std and the largest distance (mm), rotation spread and largest angle (deg)."""
    t = np.array([T[:3, 3] for T in T_base_board])
    centre = t.mean(axis=0)
    # mean rotation by projecting the averaged matrix back onto SO(3)
    Rm = np.mean([T[:3, :3] for T in T_base_board], axis=0)
    U, _, Vt = np.linalg.svd(Rm)
    Rm = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
    angles = np.array([rotation_angle_deg(Rm.T @ T[:3, :3]) for T in T_base_board])
    dist = np.linalg.norm(t - centre, axis=1) * 1000
    return {"mean_position_m": centre.round(5).tolist(), "std_xyz_mm": (t.std(axis=0) * 1000).round(2).tolist(),
            "rms_mm": float(np.sqrt(np.mean(dist ** 2))), "max_mm": float(dist.max()),
            "rotation_rms_deg": float(np.sqrt(np.mean(angles ** 2))), "rotation_max_deg": float(angles.max())}


def leave_out(T_base_link6, T_cam_board, method, folds=3, seed=0):
    """Hold out poses, solve on the rest, and measure how far the held-out board pose falls from the board position of the training poses."""
    n = len(T_base_link6)
    if n < 8:
        return None
    order = np.random.default_rng(seed).permutation(n)
    errors = []
    for fold in range(folds):
        test = set(order[fold::folds].tolist())
        train = [i for i in range(n) if i not in test]
        X = solve_one([T_base_link6[i] for i in train], [T_cam_board[i] for i in train], method)
        ref = board_in_base([T_base_link6[i] for i in train], [T_cam_board[i] for i in train], X)
        mean_t = np.mean([T[:3, 3] for T in ref], axis=0)
        for i in test:
            errors.append(float(np.linalg.norm((T_base_link6[i] @ X @ T_cam_board[i])[:3, 3] - mean_t) * 1000))
    return {"folds": folds, "rms_mm": float(np.sqrt(np.mean(np.square(errors)))), "max_mm": float(max(errors))}


def solve(T_base_link6, T_cam_board, nominal=None, method=DEFAULT_METHOD):
    """All OpenCV methods on the same captures, their agreement, and the quality numbers of the chosen one. `nominal` is the CAD T_link6_optical."""
    if len(T_base_link6) < 6:
        raise ValueError(f"need at least 6 captures with a usable board, got {len(T_base_link6)}")
    results = {name: solve_one(T_base_link6, T_cam_board, name) for name in METHODS}
    X = results[method]
    report = {"method": method, "n_captures": len(T_base_link6), "T_link6_optical": X.tolist(),
              "methods": {name: {"T_link6_optical": T.tolist(), "vs_chosen_deg_mm": pose_difference(X, T),
                                 "board_spread_rms_mm": board_spread(board_in_base(T_base_link6, T_cam_board, T))["rms_mm"]}
                          for name, T in results.items()}}
    report["board_in_base"] = board_spread(board_in_base(T_base_link6, T_cam_board, X))
    report["leave_out"] = leave_out(T_base_link6, T_cam_board, method)
    if nominal is not None:
        report["vs_nominal_deg_mm"] = pose_difference(np.asarray(nominal, float), X)
        report["nominal_board_spread_rms_mm"] = board_spread(board_in_base(T_base_link6, T_cam_board, np.asarray(nominal, float)))["rms_mm"]
    try:                                            # cross-check: board and camera solved together (Shah / Li), different formulation
        Rb, tb = _stack([inv_T(A) for A in T_base_link6])
        Rw, tw = _stack(T_cam_board)
        out = cv2.calibrateRobotWorldHandEye(Rw, tw, Rb, tb, method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH)
        report["cross_check_robot_world"] = {"vs_chosen_deg_mm": pose_difference(X, inv_T(make_T(out[2], out[3])))}   # it returns T_cam_gripper, the inverse of X
    except (cv2.error, AttributeError):
        report["cross_check_robot_world"] = None
    report["verdict"] = verdict(report)
    return report


# thresholds are indicative, not validated: they say "look closer", not "safe to grasp"
def verdict(report):
    spread, issues = report["board_in_base"], []
    if report["n_captures"] < 10:
        issues.append(f"only {report['n_captures']} captures (10 or more recommended)")
    if spread["rms_mm"] > 3.0:
        issues.append(f"the board position varies by {spread['rms_mm']:.1f} mm rms over the captures (above 3 mm)")
    if spread["rotation_rms_deg"] > 0.5:
        issues.append(f"the board orientation varies by {spread['rotation_rms_deg']:.2f} deg rms (above 0.5)")
    worst = max(m["vs_chosen_deg_mm"][0] for m in report["methods"].values()), max(m["vs_chosen_deg_mm"][1] for m in report["methods"].values())
    if worst[0] > 1.0 or worst[1] > 5.0:
        issues.append(f"the OpenCV methods disagree by up to {worst[0]:.2f} deg / {worst[1]:.1f} mm (above 1 deg / 5 mm)")
    if report["leave_out"] and report["leave_out"]["rms_mm"] > 4.0:
        issues.append(f"held-out poses fall {report['leave_out']['rms_mm']:.1f} mm from the training board position (above 4 mm)")
    return {"acceptable": not issues, "issues": issues}
