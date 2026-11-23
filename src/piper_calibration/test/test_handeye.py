"""Hand-eye solver and ChArUco detector against a synthetic truth: a rendered board seen from known camera poses, a known X = T_link6_optical.

Defends: the detector returns the pose that was rendered, calibrateHandEye recovers X, and the quality numbers (board spread, leave-out)
say "good" for good data and "not acceptable" for data with a wrong X in the kinematics.
Moved from graspwise tests/unit/box (ported from piper_pick tests/test_handeye.py @ demo-reference cfc95d3).
"""
import json

import numpy as np
import cv2
import pytest

VISION = hasattr(cv2.aruco, "CharucoDetector")      # OpenCV 4.7+ with the contrib aruco API (`pip install piper_calibration[vision]`); Studio's own venv has the system 4.6
pytestmark = pytest.mark.skipif(not VISION, reason="needs OpenCV 4.7+ contrib (pip install piper_calibration[vision])")

from piper_calibration import handeye

K = np.array([[364.65, 0, 485.34], [0, 364.65, 296.66], [0, 0, 1]])
SIZE = (960, 600)


def _look_at(eye, target, roll):
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, -1.0], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z], axis=1)
    c, s = np.cos(roll), np.sin(roll)
    R = R @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return handeye.make_T(R, eye)


@pytest.fixture(scope="module")
def board():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    return cv2.aruco.CharucoBoard((6, 8), 0.03, 0.0225, dictionary)


def _render(board, T_cam_board, noise=0.0, seed=0):
    px_per_m = 2000.0
    image = board.generateImage((int(0.18 * px_per_m), int(0.24 * px_per_m)), marginSize=0, borderBits=1)
    S = np.diag([1 / px_per_m, 1 / px_per_m, 1.0])
    H = K @ np.column_stack([T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]]) @ S
    out = cv2.warpPerspective(image, H, SIZE, flags=cv2.INTER_AREA, borderValue=128)
    out = cv2.GaussianBlur(out, (3, 3), 0.8)
    if noise:
        out = np.clip(out + np.random.default_rng(seed).normal(0, noise, out.shape), 0, 255).astype(np.uint8)
    return np.stack([out] * 3, axis=-1)


def _scene(n, X, rng):
    """n camera poses looking at the board lying near (0.4, 0.0, 0) of the base, with varied position, tilt and roll."""
    # the printed face has its normal pointing away from the viewer: lying face up, the board's z points down
    T_base_board = handeye.make_T(cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0] @ cv2.Rodrigues(np.array([0.0, 0.0, 0.3]))[0], [0.36, -0.05, 0.0])
    centre = T_base_board[:3, 3] + T_base_board[:3, :3] @ [0.09, 0.12, 0.0]
    poses = []
    while len(poses) < n:
        eye = centre + np.array([rng.uniform(-0.18, 0.18), rng.uniform(-0.18, 0.18), rng.uniform(0.28, 0.45)])
        T_base_cam = _look_at(eye, centre + rng.normal(0, 0.01, 3), rng.uniform(-0.6, 0.6))
        poses.append(T_base_cam)
    return T_base_board, poses


def test_detector_returns_the_rendered_pose(board):
    T_cam_board = handeye.make_T(cv2.Rodrigues(np.array([0.3, 0.1, 0.0]))[0], [-0.08, -0.1, 0.4])
    rgb = _render(board, T_cam_board, noise=2.0)
    found = handeye.BoardDetector(board, K).detect(rgb)
    assert found is not None and found["n_corners"] >= 30
    rot, mm = handeye.pose_difference(T_cam_board, found["T_cam_board"])
    assert rot < 0.5 and mm < 3.0
    assert found["reprojection_px"] < 0.5


def test_board_not_found_in_an_empty_image(board):
    assert handeye.BoardDetector(board, K).detect(np.full((600, 960, 3), 128, np.uint8)) is None


def test_hand_eye_recovers_truth_from_rendered_images(board):
    rng = np.random.default_rng(1)
    X = handeye.make_T(cv2.Rodrigues(np.array([0.1, -0.05, 0.3]))[0], [0.03, -0.02, 0.07])       # the camera pose in link6
    T_base_board, cams = _scene(16, X, rng)
    detector = handeye.BoardDetector(board, K)
    A, B = [], []
    for T_base_cam in cams:
        T_cam_board = handeye.inv_T(T_base_cam) @ T_base_board
        found = detector.detect(_render(board, T_cam_board, noise=2.0, seed=len(A)))
        if found is None:
            continue
        A.append(T_base_cam @ handeye.inv_T(X))
        B.append(found["T_cam_board"])
    assert len(A) >= 12
    nominal = X @ handeye.make_T(cv2.Rodrigues(np.array([0.02, 0.03, -0.02]))[0], [0.01, 0.01, -0.01])   # a CAD value a bit off
    report = handeye.solve(A, B, nominal=nominal)
    rot, mm = handeye.pose_difference(X, np.array(report["T_link6_optical"]))
    assert rot < 0.3 and mm < 2.0, (rot, mm)
    assert report["board_in_base"]["rms_mm"] < 2.0
    assert report["leave_out"]["rms_mm"] < 3.0
    assert report["verdict"]["acceptable"], report["verdict"]
    assert report["cross_check_robot_world"]["vs_chosen_deg_mm"][0] < 0.5 and report["cross_check_robot_world"]["vs_chosen_deg_mm"][1] < 3.0
    # the nominal pose must be shown to be worse than the solved one
    assert report["nominal_board_spread_rms_mm"] > 2 * report["board_in_base"]["rms_mm"]


def test_wrong_kinematics_is_flagged_not_hidden(board):
    """A 1.5 degree error in one joint's zero makes the board wander in the base frame: the verdict must say so."""
    rng = np.random.default_rng(2)
    X = handeye.make_T(np.eye(3), [0.0, 0.0, 0.08])
    T_base_board, cams = _scene(14, X, rng)
    A, B = [], []
    for i, T_base_cam in enumerate(cams):
        T_cam_board = handeye.inv_T(T_base_cam) @ T_base_board
        T_link6 = T_base_cam @ handeye.inv_T(X)
        bad = T_link6.copy()
        angle = np.radians(1.5) * np.sin(i)                  # a pose-dependent kinematic error
        bad[:3, :3] = cv2.Rodrigues(np.array([0.0, angle, 0.0]))[0] @ bad[:3, :3]
        bad[:3, 3] += rng.normal(0, 0.006, 3)
        A.append(bad)
        B.append(T_cam_board)
    report = handeye.solve(A, B)
    assert not report["verdict"]["acceptable"]


def test_too_few_captures_are_refused():
    with pytest.raises(ValueError):
        handeye.solve([np.eye(4)] * 3, [np.eye(4)] * 3)


# ---- the board the user actually printed (arucogrid.com: 6x7, 25 mm squares, 18 mm markers, DICT_6X6_250) and the camera's full resolution
def _render_at(board, K_, size, T_cam_board, width_m, height_m, noise=2.0):
    px = 3000.0
    image = board.generateImage((int(width_m * px), int(height_m * px)), marginSize=0, borderBits=1)
    H = K_ @ np.column_stack([T_cam_board[:3, 0], T_cam_board[:3, 1], T_cam_board[:3, 3]]) @ np.diag([1 / px, 1 / px, 1.0])
    out = cv2.GaussianBlur(cv2.warpPerspective(image, H, size, flags=cv2.INTER_AREA, borderValue=128), (3, 3), 1.0)
    out = np.clip(out + np.random.default_rng(0).normal(0, noise, out.shape), 0, 255).astype(np.uint8)
    return np.stack([out] * 3, axis=-1)


def test_the_printed_board_is_read_at_full_resolution_where_the_small_image_fails(tmp_path):
    spec = {"squares_x": 6, "squares_y": 7, "square_m": 0.025, "marker_m": 0.018, "dictionary": "DICT_6X6_250"}
    (tmp_path / "b.json").write_text(json.dumps(spec))
    board, loaded = handeye.load_board(tmp_path / "b.json")
    T = handeye.make_T(cv2.Rodrigues(np.array([0.25, 0.1, 0.0]))[0], [-0.075, -0.0875, 0.6])          # 0.6 m, the ready pose's range
    K_hd, size_hd = K * 2, (1920, 1200)
    K_hd[2, 2] = 1.0
    found = handeye.BoardDetector(board, K_hd).detect(_render_at(board, K_hd, size_hd, T, 0.15, 0.175))
    assert found is not None and found["n_corners"] == 30
    rot, mm = handeye.pose_difference(T, found["T_cam_board"])
    assert rot < 0.5 and mm < 3.0
    # the same board at 960x600 is NOT readable at that range: this is why the calibration captures at 1920x1200 by default
    small = handeye.BoardDetector(board, K)
    assert small.detect(_render_at(board, K, SIZE, T, 0.15, 0.175)) is None


def test_marker_ids_can_start_anywhere(tmp_path):
    spec = {"squares_x": 6, "squares_y": 7, "square_m": 0.025, "marker_m": 0.018, "dictionary": "DICT_6X6_250", "first_marker_id": 40}
    (tmp_path / "b.json").write_text(json.dumps(spec))
    board, _ = handeye.load_board(tmp_path / "b.json")
    assert board.getIds().ravel().tolist() == list(range(40, 61))


def test_our_board_is_the_one_arucogrid_hands_out(tmp_path):
    """arucogrid.com's own snippet for the printed 6x7 / 25 mm / 18 mm / 6x6_250 board is `cv2.aruco.CharucoBoard((6, 7), 0.025, 0.018, DICT_6X6_250)` with default
    ids and layout and markerBorderBits = 1 (the detector default). Our JSON-built board must be identical: corners, ids, rendered pattern."""
    reference = cv2.aruco.CharucoBoard((6, 7), 0.025, 0.018, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250))
    (tmp_path / "b.json").write_text(json.dumps({"squares_x": 6, "squares_y": 7, "square_m": 0.025, "marker_m": 0.018, "dictionary": "DICT_6X6_250"}))
    ours, _ = handeye.load_board(tmp_path / "b.json")
    assert np.array_equal(ours.getChessboardCorners(), reference.getChessboardCorners())
    assert ours.getIds().ravel().tolist() == reference.getIds().ravel().tolist() == list(range(21))
    assert ours.getLegacyPattern() == reference.getLegacyPattern()
    assert np.array_equal(ours.generateImage((600, 700), marginSize=0, borderBits=1), reference.generateImage((600, 700), marginSize=0, borderBits=1))
    assert cv2.aruco.DetectorParameters().markerBorderBits == 1
