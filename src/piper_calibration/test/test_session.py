"""The calibration app (piper_calibration.app) against a fake arm and a fake camera that render the board from the arm's own joints and a TRUE camera pose.

Defends: poses are planned only where the board is visible and clear of it; every path stays clear; the run asks before it moves, never moves after a refusal,
ends with the arm's session end; the solve recovers the true camera pose; a missing board stops before any motion beyond the ready pose.
Moved from graspwise tests/unit/box (ported from piper_pick tests/test_calibrate.py @ demo-reference cfc95d3); the camera and the arm are passed in, not monkeypatched.
"""
from types import SimpleNamespace

import numpy as np
import cv2
import pytest

VISION = hasattr(cv2.aruco, "CharucoDetector")      # OpenCV 4.7+ with the contrib aruco API (`pip install piper_calibration[vision]`); Studio's own venv has the system 4.6
pytestmark = pytest.mark.skipif(not VISION, reason="needs OpenCV 4.7+ contrib (pip install piper_calibration[vision])")

from piper_calibration import app, handeye, planning, session
from piper_calibration import kinematics as kin
from test_handeye import K, SIZE, _render

pytest.importorskip("piper_assets")


def _board():
    return cv2.aruco.CharucoBoard((6, 8), 0.03, 0.0225, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100))


SPEC = {"squares_x": 6, "squares_y": 8, "square_m": 0.03, "marker_m": 0.0225, "dictionary": "DICT_5X5_100"}
# face up on the bench in front of the arm: the printed face's normal points down, board centre at (0.40, 0, -0.17)
T_BASE_BOARD = handeye.make_T(cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0], [0.31, 0.12, -0.17])
X_TRUE = kin.nominal_link6_optical() @ handeye.make_T(cv2.Rodrigues(np.array([0.015, -0.02, 0.01]))[0], [0.006, -0.004, 0.008])


@pytest.fixture()
def board_json(tmp_path):
    import json
    path = tmp_path / "board.json"
    path.write_text(json.dumps(SPEC))
    return path


class FakeCamera:
    K = K
    size = SIZE
    arm = None

    def __init__(self, *a, **k):
        FakeCamera.instance = self
        self.closed = False

    def grab(self, settle_frames=8, **k):
        T_base_cam = kin.link6_pose(FakeCamera.arm.q) @ X_TRUE
        rgb = _render(_board(), handeye.inv_T(T_base_cam) @ T_BASE_BOARD, noise=2.0, seed=len(FakeCamera.arm.log))
        return {}, {"rgb": rgb, "depth": np.full(SIZE[::-1], 0.5, np.float32)}

    def hello(self):
        return {"fake": True}

    def close(self):
        self.closed = True


class FakeArm:
    """Studio's RealArm surface that calibrate uses; records everything it is asked to do."""
    instance = None

    def __init__(self):
        self.q = np.array([0.0, 0.01, -0.01, 0.0, 0.0, 0.0])
        self.log, self.started, self.stopped, self.closed, self.held = [], False, False, False, False
        self.speed_percent = None
        FakeCamera.arm = self
        FakeArm.instance = self

    @classmethod
    def connect(cls, bus):
        return cls()

    def start(self, speed=None):
        self.started = True
        self.log.append("start")
        return SimpleNamespace(ok=True, message="")

    def zero(self):
        return {f"joint{i + 1}": v for i, v in enumerate([0.0, 0.01, -0.01, 0.0, 0.0, 0.0])}

    def state(self):
        return SimpleNamespace(q=self.q.copy())

    def execute(self, trajectory):
        q = np.asarray(trajectory.q, float)
        assert np.abs(q[0] - self.q).max() < 0.05, "the first point must be near the present pose (Studio refuses otherwise)"
        assert np.abs(np.diff(q, axis=0)).max() < 0.25 * 0.025 * 1.6, "no step above the slow-move peak speed"
        self.log.append(("move", q[-1].round(3).tolist()))
        self.q = q[-1].copy()
        self.min_height = min(getattr(self, "min_height", 9.0), min(planning.point_heights(r) for r in q[::6]))
        return SimpleNamespace(ok=True, reason="done", detail="")

    def hold(self):
        self.held = True

    def stop(self, speed=None):
        self.stopped = True
        self.log.append("stop")
        return SimpleNamespace(ok=True, message="")

    def close(self):
        self.closed = True


@pytest.fixture()
def fakes():
    """What `app.main` is given instead of the ZED and the real arm."""
    return {"open_camera": lambda resolution: FakeCamera(), "connect_arm": lambda canbus: FakeArm()}


def test_planned_poses_see_the_whole_board_and_stay_clear():
    poses, n = planning.plan_poses(T_BASE_BOARD, SPEC, K, SIZE, n=14)
    assert len(poses) == 14 and n >= 14
    for q in poses:
        assert planning.visible(q, T_BASE_BOARD, SPEC, K, SIZE)[0]
        assert planning.point_heights(q) >= T_BASE_BOARD[2, 3] + planning.CLEARANCE_POINT_M
    # the views must differ enough to separate rotation from translation: camera axes spread over a real range
    axes = np.array([kin.camera_optical(q)[:3, 2] for q in poses])
    assert np.degrees(np.arccos(np.clip(axes @ axes.mean(axis=0) / np.linalg.norm(axes.mean(axis=0)), -1, 1))).max() > 15


def test_the_board_out_of_reach_is_refused_before_planning():
    far = T_BASE_BOARD.copy()
    far[:3, 3] = [1.5, 0.0, -0.17]
    with pytest.raises(planning.CalibrationError):
        planning.plan_poses(far, SPEC, K, SIZE, n=14)


def test_full_run_with_fakes_recovers_the_true_camera_pose(fakes, board_json, tmp_path, monkeypatch):
    monkeypatch.setattr("builtins.input", lambda prompt="": "")
    monkeypatch.setattr(app.time, "sleep", lambda s: None)
    code = app.main(["--board", str(board_json), "--session", "t", "--root", str(tmp_path)], **fakes)
    arm = FakeArm.instance
    assert arm.started and arm.stopped and arm.closed and FakeCamera.instance.closed
    assert arm.log[0] == "start" and arm.log[-1] == "stop"
    assert arm.min_height >= T_BASE_BOARD[2, 3] + planning.CLEARANCE_PATH_M - 1e-6
    import json
    result = json.loads((tmp_path / "t" / "hand_eye.json").read_text())
    rot, mm = handeye.pose_difference(X_TRUE, np.array(result["T_link6_optical"]))
    assert rot < 0.5 and mm < 3.0, (rot, mm)
    assert code == 0 and result["verdict"]["acceptable"], result["verdict"]
    # the solved pose explains the data better than the CAD one, and the file says what produced it
    assert result["nominal_board_spread_rms_mm"] > result["board_in_base"]["rms_mm"]
    assert "opencv" in result["versions"] and result["n_captures"] >= 10


def test_nothing_moves_if_the_operator_declines(fakes, board_json, tmp_path, monkeypatch):
    monkeypatch.setattr("builtins.input", lambda prompt="": "no")
    code = app.main(["--board", str(board_json), "--session", "t", "--root", str(tmp_path)], **fakes)
    arm = FakeArm.instance
    assert code == 1 and not arm.started and arm.log == []
    assert not (tmp_path / "t" / "hand_eye.json").exists()


def test_dry_run_reads_the_arm_and_never_moves_it(fakes, board_json, tmp_path, monkeypatch):
    monkeypatch.setattr("builtins.input", lambda prompt="": pytest.fail("a dry run must not ask to move anything"))
    arm_q = np.array(planning.READY)

    class ReadyArm(FakeArm):
        def __init__(self):
            super().__init__()
            self.q = arm_q.copy()
    code = app.main(["--board", str(board_json), "--session", "d", "--root", str(tmp_path), "--dry"], open_camera=fakes["open_camera"], connect_arm=lambda canbus: ReadyArm())
    arm = FakeArm.instance
    assert code == 0 and arm.log == [] and not arm.started and arm.closed
    assert (tmp_path / "d" / "plan.json").exists()


def test_board_not_visible_stops_with_the_arm_held(fakes, board_json, tmp_path, monkeypatch):
    monkeypatch.setattr("builtins.input", lambda prompt="": "")
    monkeypatch.setattr(app.time, "sleep", lambda s: None)
    monkeypatch.setattr(FakeCamera, "grab", lambda self, **k: ({}, {"rgb": np.full((600, 960, 3), 128, np.uint8), "depth": np.zeros(SIZE[::-1], np.float32)}))
    code = app.main(["--board", str(board_json), "--session", "n", "--root", str(tmp_path)], **fakes)
    arm = FakeArm.instance
    assert code == 1 and arm.held and not arm.stopped          # held where it stands; no automatic motion after an error
    assert sum(1 for entry in arm.log if isinstance(entry, tuple)) == 1         # only the move to ready happened


def test_look_uses_the_camera_only(fakes, board_json, tmp_path, monkeypatch):
    """--look must not touch the arm at all (no connection, no CAN)."""
    fakes = dict(fakes, connect_arm=lambda canbus: pytest.fail("--look must not connect the arm"))
    FakeCamera.arm = FakeArm()
    FakeCamera.arm.q = np.array(planning.READY)
    code = app.main(["--board", str(board_json), "--session", "l", "--root", str(tmp_path), "--look"], **fakes)
    assert code == 0 and (tmp_path / "l" / "look" / "pose-00.jpg").exists() and FakeCamera.instance.closed
    monkeypatch.setattr(FakeCamera, "grab", lambda self, **k: ({}, {"rgb": np.full((600, 960, 3), 128, np.uint8), "depth": np.zeros(SIZE[::-1], np.float32)}))
    assert app.main(["--board", str(board_json), "--session", "l2", "--root", str(tmp_path), "--look"], **fakes) == 2
