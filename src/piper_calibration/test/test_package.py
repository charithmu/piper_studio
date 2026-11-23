"""The package stays light: importing it loads no OpenCV, no ZED SDK, no vendor arm SDK. The board maker writes a sheet its own detector reads completely."""
import json
import subprocess
import sys

import pytest


def test_importing_the_package_loads_no_hardware_or_vision_library():
    code = ("import sys, piper_calibration, piper_calibration.kinematics, piper_calibration.planning, piper_calibration.zed\n"
            "bad = [m for m in ('cv2', 'pyzed', 'pyAgxArm') if m in sys.modules]\nassert not bad, bad")
    subprocess.run([sys.executable, "-c", code], check=True)


def test_the_board_maker_writes_a_sheet_the_detector_reads(tmp_path, capsys):
    pytest.importorskip("PIL")
    cv2 = pytest.importorskip("cv2")
    if not hasattr(cv2.aruco, "CharucoDetector"):
        pytest.skip("needs OpenCV 4.7+ contrib")
    from piper_calibration import board
    board.main(["--out", str(tmp_path)])
    spec = json.loads((tmp_path / "charuco_6x8_30mm.json").read_text())
    assert spec["square_m"] == 0.03 and spec["dictionary"] == "DICT_5X5_100"
    assert (tmp_path / "charuco_6x8_30mm.pdf").exists()
    assert "self-test: 35 of 35" in capsys.readouterr().out
