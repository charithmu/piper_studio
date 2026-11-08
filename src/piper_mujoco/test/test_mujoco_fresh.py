"""The shipped MuJoCo data equals a fresh build from the workspace (needs the ROS environment; skipped without it)."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("piper_sim")
from piper_mujoco.sim import data_dir

BUILDER = Path(__file__).resolve().parents[3] / "tools/release/build_mujoco.py"


def test_built_data_is_current(tmp_path):
    subprocess.run([sys.executable, str(BUILDER), "--out", str(tmp_path / "d")], check=True, capture_output=True)
    fresh = json.loads((tmp_path / "d/manifest.json").read_text())["files"]
    assert fresh == json.loads((data_dir() / "manifest.json").read_text())["files"], "stale: run tools/release/build_mujoco.py"
