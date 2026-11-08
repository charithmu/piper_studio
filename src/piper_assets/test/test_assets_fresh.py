"""The shipped data equals a fresh build from the workspace (needs the ROS environment; skipped without it)."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

pytest.importorskip("xacro")
pytest.importorskip("piper_sim")
import piper_assets as pa

BUILDER = Path(__file__).resolve().parents[3] / "tools/release/build_assets.py"


def test_built_data_is_current(tmp_path):
    subprocess.run([sys.executable, str(BUILDER), "--out", str(tmp_path / "d")], check=True, capture_output=True)
    fresh = json.loads((tmp_path / "d/manifest.json").read_text())["files"]
    shipped = pa.manifest()["files"]
    assert fresh == shipped, "piper_assets data is stale: run tools/release/build_assets.py (or scripts/build_wheels.sh)"
