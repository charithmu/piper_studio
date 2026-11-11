"""piper_tools: paths, entry points, the repo shims. Nothing here touches CAN or an arm."""
import importlib
import subprocess
import sys
import tomllib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src/piper_tools"


def test_paths_follow_the_environment(monkeypatch, tmp_path):
    from piper_tools import paths
    monkeypatch.setenv("PIPER_DATA_DIR", str(tmp_path / "d"))
    monkeypatch.setenv("PIPER_ARMS_DIR", str(tmp_path / "a"))
    assert paths.data_dir("x", "y") == tmp_path / "d/x/y"
    assert paths.arms_dir() == tmp_path / "a"


def test_paths_defaults(monkeypatch, tmp_path):
    from piper_tools import paths
    monkeypatch.delenv("PIPER_DATA_DIR", raising=False)
    monkeypatch.delenv("PIPER_ARMS_DIR", raising=False)
    monkeypatch.setenv("HOME", str(tmp_path))
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / ".local/share"))
    monkeypatch.setattr(paths, "WORKSTATION_DATA", tmp_path / "nope")
    assert paths.data_dir() == tmp_path / ".local/share/piper"           # a box: no workstation folder
    (tmp_path / "ws").mkdir()
    monkeypatch.setattr(paths, "WORKSTATION_DATA", tmp_path / "ws")
    assert paths.data_dir("arm_info") == tmp_path / "ws/arm_info"        # the workstation keeps its old place
    assert (paths.arms_dir() / "piper-01.yaml").exists()                 # running from the checkout


def test_arm_config_reads_the_arms_dir(monkeypatch, tmp_path):
    from piper_tools import arm_config
    (tmp_path / "box-arm.yaml").write_text("arm_id: box-arm\ncamera: none\n")
    monkeypatch.setenv("PIPER_ARMS_DIR", str(tmp_path))
    assert arm_config.camera("box-arm") is None


def test_every_entry_point_resolves():
    scripts = tomllib.loads((PKG / "pyproject.toml").read_text())["project"]["scripts"]
    assert "piper-estop" in scripts and "piper-info" in scripts and "piper-session" in scripts
    pytest.importorskip("pyAgxArm")
    for cmd, target in scripts.items():
        mod, func = target.split(":")
        assert callable(getattr(importlib.import_module(mod), func)), cmd


def test_the_estop_does_not_take_the_bus_lock():
    """The e-stop must work beside a running tool: it opens the arm with lock=False."""
    assert "lock=False" in (PKG / "piper_tools/arm_stop.py").read_text()


def test_repo_shims_still_run_and_import():
    pytest.importorskip("pyAgxArm")
    for name in ("arm_stop", "arm_info", "arm_session", "arm_pose", "arm_joints", "gripper_test", "arm_zero"):
        r = subprocess.run([sys.executable, str(ROOT / f"tools/real_arm/{name}.py"), "--help"], capture_output=True, text=True, timeout=60)
        assert r.returncode == 0, (name, r.stderr[-400:])
    r = subprocess.run([sys.executable, "-c", f"import sys; sys.path.insert(0, '{ROOT}/tools/real_arm'); import arm_config, arm_common; print(arm_config.camera())"],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr[-400:]
