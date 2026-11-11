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


def _fake_sysfs(tmp_path, monkeypatch, buses):
    """buses: {name: (state, device path)}; fakes /sys/class/net for piper_core.canbus."""
    import piper_core.canbus as cb
    net = tmp_path / "net"
    for name, (state, dev) in buses.items():
        d = net / name
        d.mkdir(parents=True)
        (d / "type").write_text("280\n")
        (d / "operstate").write_text(state + "\n")
        target = tmp_path / dev.lstrip("/")
        target.mkdir(parents=True, exist_ok=True)
        (d / "device").symlink_to(target)
    real = cb.Path
    monkeypatch.setattr(cb, "Path", lambda p: real(net) if p == "/sys/class/net" else real(p))
    return cb


JETSON = {"can0": ("up", "/devices/platform/bus@0/3610000.usb/usb1/1-2/1-2:1.0"), "can1": ("down", "/devices/platform/bus@0/c310000.mttcan")}


def test_the_usb_adapter_is_found_by_usb_not_by_name(tmp_path, monkeypatch):
    for k, names in enumerate((JETSON, {"can1": JETSON["can0"], "can0": JETSON["can1"]})):          # the names swap between boots
        cb = _fake_sysfs(tmp_path / f"boot{k}", monkeypatch, names)
        name, why = cb.usb_canbus()
        assert why is None and "usb1" in cb.can_info()[name]["path"] and not cb.can_info()[name]["path"].endswith("mttcan")


def test_resolve_refuses_a_down_usb_adapter_and_never_picks_the_soc(tmp_path, monkeypatch):
    buses = {"can0": ("down", JETSON["can0"][1]), "can1": ("up", JETSON["can1"][1])}        # the SoC bus is up, the adapter is not
    cb = _fake_sysfs(tmp_path, monkeypatch, buses)
    with pytest.raises(SystemExit, match="piper-can-up"):
        cb.resolve_canbus()


def test_no_usb_adapter_is_an_error(tmp_path, monkeypatch):
    cb = _fake_sysfs(tmp_path, monkeypatch, {"can1": JETSON["can1"]})
    with pytest.raises(SystemExit, match="no USB-CAN adapter"):
        cb.resolve_canbus()
