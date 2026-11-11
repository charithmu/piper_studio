"""The real-camera launch (zed_camera.launch.py) gives the ZED wrapper the names the description and the simulators use, with the wrapper's TF off."""

import importlib.util
from pathlib import Path

import pytest

LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "zed_camera.launch.py"


@pytest.fixture(scope="module")
def zl():
    spec = importlib.util.spec_from_file_location("zed_camera_launch", LAUNCH)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.mark.parametrize("prefix", ["piper_", "arm_", ""])
def test_names_equal_the_description(zl, prefix):
    from piper_description import camera_topics
    assert zl.zed_names(prefix) == {k: v for k, v in camera_topics(prefix).items()}


def test_override_turns_tf_and_tracking_off(zl):
    import yaml
    p = yaml.safe_load(zl.OVERRIDE.format(resolution="SVGA", fps=30))["/**"]["ros__parameters"]
    assert p["pos_tracking"] == {"pos_tracking_enabled": False, "publish_tf": False, "publish_map_tf": False}
    assert p["general"] == {"grab_resolution": "SVGA", "grab_frame_rate": 30}


def test_a_missing_wrapper_is_a_clear_error(zl, monkeypatch):
    from launch import LaunchContext
    monkeypatch.setattr(zl, "get_package_share_directory", lambda n: (_ for _ in ()).throw(zl.PackageNotFoundError(n)))
    ctx = LaunchContext()
    for n, v in (("prefix", "piper_"), ("camera_model", "zedxm"), ("resolution", "SVGA"), ("fps", "30"), ("params_file", "")):
        ctx.launch_configurations[n] = v
    with pytest.raises(RuntimeError, match="zed_wrapper"):
        zl.launch_setup(ctx)
