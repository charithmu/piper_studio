"""piper_assets reads only its generated data: no ROS needed. (Build the data first: tools/release/build_assets.py.)"""

import re
import xml.etree.ElementTree as ET

import pytest

import piper_assets as pa

try:
    pa.data_dir()
except FileNotFoundError:
    pytest.skip("data not built", allow_module_level=True)


@pytest.mark.parametrize("camera", [None, "d435"])
@pytest.mark.parametrize("width", [0.0, 0.07, None])
def test_urdf_is_flat_unprefixed_and_its_meshes_exist(camera, width):
    path = pa.urdf_path(width, camera)
    root = ET.fromstring(path.read_text())
    names = {j.get("name") for j in root.findall("joint")}
    assert set(pa.JOINT_NAMES) <= names and pa.TCP_LINK in {l.get("name") for l in root.findall("link")}
    moving = {j.get("name") for j in root.findall("joint") if j.get("type") in ("revolute", "prismatic")}
    assert moving == set(pa.JOINT_NAMES) | ({"gripper", "gripper_joint1", "gripper_joint2"} if width is None else set())
    files = re.findall(r'filename="([^"]+)"', path.read_text())
    assert files and all(not f.startswith(("/", "file:", "package:")) and (path.parent / f).is_file() for f in files)
    assert ("camera_link" in {l.get("name") for l in root.findall("link")}) == (camera == "d435")


def test_other_gripper_width_is_made_in_the_cache(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path))
    p = pa.urdf_path(0.035)
    assert p.parent.parent.parent == tmp_path and all(re.findall(r'filename="(/[^"]+)"', p.read_text()))
    with pytest.raises(ValueError):
        pa.urdf_path(0.08)


def test_limits_have_sources_and_planning_is_inside_hard():
    lim = pa.limits()
    assert lim.joints == pa.JOINT_NAMES and lim.velocity == 3.0 and lim.acceleration == 5.0 and lim.gripper_stroke == 0.07
    for j in lim.joints:
        assert lim.planning[j][0] == pytest.approx(lim.hard[j][0] + 0.01) and lim.planning[j][1] == pytest.approx(lim.hard[j][1] - 0.01)
    assert lim.flags["jerk"] == "derived" and "rating" in lim.flags["effort"] and set(lim.sources) >= {"position", "velocity", "jerk"}
    assert lim.hard["joint2"][0] == 0.0 and lim.hard["joint3"][1] == 0.0


def test_zero_pose_is_the_guard_zero_and_passes_the_trajectory_check():
    import numpy as np
    from piper_core import Trajectory, check_trajectory
    lim, z = pa.limits(), pa.zero_pose()
    assert z == pytest.approx([0.0, 0.01, -0.01, 0.0, 0.0, 0.0])
    tr = Trajectory.from_pairs([(0.0, z), (1.0, [0.5] + z[1:]), (2.0, z)])
    assert check_trajectory(tr, lim.hard) is None


def test_scene_and_manifest():
    s = pa.scene()
    assert s["frame"] == "base_link" and {o["name"] for o in s["objects"]} >= {"cube_red", "cube_green", "cylinder_blue"}
    assert s["table"]["measured"] is False and s["table"]["size"] == [1.4, 1.4, 0.05]
    m = pa.manifest()
    assert m["base_link"] == pa.BASE_LINK and m["tcp_xyz_in_gripper_base"] == list(pa.TCP_OFFSET)


def test_srdf_names_match_the_urdf():
    srdf = ET.fromstring(pa.srdf_path().read_text())
    links = {l.get("name") for l in ET.fromstring(pa.urdf_path().read_text()).findall("link")}
    assert srdf.findall("disable_collisions") and all(d.get("link1") in links and d.get("link2") in links for d in srdf.findall("disable_collisions"))
