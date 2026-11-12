"""piper_assets reads only its generated data: no ROS needed. (Build the data first: tools/release/build_assets.py.)"""

import re
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import piper_assets as pa

try:
    pa.data_dir()
except FileNotFoundError:
    pytest.skip("data not built", allow_module_level=True)


@pytest.mark.parametrize("camera", [None, "zedxm"])
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
    assert ("zed_camera_center" in {l.get("name") for l in root.findall("link")}) == (camera == "zedxm")


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


# ---- the three setups: arm | arm_cam | go2 ----------------------------------------------------------------------------------------------------------------

needs_go2 = pytest.mark.skipif("go2" not in pa.manifest().get("setups", []), reason="this build has no go2 setup")


def test_setups_and_the_older_arguments_keep_their_meaning():
    assert pa.SETUPS == ("arm", "arm_cam", "go2") and pa.DEFAULT_SETUP == "go2"
    assert pa.urdf_path().name == "piper_zedxm_w0.07.urdf"                       # setup=None: a real arm on a table, as before
    assert pa.urdf_path(camera=None).name == "piper_none_w0.07.urdf"
    assert pa.urdf_path(setup="arm").name == "piper_none_w0.07.urdf" and pa.urdf_path(setup="arm_cam").name == "piper_zedxm_w0.07.urdf"
    assert pa.world("arm") == {} and pa.curobo_world("arm_cam") == {}
    with pytest.raises(ValueError):
        pa.urdf_path(setup="dog")


@needs_go2
def test_go2_urdf_has_the_same_arm_names_the_dog_beside_them_and_every_mesh():
    root = ET.parse(pa.urdf_path(setup="go2")).getroot()
    links = {l.get("name") for l in root.iter("link")}
    joints = {j.get("name") for j in root.iter("joint")}
    assert {*pa.JOINT_NAMES, "gripper"} <= joints and {pa.BASE_LINK, pa.TCP_LINK, "gripper_base", "zed_left_camera_frame_optical"} <= links
    assert {"base", "FL_hip", "RR_foot", "radar", "dock_plate", "mount"} <= links
    children = {j.find("child").get("link"): j.find("parent").get("link") for j in root.iter("joint")}
    assert set(links) - set(children) == {"base"} and children["mount"] == "dock_plate" and children["base_link"] == "mount"
    d = pa.data_dir()
    meshes = [m.get("filename") for m in root.iter("mesh")]
    assert all((d / f).exists() for f in meshes) and any("go2_description" in f for f in meshes)
    assert (d / "licenses/unitree_ros-BSD3.txt").exists()


@needs_go2
def test_go2_world_mount_and_cuRobo_pieces_agree():
    base, arm = pa.world("go2", "base"), pa.world("go2", "arm_base")
    assert set(base) == set(arm) >= {"dog", "plate"} and not any("floor" in n for n in base)      # the dog as one generated box, the plate (and payload); no floor
    m = pa.mount()
    assert "NOMINAL" in m and m["plate_size"][2] == pytest.approx(0.045)  # measured by touch and np.array(m["base_in_go2"]).shape == (4, 4)
    T = np.array(m["base_in_go2"])
    assert (T @ [*arm["dog"]["pose"][:3], 1.0])[:3] == pytest.approx(base["dog"]["pose"][:3])
    assert arm["plate"]["pose"][:3] == pytest.approx([0, 0, -m["plate_size"][2] / 2], abs=1e-9)   # the arm base frame is the plate's top face
    rep = pa.cover_report()
    assert rep["all_points_inside"] and rep["dims_m"] == pytest.approx(base["dog"]["dims"], abs=1e-4)
    cfg = pa.curobo_config(setup="go2")                                         # the arm alone: the dog is the world
    assert cfg["kinematics"]["urdf_path"].endswith("piper_zedxm_w0.07.urdf")
    assert pa.curobo_world()["cuboid"] == arm
    assert pa.srdf_path(setup="go2").name == "go2_piper.srdf"
    srdf = ET.parse(pa.srdf_path(setup="go2")).getroot()
    assert {("base_link", "dock_plate")} <= {(e.get("link1"), e.get("link2")) for e in srdf.iter("disable_collisions")}
