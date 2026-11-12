"""The optional Go2 + Piper assembly (nominal mount): expands, one connected tree, no name clashes, boxes consistent."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from piper_description import GO2_URDF, dog_cover, go2_piper_description

pytestmark = pytest.mark.skipif(not GO2_URDF.exists(), reason="go2_studio's unitree_ros checkout is not here")


def tree(xml):
    root = ET.fromstring(xml)
    links = {l.get("name") for l in root.iter("link")}
    edges = {j.find("child").get("link"): j.find("parent").get("link") for j in root.iter("joint")}
    return root, links, edges


def test_one_connected_tree_rooted_at_the_go2_base():
    _, links, edges = tree(go2_piper_description())
    assert set(edges) == links - {"base"}                     # every link but the root has exactly one parent
    for link in links:                                         # and every chain ends at `base`
        seen = link
        for _ in range(len(links)):
            if seen == "base":
                break
            seen = edges[seen]
        assert seen == "base", link


def test_arm_chain_runs_through_plate_and_mount_and_all_arm_names_have_the_prefix():
    root, links, edges = tree(go2_piper_description())
    assert edges["piper_dock_plate"] == "base" and edges["piper_mount"] == "piper_dock_plate" and edges["piper_base_link"] == "piper_mount"
    arm = {l for l in links if l.startswith("piper_")}
    assert {"piper_link6", "piper_gripper_base", "piper_tcp_link"} <= arm
    go2 = links - arm
    assert {"base", "FL_foot", "radar", "Head_upper"} <= go2 and not any(n.startswith("piper") for n in go2)
    joints = [j.get("name") for j in root.iter("joint")]
    assert len(joints) == len(set(joints)) and len(links) == len(set(l.get("name") for l in root.iter("link")))


def test_plate_pose_is_a_parameter_and_moves_the_arm_chain_only():
    root, _, _ = tree(go2_piper_description(plate_xyz="0.1 0 0.07"))
    plate = next(j for j in root.iter("joint") if j.get("name") == "piper_dock_plate_joint")
    assert plate.find("origin").get("xyz") == "0.1 0 0.07"


# ---- camera, frames, cuRobo -------------------------------------------------------------------------------------------------------------------------

import numpy as np  # noqa: E402

from piper_description import arm_base_pose, go2_piper_curobo, robot_description  # noqa: E402


def joint_origins(xml, prefix_filter):
    return {j.get("name"): (j.find("origin").get("xyz"), j.find("origin").get("rpy")) for j in ET.fromstring(xml).iter("joint") if prefix_filter in j.get("name")}


def test_wrist_camera_is_the_standalone_one_and_default():
    cam = ("camera_mount", "zed_")
    go2 = go2_piper_description()
    alone = robot_description("mock")
    for key in cam:
        a, b = joint_origins(go2, key), joint_origins(alone, key)
        assert a and a == b, key
    assert "piper_zed_left_camera_frame_optical" in go2
    assert "zed_" not in go2_piper_description(camera="none").replace("zed_ros2", "")


def test_the_mount_comes_from_one_file_for_the_xacro_and_the_python_helpers():
    from piper_description import setup_config
    m = setup_config()["mount"]
    root = ET.fromstring(go2_piper_description())
    j = next(j for j in root.iter("joint") if j.get("name") == "piper_dock_plate_joint")
    assert [float(x) for x in j.find("origin").get("xyz").split()] == m["plate_xyz"]
    box = next(l for l in root.iter("link") if l.get("name") == "piper_dock_plate").find("collision/geometry/box")
    assert [float(x) for x in box.get("size").split()] == m["plate_size"]
    assert arm_base_pose()[:3, 3].tolist() == pytest.approx([m["plate_xyz"][0], m["plate_xyz"][1], m["plate_xyz"][2] + m["plate_size"][2] / 2])
    assert m["nominal"] is True                       # until the plate is measured: flip it then, and the docs say so


def test_export_for_simulators_is_self_contained(tmp_path, monkeypatch):
    """tools/release/export_go2_piper.py: one URDF whose every mesh exists next to it, the dog's cover in both frames, the nominal warning."""
    import importlib.util
    import json
    import os
    root = Path(__file__).resolve().parents[3]
    spec = importlib.util.spec_from_file_location("export_go2_piper", root / "tools/release/export_go2_piper.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    monkeypatch.setattr("sys.argv", ["export", "--out", str(tmp_path)])
    mod.main()
    urdf = ET.parse(tmp_path / "go2_piper.urdf").getroot()
    meshes = [m.get("filename") for m in urdf.iter("mesh")]
    assert meshes and all(os.path.exists(tmp_path / f) for f in meshes) and not any(f.startswith(("package://", "/")) for f in meshes)
    assert any("go2_description" in f for f in meshes) and any("piper_description" in f or "agx_arm" in f for f in meshes)
    for frame in ("base", "arm_base"):
        assert {"dog", "plate"} <= set(json.loads((tmp_path / f"world_{frame}.json").read_text()))
    assert "NOMINAL" in json.loads((tmp_path / "mount.json").read_text())
