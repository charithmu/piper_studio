"""The optional Go2 + Piper assembly (nominal mount): expands, one connected tree, no name clashes, boxes consistent."""

import xml.etree.ElementTree as ET

import pytest

from piper_description import GO2_URDF, go2_dock_world, go2_piper_description

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


def test_dock_world_matches_the_default_plate_and_margin_grows_the_boxes():
    w, g = go2_dock_world(), go2_dock_world(margin=0.02)
    assert set(w) == {"go2_trunk", "dock_plate", "go2_head", "go2_radar"}
    assert w["dock_plate"]["pose"][:3] == [-0.05, 0.0, 0.062]
    assert g["go2_trunk"]["dims"] == pytest.approx([d + 0.04 for d in w["go2_trunk"]["dims"]])
