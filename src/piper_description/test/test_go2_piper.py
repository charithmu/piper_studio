"""The optional Go2 + Piper assembly (nominal mount): expands, one connected tree, no name clashes, boxes consistent."""

import xml.etree.ElementTree as ET
from pathlib import Path

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


def test_defaults_of_the_python_helpers_equal_the_xacro_defaults():
    from piper_description import go2
    root = ET.fromstring(go2_piper_description())
    j = next(j for j in root.iter("joint") if j.get("name") == "piper_dock_plate_joint")
    assert [float(x) for x in j.find("origin").get("xyz").split()] == list(go2.PLATE_XYZ)
    pose = arm_base_pose()
    assert pose[:3, 3].tolist() == pytest.approx([-0.05, 0.0, 0.067])


def test_world_in_the_arm_frame_is_the_inverse_of_the_mount():
    w_base = go2_dock_world()
    w_arm = go2_dock_world(frame="arm_base")
    T = arm_base_pose()
    for name in w_base:
        p_arm = np.array(w_arm[name]["pose"][:3])
        assert (T @ [*p_arm, 1.0])[:3] == pytest.approx(w_base[name]["pose"][:3])
    # a yawed mount rotates the boxes (a 180 degree yaw puts the head behind the arm base)
    yawed = go2_dock_world(frame="arm_base", mount_rpy=(0, 0, np.pi))
    assert yawed["go2_head"]["pose"][0] < 0 < w_arm["go2_head"]["pose"][0]


def _box_clearance(center, radius, box):
    q = np.array(box["pose"][3:])
    w, x, y, z = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    local = R.T @ (np.asarray(center) - np.array(box["pose"][:3]))
    half = np.array(box["dims"]) / 2
    d = np.maximum(np.abs(local) - half, 0.0)
    inside = np.all(np.abs(local) < half)
    return -float(np.min(half - np.abs(local))) - radius if inside else float(np.linalg.norm(d)) - radius


@pytest.mark.parametrize("pose", ["zero", "ready"])
def test_arm_spheres_clear_the_dog_at_zero_and_ready(pose):
    """CPU check (no cuRobo): every collision sphere of the arm with the camera vs the dog's boxes, in the arm base frame. Nominal mount, nominal boxes."""
    pytest.importorskip("piper_assets")
    from piper_core.kinematics import Kinematics
    from piper_core.arm_cli import READY
    from piper_description import go2 as g
    cfg = go2_piper_curobo(ground=False)
    spheres = cfg["robot"]["robot_cfg"]["kinematics"]["collision_spheres"] if "robot_cfg" in cfg["robot"] else cfg["robot"]["kinematics"]["collision_spheres"]
    q = [0.0, 0.01, -0.01, 0.0, 0.0, 0.0] if pose == "zero" else list(READY)
    kin = Kinematics.from_assets(camera="zedxm")
    worst = np.inf
    for link, lst in spheres.items():
        T = kin.pose(q, link)
        for s in lst:
            c = (T @ [*s["center"], 1.0])[:3]
            for box in cfg["world"]["cuboid"].values():
                worst = min(worst, _box_clearance(c, s["radius"], box))
    assert worst > 0.0, f"arm penetrates the dog by {-worst * 1000:.0f} mm"
    print(f"{pose}: minimum clearance {worst * 1000:.0f} mm")


def test_export_for_simulators_is_self_contained(tmp_path, monkeypatch):
    """tools/release/export_go2_piper.py: one URDF whose every mesh exists next to it, the boxes in both frames, the nominal warning."""
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
    assert "dock_plate" in json.loads((tmp_path / "world_base.json").read_text())
    arm_world = json.loads((tmp_path / "world_arm_base.json").read_text())
    assert "dock_plate" not in arm_world and "floor" in arm_world
    assert "NOMINAL" in json.loads((tmp_path / "mount.json").read_text())
