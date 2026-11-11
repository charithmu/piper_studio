"""The cuRobo model shipped with piper_assets is consistent with the URDF, the SRDF and the limits, and current (CPU only, no cuRobo, no ROS).
The model is tuned and regenerated in this repo (tools/curobo/, docs/CUROBO.md); this test fails when the description moves on without a regeneration, or when a refit overlaps itself."""
import itertools
import xml.etree.ElementTree as ET

import numpy as np
import pytest

import piper_assets as pa

CAMERAS = [None, "zedxm"]


def _rot(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def _axis_rot(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


def link_poses(urdf, q):
    """{link: 4x4 in base_link} of the fixed-gripper URDF at the 6 arm joint angles q (revolute joints rotate, everything else is fixed)."""
    root = ET.parse(urdf).getroot()
    vec = lambda s: [float(x) for x in (s or "0 0 0").split()]  # noqa: E731
    pose = {"world": np.eye(4), "base_link": np.eye(4)}
    joints = [j for j in root.iter("joint") if j.find("parent") is not None]
    pending = list(joints)
    while pending:
        rest = []
        for j in pending:
            par, child = j.find("parent").get("link"), j.find("child").get("link")
            if par not in pose:
                rest.append(j)
                continue
            o = j.find("origin")
            T = np.eye(4)
            T[:3, :3] = _rot(vec(o.get("rpy") if o is not None else None))
            T[:3, 3] = vec(o.get("xyz") if o is not None else None)
            if j.get("type") in ("revolute", "continuous") and j.get("name") in pa.JOINT_NAMES:
                R = np.eye(4)
                R[:3, :3] = _axis_rot(vec(j.find("axis").get("xyz")), q[pa.JOINT_NAMES.index(j.get("name"))])
                T = T @ R
            pose[child] = pose[par] @ T
        assert len(rest) < len(pending), "URDF has a joint whose parent is never reached"
        pending = rest
    return pose


def _model(camera):
    return pa.curobo_config(camera), ET.parse(pa.srdf_path(camera)).getroot()


def overlaps(camera, q):
    """Self-collision as cuRobo counts it: sphere distance minus both radii (each plus the sphere buffer) minus the per-link buffers, for pairs not in the ignore list. {pair: mm}."""
    top, _ = _model(camera)
    k = top["kinematics"]
    poses = link_poses(pa.urdf_path(0.07, camera), q)
    ign = {a: set(b) for a, b in k["self_collision_ignore"].items()}
    buf, sb = k["self_collision_buffer"], k.get("collision_sphere_buffer", 0.0)
    spheres = []
    for link in k["collision_link_names"]:
        for s in k["collision_spheres"][link]:
            c = poses[link] @ np.append(s["center"], 1.0)
            spheres.append((link, c[:3], s["radius"] + sb))
    bad = {}
    for (la, ca, ra), (lb, cb, rb) in itertools.combinations(spheres, 2):
        if la == lb or lb in ign.get(la, ()) or la in ign.get(lb, ()):
            continue
        d = np.linalg.norm(ca - cb) - ra - rb - buf.get(la, 0) - buf.get(lb, 0)
        if d < 0:
            key = f"{la}|{lb}"
            bad[key] = min(bad.get(key, 9.0), d)
    return {k: round(v * 1000, 1) for k, v in bad.items()}


def named_pose(camera, name):
    srdf = ET.parse(pa.srdf_path(camera)).getroot()
    for g in srdf.iter("group_state"):
        if g.get("name") == name and g.get("group") == "arm":
            return [float(next(j.get("value") for j in g.findall("joint") if j.get("name") == n)) for n in pa.JOINT_NAMES]
    return None


@pytest.mark.parametrize("camera", CAMERAS)
def test_model_is_current_for_the_shipped_description(camera):
    """Built for the very URDF (joints, collision meshes) and SRDF pairs this package ships; else: scripts/curobo_env.sh run tools/curobo/build_model.py (docs/CUROBO.md)."""
    assert pa.curobo_model_is_current(camera), "the cuRobo model is stale: the description changed, regenerate it (docs/CUROBO.md)"


@pytest.mark.parametrize("camera", CAMERAS)
def test_model_agrees_with_urdf_srdf_and_limits(camera):
    top, srdf = _model(camera)
    k, L = top["kinematics"], pa.limits()
    urdf = ET.parse(pa.urdf_path(0.07, camera)).getroot()
    links = {link.get("name") for link in urdf.findall("link")}
    assert k["cspace"]["joint_names"] == pa.JOINT_NAMES and k["base_link"] == pa.BASE_LINK and k["tool_frames"] == [pa.TCP_LINK]
    assert set(k["collision_spheres"]) <= links
    assert {"camera_mount_link", "zed_camera_center"} <= set(k["collision_spheres"]) if camera else not {"camera_mount_link", "zed_camera_center"} & set(k["collision_spheres"])
    ign = {frozenset((a, b)) for a, v in k["self_collision_ignore"].items() for b in v}
    pairs = {frozenset((e.get("link1"), e.get("link2"))) for e in srdf.iter("disable_collisions")}
    have = set(k["collision_spheres"])
    assert not {p for p in pairs if p <= have} - ign, "an SRDF pair between sphere links is not in the model's ignore list"
    cs, n = k["cspace"], len(pa.JOINT_NAMES)
    assert cs["max_acceleration"] == [L.acceleration] * n and cs["max_jerk"] == [L.jerk] * n
    assert np.allclose(cs["default_joint_position"], pa.zero_pose()) and cs["position_limit_clip"] == pytest.approx(L.margin)
    prov = pa.curobo_provenance(camera)
    assert cs["acceleration_scale"][0] == prov["decisions"]["operating_scale"] and cs["velocity_scale"][0] == pytest.approx(prov["decisions"]["operating_scale"] ** 0.5)
    assert pa.curobo_planner_defaults()["graph_planner_algorithm"]


@pytest.mark.parametrize("camera", CAMERAS)
@pytest.mark.parametrize("pose", ["zero", "ready"])
def test_no_self_overlap_at_the_named_poses(camera, pose):
    """The failure the bracket fit had (its spheres inside the finger spheres at zero: 0 of 100 plans): caught here without a GPU."""
    q = pa.zero_pose() if pose == "zero" else named_pose(camera, "ready")
    assert q is not None
    assert overlaps(camera, q) == {}


@pytest.mark.parametrize("camera", CAMERAS)
def test_validation_was_run_and_planned(camera):
    v = pa.curobo_provenance(camera)["validation"]
    assert v is not None and v["audit_flags"] == [] and v["effective_limits"]["ok"] and v["self_overlap_at_zero"] == {}
    p = v["planning"]
    assert p["success"] >= 0.85 * p["n"] and p["n_over_vel"] == p["n_over_acc"] == p["n_over_jerk"] == 0 and p["max_pos_violation_rad"] == 0
