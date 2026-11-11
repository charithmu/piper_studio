"""The generated MuJoCo model must reproduce the official kinematics and couple the fingers."""

import sys
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import piper_sim
from piper_sim import robot_description

sys.path.insert(0, str(Path(get_package_prefix("piper_description")) / "lib" / "piper_description"))
import model_audit  # noqa: E402

ARM = [f"joint{i}" for i in range(1, 7)]
P = "piper_"  # the model carries the default prefix; the official model does not
OFFICIAL = (Path(get_package_share_directory("agx_arm_description"))
            / "agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro")


def test_generated_mujoco_model_matches_official(tmp_path):
    """MJCF from mujoco_ros2_control's converter + our inputs: exact kinematics, coupled fingers."""
    mujoco = pytest.importorskip("mujoco")
    cfg = Path(get_package_share_directory("piper_sim")) / "config" / "mujoco"
    scene = piper_sim.mujoco_model(robot_description("mujoco"), cfg / "inputs.xml", cfg / "scene.xml",
                                           cache_dir=tmp_path)
    mj = model_audit.load_mjcf("mjcf", scene)
    ref = model_audit.load_urdf("official", OFFICIAL)
    rng = np.random.default_rng(1)
    lo = np.array([ref.joints[j].lower for j in ARM]); hi = np.array([ref.joints[j].upper for j in ARM])
    for q in [np.zeros(6)] + [rng.uniform(lo, hi) for _ in range(100)]:
        assert np.allclose(mj.fk(dict(zip([P + j for j in ARM], q))), ref.fk(dict(zip(ARM, q))), atol=1e-6)
    for j in ARM:
        assert (mj.joints[P + j].lower, mj.joints[P + j].upper) == pytest.approx(
            (ref.joints[j].lower, ref.joints[j].upper), abs=1e-5)
    m = mujoco.MjModel.from_xml_path(str(scene)); d = mujoco.MjData(m)
    d.ctrl[m.actuator(P + "gripper").id] = 0.06
    while d.time < 1.0:
        mujoco.mj_step(m, d)
    q = {n: d.qpos[m.jnt_qposadr[m.joint(n).id]] for n in (P + "gripper", P + "gripper_joint1", P + "gripper_joint2")}
    assert q[P + "gripper"] == pytest.approx(0.06, abs=1e-3)
    assert q[P + "gripper_joint1"] == pytest.approx(0.5 * q[P + "gripper"], abs=1e-4)
    assert q[P + "gripper_joint2"] == pytest.approx(-0.5 * q[P + "gripper"], abs=1e-4)


def _urdf_collision_boxes(urdf_text, q):
    """World-frame bounding boxes (min, max) of every collision mesh in the description at joint values q, from the URDF alone."""
    import xml.etree.ElementTree as ET
    import trimesh

    path = Path(urdf_text)
    ref = model_audit.load_urdf("desc", path)
    out = []
    for link in ET.parse(path).getroot().findall("link"):
        for col in link.findall("collision"):
            mesh = col.find("geometry/mesh")
            if mesh is None:
                continue
            f = mesh.get("filename")
            if f.startswith("package://"):
                pkg, rest = f[len("package://"):].split("/", 1)
                f = str(Path(get_package_share_directory(pkg)) / rest)
            m = trimesh.load(f.replace("file://", ""), force="mesh")
            o = col.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            T = ref.fk(q, link.get("name")) @ _pose(xyz, rpy)
            v = (T[:3, :3] @ m.vertices.T).T + T[:3, 3]
            out.append((link.get("name"), v.min(0), v.max(0)))
    return out


def _pose(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]
    T[:3, 3] = xyz
    return T


def test_mujoco_collision_meshes_sit_where_the_description_puts_them(tmp_path):
    """With the wrist camera: every collision mesh of the description (arm, gripper, bracket, ZED X Mini) appears in the MuJoCo model at the same place.

    The converter fuses fixed links into link6, so this compares the world bounding box of each collision mesh geom with the box the URDF
    predicts, at the zero pose and at random arm poses: a wrong bracket or camera pose, a missing mesh or a rotated one fails here.
    """
    mujoco = pytest.importorskip("mujoco")
    pytest.importorskip("trimesh")
    from piper_description import robot_description as plain

    urdf_file = tmp_path / "desc.urdf"
    urdf_file.write_text(plain("mujoco"))
    cfg = Path(get_package_share_directory("piper_sim")) / "config" / "mujoco"
    scene = piper_sim.mujoco_model(robot_description("mujoco"), cfg / "inputs.xml", cfg / "scene.xml", cache_dir=tmp_path)
    m = mujoco.MjModel.from_xml_path(str(scene))
    d = mujoco.MjData(m)
    ref = model_audit.load_urdf("desc", urdf_file)
    rng = np.random.default_rng(3)
    lo = np.array([ref.joints[P + j].lower for j in ARM]); hi = np.array([ref.joints[P + j].upper for j in ARM])
    for q in [np.zeros(6), rng.uniform(lo, hi), rng.uniform(lo, hi)]:
        for name, v in zip(ARM, q):
            d.qpos[m.jnt_qposadr[m.joint(P + name).id]] = v
        d.qpos[m.jnt_qposadr[m.joint(P + "gripper").id]] = 0.0
        mujoco.mj_forward(m, d)
        qd = dict(zip([P + j for j in ARM], q))
        want = [(n, lo_, hi_) for n, lo_, hi_ in _urdf_collision_boxes(urdf_file, qd)]
        got = []
        for g in range(m.ngeom):
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_contype[g] == 1:
                mid = m.geom_dataid[g]
                v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
                w = (d.geom_xmat[g].reshape(3, 3) @ v.T).T + d.geom_xpos[g]
                got.append((w.min(0), w.max(0)))
        assert len(got) == len(want) == 13, (len(got), len(want))
        # match each expected box to the nearest geom box: all 6 bounds within 1.5 mm
        for name, lo_, hi_ in want:
            best = min(max(np.abs(a - lo_).max(), np.abs(b - hi_).max()) for a, b in got)
            assert best < 0.0015, f"{name}: no MuJoCo collision mesh within 1.5 mm of the description's (nearest {best * 1000:.1f} mm)"
