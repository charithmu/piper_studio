#!/usr/bin/env python3
"""Write reference forward kinematics (from the URDF) for isaac/check_model.py: random configs, TCP pose, total mass.

    python isaac/make_fk_reference.py OUT.json [N] [--no-camera]

The description with the ZED X Mini on its bracket (the default); the reference then also holds, per configuration, the pose of every link frame of the wrist
(tcp, bracket, camera, optical frame) and the world bounding box of the bracket's and the camera's collision mesh, so check_model.py can verify that the camera
and bracket are where the description puts them. --no-camera: the robot only.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/piper_description/tools"))
import model_audit  # noqa: E402
from piper_sim import physics_model, robot_description  # noqa: E402

args = [a for a in sys.argv[1:] if not a.startswith("--")]
camera = "--no-camera" not in sys.argv
FRAMES = ["gripper_base", "tcp_link", "camera_mount_link", "camera_link", "camera_color_optical_frame"]


def box(urdf_file, link, q):
    """World bounding box (min, max) of the link's collision mesh at joint values q, from the URDF alone."""
    import xml.etree.ElementTree as ET
    import trimesh
    from ament_index_python.packages import get_package_share_directory
    el = next(l for l in ET.parse(urdf_file).getroot().findall("link") if l.get("name") == link)
    col = el.find("collision")
    f = col.find("geometry/mesh").get("filename")
    if f.startswith("package://"):
        pkg, rest = f[len("package://"):].split("/", 1)
        f = str(Path(get_package_share_directory(pkg)) / rest)
    mesh = trimesh.load(f.replace("file://", ""), force="mesh")
    o = col.find("origin")
    xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    O = np.eye(4)
    O[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]
    O[:3, 3] = xyz
    T = m.fk(q, link) @ O
    v = (T[:3, :3] @ mesh.vertices.T).T + T[:3, 3]
    return v.min(0), v.max(0)


out, n = Path(args[0]), int(args[1]) if len(args) > 1 else 25
urdf = physics_model(robot_description("isaac", prefix="", **({} if camera else {"camera": "none"})))  # official names; the USD (prefixed) is compared by DOF order
tmp = out.with_suffix(".urdf")
tmp.write_text(urdf)
m = model_audit.load_urdf("ref", tmp)
lo = np.array([m.joints[j].lower for j in model_audit.ARM_JOINTS]) + 0.02
hi = np.array([m.joints[j].upper for j in model_audit.ARM_JOINTS]) - 0.02
rng = np.random.default_rng(3)
qs = [np.zeros(6) + [0, 0.05, -0.05, 0, 0, 0]] + [rng.uniform(lo, hi) for _ in range(n)]
refs = []
for q in qs:
    T = m.fk(dict(zip(model_audit.ARM_JOINTS, q)), "tcp_link")
    ref = {"q": q.tolist(), "pos": T[:3, 3].tolist(), "rot": T[:3, :3].tolist()}
    if camera:
        qd = dict(zip(model_audit.ARM_JOINTS, q))
        ref["frames"] = {name: {"pos": (F := m.fk(qd, name))[:3, 3].tolist(), "rot": F[:3, :3].tolist()}
                         for name in FRAMES}
        ref["col_boxes"] = {name: [b.tolist() for b in box(tmp, name, qd)] for name in ("camera_mount_link", "camera_link")}
    refs.append(ref)
json.dump({"configs": refs, "total_mass": sum(m.masses.values()), "masses": m.masses}, out.open("w"))
print("wrote", out, "total mass", round(sum(m.masses.values()), 4))
