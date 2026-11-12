#!/usr/bin/env python3
"""Generate the Go2's collision cover for planners: one box that contains the whole dog at its stand pose.

    source scripts/env.sh && python tools/go2_cover/build_dog_cover.py [--go2-pkg DIR] [--out src/piper_description/config/go2]

Input: the Go2 description (go2_description: urdf + dae meshes, from go2_studio's unitree_ros checkout), the stand pose in config/go2/go2_setup.yaml, the policy in tools/go2_cover/policy.yaml.
Every link's visual mesh and every URDF collision shape is put in the Go2 `base` frame at the stand pose (forward kinematics of the URDF); the cover is their axis-aligned bounding box (+ the policy margin).
Writes dog_cover.json (the box, Go2 `base` frame) and dog_cover_report.json (provenance: Go2 description hashes, stand pose, policy; size, volume, what the box contains).
The box does not depend on the mount: moving the plate does not regenerate it. Needs trimesh (the workspace venv has it); no ROS, no cuRobo.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import logging
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import yaml

logging.getLogger("trimesh").setLevel(logging.ERROR)
WS = Path(__file__).resolve().parents[2]
GO2_PKG = Path.home() / "projects/ros/go2_studio/third_party/unitree_ros/robots/go2_description"
CFG = WS / "src/piper_description/config/go2"


def _floats(text, default):
    return np.array([float(v) for v in (text or default).split()])


def _T(xyz, rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]]
    T[:3, 3] = xyz
    return T


def _axis_T(axis, angle):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
    return T


def link_frames(root, q: dict) -> dict:
    """T_base_link for every link of the Go2 URDF at joint angles q (by joint name; missing = 0)."""
    T = {"base": np.eye(4)}
    pending = list(root.findall("joint"))
    while pending:
        rest = []
        for j in pending:
            parent = j.find("parent").get("link")
            if parent not in T:
                rest.append(j)
                continue
            o = j.find("origin")
            Tj = _T(_floats(o.get("xyz") if o is not None else None, "0 0 0"), _floats(o.get("rpy") if o is not None else None, "0 0 0"))
            if j.get("type") in ("revolute", "continuous"):
                Tj = Tj @ _axis_T(_floats(j.find("axis").get("xyz"), "1 0 0"), q.get(j.get("name"), 0.0))
            T[j.find("child").get("link")] = T[parent] @ Tj
        if len(rest) == len(pending):
            raise ValueError("the URDF has joints whose parent is never reached")
        pending = rest
    return T


def link_geometry(link, pkg: Path) -> list[trimesh.Trimesh]:
    """The link's visual meshes and collision shapes as meshes in the LINK frame."""
    out = []
    for tag in ("visual", "collision"):
        for el in link.findall(tag):
            g = el.find("geometry")
            if g is None or len(g) == 0:
                continue
            o = el.find("origin")
            T0 = _T(_floats(o.get("xyz") if o is not None else None, "0 0 0"), _floats(o.get("rpy") if o is not None else None, "0 0 0"))
            s = g[0]
            if s.tag == "mesh":
                if tag == "collision":
                    continue                      # (the Go2's collision shapes are primitives; a mesh here would duplicate the visual)
                m = trimesh.load(pkg / s.get("filename").split("package://go2_description/")[-1], force="mesh")
                m.apply_scale(_floats(s.get("scale"), "1 1 1"))
            elif s.tag == "box":
                m = trimesh.creation.box(extents=_floats(s.get("size"), "0 0 0"))
            elif s.tag == "sphere":
                m = trimesh.creation.icosphere(subdivisions=2, radius=float(s.get("radius")))
            elif s.tag == "cylinder":
                m = trimesh.creation.cylinder(radius=float(s.get("radius")), height=float(s.get("length")), sections=24)
            else:
                continue
            m.apply_transform(T0)
            out.append(m)
    return out


def dog_meshes(urdf: Path, stand: dict) -> dict:
    """{link: mesh in the Go2 `base` frame} for every link with geometry, at joint angles `stand`."""
    pkg = urdf.parent.parent
    root = ET.parse(urdf).getroot()
    frames = link_frames(root, stand)
    out = {}
    for link in root.findall("link"):
        parts = link_geometry(link, pkg)
        if parts:
            m = trimesh.util.concatenate(parts)
            m.apply_transform(frames[link.get("name")])
            out[link.get("name")] = m
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go2-pkg", default=str(GO2_PKG))
    ap.add_argument("--out", default=str(CFG))
    a = ap.parse_args()
    pkg, out = Path(a.go2_pkg), Path(a.out)
    urdf = pkg / "urdf/go2_description.urdf"
    setup = yaml.safe_load((CFG / "go2_setup.yaml").read_text())
    policy = yaml.safe_load((WS / "tools/go2_cover/policy.yaml").read_text())
    stand = {f"{leg}_{j}_joint": v for leg in ("FL", "FR", "RL", "RR") for j, v in setup["stand_pose"].items()}
    meshes = dog_meshes(urdf, stand)
    pts = np.vstack([m.vertices for m in meshes.values()])
    lo, hi = pts.min(0) - policy["margin"], pts.max(0) + policy["margin"]
    dims, centre = hi - lo, (hi + lo) / 2
    hull_volume = float(trimesh.convex.convex_hull(trimesh.PointCloud(pts)).volume)
    src = subprocess.run(["git", "-C", str(pkg.parents[2]), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    report = {
        "go2_description": {"urdf_sha256": hashlib.sha256(urdf.read_bytes()).hexdigest()[:16], "unitree_ros_commit": src,
                            "meshes_sha256": hashlib.sha256(b"".join((pkg / "dae" / f).read_bytes() for f in sorted(p.name for p in (pkg / "dae").glob("*.dae")))).hexdigest()[:16]},
        "stand_pose": setup["stand_pose"], "stand_source": setup["stand_source"], "policy": policy,
        "links_used": sorted(meshes), "points": int(len(pts)), "all_points_inside": bool(np.all((pts >= lo - 1e-9) & (pts <= hi + 1e-9))),
        "dims_m": [round(float(x), 4) for x in dims], "box_volume_m3": round(float(np.prod(dims)), 5), "convex_hull_of_the_dog_m3": round(hull_volume, 5),
        "frame": "Go2 `base`; the dog at the stand pose; the plate and the arm are not part of the cover",
    }
    box = {"dog": {"dims": [round(float(x), 5) for x in dims], "pose": [round(float(x), 5) for x in [*centre, 1.0, 0.0, 0.0, 0.0]]}}
    out.mkdir(parents=True, exist_ok=True)
    (out / "dog_cover.json").write_text(json.dumps({"frame": "go2_base", "boxes": box}, indent=1))
    (out / "dog_cover_report.json").write_text(json.dumps(report, indent=1))
    print(f"cover: one box {report['dims_m']} m centred at {[round(float(c), 3) for c in centre]} (Go2 base frame), {report['box_volume_m3'] * 1000:.0f} L "
          f"(the dog's convex hull {hull_volume * 1000:.0f} L); all {len(pts)} points inside: {report['all_points_inside']}")


if __name__ == "__main__":
    main()
