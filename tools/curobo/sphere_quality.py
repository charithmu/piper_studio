"""Mesh-based quality of a cuRobo sphere set, per link (the check the builder's own metrics do not give).
For every link with collision meshes: sample the mesh SURFACE and measure the distance to the nearest sphere surface.
  covered      share of surface points inside the sphere union (safety: uncovered points are unprotected)
  worst_mm     largest distance from any surface point to the sphere union (how far the arm can poke out of the spheres)
  p99_mm       99th percentile of that distance
  excess       share of sphere volume (sampled) that lies OUTSIDE the mesh (conservatism; needs a watertight mesh, else n/a)
    python -I tools/curobo/sphere_quality.py <urdf> <yml> [out.json]     (CPU only; ~1 min)
"""
import json, re, sys
from pathlib import Path
import numpy as np, trimesh, yaml
import xml.etree.ElementTree as ET
from trimesh.transformations import euler_matrix, translation_matrix

def tf(xyz, rpy): return translation_matrix(xyz) @ euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")
def vec(s): return [float(x) for x in (s or "0 0 0").split()]

def main(urdf, ymlp, out=None, n=20000, seed=0):
    rng = np.random.default_rng(seed)
    root = ET.fromstring(re.sub(r"<!--.*?-->", "", open(urdf).read(), flags=re.S))
    k = yaml.safe_load(open(ymlp)); k = k.get("robot_cfg", k)["kinematics"]; res = {}
    for l in root.findall("link"):
        name = l.get("name"); sph = k["collision_spheres"].get(name, [])
        meshes = []
        for c in l.findall("collision"):
            g = c.find("geometry"); o = c.find("origin"); T = tf(vec(o.get("xyz")) if o is not None else [0] * 3, vec(o.get("rpy")) if o is not None else [0] * 3)
            if g.find("mesh") is not None: m = trimesh.load(str((Path(urdf).parent / g.find("mesh").get("filename")).resolve()), force="mesh")
            elif g.find("box") is not None: m = trimesh.creation.box(vec(g.find("box").get("size")))
            else: continue
            m = m.copy(); m.apply_transform(T); meshes.append(m)
        if not meshes: continue
        m = trimesh.util.concatenate(meshes)
        P = m.sample(n, seed=seed) if False else m.sample(n)
        if not sph: res[name] = {"n_spheres": 0, "covered": 0.0, "worst_mm": float(np.linalg.norm(P - P.mean(0), axis=1).max() * 1000), "p99_mm": None, "excess": None}; continue
        C = np.array([s["center"] for s in sph]); Rr = np.array([s["radius"] for s in sph])
        d = (np.linalg.norm(P[:, None, :] - C[None], axis=2) - Rr[None]).min(axis=1)            # <=0 inside union
        covered = float((d <= 0).mean()); worst = float(max(d.max(), 0) * 1000); p99 = float(max(np.percentile(d, 99), 0) * 1000)
        excess = None
        if m.is_watertight:
            Q = []
            for c_, r_ in zip(C, Rr):
                v = rng.normal(size=(1500, 3)); v /= np.linalg.norm(v, axis=1)[:, None]; Q.append(c_ + v * (r_ * rng.random((1500, 1)) ** (1 / 3)))
            Q = np.vstack(Q); excess = float(1 - m.contains(Q).mean())
        res[name] = {"n_spheres": len(sph), "covered": round(covered, 3), "worst_mm": round(worst, 1), "p99_mm": round(p99, 1), "excess": None if excess is None else round(excess, 2),
                     "r_min_mm": round(float(Rr.min() * 1000), 1), "r_max_mm": round(float(Rr.max() * 1000), 1)}
    print(f"{'link':16s} {'n':>3s} {'covered':>8s} {'worst_mm':>9s} {'p99_mm':>7s} {'excess':>7s} {'r_min..r_max mm':>16s}")
    for n_, r in res.items(): print(f"{n_:16s} {r['n_spheres']:3d} {r['covered']:8.2f} {r['worst_mm']:9.1f} {str(r['p99_mm']):>7s} {str(r['excess']):>7s} {str(r.get('r_min_mm'))+'..'+str(r.get('r_max_mm')):>16s}")
    if out: json.dump(res, open(out, "w"), indent=2)
    return res

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else None)
