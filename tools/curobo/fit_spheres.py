"""Inscribed greedy-cover sphere fit for cuRobo, from the URDF collision meshes. The hand-tuned values are in policy.yaml (section fit).
Each sphere sits INSIDE the mesh with radius = its depth below the surface + m (so it never sticks out of the mesh by more than ~m); spheres are chosen
greedily to cover the mesh surface (a surface point counts as covered when it is within r + tol of a centre). Result: the sphere union hugs the mesh
(protrusion <= ~m, uncovered surface <= ~tol) instead of ballooning, which is what keeps near-contact poses (the zero pose) collision free.
Per-link policy is policy.yaml (explicit; reviewed by hand). Output: a copy of a builder YAML with new collision_spheres.
    python -I tools/curobo/fit_spheres.py <urdf> <builder.yml> <out.yml> [policy.yaml]
"""
import re, sys
from pathlib import Path
import numpy as np, trimesh, yaml
import xml.etree.ElementTree as ET
from scipy.spatial import cKDTree
from scipy.sparse import csr_matrix
from trimesh.transformations import euler_matrix, translation_matrix

POLICY_FILE = Path(__file__).with_name("policy.yaml")


def load_policy(path=POLICY_FILE):
    """policy.yaml -> (default overhang mm, tolerance mm, {link: policy}); 'default' holds the per-link defaults. Keys: max, stop, min_depth_mm, overhang_mm (per link),
    floor_z / floor_exempt (base only: spheres may not reach below floor_z, surface points below floor_z + floor_exempt are ignored), skip (no spheres)."""
    d = yaml.safe_load(Path(path).read_text())["fit"]
    return d["overhang_mm"], d["tolerance_mm"], {"default": d["default"], **d["links"]}


vec = lambda s: [float(x) for x in (s or "0 0 0").split()]
tf = lambda xyz, rpy: translation_matrix(xyz) @ euler_matrix(rpy[0], rpy[1], rpy[2], axes="sxyz")


def candidates(mesh, pitch=0.004, cap=5000, seed=0):
    v = mesh.voxelized(pitch).fill()
    pts = np.asarray(v.points)
    inside = pts      # filled voxels only: mesh.contains() uses ray parity and drops regions where the mesh has overlapping bodies (link5 has 6)
    rng = np.random.default_rng(seed)
    return inside[rng.permutation(len(inside))[:cap]] if len(inside) > cap else inside


def fit_link(mesh, pol, m, tol):
    S_dense = mesh.sample(60000); tree = cKDTree(S_dense)
    S = S_dense[:20000]
    floor = pol.get("floor_z")
    if floor is not None: S = S[S[:, 2] > floor + pol["floor_exempt"]]
    C = candidates(mesh)
    if floor is not None: C = C[C[:, 2] > floor + 0.003]
    depth = tree.query(C)[0]; keep = depth >= pol["min_depth_mm"] / 1000; C, depth = C[keep], depth[keep]
    R = depth + m
    if floor is not None: R = np.minimum(R, C[:, 2] - floor); k2 = R >= 0.003; C, R = C[k2], R[k2]
    Stree = cKDTree(S); rows, cols = [], []
    for i, (c, r) in enumerate(zip(C, R)):
        idx = Stree.query_ball_point(c, r + tol); rows += [i] * len(idx); cols += idx
    M = csr_matrix((np.ones(len(rows), dtype=np.float32), (rows, cols)), shape=(len(C), len(S)))
    unc = np.ones(len(S), dtype=np.float32); chosen = []
    while len(chosen) < pol["max"] and unc.mean() > pol["stop"]:
        gain = M @ unc; i = int(np.argmax(gain))
        if gain[i] <= 0: break
        chosen.append(i); unc = unc * (1 - M[i].toarray().ravel())
    return [(C[i], float(R[i])) for i in chosen], float(unc.mean())


def main(urdf, ymlp, outp, policy=POLICY_FILE):
    m_mm, tol_mm, POLICY = load_policy(policy)
    m, tol = float(m_mm) / 1000, float(tol_mm) / 1000
    root = ET.fromstring(re.sub(r"<!--.*?-->", "", open(urdf).read(), flags=re.S))
    d = yaml.safe_load(open(ymlp)); k = d.get("robot_cfg", d)["kinematics"]; new = {}
    for l in root.findall("link"):
        name = l.get("name"); pol = {**POLICY["default"], **POLICY.get(name, {})}
        if name not in k["collision_spheres"] and name not in POLICY: continue
        ms = []
        for c in l.findall("collision"):
            g = c.find("geometry"); o = c.find("origin"); T = tf(vec(o.get("xyz")) if o is not None else [0] * 3, vec(o.get("rpy")) if o is not None else [0] * 3)
            if g.find("mesh") is not None: mm = trimesh.load(str((Path(urdf).parent / g.find("mesh").get("filename")).resolve()), force="mesh")
            elif g.find("box") is not None: mm = trimesh.creation.box(vec(g.find("box").get("size")))
            else: continue
            mm = mm.copy(); mm.apply_transform(T); ms.append(mm)
        if not ms: continue
        if pol.get("skip"): new[name] = []; print(f"{name:14s} skipped (policy)"); continue
        sph, unc = fit_link(trimesh.util.concatenate(ms), pol, pol["overhang_mm"] / 1000 if "overhang_mm" in pol else m, tol)
        new[name] = [{"center": [round(float(x), 5) for x in c], "radius": round(r, 5)} for c, r in sph]
        print(f"{name:14s} {len(sph):3d} spheres  r {min(r for _, r in sph)*1000:.1f}..{max(r for _, r in sph)*1000:.1f} mm  uncovered {unc*100:.1f}%")
    k["collision_spheres"].update(new)
    k["collision_link_names"] = [x for x in k["collision_link_names"] if new.get(x, k["collision_spheres"].get(x))]
    for x in list(k["collision_spheres"]):
        if not k["collision_spheres"][x]: k["collision_spheres"].pop(x); k["self_collision_buffer"].pop(x, None)
    yaml.safe_dump(d, open(outp, "w"), sort_keys=False)

if __name__ == "__main__":
    main(*sys.argv[1:5])
