"""Check Isaac's USD against the URDF: fingertip pose over random configurations and total mass; for the wrist-camera USD also the pose of
every wrist frame (tcp, bracket, camera, optical frame) and the world bounding box of the bracket's and the camera's collision mesh.

    scripts/isaac.sh check            (writes the references from the ROS side, then runs this in Isaac's env, for both USDs)
Compares in the base_link frame. No ROS needed here.
"""
import json
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from pxr import Gf, Usd, UsdGeom, UsdPhysics  # noqa: E402

usd, ref_file = sys.argv[1], Path(sys.argv[2])
PREFIX = sys.argv[3] if len(sys.argv) > 3 else "piper_"  # the description's name prefix
ref = json.loads(ref_file.read_text())
stage_utils.create_new_stage()
stage = omni.usd.get_context().get_stage()
stage_utils.add_reference_to_stage(usd_path=usd, path="/World/piper")
stage.GetPrimAtPath("/World/piper").GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityMagnitudeAttr().Set(0.0)  # pose check only: no sag
for _ in range(5):
    app.update()
from isaacsim.core.experimental.prims import Articulation  # noqa: E402

root = next(p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)).GetPath().pathString
byname = {p.GetName(): p for p in stage.Traverse()}
robot = Articulation(root)
app_utils.play(commit=True)
for _ in range(5):
    app.update()
names = list(robot.dof_names)


def world(prim):
    return np.array(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0.0)).T  # column-vector convention


def collision_box(link_prim):
    """World bounding box (min, max) of the collision meshes directly under a link prim (not of its child links), in metres."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    stack = list(link_prim.GetFilteredChildren(Usd.TraverseInstanceProxies()))      # the meshes are instanceable: look through the instance proxies
    while stack:
        p = stack.pop()
        if p.GetName().startswith(PREFIX) and p.GetName() != link_prim.GetName():
            continue                                   # a child link: its frame and its own geometry
        stack.extend(p.GetFilteredChildren(Usd.TraverseInstanceProxies()))
        if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI):
            pts = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(), dtype=float)               # exact vertices, not the (looser) authored extent
            w = (world(p) @ np.hstack([pts, np.ones((len(pts), 1))]).T).T[:, :3]
            lo, hi = np.minimum(lo, w.min(0)), np.maximum(hi, w.max(0))
    return lo, hi


perr, rerr, ferr, berr = [], [], {}, {}
for cfg in ref["configs"]:
    q = np.zeros((1, len(names)), dtype=np.float32)
    for n, v in zip(names, cfg["q"]):
        q[0, names.index(n)] = v
    for _ in range(3):
        robot.set_dof_positions(q)
        robot.set_dof_position_targets(q)
        app.update()
    T = np.linalg.inv(world(byname[PREFIX + "base_link"])) @ world(byname[PREFIX + "tcp_link"])
    perr.append(np.linalg.norm(T[:3, 3] - np.array(cfg["pos"])) * 1000)
    c = (np.trace(T[:3, :3].T @ np.array(cfg["rot"])) - 1) / 2
    rerr.append(np.degrees(np.arccos(np.clip(c, -1, 1))))
    inv_base = np.linalg.inv(world(byname[PREFIX + "base_link"]))
    for name, f in cfg.get("frames", {}).items():
        Tf = inv_base @ world(byname[PREFIX + name])
        ferr.setdefault(name, []).append((np.linalg.norm(Tf[:3, 3] - np.array(f["pos"])) * 1000,
                                          np.degrees(np.arccos(np.clip((np.trace(Tf[:3, :3].T @ np.array(f["rot"])) - 1) / 2, -1, 1)))))
    for name, (rlo, rhi) in cfg.get("col_boxes", {}).items():
        lo, hi = collision_box(byname[PREFIX + name])
        lo, hi = (inv_base @ np.append(lo, 1))[:3], (inv_base @ np.append(hi, 1))[:3]
        if name not in berr:
            print(f"[check_model] {name} collision box at config 0: usd {np.round(lo, 4).tolist()} .. {np.round(hi, 4).tolist()}, urdf {np.round(rlo, 4).tolist()} .. {np.round(rhi, 4).tolist()}")
        berr.setdefault(name, []).append(max(np.abs(lo - np.array(rlo)).max(), np.abs(hi - np.array(rhi)).max()) * 1000)
mass = 0.0
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.MassAPI):
        m = UsdPhysics.MassAPI(p).GetMassAttr().Get()
        mass += float(m) if m else 0.0
print(f"[check_model] dofs {names}")
print(f"[check_model] fingertip position error over {len(perr)} configs: max {max(perr):.3f} mm, mean {np.mean(perr):.3f} mm; "
      f"orientation max {max(rerr):.3f} deg")
print(f"[check_model] total mass: usd {mass:.4f} kg vs urdf {ref['total_mass']:.4f} kg")
for name, e in ferr.items():
    print(f"[check_model] frame {name}: position error max {max(a for a, _ in e):.3f} mm, orientation max {max(b for _, b in e):.3f} deg")
for name, e in berr.items():
    print(f"[check_model] collision mesh of {name}: world bounding box differs from the URDF's by at most {max(e):.2f} mm")
bad = [n for n, e in ferr.items() if max(a for a, _ in e) > 0.5 or max(b for _, b in e) > 0.05] + [n for n, e in berr.items() if not max(e) <= 1.5]      # not <= also catches "no collision mesh found" (nan)
print("[check_model] RESULT:", "FAIL " + ", ".join(bad) if bad or max(perr) > 0.5 else "ok")
app.close()
