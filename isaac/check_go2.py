"""Check the Go2 setup in Isaac: the dog's visual (its USD, placed around the arm) sits inside the planner's box for the dog, fills it (the box is the dog's bounding box, so every side touches), and stands on the ground plane.
The same World calls the server makes; the boxes, the dog's pose and the ground come from the built piper_assets data. (Blocking and contact reporting are checked by the GPU tests: scripts/isaac.sh check-go2.)

    scripts/isaac.sh check-go2            python isaac/check_go2.py <arm usd> <dog usd>
"""
import json
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from world import World  # noqa: E402

arm_usd, dog_usd = sys.argv[1], sys.argv[2]
data = Path(__file__).resolve().parents[1] / "src/piper_assets/piper_assets/data/go2"
mount = json.loads((data / "mount.json").read_text())
world_json = json.loads((data / "world_arm_base.json").read_text())
boxes = [{"name": n, "size": b["dims"], "pos": b["pose"][:3], "quat": b["pose"][3:], "visible": n != "dog"} for n, b in world_json.items()]

w = World(app, arm_usd, ground_z=mount["floor_z_in_arm_base"], contacts=True)
w.add_static_boxes(boxes)
w.add_visual_usd(dog_usd, mount["dog_in_arm_base"])
pts = []
for p in w.stage.Traverse(Usd.TraverseInstanceProxies()):
    if p.GetPath().pathString.startswith("/World/dog") and p.IsA(UsdGeom.Mesh):
        v = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(), dtype=float)
        T = np.array(UsdGeom.Xformable(p).ComputeLocalToWorldTransform(0.0)).T
        pts.append((T @ np.hstack([v, np.ones((len(v), 1))]).T).T[:, :3])
pts = np.vstack(pts)
lo, hi = pts.min(0), pts.max(0)
dog = world_json["dog"]
blo, bhi = np.array(dog["pose"][:3]) - np.array(dog["dims"]) / 2, np.array(dog["pose"][:3]) + np.array(dog["dims"]) / 2
inside = bool(np.all(lo >= blo - 2e-3) and np.all(hi <= bhi + 2e-3))
fills = float(np.abs(np.concatenate([lo - blo, bhi - hi])).max()) * 1000
floor = abs(lo[2] - mount["floor_z_in_arm_base"]) * 1000
print(f"[check_go2] dog visual: {len(pts)} vertices; bounding box {np.round(lo, 4).tolist()} .. {np.round(hi, 4).tolist()}")
print(f"[check_go2] planner's box for the dog: {np.round(blo, 4).tolist()} .. {np.round(bhi, 4).tolist()}; visual inside it: {inside}; largest gap between them {fills:.2f} mm")
print(f"[check_go2] lowest point of the dog is {floor:.2f} mm from the ground plane (z = {mount['floor_z_in_arm_base']:.4f} m)")
print("[check_go2] RESULT:", "ok" if inside and fills < 10.0 and floor < 2.0 else "FAIL")   # the box also covers the Go2 URDF's own collision shapes (the head primitives overhang the head mesh by up to ~8 mm)
app.close()
