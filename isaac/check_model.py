"""Check Isaac's USD against the URDF: fingertip pose over random configurations and total mass.

    scripts/isaac.sh check            (writes the reference from the ROS side, then runs this in Isaac's env)
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
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

usd, ref_file = sys.argv[1], Path(sys.argv[2])
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


perr, rerr = [], []
for cfg in ref["configs"]:
    q = np.zeros((1, len(names)), dtype=np.float32)
    for n, v in zip(names, cfg["q"]):
        q[0, names.index(n)] = v
    for _ in range(3):
        robot.set_dof_positions(q)
        robot.set_dof_position_targets(q)
        app.update()
    T = np.linalg.inv(world(byname["base_link"])) @ world(byname["tcp_link"])
    perr.append(np.linalg.norm(T[:3, 3] - np.array(cfg["pos"])) * 1000)
    c = (np.trace(T[:3, :3].T @ np.array(cfg["rot"])) - 1) / 2
    rerr.append(np.degrees(np.arccos(np.clip(c, -1, 1))))
mass = 0.0
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.MassAPI):
        m = UsdPhysics.MassAPI(p).GetMassAttr().Get()
        mass += float(m) if m else 0.0
print(f"[check_model] dofs {names}")
print(f"[check_model] fingertip position error over {len(perr)} configs: max {max(perr):.3f} mm, mean {np.mean(perr):.3f} mm; "
      f"orientation max {max(rerr):.3f} deg")
print(f"[check_model] total mass: usd {mass:.4f} kg vs urdf {ref['total_mass']:.4f} kg")
app.close()
