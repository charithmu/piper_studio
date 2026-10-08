"""Convert the exported Piper URDF to USD with Isaac Sim 6.1's URDF importer (run in Isaac's env).

    python isaac/convert_urdf.py piper_isaac.urdf out_dir

The URDF comes from `ros2 run piper_description export_urdf.py --hardware isaac --physics`, so the USD
is generated from the same single description as every other backend.
"""

import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402

urdf, out = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
out.mkdir(parents=True, exist_ok=True)
config = URDFImporterConfig(
    urdf_path=str(urdf),
    usd_path=str(out),
    merge_fixed_joints=False,   # keep tcp_link, flange_link and the virtual gripper frames
    merge_mesh=False,
    collision_from_visuals=False,
    allow_self_collision=False,
    fix_base=True,
    joint_target_type="position",
)
usd = URDFImporter(config).import_urdf()
print("USD:", usd)
app.close()
