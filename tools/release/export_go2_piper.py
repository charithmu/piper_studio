#!/usr/bin/env python3
"""Export the Go2 + Piper assembly for simulators and planners that have no ROS (optional; not part of the wheels: the Go2 meshes are Unitree's, ~25 MB).

    source scripts/env.sh && python tools/release/export_go2_piper.py --out DIR [--plate-xyz "-0.02 0 0.072"] [--mount-rpy "0 0 0"] [--camera zedxm|none]

Writes, in DIR (put it under ~/data/ml/..., not in the repo; a SOURCE.md naming this tool is written next to the files):
  go2_piper.urdf          the whole robot, meshes referenced by relative paths (Go2 `base` is the root; the arm and the camera keep the `piper_` prefix)
  meshes/                 every mesh the URDF uses (the Go2's from go2_studio's unitree_ros checkout, the arm's and camera's from the workspace)
  world_base.json         the dog's boxes in the Go2 `base` frame (trunk, plate, head, radar, floor): collision for a simulator or a locomotion stack
  world_arm_base.json     the same boxes in the arm's base_link frame (without the plate): the cuRobo world, with the arm model from piper_assets
  mount.json              the mount used, `base_in_go2` (4x4) and the NOMINAL warning, until the dock plate is measured
Backends: Isaac/MuJoCo/Gazebo import go2_piper.urdf (fixed or floating base is their choice; the Go2's leg joints are the dog's own and are not driven here).
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import re
import shutil
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[2]
GO2_PKG = Path.home() / "projects/ros/go2_studio/third_party/unitree_ros/robots/go2_description"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--plate-xyz", default="-0.02 0 0.072")
    ap.add_argument("--plate-size", default="0.14 0.09 0.030")
    ap.add_argument("--mount-rpy", default="0 0 0")
    ap.add_argument("--camera", default="zedxm")
    ap.add_argument("--go2-pkg", default=str(GO2_PKG), help="the go2_description package dir (urdf/, dae/)")
    a = ap.parse_args()
    out = Path(a.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    from piper_description import arm_base_pose, go2_dock_world, go2_piper_description
    spec = importlib.util.spec_from_file_location("build_assets", WS / "tools/release/build_assets.py")
    build_assets = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(build_assets)

    floats = lambda s: tuple(float(x) for x in s.split())
    urdf = go2_piper_description(go2_urdf=Path(a.go2_pkg) / "urdf/go2_description.urdf", plate_xyz=a.plate_xyz, plate_size=a.plate_size,
                                 mount_rpy=a.mount_rpy, camera=a.camera)

    def go2_mesh(m):                      # package://go2_description/dae/x.dae  ->  meshes/go2_description/dae/x.dae
        rest = m.group(1)
        dst = out / "meshes/go2_description" / rest
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(Path(a.go2_pkg) / rest, dst)
        return f'filename="meshes/go2_description/{rest}"'

    # build_assets.localise_meshes resolves every filename="..." through ament: hide the Go2's under another attribute name first
    urdf = urdf.replace('filename="package://go2_description/', 'go2file="')
    urdf = build_assets.localise_meshes(urdf, out)
    urdf = re.sub(r'go2file="([^"]+)"', go2_mesh, urdf)
    (out / "go2_piper.urdf").write_text(urdf)

    mount = dict(plate_xyz=floats(a.plate_xyz), plate_size=floats(a.plate_size), mount_rpy=floats(a.mount_rpy))
    (out / "world_base.json").write_text(json.dumps(go2_dock_world(frame="base", ground=True, **mount), indent=1))
    world = go2_dock_world(frame="arm_base", ground=True, **mount)
    world.pop("dock_plate")
    (out / "world_arm_base.json").write_text(json.dumps(world, indent=1))
    (out / "mount.json").write_text(json.dumps({"NOMINAL": "placeholder pose, not measured on the Go2 expansion dock", **{k: list(v) for k, v in mount.items()},
                                                "base_in_go2": arm_base_pose(**mount).tolist(), "camera": a.camera}, indent=1))
    (out / "SOURCE.md").write_text(f"Owner: {WS} (tools/release/export_go2_piper.py). Go2 meshes: {a.go2_pkg} (Unitree, BSD-3). Nominal mount; regenerate after measuring the plate.\n")
    print(f"wrote {out}: go2_piper.urdf, {sum(1 for _ in (out / 'meshes').rglob('*') if _.is_file())} mesh files, world_base.json, world_arm_base.json, mount.json")


if __name__ == "__main__":
    sys.exit(main())
