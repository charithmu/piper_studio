#!/usr/bin/env python3
"""Import the ZED X Mini wrist bracket (user's Fusion design) into the description: millimetres -> metres, plus a light collision mesh.

    python tools/build_zedxm_bracket_mesh.py SRC_mm.stl      # writes meshes/zedxm_bracket.stl and meshes/zedxm_bracket_collision.stl

SRC is the Fusion export: millimetres, origin at the middle of the clamp ring (ring axis = Fusion Y). The frame is kept as it is, so the mesh origin
is the frame `piper_camera_mount_link` and config/camera_mounts/zedxm_bracket.yaml says where that frame is in gripper_base.
The collision mesh is the visual with vertices clustered on a 3 mm grid (not a hull: a hull would fill the ring bore and collide with the gripper).
"""
import sys
from pathlib import Path

import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_collision_meshes import MESHES, STAND_CELL, cluster

m = trimesh.load(sys.argv[1], force="mesh")
m.apply_scale(0.001)
m.export(MESHES / "zedxm_bracket.stl")
cluster(m, STAND_CELL).export(MESHES / "zedxm_bracket_collision.stl")
print("bounds (m)", m.bounds.round(4).tolist())
