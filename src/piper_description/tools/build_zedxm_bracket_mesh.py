#!/usr/bin/env python3
"""Import the ZED X Mini wrist bracket (user's Fusion design) into the description: millimetres -> metres, into the gripper-base frame, plus a light collision mesh.

    python tools/build_zedxm_bracket_mesh.py SRC_mm.stl [--offset-mm X Y Z] [--rot-x-deg D]   # writes meshes/<name>.stl and <name>_collision.stl

SRC is the Fusion export: millimetres, world axes, origin at the middle of the clamp ring (ring axis = Fusion Y). Version 2 (2026-11-11): the user's UCS at the gripper base
is 30.0 mm behind the ring origin (Y) and 0.174 deg rotated about X against the world axes (his measure of the camera point: world delta (0, 16.314, 91.710) = UCS
delta (0, 16.036, 91.759), same length 93.15), so `--name zedxm_bracket_v2 --offset-mm 0 30 0 --rot-x-deg 0.174` expresses (version 1 was `--name zedxm_bracket_v1`, no offset: the ring-centre frame) the mesh in that UCS: p_ucs = Rx(d) (p_src + offset).
The mesh origin is then the frame `piper_camera_mount_link` = gripper_base (translation 0 in config/camera_mounts/zedxm_bracket.yaml).
The collision mesh is the visual with vertices clustered on a 3 mm grid (not a hull: a hull would fill the ring bore and collide with the gripper).
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_collision_meshes import MESHES, STAND_CELL, cluster

ap = argparse.ArgumentParser()
ap.add_argument("src")
ap.add_argument("--name", required=True, help="output name, e.g. zedxm_bracket_v2")
ap.add_argument("--offset-mm", nargs=3, type=float, default=[0, 0, 0])
ap.add_argument("--rot-x-deg", type=float, default=0.0)
a = ap.parse_args()
m = trimesh.load(a.src, force="mesh")
m.apply_translation(a.offset_mm)
m.apply_transform(trimesh.transformations.rotation_matrix(np.radians(a.rot_x_deg), [1, 0, 0]))
m.apply_scale(0.001)
m.export(MESHES / f"{a.name}.stl")
cluster(m, STAND_CELL).export(MESHES / f"{a.name}_collision.stl")
print("bounds (m)", m.bounds.round(4).tolist())
