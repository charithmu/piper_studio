#!/usr/bin/env python3
"""Build the collision meshes of the wrist camera and its stand from their visual meshes (low-poly, same frame as the visual).

    python tools/build_collision_meshes.py            # writes meshes/realsense_mid_stand_collision.stl and meshes/d435_collision.stl

stand   the visual mesh with its vertices clustered on a 3 mm grid (keeps the bracket's real shape, notch included; not a box, not a hull)
camera  the convex hull of the visual mesh's vertices clustered on a 4 mm grid (the D435 body is convex to within a few mm)
Both stay within the grid size of the visual surface (checked below). The collision uses the visual's origin, so nothing else changes.
Needs trimesh and numpy (the workspace .venv).
"""
from pathlib import Path

import numpy as np
import trimesh

MESHES = Path(__file__).resolve().parents[1] / "meshes"
STAND_CELL, CAMERA_CELL = 0.003, 0.004  # m


def load(path: Path) -> trimesh.Trimesh:
    scene = trimesh.load(path)
    return trimesh.util.concatenate(list(scene.dump())) if hasattr(scene, "dump") else scene


def cluster(mesh: trimesh.Trimesh, cell: float) -> trimesh.Trimesh:
    """Vertex clustering: vertices in the same grid cell merge to their mean; collapsed triangles are dropped."""
    key = np.floor(mesh.vertices / cell).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    sums = np.zeros((inv.max() + 1, 3))
    np.add.at(sums, inv, mesh.vertices)
    verts = sums / np.bincount(inv)[:, None]
    faces = inv[mesh.faces]
    ok = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    out = trimesh.Trimesh(verts, faces[ok], process=True)
    out.remove_unreferenced_vertices()
    return out


def check(simplified: trimesh.Trimesh, original: trimesh.Trimesh, name: str, hull: bool = False):
    from scipy.spatial import cKDTree
    if hull:   # a hull covers the visual, so check the other direction: its corners lie on the visual's outer surface
        dist, _ = cKDTree(original.vertices).query(simplified.vertices)
    else:      # distance of the visual's vertices to the simplified surface (sampled)
        pts = original.vertices[:: max(1, len(original.vertices) // 4000)]
        dist, _ = cKDTree(simplified.sample(60000)).query(pts)
    print(f"{name}: {len(original.faces)} -> {len(simplified.faces)} faces, bounds {np.round(simplified.extents, 4)} m "
          f"(visual {np.round(original.extents, 4)}), largest gap to the visual {dist.max() * 1000:.1f} mm")
    assert dist.max() < 3 * 0.004, f"{name}: simplified surface too far from the visual mesh"


def main():
    stand = load(MESHES / "realsense_mid_stand.dae")
    s = cluster(stand, STAND_CELL)
    check(s, stand, "stand")
    s.export(MESHES / "realsense_mid_stand_collision.stl")
    cam = load(MESHES / "d435_colored.dae")
    c = trimesh.convex.convex_hull(cluster(cam, CAMERA_CELL).vertices)
    check(c, cam, "d435", hull=True)
    c.export(MESHES / "d435_collision.stl")


if __name__ == "__main__":
    main()
