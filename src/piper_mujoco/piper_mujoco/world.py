"""A `piper_core.World` built into a MuJoCo model (the MuJoCo half of the world seam; the Isaac half is isaac/world.py `add_world`).

The robot's model is loaded by `PiperSim`; this adds what is around it through MuJoCo's own model-editing API (`MjSpec`), so nothing is written to disk and no
robot file is included by name. Meshes collide as their convex hull (MuJoCo's rule for mesh geoms); a mesh is placed by its file's own origin.
"""

from __future__ import annotations

import math
from pathlib import Path

import mujoco

_GEOM = {"box": mujoco.mjtGeom.mjGEOM_BOX, "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER, "sphere": mujoco.mjtGeom.mjGEOM_SPHERE, "mesh": mujoco.mjtGeom.mjGEOM_MESH}


def _half(body) -> list:
    """MuJoCo geom size from a `Body.size`: box half extents, cylinder (radius, half height), sphere radius."""
    if body.shape == "box":
        return [s / 2 for s in body.size]
    if body.shape == "cylinder":
        return [body.size[0], body.size[1] / 2, 0.0]
    if body.shape == "sphere":
        return [body.size[0], 0.0, 0.0]
    return [0.0, 0.0, 0.0]


def apply_world(spec: "mujoco.MjSpec", world) -> None:
    """Add the world's ground height, bodies and fixed cameras to `spec` (an `MjSpec` of the robot's scene)."""
    wb = spec.worldbody
    if world.ground_z is not None:
        floor = next((g for g in wb.geoms if g.name == "floor"), None)
        if floor is None:
            raise ValueError("the robot's scene has no 'floor' geom to place; give World(ground_z=None)")
        floor.pos = [floor.pos[0], floor.pos[1], world.ground_z]
    for b in world.bodies:
        kw = dict(name=b.name, type=_GEOM[b.shape], rgba=list(b.rgba), friction=list(b.friction))
        if b.shape == "mesh":
            path = Path(b.mesh)
            if not path.exists():
                raise FileNotFoundError(f"{b.name}: mesh file {path} does not exist")
            scale = b.size[0] if b.size else 1.0
            spec.add_mesh(name=f"mesh_{b.name}", file=str(path), scale=[scale, scale, scale])
            kw["meshname"] = f"mesh_{b.name}"
            if b.texture:
                tex = spec.add_texture(name=f"tex_{b.name}", type=mujoco.mjtTexture.mjTEXTURE_2D, file=str(b.texture))
                mat = spec.add_material(name=f"mat_{b.name}", specular=0.2, shininess=0.2)
                mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name
                kw["material"] = mat.name
                kw["rgba"] = [1, 1, 1, 1]
        else:
            kw["size"] = _half(b)
        if b.static:
            wb.add_geom(pos=list(b.pos), quat=list(b.quat), **kw)
        else:
            body = wb.add_body(name=b.name, pos=list(b.pos), quat=list(b.quat))
            body.add_freejoint(name=b.name)
            body.add_geom(mass=b.mass, **kw)
    for c in world.cameras:
        wb.add_camera(name=c.name, pos=list(c.pos), xyaxes=list(c.xyaxes), fovy=c.fovy_deg, resolution=list(c.resolution))


def fovy_from_hfov(hfov_deg: float, size: tuple) -> float:
    """Vertical field of view (degrees) of a pinhole camera of `size` (w, h) with a horizontal one of `hfov_deg`."""
    return math.degrees(2 * math.atan(math.tan(math.radians(hfov_deg) / 2) * size[1] / size[0]))
