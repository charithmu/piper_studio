"""The world around the robot, described once for every simulator (the SEAM between Piper Studio and a downstream project).

Studio owns the robot: arm, gripper (with its pads), wrist camera, their physics. A downstream project owns what is around it: tables, shelves, objects,
their friction and mass, extra cameras. It says so with a `World`; `piper_mujoco.PiperSim(world=...)` and `piper_isaac.PiperIsaacSim(world=...)` build the same
scene from it, so a scenario runs unchanged on either. Pure Python, JSON round trip (`to_json` is what the Isaac server receives).

Frame: the robot's base frame (`base_link`), which is the simulator world's origin. Quaternions are wxyz. Lengths in metres, mass in kg.

    World(bodies=[Body("table", "box", (1.4, 0.9, 0.04), pos=(0.55, 0, -0.02)),                 # static: its top face is the base's mounting plane (z = 0)
                  Body("cup", "cylinder", (0.04, 0.10), pos=(0.3, 0, 0.05), static=False, mass=0.2)], ground_z=-0.5)
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field

SHAPES = ("box", "cylinder", "sphere", "mesh")


@dataclass(frozen=True)
class Body:
    """One solid. `size`: box = full extents (x, y, z); cylinder = (radius, height) with its axis along z; sphere = (radius,); mesh = (scale,) of the file (default 1).
    A mesh collides as its convex hull (both simulators); `texture` is an image for its visual only. `friction` = (sliding, torsional, rolling) as in MuJoCo;
    Isaac uses the sliding value for static and dynamic friction. Where two surfaces meet, the larger friction wins, in both simulators."""
    name: str
    shape: str
    size: tuple = ()
    pos: tuple = (0.0, 0.0, 0.0)
    quat: tuple = (1.0, 0.0, 0.0, 0.0)
    rgba: tuple = (0.7, 0.7, 0.7, 1.0)
    static: bool = True
    mass: float = 0.05
    friction: tuple = (1.0, 0.005, 0.0001)
    mesh: str | None = None
    texture: str | None = None

    def __post_init__(self):
        n = {"box": 3, "cylinder": 2, "sphere": 1, "mesh": (0, 1)}
        if self.shape not in SHAPES:
            raise ValueError(f"{self.name}: shape must be one of {SHAPES}")
        want = n[self.shape]
        if len(self.size) not in (want if isinstance(want, tuple) else (want,)):
            raise ValueError(f"{self.name}: a {self.shape} needs size of length {want}, got {self.size}")
        if self.shape == "mesh" and not self.mesh:
            raise ValueError(f"{self.name}: a mesh body needs `mesh` (a file path)")
        if self.shape != "mesh" and (self.mesh or self.texture):
            raise ValueError(f"{self.name}: `mesh` and `texture` belong to shape 'mesh'")
        if len(self.pos) != 3 or len(self.quat) != 4 or len(self.rgba) != 4 or len(self.friction) != 3:
            raise ValueError(f"{self.name}: pos needs 3, quat 4, rgba 4 and friction 3 numbers")
        if not self.static and not self.mass > 0:
            raise ValueError(f"{self.name}: a dynamic body needs mass > 0")
        if any(f < 0 for f in self.friction):
            raise ValueError(f"{self.name}: friction must not be negative")


@dataclass(frozen=True)
class FixedCamera:
    """A camera fixed in the world for pictures (MuJoCo only for now; Isaac refuses a world that has one). `xyaxes`: the camera's x and y axes in the world (6 numbers, as MuJoCo)."""
    name: str
    pos: tuple
    xyaxes: tuple
    fovy_deg: float
    resolution: tuple = (640, 480)


@dataclass(frozen=True)
class World:
    bodies: tuple = ()
    ground_z: float | None = None       # height of an infinite ground plane in the base frame; None keeps the robot model's own (floor)
    cameras: tuple = field(default=())

    def __post_init__(self):
        object.__setattr__(self, "bodies", tuple(self.bodies))
        object.__setattr__(self, "cameras", tuple(self.cameras))
        names = [b.name for b in self.bodies] + [c.name for c in self.cameras]
        if len(set(names)) != len(names):
            raise ValueError("body and camera names must be unique")

    def to_json(self) -> str:
        return json.dumps({"schema": "piper.world/1", "ground_z": self.ground_z, "bodies": [asdict(b) for b in self.bodies], "cameras": [asdict(c) for c in self.cameras]})

    @classmethod
    def from_json(cls, text: str) -> "World":
        d = json.loads(text)
        if d.get("schema") != "piper.world/1":
            raise ValueError(f"not a piper.world/1 document: {d.get('schema')!r}")
        tup = lambda v: tuple(v) if isinstance(v, list) else v
        return cls(bodies=[Body(**{k: tup(v) for k, v in b.items()}) for b in d["bodies"]], ground_z=d["ground_z"],
                   cameras=[FixedCamera(**{k: tup(v) for k, v in c.items()}) for c in d["cameras"]])
