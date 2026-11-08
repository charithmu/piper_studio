"""The Piper's Isaac Sim world, shared by every front end: stage, USD, physics, servo drives, gravity compensation, finger coupling, scene.

One owner for these facts (ARCHITECTURE: one owner, thin wrappers). The front ends differ only in how commands arrive and state leaves:
  isaac/run_server.py   ROS-free: JSON lines over a unix socket (the `piper_isaac` client, an ArmBackend)
  isaac/run_piper.py    ROS 2 bridge nodes (OmniGraph); not yet moved onto this module (CLEANUP G13, phase 2)

Runs inside Isaac Sim's own Python environment (never in .venv). Import this module only after the `SimulationApp` exists: every Isaac import is
inside a method. Nothing Piper-specific is installed into Isaac's environment: the caller passes plain data (paths, lists).

    app = SimulationApp({...})
    world = World(app, usd, prefix="piper_", ctrl_hz=500, physics_hz=1000, ground_z=-0.5)
    world.add_scene(objects, table, dynamic=True)
    world.start(zero, gripper_width)
    world.set_targets(arm=q); world.step(); q = world.q()
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from servo_gains import GAINS  # same directory; kept equal to MuJoCo's by piper_sim/test/test_servo_gains.py

FINGER_RATIO = {"gripper_joint1": 0.5, "gripper_joint2": -0.5}      # URDF mimic: multiplier x gripper width (the importer's PhysX variant has no mimic)
OBJECT_MASS = 0.05                                                  # kg and friction: the MuJoCo scene's free bodies (tools/release/build_mujoco.py)
OBJECT_FRICTION = 1.0


class World:
    def __init__(self, app, usd, *, prefix="piper_", ctrl_hz=500.0, physics_hz=1000.0, ground_z=0.0, contacts=False):
        import carb.settings
        import omni.usd
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.objects import DomeLight, GroundPlane
        from pxr import PhysxSchema, UsdPhysics

        if abs(physics_hz / ctrl_hz - round(physics_hz / ctrl_hz)) > 1e-9 or physics_hz < ctrl_hz:
            raise ValueError("physics_hz must be a whole multiple of ctrl_hz")
        self.app, self.prefix, self.ctrl_hz, self.physics_hz = app, prefix, float(ctrl_hz), float(physics_hz)
        self.ctrl_dt = 1.0 / self.ctrl_hz
        self.steps = 0
        self.hooks = []                                     # called after every step with the world (video recording)
        self.objects = {}                                   # name -> (path, initial position, initial quaternion wxyz)
        self._contact_cb = None
        self.contacts = contacts                            # apply the PhysX contact report schema (must happen BEFORE start(): applying it to a running body invalidates the tensor views)

        stage_utils.create_new_stage()
        self.stage = omni.usd.get_context().get_stage()
        GroundPlane("/World/ground", positions=[0, 0, ground_z])
        DomeLight("/World/dome").set_intensities(2500)
        stage_utils.add_reference_to_stage(usd_path=str(usd), path="/World/piper")
        self.stage.GetPrimAtPath("/World/piper").GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
        for _ in range(5):
            app.update()

        s = carb.settings.get_settings()                    # manual stepping: one app.update advances the simulation by exactly 1 / ctrl_hz
        s.set("/app/runLoops/main/rateLimitFrequency", self.ctrl_hz)
        s.set("/app/runLoops/main/rateLimitEnabled", False)
        s.set("/app/runLoops/present/rateLimitEnabled", False)
        s.set("/app/runLoops/rendering_0/rateLimitEnabled", False)

        scene = UsdPhysics.Scene.Define(self.stage, "/World/physicsScene")
        scene.CreateGravityDirectionAttr().Set((0, 0, -1))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr().Set(int(self.physics_hz))

        joints = {p.GetName(): p for p in self.stage.Traverse() if p.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint")}
        missing = [prefix + n for n in GAINS if prefix + n not in joints]      # GAINS is keyed by the official joint names
        assert not missing, f"USD lacks joints {missing}; has {sorted(joints)}"
        for name, (kp, kd, fmax) in GAINS.items():
            j = joints[prefix + name]
            drive = UsdPhysics.DriveAPI.Get(j, "angular" if j.GetTypeName() == "PhysicsRevoluteJoint" else "linear")
            drive.CreateStiffnessAttr().Set(float(kp))
            drive.CreateDampingAttr().Set(float(kd))
            drive.CreateMaxForceAttr().Set(float(fmax))

        # The real arm's firmware and the MuJoCo model compensate gravity; a bare PhysX PD drive would sag (about 9 mm at the TCP).
        self.arm_bodies = []
        for p in self.stage.Traverse():
            if p.GetPath().pathString.startswith("/World/piper") and p.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)
                self.arm_bodies.append(p)
        roots = [p.GetPath().pathString for p in self.stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        assert len(roots) == 1, roots
        self.robot_path = roots[0]
        if contacts:
            self._enable_report(p for p in self.arm_bodies)

    # ------------------------------------------------------------------ scene
    def add_scene(self, objects=(), table=None, *, dynamic=False):
        """Objects as in piper_sim/config/scene_objects.yaml (name, shape box|cylinder, size, pos, rgba), in the base frame (= world).
        dynamic=False: static colliders (the ROS runner). dynamic=True: free rigid bodies (mass 0.05 kg, friction 1.0), like the MuJoCo scene.
        `table`: {size, top_z, pos_xy}: a static box whose top face is at top_z."""
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

        mat = UsdShade.Material.Define(self.stage, "/World/scene_material")          # friction 1.0; the larger of the two bodies' values wins (as in MuJoCo)
        UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        m = UsdPhysics.MaterialAPI(mat.GetPrim())
        m.CreateStaticFrictionAttr().Set(OBJECT_FRICTION)
        m.CreateDynamicFrictionAttr().Set(OBJECT_FRICTION)
        m.CreateRestitutionAttr().Set(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr().Set("max")

        def bind(prim):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")

        def geometry(path, o):
            if o["shape"] == "box":
                g = UsdGeom.Cube.Define(self.stage, path)
                g.GetSizeAttr().Set(1.0)
                return g, Gf.Vec3f(*o["size"])
            g = UsdGeom.Cylinder.Define(self.stage, path)           # [radius, height], axis z
            g.GetRadiusAttr().Set(float(o["size"][0]))
            g.GetHeightAttr().Set(float(o["size"][1]))
            return g, None

        if table:
            sx, sy, sz = table["size"]
            g = UsdGeom.Cube.Define(self.stage, "/World/scene/table")
            g.GetSizeAttr().Set(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(table["pos_xy"][0], table["pos_xy"][1], table["top_z"] - sz / 2))
            g.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
            g.GetDisplayColorAttr().Set([Gf.Vec3f(0.55, 0.45, 0.35)])
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            bind(g.GetPrim())
            self.table_path = "/World/scene/table"
        for o in objects:
            path = f"/World/scene/{o['name']}"
            if dynamic:
                body = UsdGeom.Xform.Define(self.stage, path)       # rigid body on the parent, the (scaled) collider on a child
                body.AddTranslateOp().Set(Gf.Vec3d(*o["pos"]))
                UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
                UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr().Set(OBJECT_MASS)
                g, scale = geometry(path + "/geom", o)
                if scale is not None:
                    g.AddScaleOp().Set(scale)
                UsdPhysics.CollisionAPI.Apply(g.GetPrim())
                bind(g.GetPrim())
            else:
                g, scale = geometry(path, o)
                g.AddTranslateOp().Set(Gf.Vec3d(*o["pos"]))
                if scale is not None:
                    g.AddScaleOp().Set(scale)
                UsdPhysics.CollisionAPI.Apply(g.GetPrim())
                bind(g.GetPrim())
            g.GetDisplayColorAttr().Set([Gf.Vec3f(*o["rgba"][:3])])
            self.objects[o["name"]] = (path, np.array(o["pos"], float), np.array([1.0, 0, 0, 0]))
        if self.contacts:
            self._enable_report([self.stage.GetPrimAtPath(p) for p, _, _ in self.objects.values()] + ([self.stage.GetPrimAtPath(self.table_path)] if getattr(self, 'table_path', None) else []))
        self.app.update()

    # ------------------------------------------------------------------ run
    def start(self, zero, gripper_width):
        """Play, create the articulation, put the arm at `zero` (6 values) with the gripper at `gripper_width`."""
        import isaacsim.core.experimental.utils.app as app_utils
        import omni.timeline
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from omni.kit.loop import _loop as omni_loop

        timeline = omni.timeline.get_timeline_interface()
        timeline.set_time_codes_per_second(self.ctrl_hz)
        loop = omni_loop.acquire_loop_interface()
        loop.set_manual_mode(True)
        loop.set_manual_step_size(self.ctrl_dt)             # each app.update advances the simulation by exactly this much
        self.robot = Articulation(self.robot_path)
        self.rigid = {n: RigidPrim(p) for n, (p, _, _) in self.objects.items()}
        app_utils.play(commit=True)
        for _ in range(3):
            self.app.update()
        names = list(self.robot.dof_names)
        self.names = names
        self.i_arm = [names.index(f"{self.prefix}joint{i}") for i in range(1, 7)]
        self.i_grip = names.index(self.prefix + "gripper")
        self.i_fingers = [names.index(self.prefix + f) for f in FINGER_RATIO]
        self.ratios = np.array(list(FINGER_RATIO.values()), dtype=np.float32)
        self.target = np.zeros(len(names), dtype=np.float32)
        self.reset(zero, gripper_width)

    def reset(self, zero, gripper_width):
        """Arm at `zero`, gripper at `gripper_width`, every object back where it started, all velocities zero. Sim time restarts at 0."""
        q = np.zeros((1, len(self.names)), dtype=np.float32)
        q[0, self.i_arm] = zero
        q[0, self.i_grip] = gripper_width
        q[0, self.i_fingers] = self.ratios * gripper_width
        self.robot.set_dof_positions(q)
        self.robot.set_dof_velocities(np.zeros_like(q))
        self.target[:] = q[0]
        self.robot.set_dof_position_targets(q)
        for name, (_, pos, quat) in self.objects.items():
            self.rigid[name].set_world_poses(positions=pos.reshape(1, 3), orientations=quat.reshape(1, 4))
            self.rigid[name].set_velocities(linear_velocities=np.zeros((1, 3)), angular_velocities=np.zeros((1, 3)))
        self.steps = 0

    @property
    def time(self):
        return self.steps * self.ctrl_dt

    def set_targets(self, arm=None, grip=None):
        if arm is not None:
            self.target[self.i_arm] = np.asarray(arm, dtype=np.float32)
        if grip is not None:
            self.target[self.i_grip] = grip
        self.robot.set_dof_position_targets(self.target.reshape(1, -1))
        self.robot.set_dof_position_targets((self.ratios * self.target[self.i_grip]).reshape(1, -1), dof_indices=self.i_fingers)

    def step(self, n=1):
        for _ in range(n):
            self.app.update()
            self.steps += 1
            for h in self.hooks:
                h(self)

    def q(self):
        return self.robot.get_dof_positions().numpy()[0][self.i_arm].astype(float)

    def dq(self):
        return self.robot.get_dof_velocities().numpy()[0][self.i_arm].astype(float)

    def gripper(self):
        return float(self.robot.get_dof_positions().numpy()[0][self.i_grip]), float(self.robot.get_dof_velocities().numpy()[0][self.i_grip])

    def object_poses(self):
        """{name: (position (3,), quaternion wxyz (4,))} in the base frame (= world)."""
        out = {}
        for name, prim in self.rigid.items():
            p, o = prim.get_world_poses()
            out[name] = (p.numpy()[0].astype(float), o.numpy()[0].astype(float))
        return out

    # ------------------------------------------------------------------ contacts
    @staticmethod
    def _enable_report(prims):
        from pxr import PhysxSchema

        for prim in prims:
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)

    def report_contacts(self, sink):
        """Call `sink(step, name_a, name_b, depth)` for every penetration (separation < 0) that involves an arm link (the fixed base excluded, like
        PiperSim: self, arm-table and arm-object). Names are the body names without the prefix. Needs `World(..., contacts=True)`."""
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        if not self.contacts:
            raise RuntimeError("World was created without contacts=True")
        base = "/" + self.prefix + "base_link"
        arm = {p.GetPath().pathString for p in self.arm_bodies if not p.GetPath().pathString.endswith(base)}

        def short(path):
            leaf = path.rsplit("/", 1)[-1]
            if path.startswith("/World/scene/") and leaf == "geom":
                return path.split("/")[3]
            return leaf[len(self.prefix):] if leaf.startswith(self.prefix) else leaf

        def on_contact(headers, data):
            for h in headers:
                a, b = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                if a not in arm and b not in arm:
                    continue
                for k in range(h.contact_data_offset, h.contact_data_offset + h.num_contact_data):
                    sep = float(data[k].separation)
                    if sep < 0.0:
                        sink(self.steps, short(a), short(b), -sep)

        self._contact_cb = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)
        return True
