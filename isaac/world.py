"""The Piper's Isaac Sim world, shared by every front end: stage, USD, physics, servo drives, gravity compensation, finger coupling, scene.

One owner for these facts (ARCHITECTURE: one owner, thin wrappers). The front ends differ only in how commands arrive and state leaves:
  isaac/run_server.py   ROS-free: JSON lines over a unix socket (the `piper_isaac` client, an ArmBackend)
  isaac/run_piper.py    ROS 2 bridge nodes (OmniGraph); not yet moved onto this module (CLEANUP G13, phase 2)

Runs inside Isaac Sim's own Python environment (never in .venv). Import this module only after the `SimulationApp` exists: every Isaac import is
inside a method. Nothing Piper-specific is installed into Isaac's environment: the caller passes plain data (paths, lists).

    app = SimulationApp({...})
    world = World(app, usd, prefix="piper_", ctrl_hz=250, physics_hz=250, ground_z=-0.5)
    world.add_world(piper_world_dict)          # a piper_core.World as a dict (bodies, ground, cameras): the same description MuJoCo builds from
    world.start(zero, gripper_width)
    world.set_targets(arm=q); world.step(); q = world.q()
"""

from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np

from servo_gains import gains  # same directory; kept equal to MuJoCo's by piper_sim/test/test_servo_gains.py

FINGER_RATIO = {"gripper_joint1": 0.5, "gripper_joint2": -0.5}      # URDF mimic: multiplier x gripper width (the importer's PhysX variant has no mimic)
OBJECT_MASS = 0.05                                                  # kg and friction: the MuJoCo scene's free bodies (tools/release/build_mujoco.py)
OBJECT_FRICTION = 1.0


class World:
    def __init__(self, app, usd, *, prefix="piper_", ctrl_hz=250.0, physics_hz=250.0, ground_z=0.0, contacts=False, camera=True, actuation=None):
        import carb.settings
        import omni.usd
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.experimental.objects import DomeLight, GroundPlane
        from pxr import PhysxSchema, UsdPhysics

        if abs(physics_hz / ctrl_hz - round(physics_hz / ctrl_hz)) > 1e-9 or physics_hz < ctrl_hz:
            raise ValueError("physics_hz must be a whole multiple of ctrl_hz")
        self.app, self.prefix, self.ctrl_hz, self.physics_hz = app, prefix, float(ctrl_hz), float(physics_hz)
        self.act = actuation or {}                          # piper_assets.actuation(): the gripper's speed, force plateau, pad friction (the same numbers MuJoCo uses)
        self.ctrl_dt = 1.0 / self.ctrl_hz
        self.steps = 0
        self.hooks = []                                     # called after every step with the world (video recording); `rendered` says whether this step drew a frame
        self.render_hz = 0.0                                # frames per simulated second the app renders at (0 = never: physics only); a front end sets it when it needs pixels
        self.rendered = False
        self._next_render = 0.0
        self.gui = False
        self._last_gui_render = 0.0
        self.objects = {}                                   # name -> (path, initial position, initial quaternion wxyz)
        self._contact_cb = None
        self.contacts = contacts                            # apply the PhysX contact report schema (must happen BEFORE start(): applying it to a running body invalidates the tensor views)

        stage_utils.create_new_stage()
        self.stage = omni.usd.get_context().get_stage()
        self.ground_z = float(ground_z)
        GroundPlane("/World/ground", positions=[0, 0, ground_z])
        DomeLight("/World/dome").set_intensities(2500)
        stage_utils.add_reference_to_stage(usd_path=str(usd), path="/World/piper")
        self.stage.GetPrimAtPath("/World/piper").GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
        for _ in range(5):
            app.update()

        s = carb.settings.get_settings()                    # app.update only renders (see start()); the loop settings keep the timeline and its clock fixed-rate
        s.set("/app/runLoops/main/rateLimitFrequency", self.ctrl_hz)
        s.set("/app/runLoops/main/rateLimitEnabled", False)
        s.set("/app/runLoops/present/rateLimitEnabled", False)
        s.set("/app/runLoops/rendering_0/rateLimitEnabled", False)

        scene = UsdPhysics.Scene.Define(self.stage, "/World/physicsScene")
        scene.CreateGravityDirectionAttr().Set((0, 0, -1))
        scene.CreateGravityMagnitudeAttr().Set(9.81)
        PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr().Set(int(self.physics_hz))

        joints = {p.GetName(): p for p in self.stage.Traverse() if p.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint")}
        GAINS = gains(camera)                                  # the servo table of the model that is loaded (with or without the wrist camera), generated from actuation.yaml (tools/release/build_gains.py)
        missing = [prefix + n for n in GAINS if prefix + n not in joints]      # GAINS is keyed by the official joint names
        assert not missing, f"USD lacks joints {missing}; has {sorted(joints)}"
        for name, (kp, kd, fmax) in GAINS.items():
            j = joints[prefix + name]
            drive = UsdPhysics.DriveAPI.Get(j, "angular" if j.GetTypeName() == "PhysicsRevoluteJoint" else "linear")
            drive.CreateStiffnessAttr().Set(float(kp))
            drive.CreateDampingAttr().Set(float(kd))
            drive.CreateMaxForceAttr().Set(float(fmax))

        self._bind_pad_friction()

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

    # ------------------------------------------------------------------ the gripper's pads
    def _bind_pad_friction(self):
        """The fingers' collision shapes get the pad friction of actuation.yaml (the gripper is the robot's, so its friction is Studio's; MuJoCo's finger geoms carry the same number)."""
        from pxr import PhysxSchema, UsdPhysics, UsdShade

        pad = self.act.get("gripper", {}).get("pad_friction")
        if pad is None:
            return
        mat = UsdShade.Material.Define(self.stage, "/World/pad_material")
        UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        m = UsdPhysics.MaterialAPI(mat.GetPrim())
        m.CreateStaticFrictionAttr().Set(float(pad[0]))
        m.CreateDynamicFrictionAttr().Set(float(pad[0]))
        m.CreateRestitutionAttr().Set(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr().Set("max")
        from pxr import Usd

        fingers = [p for p in self.stage.Traverse() if p.GetPath().pathString.startswith("/World/piper") and p.GetName() in (f"{self.prefix}gripper_link1", f"{self.prefix}gripper_link2")]
        assert len(fingers) == 2, f"expected the two finger links, found {[str(p.GetPath()) for p in fingers]}"
        n = 0
        for finger in fingers:               # the colliders sit in instanced prototypes below the link, where nothing can be authored: bind on the link, stronger than anything below it
            UsdShade.MaterialBindingAPI.Apply(finger).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")
            n += sum(1 for q in Usd.PrimRange(finger, Usd.TraverseInstanceProxies()) if q.HasAPI(UsdPhysics.CollisionAPI))
        assert n >= 2, f"found {n} finger collision shapes under the two finger links"
        self.pad_colliders = n

    # ------------------------------------------------------------------ the world around the robot (piper_core.World)
    def add_world(self, spec):
        """Build a `piper_core.World` (as the dict `json.loads(World.to_json())` gives) into the stage: ground height, static and free bodies with their friction and mass, meshes
        (convex hull collision, texture as a preview surface). The same description MuJoCo's `apply_world` builds from. Fixed cameras are not supported here: refused."""
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

        if spec.get("cameras"):
            raise ValueError("fixed cameras in a World are MuJoCo only; this simulator's pictures are the wrist camera's and the recorder's")
        materials = {}

        def friction_material(friction):
            key = float(friction[0])
            if key not in materials:
                mat = UsdShade.Material.Define(self.stage, f"/World/materials/mu_{len(materials)}")
                UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
                m = UsdPhysics.MaterialAPI(mat.GetPrim())
                m.CreateStaticFrictionAttr().Set(key)
                m.CreateDynamicFrictionAttr().Set(key)
                m.CreateRestitutionAttr().Set(0.0)
                PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr().Set("max")      # the larger of the two surfaces wins, as in MuJoCo
                materials[key] = mat
            return materials[key]

        def shape(path, b):
            """The collider/visual prim of a body, placed relative to its parent; returns (prim, scale op or None)."""
            if b["shape"] == "box":
                g = UsdGeom.Cube.Define(self.stage, path)
                g.GetSizeAttr().Set(1.0)
                return g, Gf.Vec3f(*b["size"])
            if b["shape"] == "cylinder":
                g = UsdGeom.Cylinder.Define(self.stage, path)           # [radius, height], axis z
                g.GetRadiusAttr().Set(float(b["size"][0]))
                g.GetHeightAttr().Set(float(b["size"][1]))
                return g, None
            if b["shape"] == "sphere":
                g = UsdGeom.Sphere.Define(self.stage, path)
                g.GetRadiusAttr().Set(float(b["size"][0]))
                return g, None
            return self._mesh_prim(path, b), None

        if spec.get("ground_z") is not None and abs(self.ground_z - spec["ground_z"]) > 1e-9:
            raise ValueError("the World's ground_z differs from the ground this World object was created with: pass ground_z=spec['ground_z'] to World(...)")
        for b in spec["bodies"]:
            path = f"/World/scene/{b['name']}"
            pos, quat = Gf.Vec3d(*b["pos"]), Gf.Quatd(*b["quat"])
            if b["static"]:
                g, scale = shape(path, b)
                g.AddTranslateOp().Set(pos)
                g.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
                if scale is not None:
                    g.AddScaleOp().Set(scale)
                collider = g
            else:
                body = UsdGeom.Xform.Define(self.stage, path)           # rigid body on the parent, the (scaled) collider on a child
                body.AddTranslateOp().Set(pos)
                body.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)
                UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
                UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr().Set(float(b["mass"]))
                g, scale = shape(path + "/geom", b)
                if scale is not None:
                    g.AddScaleOp().Set(scale)
                collider = g
            UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
            UsdShade.MaterialBindingAPI.Apply(collider.GetPrim()).Bind(friction_material(b["friction"]), UsdShade.Tokens.weakerThanDescendants, "physics")
            if b["shape"] != "mesh" or not b.get("texture"):
                collider.GetDisplayColorAttr().Set([Gf.Vec3f(*b["rgba"][:3])])
            if not b["static"]:
                self.objects[b["name"]] = (path, np.array(b["pos"], float), np.array(b["quat"], float))
            else:
                self.static_bodies = getattr(self, "static_bodies", {})
                self.static_bodies[b["name"]] = path
        if self.contacts:
            self._enable_report([self.stage.GetPrimAtPath(p) for p, _, _ in self.objects.values()] + [self.stage.GetPrimAtPath(p) for p in getattr(self, "static_bodies", {}).values()])
        self.app.update()

    def _mesh_prim(self, path, b):
        """A mesh file as a USD mesh with a convex-hull collider (and its texture as a preview surface when the file has UVs)."""
        import trimesh
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Vt

        m = trimesh.load(b["mesh"], force="mesh", process=False)
        scale = float(b["size"][0]) if b.get("size") else 1.0
        g = UsdGeom.Mesh.Define(self.stage, path)
        g.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy((np.asarray(m.vertices, float) * scale).astype(np.float32)))
        g.GetFaceVertexCountsAttr().Set(Vt.IntArray.FromNumpy(np.full(len(m.faces), 3, dtype=np.int32)))
        g.GetFaceVertexIndicesAttr().Set(Vt.IntArray.FromNumpy(np.asarray(m.faces, dtype=np.int32).reshape(-1)))
        g.GetSubdivisionSchemeAttr().Set("none")
        UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr().Set("convexHull")
        uv = getattr(m.visual, "uv", None)
        if b.get("texture") and uv is not None and len(uv) == len(m.vertices):
            st = UsdGeom.PrimvarsAPI(g).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
            st.Set(Vt.Vec2fArray.FromNumpy(np.asarray(uv, np.float32)))
            mat = UsdShade.Material.Define(self.stage, f"{path}_visual")
            shader = UsdShade.Shader.Define(self.stage, f"{path}_visual/surface")
            shader.CreateIdAttr("UsdPreviewSurface")
            tex = UsdShade.Shader.Define(self.stage, f"{path}_visual/texture")
            tex.CreateIdAttr("UsdUVTexture")
            tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(b["texture"])))
            reader = UsdShade.Shader.Define(self.stage, f"{path}_visual/st")
            reader.CreateIdAttr("UsdPrimvarReader_float2")
            reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8)
            mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
        elif b.get("texture"):
            raise ValueError(f"{b['name']}: the texture needs one UV per vertex in the mesh file (got {None if uv is None else len(uv)} for {len(m.vertices)} vertices)")
        return g

    def add_scene(self, objects=(), table=None, *, dynamic=False):
        """The ROS runner's static scene (objects as in piper_sim/config/scene_objects.yaml; `table`: {size, top_z, pos_xy}) as a World: kept for isaac/run_piper.py."""
        bodies = []
        if table:
            sx, sy, sz = table["size"]
            bodies.append({"name": "table", "shape": "box", "size": [sx, sy, sz], "pos": [table["pos_xy"][0], table["pos_xy"][1], table["top_z"] - sz / 2], "quat": [1, 0, 0, 0],
                           "rgba": [0.55, 0.45, 0.35, 1.0], "static": True, "mass": 0.05, "friction": [OBJECT_FRICTION, 0.005, 0.0001], "mesh": None, "texture": None})
        for o in objects:
            bodies.append({"name": o["name"], "shape": o["shape"], "size": list(o["size"]), "pos": list(o["pos"]), "quat": [1, 0, 0, 0], "rgba": list(o["rgba"]), "static": not dynamic,
                           "mass": OBJECT_MASS, "friction": [OBJECT_FRICTION, 0.005, 0.0001], "mesh": None, "texture": None})
        self.add_world({"bodies": bodies, "ground_z": None, "cameras": []})

    # ------------------------------------------------------------------ static world (the dog and the plate of the Go2 setup)
    def add_static_boxes(self, boxes):
        """Static colliders from the planner's world: [{name, size [x, y, z], pos [x, y, z], quat [w, x, y, z], rgba, visible}] in the base frame (= world), the SAME boxes cuRobo plans against
        (piper_assets.world). They are not part of the arm's articulation (so the arm collides with them, unlike links of its own articulation), friction like the other scene objects, contact reporting on.
        `visible: false` keeps the collider and hides its drawing (the dog is drawn by its own USD)."""
        from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema

        mat = UsdShade.Material.Define(self.stage, "/World/scene_material")
        UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        m = UsdPhysics.MaterialAPI(mat.GetPrim())
        m.CreateStaticFrictionAttr().Set(OBJECT_FRICTION)
        m.CreateDynamicFrictionAttr().Set(OBJECT_FRICTION)
        m.CreateRestitutionAttr().Set(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr().Set("max")
        self.static_boxes = {}
        for b in boxes:
            path = f"/World/world/{b['name']}"
            g = UsdGeom.Cube.Define(self.stage, path)
            g.GetSizeAttr().Set(1.0)
            g.AddTranslateOp().Set(Gf.Vec3d(*b["pos"]))
            g.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(*b.get("quat", [1.0, 0.0, 0.0, 0.0])))
            g.AddScaleOp().Set(Gf.Vec3f(*b["size"]))
            g.GetDisplayColorAttr().Set([Gf.Vec3f(*b.get("rgba", [0.5, 0.5, 0.5, 1.0])[:3])])
            UsdPhysics.CollisionAPI.Apply(g.GetPrim())
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            if not b.get("visible", True):
                UsdGeom.Imageable(g).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            self.static_boxes[b["name"]] = path
        if self.contacts:
            self._enable_report([self.stage.GetPrimAtPath(p) for p in self.static_boxes.values()])
        self.app.update()

    def add_visual_usd(self, usd, T, path="/World/dog"):
        """A USD asset as scenery only (no physics: its `Physics` variant is `none`), placed by the 4x4 pose T (column-vector convention) in the base frame (= world)."""
        import isaacsim.core.experimental.utils.stage as stage_utils
        from pxr import Gf, UsdGeom

        stage_utils.add_reference_to_stage(usd_path=str(usd), path=path)
        prim = self.stage.GetPrimAtPath(path)
        variants = prim.GetVariantSets()
        if variants.HasVariantSet("Physics"):
            variants.GetVariantSet("Physics").SetVariantSelection("none")
        x = UsdGeom.Xformable(prim)
        x.ClearXformOpOrder()
        x.AddTransformOp().Set(Gf.Matrix4d(np.asarray(T, float).T.tolist()))            # pxr matrices are row-vector (transposed)
        self.app.update()

    # ------------------------------------------------------------------ run
    def start(self, zero, gripper_width):
        """Play, create the articulation, put the arm at `zero` (6 values) with the gripper at `gripper_width`."""
        import carb.settings
        import isaacsim.core.experimental.utils.app as app_utils
        import omni.timeline
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from omni.kit.loop import _loop as omni_loop
        from omni.physx import get_physx_simulation_interface

        timeline = omni.timeline.get_timeline_interface()
        timeline.set_time_codes_per_second(self.ctrl_hz)
        loop = omni_loop.acquire_loop_interface()
        loop.set_manual_mode(True)
        loop.set_manual_step_size(self.ctrl_dt)             # the timeline clock advances by this much per rendered frame; physics time is World.time
        self.robot = Articulation(self.robot_path)
        self.rigid = {n: RigidPrim(p) for n, (p, _, _) in self.objects.items()}
        app_utils.play(commit=True)
        for _ in range(3):
            self.app.update()
        # Physics and rendering are decoupled (as in Isaac Lab): from here app.update() draws and syncs the stage but does not advance physics; step() advances
        # PhysX itself, once per control step, and calls app.update() only when a frame is due.
        carb.settings.get_settings().set_bool("/app/player/playSimulations", False)
        self._physx = get_physx_simulation_interface()
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
        self._next_render = 0.0

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

    def render(self):
        """Draw one frame now (no physics time passes). Render products and the stage reflect the state after the last step."""
        self.app.update()
        self._last_gui_render = time.monotonic()

    def step(self, n=1):
        """n control steps of physics (1 / ctrl_hz each; PhysX substeps at physics_hz); a frame is drawn only when `render_hz` says one is due."""
        for _ in range(n):
            self._physx.simulate(self.ctrl_dt, self.time)
            self._physx.fetch_results()
            self.steps += 1
            self.rendered = self.render_hz > 0 and self.time >= self._next_render - 1e-9
            gui_due = self.gui and time.monotonic() - self._last_gui_render >= 1 / 30
            if self.rendered:
                self._next_render = max(self._next_render + 1.0 / self.render_hz, self.time - 1.0 / self.render_hz)
            if self.rendered or gui_due:
                self.render()
                self.rendered = True
            for h in self.hooks:
                h(self)

    def q(self):
        return self.robot.get_dof_positions().numpy()[0][self.i_arm].astype(float)

    def dq(self):
        return self.robot.get_dof_velocities().numpy()[0][self.i_arm].astype(float)

    def gripper(self):
        """(opening in m, its speed): the PHYSICAL opening, the two finger joints' travel (joint1 - joint2). The `gripper` dof is a virtual joint that no object blocks,
        so it would read the commanded width even with a cube between the fingers."""
        q, v = self.robot.get_dof_positions().numpy()[0], self.robot.get_dof_velocities().numpy()[0]
        a, b = self.i_fingers
        return float(q[a] - q[b]), float(v[a] - v[b])

    def gripper_force(self):
        """The gripper's force in N as the real one reads it: negative while it grips. Each finger's joint force along its motion axis (the drive's push, which the object's contact
        balances), finger 1 closing towards negative and finger 2 towards positive, averaged."""
        e = self.robot.get_dof_projected_joint_forces().numpy()[0]
        a, b = self.i_fingers
        return float((e[a] - e[b]) / 2.0)

    def set_gripper_force_limit(self, newtons):
        """The most force each finger's drive may apply (the commanded effort, up to the gripper's plateau): `PhysxDrive maxForce`, set on both fingers."""
        efforts = np.full((1, len(self.i_fingers)), float(newtons), dtype=np.float32)
        self.robot.set_dof_max_efforts(efforts, dof_indices=self.i_fingers)

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
