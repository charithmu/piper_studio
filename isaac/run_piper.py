"""Run the Piper in Isaac Sim 6.1 and bridge it to ROS 2 (run in Isaac's own environment via isaac.sh).

    isaac.sh run [--video out.mp4] [--seconds N] [--gui]

Topics (Isaac's bundled ROS 2 Jazzy, Fast DDS, same ROS_DOMAIN_ID as the workspace):
    /clock                  simulation time
    /isaac/joint_states     measured state of all 9 DOFs (arm, gripper width, both fingers)
    /isaac/joint_commands   position targets by joint name (what ros2_control's JointStateTopicSystem sends)

The ROS side is unchanged: piper_bringup with backend:=isaac runs the shared controllers and MoveIt, and
JointStateTopicSystem exchanges the two joint topics with this process. The USD comes from the same single
description as every other backend (isaac/convert_urdf.py).
"""

import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.environ.get("PIPER_USD"), required=os.environ.get("PIPER_USD") is None)
ap.add_argument("--seconds", type=float, default=0.0, help="exit after this much simulated time (0 = run until signalled)")
ap.add_argument("--gui", action="store_true", help="open a window (default: headless)")
ap.add_argument("--video", type=Path, help="record the camera view to this mp4 (needs ffmpeg)")
ap.add_argument("--video-fps", type=float, default=30.0)
ap.add_argument("--camera", default="none", help="none | d435: publish the wrist camera topics (USD must contain the camera)")
ap.add_argument("--camera-pose", type=Path, help="camera pose in link6 (JSON from export_urdf.py --camera-pose-out); needed with --camera")
ap.add_argument("--scene", type=Path, help="JSON list of static objects (same file content as piper_bringup/config/scene_objects.yaml)")
ap.add_argument("--frame-hz", type=float, default=float(os.environ.get("PIPER_ISAAC_FRAME_HZ", "0")),
                help="app updates (and ROS publishing) per second; 0 = auto: 240 without rendering, 90 with a camera/video "
                     "(rendering costs ~6 ms per update). Higher = less control latency, more CPU")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

# Without a camera/video nothing needs rendering: skipping viewport updates cuts the per-frame cost a lot.
_needs_render = bool(args.video) or args.gui or args.camera != "none"
args.frame_hz = args.frame_hz or (90.0 if _needs_render else 240.0)
app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720,
                     "disable_viewport_updates": not _needs_render})

import numpy as np  # noqa: E402
import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import usdrt.Sdf  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

for ext in ("isaacsim.ros2.core", "isaacsim.ros2.bridge", "isaacsim.core.nodes"):
    enable_extension(ext)
app.update()

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.experimental.objects import DomeLight, GroundPlane  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402
from pxr import PhysxSchema, UsdPhysics  # noqa: E402

FRAME_HZ = float(args.frame_hz)
PHYSICS_HZ = FRAME_HZ * max(1, round(240.0 / FRAME_HZ))  # a whole number of physics substeps per frame (>= 240 Hz)  # app updates (and ROS publishing) per second; PHYSICS_HZ / FRAME_HZ substeps each
ARM = [f"joint{i}" for i in range(1, 7)]
from servo_gains import GAINS  # noqa: E402  (same directory; kept equal to MuJoCo's servos)
INITIAL = {"joint2": 0.01, "joint3": -0.01}  # config/initial_positions.yaml

stage_utils.create_new_stage()
stage = omni.usd.get_context().get_stage()
GroundPlane("/World/ground", positions=[0, 0, 0])
DomeLight("/World/dome").set_intensities(2500)
stage_utils.add_reference_to_stage(usd_path=args.usd, path="/World/piper")
stage.GetPrimAtPath("/World/piper").GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
for _ in range(5):
    app.update()

# Frames are paced to wall-clock time by the loop below.
import carb.settings  # noqa: E402

_settings = carb.settings.get_settings()
_settings.set("/app/runLoops/main/rateLimitFrequency", float(FRAME_HZ))  # manual mode: dt is fixed at this rate
_settings.set("/app/runLoops/main/rateLimitEnabled", False)
_settings.set("/app/runLoops/present/rateLimitEnabled", False)
_settings.set("/app/runLoops/rendering_0/rateLimitEnabled", False)

# ---- physics: 200 Hz, drives, finger mimic ---------------------------------------------------------------
scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr().Set((0, 0, -1))
scene.CreateGravityMagnitudeAttr().Set(9.81)
px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
px.CreateTimeStepsPerSecondAttr().Set(int(PHYSICS_HZ))

joints = {p.GetName(): p for p in stage.Traverse()
          if p.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint")}
missing = [n for n in GAINS if n not in joints]
assert not missing, f"USD lacks joints {missing}; has {sorted(joints)}"
for name, (kp, kd, fmax) in GAINS.items():
    j = joints[name]
    drive = UsdPhysics.DriveAPI.Get(j, "angular" if j.GetTypeName() == "PhysicsRevoluteJoint" else "linear")
    drive.CreateStiffnessAttr().Set(float(kp))
    drive.CreateDampingAttr().Set(float(kd))
    drive.CreateMaxForceAttr().Set(float(fmax))

# The real arm's firmware and the MuJoCo model compensate gravity; a bare PhysX PD drive would sag (about 9 mm at the TCP)
# and the sag would add to every tracking error. Link bodies of the robot do not feel gravity here.
for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/World/piper") and p.HasAPI(UsdPhysics.RigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr(True)

roots = [p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
assert len(roots) == 1, roots
ROBOT = roots[0]

# ---- shared scene objects (same as Gazebo and MuJoCo) ---------------------------------------------------------------
if args.scene:
    from pxr import Gf, UsdGeom

    for o in json.loads(args.scene.read_text()):
        path = f"/World/scene/{o['name']}"
        if o["shape"] == "box":
            geom = UsdGeom.Cube.Define(stage, path)
            geom.GetSizeAttr().Set(1.0)
            scale = Gf.Vec3f(*o["size"])
        else:  # cylinder: [radius, height]
            geom = UsdGeom.Cylinder.Define(stage, path)
            geom.GetRadiusAttr().Set(float(o["size"][0]))
            geom.GetHeightAttr().Set(float(o["size"][1]))
            scale = Gf.Vec3f(1, 1, 1)
        geom.AddTranslateOp().Set(Gf.Vec3d(*o["pos"]))
        if o["shape"] == "box":
            geom.AddScaleOp().Set(scale)
        geom.GetDisplayColorAttr().Set([Gf.Vec3f(*o["rgba"][:3])])
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())

# ---- ROS 2 graph ------------------------------------------------------------------------------------------
og.Controller.edit(
    {"graph_path": "/ROS", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("simtime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("pub", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("sub", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "clock.inputs:execIn"),
            ("tick.outputs:tick", "pub.inputs:execIn"),
            ("tick.outputs:tick", "sub.inputs:execIn"),
            ("tick.outputs:tick", "ctrl.inputs:execIn"),
            ("ctx.outputs:context", "clock.inputs:context"),
            ("ctx.outputs:context", "pub.inputs:context"),
            ("ctx.outputs:context", "sub.inputs:context"),
            ("simtime.outputs:simulationTime", "clock.inputs:timeStamp"),
            ("simtime.outputs:simulationTime", "pub.inputs:timeStamp"),
            ("sub.outputs:jointNames", "ctrl.inputs:jointNames"),
            ("sub.outputs:positionCommand", "ctrl.inputs:positionCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("pub.inputs:topicName", "isaac/joint_states"),
            ("pub.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT)]),
            ("sub.inputs:topicName", "isaac/joint_commands"),
            ("ctrl.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT)]),
        ],
    },
)

# ---- wrist camera: a USD camera under link6 at the D435 colour optical frame, published like the RealSense driver ----------
if args.camera != "none":
    import omni.replicator.core as rep
    from isaacsim.ros2.nodes import Ros2CameraGraphConfig, create_ros2_camera_graph
    from pxr import Gf, UsdGeom

    cfg = json.loads(args.camera_pose.read_text())
    link = next(p for p in stage.Traverse() if p.GetName() == cfg["link"] and p.GetPath().pathString.startswith("/World/piper"))
    cam_path = f"{link.GetPath()}/wrist_camera"
    cam_prim = UsdGeom.Camera.Define(stage, cam_path)
    xf = UsdGeom.Xformable(cam_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*cfg["pos"]))
    qx, qy, qz, qw = cfg["quat_xyzw"]
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))
    aperture = 20.955  # mm (USD default); the focal length sets the field of view
    cam_prim.GetHorizontalApertureAttr().Set(aperture)
    cam_prim.GetVerticalApertureAttr().Set(aperture * cfg["height"] / cfg["width"])
    cam_prim.GetFocalLengthAttr().Set(aperture / (2.0 * math.tan(math.radians(cfg["hfov_deg"]) / 2.0)))
    cam_prim.GetClippingRangeAttr().Set(Gf.Vec2f(cfg["near"], cfg["far"]))
    wrist_rp = rep.create.render_product(cam_path, (cfg["width"], cfg["height"]))
    create_ros2_camera_graph(Ros2CameraGraphConfig(
        graph_path="/ROS_Camera", camera_prim=cam_path, frame_id="camera_color_optical_frame",
        camera_info_topic="/camera/color/camera_info", rgb_topic="/camera/color/image_raw",
        publish_depth=True, depth_topic="/camera/aligned_depth_to_color/image_raw",
        render_product_prim=wrist_rp.path))
    print(f"[piper_isaac] wrist camera at {cam_path}, {cfg['width']}x{cfg['height']}, hfov {cfg['hfov_deg']} deg", flush=True)

# ---- optional camera recording --------------------------------------------------------------------------------
video = None
if args.video:
    import omni.replicator.core as rep
    import imageio.v2 as imageio

    cam = rep.create.camera(position=(1.05, -0.95, 0.62), look_at=(0.12, 0.0, 0.2), focal_length=20)
    rp = rep.create.render_product(cam, (1280, 720))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([rp])
    args.video.parent.mkdir(parents=True, exist_ok=True)
    video = imageio.get_writer(str(args.video), fps=args.video_fps, codec="libx264", quality=7)

# ---- start -------------------------------------------------------------------------------------------------
# 60 Hz frames with 4 physics substeps each (240 Hz physics).
timeline = omni.timeline.get_timeline_interface()
timeline.set_time_codes_per_second(FRAME_HZ)
from omni.kit.loop import _loop as omni_loop  # noqa: E402

_loop = omni_loop.acquire_loop_interface()
_loop.set_manual_mode(True)
_loop.set_manual_step_size(1.0 / FRAME_HZ)  # each app.update advances the simulation by exactly this much
robot = Articulation(ROBOT)
app_utils.play(commit=True)
for _ in range(3):
    app.update()
names = list(robot.dof_names)
q0 = np.zeros((1, len(names)), dtype=np.float32)
for i, n in enumerate(names):
    q0[0, i] = INITIAL.get(n, 0.0)
robot.set_dof_positions(q0)
robot.set_dof_position_targets(q0)
print("[piper_isaac] dofs", names, flush=True)
print("[piper_isaac] ready: publishing /clock, /isaac/joint_states; listening /isaac/joint_commands", flush=True)

# URDF mimic (gripper_joint1 = +0.5 * gripper, gripper_joint2 = -0.5 * gripper) is applied here: the importer's
# PhysX variant carries no mimic, and PhysX mimic joints authored by hand did not couple the fingers.
FINGER_RATIO = {"gripper_joint1": 0.5, "gripper_joint2": -0.5}
i_grip = names.index("gripper")
i_fingers = [names.index(f) for f in FINGER_RATIO]
ratios = np.array([FINGER_RATIO[f] for f in FINGER_RATIO], dtype=np.float32)

stop = False
pid_file = Path(os.environ.get("PIPER_ISAAC_PIDFILE", "")) if os.environ.get("PIPER_ISAAC_PIDFILE") else None
if pid_file:
    pid_file.write_text(str(os.getpid()))
signal.signal(signal.SIGINT, lambda *_: globals().__setitem__("stop", True))
signal.signal(signal.SIGTERM, lambda *_: globals().__setitem__("stop", True))

frame_dt = 1.0 / args.video_fps
next_frame = 0.0
wall0 = time.monotonic()
last_log, last_sim, frames = wall0, 0.0, 0
cost_update = cost_fingers = 0.0
t0 = timeline.get_current_time()
while app.is_running() and not stop:
    t_a = time.perf_counter()
    app.update()
    t_b = time.perf_counter()
    targets = robot.get_dof_position_targets().numpy()
    robot.set_dof_position_targets((ratios * targets[0, i_grip]).reshape(1, -1), dof_indices=i_fingers)
    t_c = time.perf_counter()
    cost_update += t_b - t_a
    cost_fingers += t_c - t_b
    sim_t = timeline.get_current_time() - t0
    if video is not None and sim_t >= next_frame:
        frame = rgb.get_data()
        if frame is not None and getattr(frame, "size", 0):
            video.append_data(np.asarray(frame)[:, :, :3])
            next_frame += frame_dt
    if args.seconds and sim_t >= args.seconds:
        break
    frames += 1
    if time.monotonic() - last_log >= 5.0:
        wall = time.monotonic() - last_log
        print(f"[piper_isaac] {frames / wall:.0f} frames/s (target {FRAME_HZ:.0f}), real-time factor "
              f"{(sim_t - last_sim) / wall:.2f}; per frame: app.update {1000 * cost_update / max(frames, 1):.1f} ms, "
              f"finger coupling {1000 * cost_fingers / max(frames, 1):.1f} ms", flush=True)
        last_log, last_sim, frames, cost_update, cost_fingers = time.monotonic(), sim_t, 0, 0.0, 0.0
    ahead = sim_t - (time.monotonic() - wall0)  # real-time pacing: never run faster than wall clock
    if ahead > 0.002:
        time.sleep(ahead)

if video is not None:
    video.close()
print("[piper_isaac] stopping", flush=True)
if pid_file:
    pid_file.unlink(missing_ok=True)
app.close()
