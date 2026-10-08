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
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720})

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

PHYSICS_HZ = 240.0
FRAME_HZ = 60.0  # app updates (and ROS publishing) per second; 4 physics substeps each
ARM = [f"joint{i}" for i in range(1, 7)]
# Provisional position-servo gains (stand-in for the unidentified firmware loop). Same kp as MuJoCo's inputs.xml;
# kd is the damping MuJoCo derives for dampratio=1 from the real link inertias (kv = 15.7/16.4/16.4/0.6/2.3/0.3 N*m*s/rad;
# gripper dampratio 2 -> 8 N*s/m), so the two simulators have equivalent servos. (stiffness, damping, max force)
GAINS = {"joint1": (400, 15.7, 100), "joint2": (400, 16.4, 100), "joint3": (400, 16.4, 100),
         "joint4": (100, 0.6, 100), "joint5": (100, 2.3, 100), "joint6": (50, 0.31, 100),
         "gripper": (400, 8, 10), "gripper_joint1": (400, 8, 10), "gripper_joint2": (400, 8, 10)}
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

roots = [p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
assert len(roots) == 1, roots
ROBOT = roots[0]

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
t0 = timeline.get_current_time()
while app.is_running() and not stop:
    app.update()
    targets = robot.get_dof_position_targets().numpy()
    robot.set_dof_position_targets((ratios * targets[0, i_grip]).reshape(1, -1), dof_indices=i_fingers)
    sim_t = timeline.get_current_time() - t0
    if video is not None and sim_t >= next_frame:
        frame = rgb.get_data()
        if frame is not None and getattr(frame, "size", 0):
            video.append_data(np.asarray(frame)[:, :, :3])
            next_frame += frame_dt
    if args.seconds and sim_t >= args.seconds:
        break
    ahead = sim_t - (time.monotonic() - wall0)  # real-time pacing: never run faster than wall clock
    if ahead > 0.002:
        time.sleep(ahead)

if video is not None:
    video.close()
print("[piper_isaac] stopping", flush=True)
if pid_file:
    pid_file.unlink(missing_ok=True)
app.close()
