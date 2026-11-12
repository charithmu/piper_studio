"""ROS-free Isaac Sim server for the Piper: the front end of the `piper_isaac` client (an ArmBackend). Run in Isaac's own environment by the client
(`piper_isaac.PiperIsaacSim` starts it with the same `env -i` / `gpu-run` pattern as `scripts/isaac.sh`), always through gpu-run, one process per run.

    python run_server.py --usd piper_isaac.usda --socket /path/run.sock --scene table --scene-json scene.json --zero '[0,0.01,-0.01,0,0,0]'

Protocol: one JSON object per line in, one per line out (`{"ok": true, ...}` or `{"ok": false, "error": "..."}`), one client at a time. The server holds NO rules:
the client applies the guard and the resampling (piper_core), the server only applies setpoints, steps physics in simulation time (never wall time) and
reports what it measured. Time advances only inside `run`, `settle`, `command`, `gripper` and `step`.

  hello                                  -> dof_names, ctrl_hz, physics_hz, objects, contacts (bool), prefix
  state                                  -> t (sim s since reset), q, dq (6 each), gripper
  reset {zero, width}                    -> arm at zero, gripper at width, objects back, sim time 0
  run {q: [[6] x N]}                     -> one control step per row: q_meas [[6] x N], contacts [[step, a, b, depth], ...] (step = 0-based control step of this call; the callback fires inside the step, before the counter advances)
  settle {goal, tol, vel_tol, max_s}     -> hold `goal` until within tol (rad) and slower than vel_tol, at most max_s: q_meas, contacts
  command {q}                            -> one setpoint, one control step: q_meas
  hold                                   -> targets := measured arm pose (no time passes)
  gripper {width, max_s}                 -> move the gripper and wait until it has settled (at most max_s): width
  objects                                -> {name: {pos, quat_wxyz}} in the base frame
  camera_info                            -> the wrist camera: width, height, hfov_deg, near, far, K (needs --camera zedxm)
  camera_pose                            -> T (4x4): the camera's optical frame (z forward, x right, y down) in the world = base frame, now
  camera_image                           -> width, height, rgb (base64 uint8 h*w*3), depth (base64 float32 h*w, metres along the optical axis, NaN outside near..far)
  record_start {path, fps, width, height, pos, look_at} / record_stop   (needs --render)
  quit
"""

import argparse
import json
import os
import queue
import select
import signal
import socket
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

ap = argparse.ArgumentParser()
ap.add_argument("--usd", required=True)
ap.add_argument("--socket", required=True, help="unix socket path (put it under the run directory; removed on exit)")
ap.add_argument("--prefix", default="piper_", help="prefix of every joint and link name in the USD (the description's prefix)")
ap.add_argument("--scene", choices=["empty", "table"], default="empty")
ap.add_argument("--scene-json", type=Path, help="JSON {objects: [...], table: {...}} as piper_assets.scene() returns it")
ap.add_argument("--world-json", type=Path, help="JSON list of static boxes [{name, size, pos, quat, rgba, visible}] in the base frame: the planner's world (the Go2 setup's dog and plate)")
ap.add_argument("--dog-usd", type=Path, help="USD of the dog's visual (no physics), placed by --dog-pose")
ap.add_argument("--dog-pose", help="JSON 4x4 pose of the dog's frame in the base frame (column-vector convention)")
ap.add_argument("--ground-z", type=float, help="height of the ground plane in the base frame (default 0, or -0.5 in the table scene)")
ap.add_argument("--zero", default="[0, 0.01, -0.01, 0, 0, 0]", help="JSON list: the zero pose (piper_assets.zero_pose())")
ap.add_argument("--camera", default="zedxm", help="zedxm (default): the ZED X Mini wrist camera (the USD with the bracket and camera, plus --camera-pose) | none: the robot only")
ap.add_argument("--camera-pose", type=Path, help="camera_pose.json of `scripts/isaac.sh build-usd` / the wheel (the optical frame in link6, size, field of view); needed unless --camera none")
ap.add_argument("--gripper-width", type=float, default=0.07)
ap.add_argument("--ctrl-hz", type=float, default=250.0)
ap.add_argument("--physics-hz", type=float, default=250.0)
ap.add_argument("--render", action="store_true", help="enable viewport updates so `record_start` works (frames are rendered only while recording, at the video fps)")
ap.add_argument("--gui", action="store_true", help="open the full Isaac Sim editor (default: headless); also enables recording")
args, _ = ap.parse_known_args()
if args.camera != "none" and args.camera_pose is None:
    sys.exit("--camera-pose is needed with the wrist camera; use --camera none for the robot only")
args.render = args.render or args.gui or args.camera != "none"      # rendering is needed for the wrist camera too (viewport updates stay off; replicator renders on request)

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720, "disable_viewport_updates": not args.render,
                     "extra_args": ["--/isaac/startup/ros_bridge_extension="] if args.gui else []},
                    experience=str(Path(os.environ["ISAAC_SIM_DIR"]) / "apps/isaacsim.exp.full.kit") if args.gui else "")

import numpy as np  # noqa: E402

from world import World  # noqa: E402

zero = json.loads(args.zero)
spec = json.loads(args.scene_json.read_text()) if args.scene_json else {"objects": [], "table": None}
table_scene = args.scene == "table"
world = World(app, args.usd, prefix=args.prefix, ctrl_hz=args.ctrl_hz, physics_hz=args.physics_hz,
              ground_z=args.ground_z if args.ground_z is not None else (-0.5 if table_scene else 0.0), contacts=True, camera=args.camera != "none")
if args.world_json:
    world.add_static_boxes(json.loads(args.world_json.read_text()))
if args.dog_usd:
    world.add_visual_usd(args.dog_usd, json.loads(args.dog_pose))
world.add_scene(spec.get("objects", []) if table_scene else [], spec.get("table") if table_scene else None, dynamic=True)   # MuJoCo's empty scene has no objects
world.start(zero, args.gripper_width)
world.gui = args.gui

# ---- the wrist camera: a USD camera under link6 at the ZED X Mini's left (colour) optical frame, rendered on request ----------------
wcam = {}
if args.camera != "none":
    import math

    import omni.replicator.core as rep
    from pxr import Gf, UsdGeom

    cfg = json.loads(args.camera_pose.read_text())
    link = next(p for p in world.stage.Traverse() if p.GetName() == cfg["link"] and p.GetPath().pathString.startswith("/World/piper"))
    cam_path = f"{link.GetPath()}/wrist_camera"
    cam_prim = UsdGeom.Camera.Define(world.stage, cam_path)
    xf = UsdGeom.Xformable(cam_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*cfg["pos"]))
    qx, qy, qz, qw = cfg["quat_xyzw"]
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(qw, qx, qy, qz))
    aperture = 20.955                                               # mm (USD default); the focal length sets the field of view
    cam_prim.GetHorizontalApertureAttr().Set(aperture)
    cam_prim.GetVerticalApertureAttr().Set(aperture * cfg["height"] / cfg["width"])
    cam_prim.GetFocalLengthAttr().Set(aperture / (2.0 * math.tan(math.radians(cfg["hfov_deg"]) / 2.0)))
    cam_prim.GetClippingRangeAttr().Set(Gf.Vec2f(cfg["near"], cfg["far"]))
    rp = rep.create.render_product(cam_path, (cfg["width"], cfg["height"]))
    wcam.update(cfg=cfg, path=cam_path, rgb=rep.AnnotatorRegistry.get_annotator("rgb"), depth=rep.AnnotatorRegistry.get_annotator("distance_to_image_plane"))
    wcam["rgb"].attach([rp])
    wcam["depth"].attach([rp])
    world.render(), world.render()                                  # the first rendered frames are empty
    print(f"[piper_isaac] wrist camera at {cam_path}, {cfg['width']}x{cfg['height']}, hfov {cfg['hfov_deg']} deg", flush=True)


def camera_optical_pose():
    """4x4 pose of the optical frame in the world: the USD camera looks along -z with +y up, the optical frame along +z with +y down."""
    m = UsdGeom.Xformable(world.stage.GetPrimAtPath(wcam["path"])).ComputeLocalToWorldTransform(0)
    T = np.array([[m[j][i] for j in range(4)] for i in range(4)])    # pxr matrices are row-vector (transposed)
    T[:3, :3] = T[:3, :3] @ np.diag([1.0, -1.0, -1.0])
    return T

contacts = []                                                   # (step, a, b, depth) since the last reply
CONTACTS_OK = False
try:
    CONTACTS_OK = world.report_contacts(lambda s, a, b, d: contacts.append([s, a, b, d]))
except Exception as e:  # noqa: BLE001  (an API difference must not kill the server; the client is told)
    print(f"[piper_isaac] contact report unavailable: {e!r}", flush=True)

rec = {"writer": None, "rgb": None, "queue": None, "thread": None, "next": 0.0, "dt": 1 / 30, "error": None}


def _encode_loop(q, writer):
    while True:
        frame = q.get()
        if frame is None:
            return
        if rec["error"] is None:
            try:
                writer.append_data(frame)
            except Exception as e:  # drain the queue so the simulation and record_stop cannot deadlock
                rec["error"] = e


def record_hook(w):
    """World renders only at the video's frame rate (render_hz), so every rendered step yields one fresh frame; H.264 encoding runs in a thread."""
    if rec["writer"] is not None and w.rendered and w.time >= rec["next"] - 1e-9:
        if rec["error"] is not None:
            raise RuntimeError(f"video encoding failed: {rec['error']}")
        frame = rec["rgb"].get_data()
        if frame is not None and getattr(frame, "size", 0):
            rec["queue"].put(np.array(frame)[:, :, :3])         # copy: the annotator reuses its buffer; put() blocks only if the encoder falls far behind
            rec["next"] += rec["dt"]


world.hooks.append(record_hook)


def _finish_recording():
    if rec["writer"] is None:
        return
    rec["queue"].put(None)
    rec["thread"].join()                                        # every queued frame is written before the file is closed
    try:
        rec["writer"].close()
    finally:
        rec["writer"], world.render_hz = None, 0.0
    if rec["error"] is not None:
        raise RuntimeError(f"video encoding failed: {rec['error']}")


if args.render and not args.gui:                                # headless recording needs only the recording camera
    from omni.kit.viewport.utility import get_active_viewport

    get_active_viewport().updates_enabled = False


def take_contacts(first_step):
    out = [[s - first_step, a, b, d] for s, a, b, d in contacts]
    contacts.clear()
    return out


def handle(c):
    cmd = c["cmd"]
    if cmd == "hello":
        return {"dof_names": world.names, "ctrl_hz": world.ctrl_hz, "physics_hz": world.physics_hz, "objects": sorted(world.objects), "contacts": CONTACTS_OK,
                "prefix": world.prefix, "render": args.render}
    if cmd == "state":
        g, _ = world.gripper()
        return {"t": world.time, "q": world.q().tolist(), "dq": world.dq().tolist(), "gripper": g}
    if cmd == "reset":
        world.reset(c.get("zero", zero), c.get("width", args.gripper_width))
        world.step(2)                                               # let the articulation and the objects register the new state
        world.steps = 0
        if rec["writer"] is not None:
            rec["next"] = rec["dt"]
        contacts.clear()
        return {}
    if cmd == "run":
        rows, first = c["q"], world.steps
        contacts.clear()
        out = []
        for row in rows:
            world.set_targets(arm=row)
            world.step()
            out.append(world.q().tolist())
        return {"q_meas": out, "contacts": take_contacts(first)}
    if cmd == "settle":
        goal, first, out = np.asarray(c["goal"], float), world.steps, []
        contacts.clear()
        world.set_targets(arm=goal)
        for _ in range(int(c["max_s"] / world.ctrl_dt)):
            if float(np.max(np.abs(world.q() - goal))) < c["tol"] and float(np.max(np.abs(world.dq()))) < c["vel_tol"]:
                break
            world.step()
            out.append(world.q().tolist())
        return {"q_meas": out, "contacts": take_contacts(first)}
    if cmd == "command":
        world.set_targets(arm=c["q"])
        world.step()
        return {"q_meas": world.q().tolist()}
    if cmd == "hold":
        world.set_targets(arm=world.q())
        return {}
    if cmd == "gripper":
        width = float(c["width"])
        world.set_targets(grip=width)
        for _ in range(int(c.get("max_s", 2.0) / world.ctrl_dt)):
            world.step()
            g, v = world.gripper()
            if abs(g - width) < 1e-4 and abs(v) < 1e-3:
                break
        return {"width": world.gripper()[0], "contacts": take_contacts(0)}
    if cmd == "objects":
        return {n: {"pos": p.tolist(), "quat_wxyz": q.tolist()} for n, (p, q) in world.object_poses().items()}
    if cmd == "camera_info":
        if not wcam:
            raise RuntimeError("the server was started with --camera none")
        c0 = wcam["cfg"]
        return {k: c0[k] for k in ("width", "height", "hfov_deg", "near", "far", "K") if k in c0}
    if cmd == "camera_pose":
        if not wcam:
            raise RuntimeError("the server was started with --camera none")
        return {"T": camera_optical_pose().tolist()}
    if cmd == "camera_image":
        if not wcam:
            raise RuntimeError("the server was started with --camera none")
        import base64
        for _ in range(4):                                          # physics time stays unchanged; the render product needs a few updates to show the present state (a stale first frame otherwise)
            world.render()
        c0 = wcam["cfg"]
        rgb = np.ascontiguousarray(np.asarray(wcam["rgb"].get_data())[..., :3], dtype=np.uint8)
        z = np.array(wcam["depth"].get_data(), dtype=np.float32).reshape(c0["height"], c0["width"])
        z[~np.isfinite(z) | (z < c0["near"]) | (z > c0["far"])] = np.nan
        return {"width": c0["width"], "height": c0["height"], "rgb": base64.b64encode(rgb.tobytes()).decode(), "depth": base64.b64encode(z.tobytes()).decode()}
    if cmd == "record_start":
        if rec["writer"] is not None:
            raise RuntimeError("recording already active; call record_stop first")
        if not args.render:
            raise RuntimeError("start the server with --render to record")
        import imageio.v2 as imageio
        import omni.replicator.core as rep

        fps = float(c.get("fps", 30))
        if not 0 < fps <= world.ctrl_hz:
            raise ValueError(f"fps must be in (0, {world.ctrl_hz}]")
        cam = rep.create.camera(position=tuple(c.get("pos", (1.05, -0.95, 0.62))), look_at=tuple(c.get("look_at", (0.12, 0.0, 0.2))), focal_length=20)
        rp = rep.create.render_product(cam, (int(c.get("width", 960)), int(c.get("height", 540))))
        rec["rgb"] = rep.AnnotatorRegistry.get_annotator("rgb")
        rec["rgb"].attach([rp])
        Path(c["path"]).parent.mkdir(parents=True, exist_ok=True)
        world.render(), world.render()                              # the first rendered frames are empty
        rec["queue"] = queue.Queue(maxsize=32)
        writer = imageio.get_writer(c["path"], fps=fps, codec="libx264", quality=7)
        rec["error"] = None
        rec["thread"] = threading.Thread(target=_encode_loop, args=(rec["queue"], writer), daemon=True)
        rec["thread"].start()
        rec["writer"] = writer
        world.render_hz, world._next_render = fps, world.time
        rec["dt"] = 1.0 / world.render_hz
        rec["next"] = world.time + rec["dt"]
        return {}
    if cmd == "record_stop":
        _finish_recording()
        return {}
    raise ValueError(f"unknown command {cmd!r}")


stop = False
signal.signal(signal.SIGINT, lambda *_: globals().__setitem__("stop", True))
signal.signal(signal.SIGTERM, lambda *_: globals().__setitem__("stop", True))
pid_file = Path(os.environ["PIPER_ISAAC_PIDFILE"]) if os.environ.get("PIPER_ISAAC_PIDFILE") else None
if pid_file:
    pid_file.write_text(str(os.getpid()))
sock_path = Path(args.socket)
sock_path.unlink(missing_ok=True)
srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(str(sock_path))
srv.listen(1)
srv.settimeout(1 / 30 if args.gui else 0.5)
print(f"[piper_isaac] ready: dofs {world.names}, ctrl {world.ctrl_hz:.0f} Hz, physics {world.physics_hz:.0f} Hz, contacts {CONTACTS_OK}, socket {sock_path}", flush=True)

quit_ = False
while not stop and not quit_ and app.is_running():
    if args.gui:
        world.render()                                          # UI events only; physics time stays unchanged
    try:
        conn, _ = srv.accept()
    except socket.timeout:
        continue
    conn.settimeout(None)
    with conn, conn.makefile("rwb") as f:
        pending = b""
        while not stop and app.is_running():
            if args.gui:
                world.render()
            if b"\n" not in pending:
                if not select.select([conn], [], [], 1 / 30 if args.gui else 0.5)[0]:
                    continue
                data = conn.recv(65536)
                if not data:
                    break                                       # client gone: wait for the next one
                pending += data
                if b"\n" not in pending:
                    continue
            line, pending = pending.split(b"\n", 1)
            try:
                c = json.loads(line)
                if c.get("cmd") == "quit":
                    f.write(b'{"ok": true}\n')
                    f.flush()
                    quit_ = True
                    break
                reply = {"ok": True, **handle(c)}
            except Exception as e:  # noqa: BLE001  (reported to the client; the server keeps running)
                reply = {"ok": False, "error": f"{type(e).__name__}: {e}"}
            f.write((json.dumps(reply) + "\n").encode())
            f.flush()

print("[piper_isaac] stopping", flush=True)
_finish_recording()
srv.close()
sock_path.unlink(missing_ok=True)
if pid_file:
    pid_file.unlink(missing_ok=True)
app.close()
