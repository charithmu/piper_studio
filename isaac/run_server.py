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
  record_start {path, fps, width, height, pos, look_at} / record_stop   (needs --render)
  quit
"""

import argparse
import json
import os
import signal
import socket
import sys
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
ap.add_argument("--zero", default="[0, 0.01, -0.01, 0, 0, 0]", help="JSON list: the zero pose (piper_assets.zero_pose())")
ap.add_argument("--gripper-width", type=float, default=0.07)
ap.add_argument("--ctrl-hz", type=float, default=500.0)
ap.add_argument("--physics-hz", type=float, default=1000.0)
ap.add_argument("--render", action="store_true", help="enable viewport updates so `record_start` works (about 6 ms per control step slower)")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 1280, "height": 720, "disable_viewport_updates": not args.render})

import numpy as np  # noqa: E402

from world import World  # noqa: E402

zero = json.loads(args.zero)
spec = json.loads(args.scene_json.read_text()) if args.scene_json else {"objects": [], "table": None}
table_scene = args.scene == "table"
world = World(app, args.usd, prefix=args.prefix, ctrl_hz=args.ctrl_hz, physics_hz=args.physics_hz, ground_z=-0.5 if table_scene else 0.0, contacts=True)
world.add_scene(spec.get("objects", []) if table_scene else [], spec.get("table") if table_scene else None, dynamic=True)   # MuJoCo's empty scene has no objects
world.start(zero, args.gripper_width)

contacts = []                                                   # (step, a, b, depth) since the last reply
CONTACTS_OK = False
try:
    CONTACTS_OK = world.report_contacts(lambda s, a, b, d: contacts.append([s, a, b, d]))
except Exception as e:  # noqa: BLE001  (an API difference must not kill the server; the client is told)
    print(f"[piper_isaac] contact report unavailable: {e!r}", flush=True)

rec = {"writer": None, "rgb": None, "next": 0.0, "dt": 1 / 30}


def record_hook(w):
    if rec["writer"] is not None and w.time >= rec["next"] - 1e-9:
        frame = rec["rgb"].get_data()
        if frame is not None and getattr(frame, "size", 0):
            rec["writer"].append_data(np.asarray(frame)[:, :, :3])
            rec["next"] += rec["dt"]


world.hooks.append(record_hook)


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
    if cmd == "record_start":
        if not args.render:
            raise RuntimeError("start the server with --render to record")
        import imageio.v2 as imageio
        import omni.replicator.core as rep

        cam = rep.create.camera(position=tuple(c.get("pos", (1.05, -0.95, 0.62))), look_at=tuple(c.get("look_at", (0.12, 0.0, 0.2))), focal_length=20)
        rp = rep.create.render_product(cam, (int(c.get("width", 960)), int(c.get("height", 540))))
        rec["rgb"] = rep.AnnotatorRegistry.get_annotator("rgb")
        rec["rgb"].attach([rp])
        rec["dt"], rec["next"] = 1.0 / float(c.get("fps", 30)), world.time
        Path(c["path"]).parent.mkdir(parents=True, exist_ok=True)
        rec["writer"] = imageio.get_writer(c["path"], fps=float(c.get("fps", 30)), codec="libx264", quality=7)
        world.step(2)                                               # the first rendered frames are empty
        rec["next"] = world.time
        return {}
    if cmd == "record_stop":
        if rec["writer"] is not None:
            rec["writer"].close()
            rec["writer"] = None
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
srv.settimeout(0.5)
print(f"[piper_isaac] ready: dofs {world.names}, ctrl {world.ctrl_hz:.0f} Hz, physics {world.physics_hz:.0f} Hz, contacts {CONTACTS_OK}, socket {sock_path}", flush=True)

quit_ = False
while not stop and not quit_:
    try:
        conn, _ = srv.accept()
    except socket.timeout:
        continue
    conn.settimeout(None)
    with conn, conn.makefile("rwb") as f:
        while not stop:
            line = f.readline()
            if not line:
                break                                               # client gone: wait for the next one, the world stays as it is
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
if rec["writer"] is not None:
    rec["writer"].close()
srv.close()
sock_path.unlink(missing_ok=True)
if pid_file:
    pid_file.unlink(missing_ok=True)
app.close()
