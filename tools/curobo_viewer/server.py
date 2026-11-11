"""Piper cuRobo model viewer: the models shipped in piper_assets (spheres, self-collision ignore list, URDF meshes), in the browser.
Serves index.html and /api/models, /api/model?name=..., /mesh?p=... (.stl/.dae under the piper_assets data directory only; nothing is copied). Binds to loopback; the :8700 gateway route
(artifacts.yaml, docs/CUROBO.md) exposes it on the tailnet. Models: piper_zedxm (the arm with the ZED X Mini on its bracket) and piper_none (the robot only); extra yml files of
experiments can be viewed with --extra DIR (they need `urdf_path` or <name>.urdf beside them).
    source scripts/env.sh && python tools/curobo_viewer/server.py 8782 [--extra DIR]        (scripts/curobo_viewer.sh start|stop|status)
"""
import json
import mimetypes
import re
import sys
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import yaml

import piper_assets as pa

HERE = Path(__file__).resolve().parent
EXTRA = None
CAMERA = {"piper_zedxm": True, "piper_none": None}


def _pose(camera, name):
    for g in ET.parse(pa.srdf_path(camera)).getroot().iter("group_state"):
        if g.get("name") == name and g.get("group") == "arm":
            return [float(next(j.get("value") for j in g.findall("joint") if j.get("name") == n)) for n in pa.JOINT_NAMES]
    return None


def model(name):
    if name in CAMERA:
        cam = CAMERA[name]
        top, urdf_file = pa.curobo_config(cam), pa.urdf_path(0.07, cam)
        prov = pa.curobo_provenance(cam)
        info = f"built for description {prov['description_commit']}, {'current' if pa.curobo_model_is_current(cam) else 'STALE: regenerate'}"
        poses = {n: _pose(cam, n) for n in ("zero", "ready")}
    else:                                                   # an experiment's yml
        raw = yaml.safe_load((EXTRA / (Path(name).name + ".yml")).read_text())
        top = raw.get("robot_cfg", raw)
        urdf_file = EXTRA / (name + ".urdf") if (EXTRA / (name + ".urdf")).exists() else Path(top["kinematics"]["urdf_path"])
        info, poses = "an experiment's model (not shipped)", {}
    k = top["kinematics"]
    text = re.sub(r"<!--.*?-->", "", Path(urdf_file).read_text(), flags=re.S)
    text = re.sub(r'filename="(?!/)([^"]*)"', lambda m: 'filename="%s"' % (Path(urdf_file).parent / m.group(1)).resolve(), text)   # relative mesh paths -> absolute
    return {"name": name, "urdf": text, "spheres": {link: [{"c": s["center"], "r": s["radius"]} for s in v] for link, v in k["collision_spheres"].items()},
            "ignore": k.get("self_collision_ignore", {}), "buffer": k.get("self_collision_buffer", {}), "sphere_buffer": k.get("collision_sphere_buffer", 0.0),
            "tool_frames": k["tool_frames"], "default_joint_position": k["cspace"]["default_joint_position"], "poses": poses, "info": info}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        try:
            if u.path in ("/", "/index.html"):
                return self.send(200, (HERE / "index.html").read_bytes(), "text/html; charset=utf-8")
            if u.path == "/api/models":
                names = list(CAMERA) + (sorted(p.stem for p in EXTRA.glob("*.yml")) if EXTRA else [])
                return self.send(200, json.dumps(names).encode(), "application/json")
            if u.path == "/api/model":
                return self.send(200, json.dumps(model(Path(q["name"][0]).name)).encode(), "application/json")
            if u.path == "/mesh":
                p = Path(q["p"][0]).resolve()
                roots = [pa.data_dir().resolve()] + ([EXTRA.resolve()] if EXTRA else [])
                if not any(r in p.parents for r in roots) or p.suffix.lower() not in (".stl", ".dae"):
                    return self.send(403, b"forbidden", "text/plain")
                return self.send(200, p.read_bytes(), mimetypes.guess_type(str(p))[0] or "application/octet-stream")
            self.send(404, b"not found", "text/plain")
        except Exception as e:                      # noqa: BLE001
            self.send(500, f"{type(e).__name__}: {e}".encode(), "text/plain")


if __name__ == "__main__":
    args = sys.argv[1:]
    if "--extra" in args:
        EXTRA = Path(args[args.index("--extra") + 1]).resolve()
    ThreadingHTTPServer(("127.0.0.1", int(args[0]) if args and args[0].isdigit() else 8782), H).serve_forever()
