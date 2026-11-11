#!/usr/bin/env python3
"""Piper Studio model viewer: look at the robot description as the stack builds it, in a browser (Three.js).

    scripts/viewer.sh start          # http://127.0.0.1:8781/  (loopback; the :8700 gateway route /piper-studio/viewer exposes it on the tailnet)

The page asks `/api/model?...` for the model, which this server expands LIVE from the xacro (`piper_description.robot_description`) and
the SRDF, so after you change the description you only reload the page. Meshes are read in place from this workspace (nothing copied)
through `/mesh?p=<absolute path>`, and only .stl / .dae files under the workspace and its external/ clones are served.
Independent of any other viewer. Needs the workspace environment (scripts/viewer.sh sources it).
"""

from __future__ import annotations

import gzip
import hashlib
import json
import mimetypes
import os
import re
import sys
import threading
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

WS = Path(os.environ.get("PIPER_STUDIO_WS", Path(__file__).resolve().parents[2])).resolve()
STATIC = Path(__file__).resolve().parent / "static"
MESH_ROOTS = [WS.resolve()]
MESH_SUFFIX = {".stl", ".dae"}
CACHE = Path(__file__).resolve().parent / "run" / "cache"      # light copies of the big meshes (git-ignored)
LITE_CELL = 0.0025                                                # m: vertex-clustering grid of the light meshes
_gz: dict = {}                                                    # (path, mtime) -> gzip bytes, so each file is compressed once


def floats(text, default="0 0 0"):
    return [float(v) for v in (text or default).split()]


def mesh_url(filename: str) -> str:
    path = filename.removeprefix("file://")
    if path.startswith("package://"):
        from ament_index_python.packages import get_package_share_directory
        pkg, rest = path[len("package://"):].split("/", 1)
        path = str(Path(get_package_share_directory(pkg)) / rest)
    return "mesh?p=" + path   # relative: the page is also served under a gateway prefix (/piper-studio/viewer/)


def geometry(g) -> dict | None:
    if g is None:
        return None
    if g.find("mesh") is not None:
        m = g.find("mesh")
        return {"type": "mesh", "url": mesh_url(m.get("filename")), "file": Path(m.get("filename")).name,
                "scale": floats(m.get("scale"), "1 1 1")}
    if g.find("box") is not None:
        return {"type": "box", "size": floats(g.find("box").get("size"))}
    if g.find("sphere") is not None:
        return {"type": "sphere", "radius": float(g.find("sphere").get("radius"))}
    if g.find("cylinder") is not None:
        c = g.find("cylinder")
        return {"type": "cylinder", "radius": float(c.get("radius")), "length": float(c.get("length"))}
    return None


def shapes(link, tag):
    out = []
    for el in link.findall(tag):
        geo = geometry(el.find("geometry"))
        if geo:
            o = el.find("origin")
            geo["xyz"], geo["rpy"] = floats(o.get("xyz") if o is not None else None), floats(o.get("rpy") if o is not None else None)
            out.append(geo)
    return out


def build_model(camera: str = "none", gripper: bool = True, calibration: str = "", prefix: str = "piper_", compare: str = "") -> dict:
    import xacro
    from piper_core import limits_from_urdf, named_poses_from_srdf, zero_pose
    from piper_description import robot_description

    maps = {"camera": camera, "gripper": gripper, "prefix": prefix}
    if camera == "zedxm":                       # the ZED X Mini goes with the user's own bracket
        maps["camera_mount"] = "zedxm_bracket"
    if camera != "none" and calibration:
        maps["camera_calibration"] = calibration
    root = ET.fromstring(robot_description("mock", **maps))
    links = []
    for l in root.findall("link"):
        inertial = l.find("inertial")
        links.append({
            "name": l.get("name"),
            "visual": shapes(l, "visual"), "collision": shapes(l, "collision"),
            "mass": float(inertial.find("mass").get("value")) if inertial is not None and inertial.find("mass") is not None else None,
            "com": floats(inertial.find("origin").get("xyz")) if inertial is not None and inertial.find("origin") is not None else None,
        })
    joints = []
    for j in root.findall("joint"):
        o, lim, mim = j.find("origin"), j.find("limit"), j.find("mimic")
        joints.append({
            "name": j.get("name"), "type": j.get("type"), "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": floats(o.get("xyz") if o is not None else None), "rpy": floats(o.get("rpy") if o is not None else None),
            "axis": floats(j.find("axis").get("xyz"), "1 0 0") if j.find("axis") is not None else None,
            "limit": None if lim is None or lim.get("lower") is None else {
                "lower": float(lim.get("lower")), "upper": float(lim.get("upper")),
                "velocity": float(lim.get("velocity", 0)), "effort": float(lim.get("effort", 0))},
            "mimic": None if mim is None else {"joint": mim.get("joint"), "multiplier": float(mim.get("multiplier", 1)),
                                                 "offset": float(mim.get("offset", 0))},
        })
    srdf_text = xacro.process_file(str(WS / "src/piper_control/config/moveit/piper.srdf.xacro"),
                                   mappings={"robot_name": root.get("name"), "prefix": prefix, "gripper": str(gripper).lower(), "camera": camera}).toxml()
    poses = named_poses_from_srdf(srdf_text)
    srdf = ET.fromstring(srdf_text)
    disabled = [[d.get("link1"), d.get("link2"), d.get("reason")] for d in srdf.findall("disable_collisions")]
    arm = [f"{prefix}joint{i}" for i in range(1, 7)]
    limits = limits_from_urdf(ET.tostring(root, encoding="unicode"))
    # the powered-off arm: shown for checking only, it is never a command target (docs/real_arm/arms/piper-01.md)
    slumped = {f"{prefix}joint{i}": v for i, v in enumerate([0.004, -0.0382, 0.0427, -0.0073, 0.3859, 0.0], 1)}
    firmware = (WS / "src/piper_description/config/firmware_values.yaml")
    if compare and camera != "none":      # overlay: the camera mount subtree of another calibration (version) of the same camera, links renamed `<name>__cmp`
        other = build_model(camera, gripper, compare, prefix)
        mount = prefix + "camera_mount_link"
        sub, grew = {mount}, True
        while grew:
            grew = False
            for j in other["joints"]:
                if j["parent"] in sub and j["child"] not in sub:
                    sub.add(j["child"]); grew = True
        for l in other["links"]:
            if l["name"] in sub:
                links.append({**l, "name": l["name"] + "__cmp"})
        for j in other["joints"]:
            if j["child"] in sub:
                joints.append({**j, "name": j["name"] + "__cmp", "child": j["child"] + "__cmp",
                               "parent": j["parent"] + "__cmp" if j["parent"] in sub else j["parent"]})
    return {
        "name": root.get("name"), "prefix": prefix, "options": {"camera": camera, "gripper": gripper, "calibration": calibration, "compare": compare},
        "links": links, "joints": joints, "arm_joints": arm, "tcp": f"{prefix}tcp_link",
        "poses": {k: v for k, v in poses.items()}, "zero_computed": zero_pose(limits, arm, 0.01),
        "slumped": slumped, "disabled_collisions": disabled,
        "firmware_values": firmware.read_text() if firmware.exists() else "",
        # calibrations that belong to the chosen camera: the files are named after their mount (agilex_mid_stand*, zedxm_bracket_v*)
        "calibrations": sorted(p.stem for p in (WS / "src/piper_description/config/camera_mounts").glob("*.yaml")
                               if p.stem.startswith("zedxm_bracket" if camera == "zedxm" else "agilex_mid_stand")),
        "git": git_head(),
    }


def lite_mesh(path: Path) -> Path:
    """A light binary STL of a visual or collision mesh (vertices clustered on a 2.5 mm grid), cached on disk; for slow connections."""
    key = hashlib.sha1(f"{path}:{path.stat().st_mtime_ns}:{LITE_CELL}".encode()).hexdigest()[:16]
    out = CACHE / f"{path.stem}-{key}.stl"
    if not out.exists():
        sys.path.insert(0, str(WS / "src/piper_description/tools"))
        from build_collision_meshes import cluster, load
        CACHE.mkdir(parents=True, exist_ok=True)
        mesh = cluster(load(path), LITE_CELL)
        tmp = out.with_suffix(".tmp.stl")
        mesh.export(tmp)
        tmp.replace(out)
    return out


def prewarm():
    """Build the light meshes of the default models in the background, so the first visitor does not wait for them."""
    try:
        for cam in ("none", "d435"):
            m = build_model(cam)
            for l in m["links"]:
                for s in l["visual"] + l["collision"]:
                    if s["type"] == "mesh":
                        lite_mesh(Path(s["url"].split("p=", 1)[1]))
        print("viewer: light meshes ready", flush=True)
    except Exception as e:
        print("viewer: prewarm failed:", e, flush=True)


def git_head() -> str:
    import subprocess
    try:
        return subprocess.run(["git", "-C", str(WS), "log", "-1", "--format=%h %s"], capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return ""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        sys.stderr.write("viewer: " + fmt % args + "\n")

    def send(self, code, body: bytes, ctype: str, etag: str | None = None, key=None):
        if etag and self.headers.get("If-None-Match") == etag:       # unchanged file: the browser keeps its copy
            self.send_response(304)
            self.send_header("ETag", etag)
            self.end_headers()
            return
        gz = "gzip" in self.headers.get("Accept-Encoding", "") and len(body) > 1024
        if gz:
            if key is not None and key in _gz:
                body = _gz[key]
            else:
                body = gzip.compress(body, 6)
                if key is not None:
                    _gz[key] = body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        if gz:
            self.send_header("Content-Encoding", "gzip")
        self.send_header("Vary", "Accept-Encoding")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-cache" if etag else "no-store")      # no-cache = revalidate with the ETag (cheap 304)
        if etag:
            self.send_header("ETag", etag)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        try:
            if url.path == "/api/model":
                model = build_model(q.get("camera", "none"), q.get("gripper", "true") == "true", q.get("calibration", ""), compare=q.get("compare", ""))
                return self.send(200, json.dumps(model).encode(), "application/json")
            if url.path == "/mesh":
                p = Path(q.get("p", "")).resolve()
                if p.suffix.lower() not in MESH_SUFFIX or not any(r == p or r in p.parents for r in MESH_ROOTS) or not p.is_file():
                    return self.send(404, b"not served", "text/plain")
                if q.get("lite") == "1":
                    p = lite_mesh(p)
                st = p.stat()
                return self.send(200, p.read_bytes(), mimetypes.guess_type(str(p))[0] or "application/octet-stream",
                                 etag=f'"{st.st_mtime_ns}-{st.st_size}"', key=(str(p), st.st_mtime_ns))
            name = "index.html" if url.path in ("/", "") else url.path.lstrip("/")
            f = (STATIC / name).resolve()
            if STATIC not in f.parents or not f.is_file():
                return self.send(404, b"not found", "text/plain")
            return self.send(200, f.read_bytes(), mimetypes.guess_type(str(f))[0] or "text/plain")
        except Exception as e:  # show the problem in the page instead of a dead connection
            import traceback
            return self.send(500, json.dumps({"error": str(e), "trace": traceback.format_exc()[-1500:]}).encode(), "application/json")


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8781
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    threading.Thread(target=prewarm, daemon=True).start()
    print(f"Piper Studio viewer on http://127.0.0.1:{port}/  (workspace {WS})", flush=True)
    srv.serve_forever()


if __name__ == "__main__":
    main()
