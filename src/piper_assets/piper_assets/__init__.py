"""The Piper arm as data; see README.md. Reads the files in `data/` that tools/release/build_assets.py generated. No ROS."""

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from importlib import resources
from pathlib import Path

import numpy as np

try:
    from importlib.metadata import version as _version
    __version__ = _version("piper_assets")
except Exception:       # running from a source tree
    __version__ = "0.0.0+source"

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
BASE_LINK = "base_link"
TCP_LINK = "tcp_link"
GRIPPER_LINK = "gripper_base"          # the TCP frame is TCP_OFFSET along +z of this link (+z = approach direction)
TCP_OFFSET = (0.0, 0.0, 0.138)         # m, in GRIPPER_LINK
CAMERAS = (None, "zedxm")             # None = the robot only; "zedxm" = with the ZED X Mini wrist camera on its bracket (the default everywhere)


def data_dir() -> Path:
    path = Path(str(resources.files("piper_assets") / "data"))
    if not (path / "manifest.json").exists():
        raise FileNotFoundError(f"{path} has no generated data: run tools/release/build_assets.py (needs the ROS environment) or install the wheel")
    return path


def manifest() -> dict:
    return json.loads((data_dir() / "manifest.json").read_text())


def _camera(camera) -> str:
    """True / "zedxm" -> "zedxm" (with the wrist camera); None / False / "none" / "no_camera" -> "none" (the robot only)."""
    if camera is True or camera == "zedxm":
        return "zedxm"
    if camera in (None, False, "none", "no_camera"):
        return "none"
    raise ValueError(f"camera must be True/'zedxm' (with the camera) or None/False/'none' (robot only), got {camera!r}")


def urdf_path(gripper_width: float | None = 0.07, camera: str | bool | None = True) -> Path:
    """Flat URDF (WITH the ZED X Mini wrist camera by default; camera=None: the robot only) with the gripper fixed at `gripper_width` metres (0.0 closed .. 0.07 open); None keeps the moving gripper (mimic fingers).
    0.0 and 0.07 are shipped; other widths are made on first use in the user cache with absolute mesh paths."""
    cam, d = _camera(camera), data_dir()
    if gripper_width is None:
        return d / f"piper_{cam}_open.urdf"
    shipped = d / f"piper_{cam}_w{float(gripper_width)}.urdf"
    if shipped.exists():
        return shipped
    from piper_core.urdf_tools import fixed_gripper
    cache = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "piper_assets" / __version__
    out = cache / f"piper_{cam}_w{float(gripper_width):.6g}.urdf"
    if not out.exists():
        text = (d / f"piper_{cam}_open.urdf").read_text()
        text = re.sub(r'filename="(meshes/[^"]+)"', lambda m: f'filename="{d / m.group(1)}"', text)
        cache.mkdir(parents=True, exist_ok=True)
        out.write_text(fixed_gripper(text, float(gripper_width), ""))
    return out


def srdf_path(camera: str | bool | None = True) -> Path:
    return data_dir() / f"piper_{_camera(camera)}.srdf"


def urdf_fingerprint(urdf: str | Path, srdf: str | Path | None = None) -> str:
    """A hash of what a cuRobo sphere model depends on: every joint (type, parent, child, origin, axis, limits), every collision geometry (type, size, origin, the CONTENT of
    its mesh file) and the SRDF's disabled pairs. Masses, visuals and comments do not count. A model built for another fingerprint is stale (curobo_model_is_current)."""
    import hashlib
    import xml.etree.ElementTree as ET
    urdf = Path(urdf)
    root = ET.fromstring(re.sub(r"<!--.*?-->", "", urdf.read_text(), flags=re.S))
    parts = []
    for j in sorted(root.iter("joint"), key=lambda e: e.get("name")):
        if j.find("parent") is None:
            continue                                    # ros2_control <joint> entries
        o, a, lim = j.find("origin"), j.find("axis"), j.find("limit")
        parts.append(("joint", j.get("name"), j.get("type"), j.find("parent").get("link"), j.find("child").get("link"), None if o is None else (o.get("xyz"), o.get("rpy")),
                      None if a is None else a.get("xyz"), None if lim is None else (lim.get("lower"), lim.get("upper"))))
    for link in sorted(root.findall("link"), key=lambda e: e.get("name")):
        for c in link.findall("collision"):
            g, o = c.find("geometry")[0], c.find("origin")
            mesh = None
            if g.tag == "mesh":
                f = (urdf.parent / g.get("filename")).resolve()
                mesh = (g.get("scale"), hashlib.sha256(f.read_bytes()).hexdigest()) if f.exists() else (g.get("scale"), g.get("filename"))
            parts.append(("collision", link.get("name"), g.tag, None if g.tag == "mesh" else dict(g.attrib), mesh, None if o is None else (o.get("xyz"), o.get("rpy"))))
    if srdf is not None:
        pairs = sorted(tuple(sorted((e.get("link1"), e.get("link2")))) for e in ET.fromstring(Path(srdf).read_text()).iter("disable_collisions"))
        parts.append(("srdf", pairs))
    return hashlib.sha256(json.dumps(parts, sort_keys=True, default=str).encode()).hexdigest()[:16]


def curobo_model(camera: str | bool | None = True) -> Path:
    """The cuRobo model (collision spheres, limits, self-collision ignore list) for the arm WITH the ZED X Mini on its bracket (default) or the robot only (camera=None).
    Hand-tuned and regenerated in Piper Studio (tools/curobo/, docs/CUROBO.md); needs no cuRobo to read. Use `curobo_config` to load it."""
    return data_dir() / "curobo" / f"piper_{_camera(camera)}.yml"


def curobo_config(camera: str | bool | None = True, gripper_width: float = 0.07) -> dict:
    """The model as the dict cuRobo's RobotCfg.create() takes, with `kinematics.urdf_path` set to the shipped URDF of the same camera and `gripper_width` (the file itself carries no path).
    A planner may change what it needs (load_dynamics, position_limit_clip, ...) on the returned dict; the model file stays as it is."""
    import yaml
    raw = yaml.safe_load(curobo_model(camera).read_text())
    top = raw.get("robot_cfg", raw)
    top["kinematics"]["urdf_path"] = str(urdf_path(gripper_width, camera))
    return top


def curobo_provenance(camera: str | bool | None = True) -> dict:
    """How the model was made: description commit, URDF/SRDF fingerprint, every decision, sphere counts, ignore pairs; plus `validation` and `quality` (the measured results)."""
    base = curobo_model(camera)
    out = json.loads(Path(str(base) + ".provenance.json").read_text())
    for key, name in (("validation", f"validation_{_camera(camera)}.json"), ("quality", f"quality_{_camera(camera)}.json")):
        f = base.parent / name
        out[key] = json.loads(f.read_text()) if f.exists() else None
    return out


def curobo_model_is_current(camera: str | bool | None = True) -> bool:
    """True when the model was built for the URDF and SRDF this package ships (same fingerprint). False: regenerate (docs/CUROBO.md)."""
    prov = curobo_provenance(camera)
    return prov["fingerprint"] == urdf_fingerprint(urdf_path(0.07, camera), srdf_path(camera))


def curobo_planner_defaults() -> dict:
    """Starting values for a cuRobo planner (algorithm, interpolation step, collision cache sizes, warmup): tools/curobo/policy.yaml, section planner. Downstream projects tune their own on top."""
    return json.loads((data_dir() / "curobo" / "planner_defaults.json").read_text())


@dataclass(frozen=True)
class Limits:
    joints: list
    hard: dict               # {joint: (lower, upper)} rad: the firmware / URDF limits
    planning: dict           # {joint: (lower, upper)} rad: hard shrunk by `margin`; plan inside these
    margin: float
    velocity: float          # rad/s, every arm joint
    acceleration: float      # rad/s^2
    jerk: float              # rad/s^3, DERIVED (see sources)
    effort: dict             # {joint: N.m}, an SDK command range, not a motor rating
    gripper_stroke: float    # m
    max_step: float          # rad per control step the guard accepts
    sources: dict            # {quantity: text}; flags for derived / not-a-rating values in `flags`
    flags: dict

    def as_arrays(self, which: str = "planning"):
        d = getattr(self, which)
        return np.array([d[j][0] for j in self.joints]), np.array([d[j][1] for j in self.joints])


def limits() -> Limits:
    raw = json.loads((data_dir() / "limits.json").read_text())
    pair = lambda block: {j: (v["lower"], v["upper"]) for j, v in block["values"].items()}
    return Limits(
        joints=raw["joints"], hard=pair(raw["position"]), planning=pair(raw["planning_position"]), margin=raw["margin"],
        velocity=raw["velocity"]["value"], acceleration=raw["acceleration"]["value"], jerk=raw["jerk"]["value"],
        effort=raw["effort"]["values"], gripper_stroke=raw["gripper"]["stroke"], max_step=raw["max_step"],
        sources={k: raw[k]["source"] for k in ("position", "planning_position", "velocity", "acceleration", "jerk", "effort", "gripper")},
        flags={k: raw[k]["flag"] for k in ("jerk", "effort")})


def zero_pose() -> list:
    """[joint1 .. joint6] of the pose every session starts and ends at (joints 2 and 3 are held 0.01 rad inside their 0 limit)."""
    z = json.loads((data_dir() / "limits.json").read_text())["zero_pose"]
    return [z[j] for j in JOINT_NAMES]


def scene() -> dict:
    """{'frame': 'base_link', 'objects': [...], 'table': {...}}; the table is a PLACEHOLDER (`measured: false`)."""
    return json.loads((data_dir() / "scene.json").read_text())


def camera_mount_path(name: str | None = None) -> Path:
    """The bracket's mount file (default: the newest version, the one the URDF uses)."""
    d = data_dir() / "camera_mounts"
    return d / f"{name or sorted(p.stem for p in d.glob('zedxm_bracket_v*.yaml'))[-1]}.yaml"


def camera() -> dict:
    """The simulated wrist camera: model, lens, width, height, hfov_deg, vfov_deg, depth_min, depth_max, rate, baseline, K (3x3, rectified left image),
    `optical_frame` (the frame name in the URDFs: the left lens, z forward, x right, y down), `parent_link` and `T_parent_optical` / `T_tcp_optical`
    (4x4, the optical frame in gripper_base and in the TCP frame). One source for every simulator and the real ZED X Mini."""
    return json.loads((data_dir() / "camera.json").read_text())


def camera_topics(prefix: str = "") -> dict:
    """The wrist camera's frame and topics as the ZED ROS 2 wrapper names them (camera_name = `<prefix>zed`, node `zed_node`): `optical_frame` (the left lens), `color`,
    `color_info` (also valid for the depth), `depth` (32FC1 metres, registered to the left image). The same from the simulators and the real ZED X Mini; the URDFs here are unprefixed, ROS uses `piper_`."""
    name, root = prefix + "zed", f"/{prefix}zed/zed_node"
    return {"camera_name": name, "optical_frame": f"{name}_left_camera_frame_optical", "color": f"{root}/rgb/color/rect/image",
            "color_info": f"{root}/rgb/color/rect/camera_info", "depth": f"{root}/depth/depth_registered"}


def camera_pose(frame: str = "tcp_link") -> np.ndarray:
    """4x4 pose of the camera's optical frame (z forward, x right, y down) in `frame`: "tcp_link", "gripper_base" or "link6"."""
    c = camera()
    key = {TCP_LINK: "T_tcp_optical", GRIPPER_LINK: "T_parent_optical", "link6": "T_link6_optical"}.get(frame)
    if key is None:
        raise ValueError(f"frame must be one of tcp_link, gripper_base, link6, got {frame!r}")
    return np.array(c[key])
