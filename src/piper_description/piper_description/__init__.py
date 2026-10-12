"""Generate the Piper Studio robot description for a backend.

    from piper_description import robot_description
    urdf = robot_description("gazebo", controllers_file="/path/controllers.yaml")
"""

from __future__ import annotations

import hashlib
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

SIMULATORS = ("gazebo", "mujoco", "isaac")
# Placeholder inertia for links that are virtual in the official model (e.g. gripper_link, the
# child of the commanded `gripper` width joint). Physics engines drop massless moving links.
VIRTUAL_LINK_MASS = 0.01  # kg
VIRTUAL_LINK_INERTIA = 1e-6  # kg m^2


def xacro_file() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("piper_description")) / "urdf" / "piper.urdf.xacro"


def robot_description(hardware: str = "mock", **mappings) -> str:
    """Expand piper.urdf.xacro; for simulators, make virtual moving links simulatable."""
    import xacro
    args = {"hardware": hardware, **{k: str(v).lower() if isinstance(v, bool) else str(v)
                                      for k, v in mappings.items()}}
    urdf = xacro.process_file(str(xacro_file()), mappings=args).toxml()
    return fix_camera_inertia(add_virtual_inertials(urdf)) if hardware in SIMULATORS else urdf


# realsense2_description's D435 model declares ixx = 3.9e-3 kg m^2 for a 72 g body (about 70x too large), which would
# distort the wrist dynamics in physics simulators. Box 90 x 25 x 25 mm (x forward, y across, z up) instead.
_D435_MASS = 0.072
_D435_INERTIA = {"ixx": _D435_MASS / 12 * (0.09**2 + 0.025**2), "iyy": _D435_MASS / 12 * (0.025**2 + 0.025**2),
                 "izz": _D435_MASS / 12 * (0.09**2 + 0.025**2)}


def fix_camera_inertia(urdf: str) -> str:
    root = ET.fromstring(urdf)
    link = next((l for l in root.findall("link") if l.get("name") == "camera_link"), None)
    inertia = None if link is None else link.find("inertial/inertia")
    if inertia is not None:
        for k, v in _D435_INERTIA.items():
            inertia.set(k, f"{v:.6e}")
        for k in ("ixy", "ixz", "iyz"):
            inertia.set(k, "0")
    return ET.tostring(root, encoding="unicode")


# Physics-model limit margins. Gazebo/DART velocity control cannot move a joint off a limit it is
# resting on, so the simulator's hard stops sit slightly outside the URDF limits that ros2_control,
# MoveIt and piper_py enforce. Commanded targets therefore never touch a physics limit.
PHYSICS_LIMIT_MARGIN = {"revolute": 0.01, "prismatic": 0.002}  # rad, m


def mujoco_model(urdf: str, inputs: str | Path, scene: str | Path, cache_dir: str | Path | None = None,
                 camera: dict | None = None) -> Path:
    """Generate the MuJoCo scene for a description with mujoco_ros2_control's URDF converter.

    `inputs` adds actuators, equality constraints and solver options; `scene` is a world file that
    includes the generated robot. Output goes to a directory named by the hash of all inputs, so an
    unchanged model is reused. Needs the workspace venv (mujoco, trimesh, obj2mjcf, pycollada).
    Returns the path of the scene file to load.
    """
    from ament_index_python.packages import get_package_prefix
    physics = collision_meshes_as_visuals(_without_ros2_control(urdf))
    inputs, scene = Path(inputs).resolve(), Path(scene).resolve()
    inputs_text = inputs.read_text()
    if camera:  # fixed MJCF camera on the colour optical frame; fovy is the vertical field of view
        import math
        w, h, hfov = camera["width"], camera["height"], camera["hfov_deg"]
        fovy = math.degrees(2 * math.atan(math.tan(math.radians(hfov) / 2) * h / w))
        tag = (f'<camera site="camera_color_optical_frame" name="camera" fovy="{fovy:.3f}" mode="fixed" '
               f'resolution="{w} {h}"/>')
        inputs_text = inputs_text.replace("<processed_inputs>", "<processed_inputs>\n    " + tag, 1)
    digest = hashlib.sha256("\0".join(
        [physics, inputs_text, scene.read_text()]).encode()).hexdigest()[:16]
    out = Path(cache_dir or Path(tempfile.gettempdir()) / "piper_studio_mujoco") / digest
    if not (out / "scene.xml").exists():
        out.mkdir(parents=True, exist_ok=True)
        (out / "piper.urdf").write_text(physics)
        converter = (Path(get_package_prefix("mujoco_ros2_control"))
                     / "lib/mujoco_ros2_control/make_mjcf_from_robot_description.py")
        venv = os.environ.get("VIRTUAL_ENV")
        python = str(Path(venv) / "bin/python") if venv else "python3"
        (out / "inputs.xml").write_text(inputs_text)
        run = subprocess.run([python, str(converter), "-u", str(out / "piper.urdf"), "-m", str(out / "inputs.xml"),
                              "--scene", str(scene), "-o", str(out), "-c", "-s"],
                             cwd=out, capture_output=True, text=True)
        if run.returncode != 0:
            raise RuntimeError(f"MJCF conversion failed ({converter.name}):\n{run.stdout[-2000:]}{run.stderr[-2000:]}")
        (out / "scene.xml").write_text(scene.read_text())  # converter output dir + our world wrapper
    return out / "scene.xml"


def _without_ros2_control(urdf: str) -> str:
    root = ET.fromstring(urdf)
    for tag in ("ros2_control", "gazebo"):
        for el in root.findall(tag):
            root.remove(el)
    return ET.tostring(root, encoding="unicode")


def physics_model(urdf: str, keep_mimic: bool = False) -> str:
    """Model for a simulator's physics engine (not for ROS): widened joint limits, and no mimic
    constraints unless the engine implements them natively (keep_mimic=True, e.g. PhysX)."""
    root = ET.fromstring(urdf if keep_mimic else strip_mimic(urdf))
    for joint in root.findall("joint"):
        margin = PHYSICS_LIMIT_MARGIN.get(joint.get("type"))
        limit = joint.find("limit")
        if margin and limit is not None and limit.get("lower") is not None:
            limit.set("lower", repr(float(limit.get("lower")) - margin))
            limit.set("upper", repr(float(limit.get("upper")) + margin))
    return ET.tostring(root, encoding="unicode")


def collision_meshes_as_visuals(urdf: str) -> str:
    """Use each link's collision mesh for its visual too.

    mujoco_ros2_control's converter keys meshes by file stem, so link.dae (visual) and link.stl
    (collision) collide and the visual sub-meshes end up as collision geometry (some are flat and
    have no convex hull). The official STL collision meshes are complete, so visuals lose only colour.
    """
    root = ET.fromstring(urdf)
    for link in root.findall("link"):
        coll = link.find("collision/geometry/mesh")
        vis = link.find("visual/geometry/mesh")
        if coll is not None and vis is not None:
            vis.set("filename", coll.get("filename"))
            if coll.get("scale"):
                vis.set("scale", coll.get("scale"))
    return ET.tostring(root, encoding="unicode")


def strip_mimic(urdf: str) -> str:
    """Remove <mimic> tags: for the physics model when the simulator's ros2_control plugin
    implements mimic joints itself (physics-level mimic constraints then fight it)."""
    root = ET.fromstring(urdf)
    for joint in root.findall("joint"):
        for mimic in joint.findall("mimic"):
            joint.remove(mimic)
    return ET.tostring(root, encoding="unicode")


def add_virtual_inertials(urdf: str) -> str:
    """Give every massless link moved by a non-fixed joint a negligible inertial."""
    root = ET.fromstring(urdf)
    moving_children = {j.find("child").get("link") for j in root.findall("joint")
                       if j.get("type") != "fixed"}
    for link in root.findall("link"):
        if link.get("name") in moving_children and link.find("inertial") is None:
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(inertial, "mass", value=str(VIRTUAL_LINK_MASS))
            i = str(VIRTUAL_LINK_INERTIA)
            ET.SubElement(inertial, "inertia", ixx=i, iyy=i, izz=i, ixy="0", ixz="0", iyz="0")
    return ET.tostring(root, encoding="unicode")
