"""Simulator models derived from the description: limits are the official ones, virtual links get inertia, mimic and limits for physics."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory

from piper_description import robot_description as plain_description
from piper_sim import PHYSICS_LIMIT_MARGIN, physics_model, robot_description

ARM = [f"joint{i}" for i in range(1, 7)]
P = "piper_"


def expand_official():
    return plain_description("mock", prefix="")


def test_simulator_model_keeps_official_limits():
    """The ROS-side simulator description differs from the official model only by virtual inertials."""
    root = ET.fromstring(robot_description("gazebo", controllers_file="/tmp/controllers.yaml"))
    official = ET.fromstring(expand_official())
    lim = lambda r, n: r.find(f"joint[@name='{n}']/limit").attrib  # noqa: E731
    for name in [*ARM, "gripper", "gripper_joint1", "gripper_joint2"]:
        assert lim(root, P + name) == lim(official, name), name
    assert root.find(f"link[@name='{P}gripper_link']/inertial/mass") is not None
    assert root.find(f"joint[@name='{P}gripper_joint1']/mimic") is not None


def test_physics_model_widens_limits_and_drops_mimic():
    ros = ET.fromstring(robot_description("gazebo", controllers_file="/tmp/controllers.yaml"))
    phys = ET.fromstring(physics_model(ET.tostring(ros, encoding="unicode")))
    for j in ros.findall("joint"):
        margin = PHYSICS_LIMIT_MARGIN.get(j.get("type"))
        if margin is None or j.find("limit") is None:
            continue
        a, b = j.find("limit"), phys.find(f"joint[@name='{j.get('name')}']/limit")
        assert math.isclose(float(b.get("lower")), float(a.get("lower")) - margin)
        assert math.isclose(float(b.get("upper")), float(a.get("upper")) + margin)
    assert not phys.findall("joint/mimic")





def test_gazebo_sdf_collisions_match_the_description(tmp_path):
    """The SDF Gazebo builds from the description (gz sdf -p) holds every collision mesh at the pose the URDF gives it, camera and bracket included.

    sdformat lumps the fixed links (flange, gripper base, bracket, camera) into link6, so each lumped collision's pose is compared with the one
    the URDF predicts relative to its SDF link. Skipped without the gz command.
    """
    import re
    import shutil
    import subprocess
    import sys

    import numpy as np
    import pytest
    from ament_index_python.packages import get_package_prefix

    if not shutil.which("gz"):
        pytest.skip("gz command not available")
    sys.path.insert(0, str(Path(get_package_prefix("piper_description")) / "lib" / "piper_description"))
    import model_audit

    urdf_file = tmp_path / "gz.urdf"
    urdf_file.write_text(robot_description("gazebo", controllers_file="/tmp/controllers.yaml"))
    sdf = ET.fromstring(subprocess.run(["gz", "sdf", "-p", str(urdf_file)], capture_output=True, text=True, check=True).stdout).find("model")
    ref = model_audit.load_urdf("gz", urdf_file)
    urdf = ET.parse(urdf_file).getroot()

    def rpy_matrix(r, p, y):
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
        return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])

    def pose(text):
        x, y, z, r, p, yaw = (float(v) for v in text.split())
        T = np.eye(4); T[:3, :3] = rpy_matrix(r, p, yaw); T[:3, 3] = [x, y, z]
        return T

    checked = 0
    for link in sdf.findall("link"):
        for col in link.findall("collision"):
            m = re.match(r"(.+)_fixed_joint_lump__(.+)_collision_\d+$", col.get("name"))
            urdf_link = m.group(2) if m else col.get("name").removesuffix("_collision")
            el = next(l for l in urdf.findall("link") if l.get("name") == urdf_link)
            ucol = el.find("collision")
            o = ucol.find("origin")
            O = np.eye(4)
            if o is not None:
                O = pose(o.get("xyz", "0 0 0") + " " + o.get("rpy", "0 0 0"))
            expected = np.linalg.inv(ref.fk({}, link.get("name"))) @ ref.fk({}, urdf_link) @ O
            got = pose(col.find("pose").text)
            assert np.allclose(got[:3, 3], expected[:3, 3], atol=1e-5), (urdf_link, got[:3, 3], expected[:3, 3])
            assert np.allclose(got[:3, :3], expected[:3, :3], atol=1e-4), urdf_link
            assert Path(col.find("geometry/mesh/uri").text).name == Path(ucol.find("geometry/mesh").get("filename")).name
            checked += 1
    assert checked == 13
