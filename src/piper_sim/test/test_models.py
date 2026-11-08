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



