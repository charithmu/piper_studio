"""Generate the Piper Studio robot description for a backend.

    from piper_description import robot_description
    urdf = robot_description("gazebo", controllers_file="/path/controllers.yaml")
"""

from __future__ import annotations

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
    return add_virtual_inertials(urdf) if hardware in SIMULATORS else urdf


# Physics-model limit margins. Gazebo/DART velocity control cannot move a joint off a limit it is
# resting on, so the simulator's hard stops sit slightly outside the URDF limits that ros2_control,
# MoveIt and piper_py enforce. Commanded targets therefore never touch a physics limit.
PHYSICS_LIMIT_MARGIN = {"revolute": 0.01, "prismatic": 0.002}  # rad, m


def physics_model(urdf: str) -> str:
    """Model for a simulator's physics engine (not for ROS): no mimic constraints, widened limits."""
    root = ET.fromstring(strip_mimic(urdf))
    for joint in root.findall("joint"):
        margin = PHYSICS_LIMIT_MARGIN.get(joint.get("type"))
        limit = joint.find("limit")
        if margin and limit is not None and limit.get("lower") is not None:
            limit.set("lower", repr(float(limit.get("lower")) - margin))
            limit.set("upper", repr(float(limit.get("upper")) + margin))
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
