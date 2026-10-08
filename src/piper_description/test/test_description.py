"""Static checks of the Piper Studio description against the official AgileX model."""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

from piper_description import PHYSICS_LIMIT_MARGIN, physics_model, robot_description

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import model_audit  # noqa: E402

PKG = Path(get_package_share_directory("piper_description"))
OFFICIAL = (Path(get_package_share_directory("agx_arm_description"))
            / "agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro")
ARM = [f"joint{i}" for i in range(1, 7)]


def expand(**args):
    mappings = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in args.items()}
    return xacro.process_file(str(PKG / "urdf/piper.urdf.xacro"), mappings=mappings).toxml()


def write(tmp_path, xml, name="piper.urdf"):
    path = tmp_path / name
    path.write_text(xml)
    return path


@pytest.mark.parametrize("hardware", ["mock", "real"])
def test_expands(hardware):
    root = ET.fromstring(expand(hardware=hardware))
    joints = {j.get("name") for j in root.iter("joint")}
    assert {*ARM, "gripper", "tcp_joint"} <= joints
    rc = root.find("ros2_control")
    assert [j.get("name") for j in rc.iter("joint")] == [*ARM, "gripper"]


def test_unknown_hardware_fails():
    with pytest.raises(Exception):
        expand(hardware="nonexistent")


def test_official_model_unmodified(tmp_path):
    """Limits, masses and kinematics must equal the official model exactly."""
    ours = model_audit.load_urdf("ours", write(tmp_path, expand()))
    ref = model_audit.load_urdf("official", OFFICIAL)
    for name, j in ref.joints.items():
        o = ours.joints[name]
        assert (o.type, o.lower, o.upper, o.effort, o.velocity, o.mimic) == \
            (j.type, j.lower, j.upper, j.effort, j.velocity, j.mimic), name
    assert ours.masses == ref.masses
    q = dict(zip(ARM, [0.3, 1.2, -1.0, 0.4, -0.5, 0.7]))
    assert (abs(ours.fk(q) - ref.fk(q)) < 1e-12).all()


def test_tcp_at_fingertips(tmp_path):
    """Default TCP sits 0.1425 m along the flange z axis (4.5 mm base offset + 138 mm fingers)."""
    model = model_audit.load_urdf("ours", write(tmp_path, expand()))
    zero = {j: 0.0 for j in ARM}
    tcp, flange = model.fk(zero, "tcp_link"), model.fk(zero, "link6")
    offset = (abs(flange[:3, :3].T @ (tcp[:3, 3] - flange[:3, 3])))
    assert math.isclose(offset[2], 0.1425, abs_tol=1e-9)
    assert offset[0] < 1e-9 and offset[1] < 1e-9


def test_fixed_base():
    """The official model is rooted at `world`, fixed to base_link (simulators rely on this)."""
    root = ET.fromstring(expand())
    j = root.find("joint[@name='world_to_base_link']")
    assert j.get("type") == "fixed"
    assert j.find("parent").get("link") == "world" and j.find("child").get("link") == "base_link"


def test_without_gripper():
    root = ET.fromstring(expand(gripper=False))
    names = {j.get("name") for j in root.iter("joint")}
    assert "gripper" not in names and "tcp_joint" in names
    assert root.find("joint[@name='tcp_joint']/parent").get("link") == "flange_link"


def test_simulator_model_keeps_official_limits():
    """The ROS-side simulator description differs from the official model only by virtual inertials."""
    root = ET.fromstring(robot_description("gazebo", controllers_file="/tmp/controllers.yaml"))
    official = ET.fromstring(expand())
    lim = lambda r, n: r.find(f"joint[@name='{n}']/limit").attrib  # noqa: E731
    for name in [*ARM, "gripper", "gripper_joint1", "gripper_joint2"]:
        assert lim(root, name) == lim(official, name), name
    assert root.find("link[@name='gripper_link']/inertial/mass") is not None
    assert root.find("joint[@name='gripper_joint1']/mimic") is not None


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
