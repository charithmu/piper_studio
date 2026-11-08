"""Static checks of the Piper Studio description against the official AgileX model."""

import math
import yaml
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

from piper_description import robot_description

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import model_audit  # noqa: E402
import piper_arm_macro  # noqa: E402

PKG = Path(get_package_share_directory("piper_description"))
OFFICIAL = (Path(get_package_share_directory("agx_arm_description"))
            / "agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro")
ARM = [f"joint{i}" for i in range(1, 7)]
P = "piper_"  # default name prefix


def expand(**args):
    mappings = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in args.items()}
    return xacro.process_file(str(PKG / "urdf/piper.urdf.xacro"), mappings=mappings).toxml()


def expand_official(**args):
    """The description with prefix:="" , whose names are the official ones."""
    return expand(prefix="", **args)


def write(tmp_path, xml, name="piper.urdf"):
    path = tmp_path / name
    path.write_text(xml)
    return path


@pytest.mark.parametrize("hardware", ["mock", "real"])
def test_expands(hardware):
    root = ET.fromstring(expand(hardware=hardware))
    joints = {j.get("name") for j in root.iter("joint")}
    assert {*(P + j for j in ARM), P + "gripper", P + "tcp_joint", P + "mount_joint"} <= joints
    rc = root.find("ros2_control")
    assert [j.get("name") for j in rc.iter("joint")] == [*(P + j for j in ARM), P + "gripper"]


def test_every_name_has_the_prefix():
    """Links, joints and frames all carry the prefix (only `world`, the caller's frame, does not)."""
    root = ET.fromstring(expand(camera="d435"))
    bad = [e.get("name") for e in root.findall("link") + root.findall("joint")
           if not e.get("name").startswith(P) and e.get("name") != "world"]
    assert bad == []
    assert {j.get("name") for j in root.find("ros2_control").iter("joint")} <= {j.get("name") for j in root.findall("joint")}


def test_empty_prefix_gives_official_names():
    root = ET.fromstring(expand_official())
    assert root.find("joint[@name='joint1']") is not None and root.find("link[@name='base_link']") is not None


def test_arm_macro_is_the_official_model():
    """urdf/piper_arm.xacro and config/moveit/piper_official.srdf.xacro are generated from the pinned official files (DECISIONS D12)."""
    assert piper_arm_macro.check() == []


def test_unknown_hardware_fails():
    with pytest.raises(Exception):
        expand(hardware="nonexistent")


FIRMWARE = yaml.safe_load((Path(__file__).resolve().parents[1] / "config/firmware_values.yaml").read_text())
READOUT = Path(__file__).resolve().parents[3] / "docs/real_arm/arms/piper-01.readout.json"


def test_official_model_plus_firmware_values(tmp_path):
    """Limits, masses and kinematics equal the official model, except the declared firmware overrides
    (arm joint velocity and effort, gripper stroke; config/firmware_values.yaml)."""
    ours = model_audit.load_urdf("ours", write(tmp_path, expand_official()))
    ref = model_audit.load_urdf("official", OFFICIAL)
    stroke = FIRMWARE["gripper_stroke"]
    for name, j in ref.joints.items():
        o = ours.joints["mount_joint" if name == "world_to_base_link" else name]  # the macro's mount joint replaces the world joint
        want = [j.type, j.lower, j.upper, j.effort, j.velocity, j.mimic]
        if name in ARM:
            want[4] = FIRMWARE["arm_joint_max_velocity"]
            want[3] = FIRMWARE["arm_joint_max_effort"][ARM.index(name)]
        if name == "gripper":
            want[2] = stroke
        elif name == "gripper_joint1":
            want[2] = stroke / 2
        elif name == "gripper_joint2":
            want[1] = -stroke / 2
        assert [o.type, o.lower, o.upper, o.effort, o.velocity, o.mimic] == want, name
    assert ours.masses == ref.masses
    q = dict(zip(ARM, [0.3, 1.2, -1.0, 0.4, -0.5, 0.7]))
    assert (abs(ours.fk(q) - ref.fk(q)) < 1e-12).all()


def test_tcp_at_fingertips(tmp_path):
    """Default TCP sits 0.1425 m along the flange z axis (4.5 mm base offset + 138 mm fingers)."""
    model = model_audit.load_urdf("ours", write(tmp_path, expand_official()))
    zero = {j: 0.0 for j in ARM}
    tcp, flange = model.fk(zero, "tcp_link"), model.fk(zero, "link6")
    offset = (abs(flange[:3, :3].T @ (tcp[:3, 3] - flange[:3, 3])))
    assert math.isclose(offset[2], 0.1425, abs_tol=1e-9)
    assert offset[0] < 1e-9 and offset[1] < 1e-9


def test_fixed_base():
    """The arm is rooted at `world`, fixed to the base link (simulators rely on this)."""
    root = ET.fromstring(expand())
    j = root.find(f"joint[@name='{P}mount_joint']")
    assert j.get("type") == "fixed"
    assert j.find("parent").get("link") == "world" and j.find("child").get("link") == P + "base_link"


def test_mount_pose_and_parent():
    """The macro mounts the arm on any link at any pose (a carrier robot's trunk, say)."""
    import tempfile

    import xacro
    text = ('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="carrier"><link name="trunk"/>'
            f'<xacro:include filename="{PKG}/urdf/piper_arm.xacro"/>'
            '<xacro:piper_arm parent="trunk" prefix="arm_" xyz="0.1 0 0.2" rpy="0 0 1.5"/></robot>')
    with tempfile.NamedTemporaryFile("w", suffix=".xacro", delete=False) as f:
        f.write(text)
    root = ET.fromstring(xacro.process_file(f.name).toxml())
    j = root.find("joint[@name='arm_mount_joint']")
    assert j.find("parent").get("link") == "trunk" and j.find("child").get("link") == "arm_base_link"
    assert j.find("origin").attrib == {"xyz": "0.1 0 0.2", "rpy": "0 0 1.5"}
    assert root.find("joint[@name='arm_joint6']") is not None


def test_without_gripper():
    root = ET.fromstring(expand(gripper=False))
    names = {j.get("name") for j in root.iter("joint")}
    assert P + "gripper" not in names and P + "tcp_joint" in names
    assert root.find(f"joint[@name='{P}tcp_joint']/parent").get("link") == P + "flange_link"


def _fixed_chain(root, child, parent):
    """4x4 pose of `child` in `parent`, following the URDF's fixed joints upward."""
    import numpy as np

    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    pose = np.eye(4)
    while child != parent:
        j = joints[child]
        assert j.get("type") == "fixed", f"{j.get('name')} is not fixed"
        o = j.find("origin")
        xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
        r, p, y = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
        cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
        t = np.eye(4)
        t[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]]
        t[:3, 3] = xyz
        pose = t @ pose
        child = j.find("parent").get("link")
    return pose


@pytest.mark.parametrize("stand", ["packaged", "none"])
def test_wrist_camera_mount(stand):
    """camera_link in gripper_base equals AgileX's own mount numbers (agx_arm_sim piper_girpper_D435.urdf, DECISIONS D11).
    The stand is clamped to the gripper body, so it follows gripper_base, 4.5 mm out from link6 (user-confirmed, CLEANUP C5).

    Regression guard: the pose once came from a description whose link6 frame was rotated 90 deg, which put the camera in line
    with one finger. The camera must sit behind the gripper, on -x of link6, looking along the gripper (+x of link6 tilted to +z).
    """
    extra = {"camera_mount_mesh": "none"} if stand == "none" else {}
    root = ET.fromstring(expand(camera="d435", **extra))
    pose = _fixed_chain(root, P + "camera_link", P + "gripper_base")
    assert pose[:3, 3] == pytest.approx([-0.0731, -0.0115, 0.0381], abs=5e-4)
    in_link6 = _fixed_chain(root, P + "camera_link", P + "link6")
    assert in_link6[2, 3] == pytest.approx(0.0381 + 0.0045, abs=5e-4)           # the 4.5 mm gripper_base offset
    assert pose[:3, 0] == pytest.approx([0.344, 0.0, 0.939], abs=2e-3)          # viewing direction
    assert abs(pose[1, 3]) < 0.02, "camera must sit near the plane between the fingers (they open along +-y of link6)"
    mesh = [m.get("filename") for m in root.iter("mesh")]
    assert any("d435_colored.dae" in f for f in mesh)
    assert any("realsense_mid_stand.dae" in f for f in mesh) == (stand == "packaged")


def test_integration_doc_snippet_expands(tmp_path):
    """The xacro example in docs/INTEGRATION.md (a carrier robot using the macros) must keep working."""
    import re

    doc = Path(__file__).resolve().parents[3] / "docs/INTEGRATION.md"
    if not doc.exists():
        pytest.skip("docs/ not next to the sources")
    snippet = re.search(r"```xml\n(.*?)```", doc.read_text(), re.S).group(1).replace("<!-- your own links ... -->", '<link name="trunk"/>')
    path = tmp_path / "carrier.xacro"
    path.write_text(snippet)
    root = ET.fromstring(xacro.process_file(str(path)).toxml())
    assert root.find("joint[@name='piper_mount_joint']/parent").get("link") == "trunk"
    assert root.find("link[@name='piper_camera_color_optical_frame']") is not None


@pytest.mark.skipif(not READOUT.exists(), reason="no saved arm read-out")
def test_description_matches_the_arms_firmware_read_out(tmp_path):
    """The arm's own read-out (firmware S-V1.9-1) is the reference: position limits and speed of the six joints, gripper stroke.
    A difference here is a discrepancy between the vendor URDF and the arm: report it, do not hide it."""
    import json
    arm = json.loads(READOUT.read_text())
    model = model_audit.load_urdf("ours", write(tmp_path, expand_official()))
    for i, joint in enumerate(ARM, start=1):
        fw = arm["limits"][str(i)]["angle_vel"]
        j = model.joints[joint]
        assert (j.lower, j.upper) == pytest.approx((fw["min_angle_limit"], fw["max_angle_limit"]), abs=1e-6), joint
        assert j.velocity == pytest.approx(fw["max_joint_spd"]), joint
    assert model.joints["gripper"].upper == pytest.approx(arm["gripper_params"]["max_range_config"])
    assert FIRMWARE["arm_joint_max_velocity"] == pytest.approx(arm["limits"]["1"]["angle_vel"]["max_joint_spd"])
    assert FIRMWARE["gripper_stroke"] == pytest.approx(arm["gripper_params"]["max_range_config"])


@pytest.mark.parametrize("width", [0.0, 0.035, 0.07])
def test_fixed_gripper_puts_the_fingers_where_the_joints_would(width):
    """fixed_gripper(): same finger poses as the original joints at that width, but no moving or mimic joints left."""
    from piper_description import fixed_gripper, robot_description

    urdf = robot_description("mock", prefix="")
    fixed = ET.fromstring(fixed_gripper(urdf, width, ""))
    for name in ("gripper", "gripper_joint1", "gripper_joint2"):
        j = fixed.find(f"joint[@name='{name}']")
        assert j.get("type") == "fixed" and j.find("mimic") is None and j.find("limit") is None
    for name, expected_y in (("gripper_joint1", -width / 2), ("gripper_joint2", width / 2)):
        xyz = [float(v) for v in fixed.find(f"joint[@name='{name}']/origin").get("xyz").split()]
        assert xyz == pytest.approx([0.0, expected_y, 0.138], abs=1e-7)           # fingers open along -y / +y of gripper_base
    assert [float(v) for v in fixed.find("joint[@name='gripper']/origin").get("xyz").split()] == pytest.approx([0, 0, width])
    with pytest.raises(ValueError):
        fixed_gripper(urdf, 0.08, "")
