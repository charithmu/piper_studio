"""One source of truth: the description (firmware values + actuation.yaml) is what the URDF, the MuJoCo model, the Isaac drives and the real arm's gripper code all carry.

Where a number appears in more than one place, a test here says they are equal."""

import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import pytest
import yaml

import piper_assets as pa
import piper_description
import piper_core.real_arm as real_arm
from piper_core.gripper import GRIPPER_DEFAULT_EFFORT, GRIPPER_MAX_EFFORT

WS = Path(__file__).resolve().parents[3]
FIRMWARE = yaml.safe_load((WS / "src/piper_description/config/firmware_values.yaml").read_text())
ACT = piper_description.actuation()
ARM = [f"joint{i}" for i in range(1, 7)]
sys.path.insert(0, str(WS / "isaac"))
sys.path.insert(0, str(WS / "tools/release"))


def test_the_gripper_numbers_of_the_real_arm_code_are_the_yamls_and_the_firmwares():
    g = ACT["gripper"]
    assert real_arm.GRIPPER_STROKE == FIRMWARE["gripper_stroke"] == g["stroke"] == pa.limits().gripper_stroke
    assert g["command_force"] == {"min": 0.5, "default": GRIPPER_DEFAULT_EFFORT, "max": GRIPPER_MAX_EFFORT}
    assert g["force_max"] == GRIPPER_MAX_EFFORT == FIRMWARE["gripper_max_effort"]
    assert g["speed"] == g["velocity"] == FIRMWARE["gripper_max_velocity"]
    assert g["grip_force_plateau"] <= g["force_max"]


def test_the_wheel_data_is_the_description_s_actuation():
    assert pa.actuation() == ACT


@pytest.mark.parametrize("variant", ["model", "model_no_camera"])
def test_mujoco_limits_are_the_urdfs_which_are_the_firmwares(variant):
    """Every joint's range and force limit in the MJCF equals the URDF's (which equals firmware_values.yaml), and so does every actuator's force range."""
    import piper_mujoco
    folder = Path(str(__import__("importlib.resources", fromlist=["files"]).files("piper_mujoco") / "data" / variant))
    m = mujoco.MjModel.from_xml_path(str(folder / "scene_empty.xml"))
    urdf = ET.fromstring(pa.urdf_path(0.07, "zedxm" if variant == "model" else None).read_text())
    for j in urdf.iter("joint"):
        lim = j.find("limit")
        if lim is None or j.get("type") not in ("revolute", "prismatic") or j.find("mimic") is None and j.get("name") not in ARM + ["gripper"]:
            continue
        mj = m.joint(j.get("name"))
        assert mj.range[0] == pytest.approx(float(lim.get("lower")), abs=2e-5) and mj.range[1] == pytest.approx(float(lim.get("upper")), abs=2e-5), j.get("name")
        assert m.jnt_actfrcrange[mj.id][1] == pytest.approx(float(lim.get("effort"))), j.get("name")
    for i in range(m.nu):
        name = m.actuator(i).name
        want = ACT["gripper"]["force_max"] if name == "gripper" else ACT["arm"]["joints"][name]["force"]
        assert m.actuator_forcerange[i][1] == pytest.approx(want) and m.actuator_forcerange[i][0] == pytest.approx(-want), name
        if name != "gripper":
            assert m.actuator_gainprm[i, 0] == pytest.approx(ACT["arm"]["joints"][name]["kp"]), name
    assert piper_mujoco  # imported for its data


def test_the_urdf_gripper_limits_are_the_firmware_values():
    urdf = ET.fromstring((pa.data_dir() / "piper_none_open.urdf").read_text())
    lim = {j.get("name"): j.find("limit").attrib for j in urdf.iter("joint") if j.find("limit") is not None}
    assert float(lim["gripper"]["effort"]) == float(lim["gripper_joint1"]["effort"]) == float(lim["gripper_joint2"]["effort"]) == FIRMWARE["gripper_max_effort"]
    assert float(lim["gripper"]["velocity"]) == 3.0                            # the vendor placeholder, kept on purpose (piper_arm_macro.py): the measured speed is actuation.yaml's
    for i, j in enumerate(ARM):
        assert float(lim[j]["effort"]) == FIRMWARE["arm_joint_max_effort"][i] and float(lim[j]["velocity"]) == FIRMWARE["arm_joint_max_velocity"]


@pytest.mark.parametrize("camera", [True, False])
def test_isaac_drives_equal_the_actuation_and_carry_the_same_force_limits(camera):
    from servo_gains import gains
    g = gains(camera)
    for j in ARM:
        assert g[j][0] == ACT["arm"]["joints"][j]["kp"] and g[j][2] == ACT["arm"]["joints"][j]["force"]
    for name in ("gripper", "gripper_joint1", "gripper_joint2"):
        assert g[name][0] == ACT["gripper"]["servo"]["kp"] and g[name][2] == ACT["gripper"]["force_max"]


def test_gains_json_is_what_the_build_derives():
    """isaac/gains.json is committed; it must equal a fresh derivation (run tools/release/build_gains.py after changing actuation.yaml or the description)."""
    import build_gains
    doc = json.loads((WS / "isaac" / "gains.json").read_text())
    for key, cam in (("camera", "zedxm"), ("no_camera", "none")):
        assert doc[key] == build_gains.derive(cam), key


def test_the_fingers_of_both_mujoco_models_carry_the_pad_friction():
    for variant in ("model", "model_no_camera"):
        folder = Path(str(__import__("importlib.resources", fromlist=["files"]).files("piper_mujoco") / "data" / variant))
        root = ET.parse(folder / "mujoco_description_formatted.xml").getroot()
        pads = [g.get("friction") for b in root.iter("body") if b.get("name") in ("gripper_link1", "gripper_link2") for g in b.findall("geom") if g.get("class") == "collision"]
        assert len(pads) == 2 and all(tuple(map(float, p.split())) == tuple(ACT["gripper"]["pad_friction"]) for p in pads)
