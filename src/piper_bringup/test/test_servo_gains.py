"""MuJoCo and Isaac must use the same provisional servo gains (one actuator model across simulators)."""

import sys
from pathlib import Path

import mujoco
import pytest
from ament_index_python.packages import get_package_share_directory

from piper_description import mujoco_model, robot_description

ISAAC = Path(__file__).resolve().parents[3] / "isaac"
sys.path.insert(0, str(ISAAC))
from servo_gains import GAINS  # noqa: E402


def test_isaac_gains_equal_mujoco():
    if not ISAAC.exists():
        pytest.skip("isaac/ not next to the sources")
    cfg = Path(get_package_share_directory("piper_bringup")) / "config" / "mujoco"
    scene = mujoco_model(robot_description("mujoco"), cfg / "inputs.xml", cfg / "scene.xml")
    m = mujoco.MjModel.from_xml_path(str(scene))
    for i in range(m.nu):
        name = m.actuator(i).name
        kp, kd, _ = GAINS[name]
        assert m.actuator_gainprm[i, 0] == pytest.approx(kp, rel=0.02), name
        assert -m.actuator_biasprm[i, 2] == pytest.approx(kd, rel=0.03), name
