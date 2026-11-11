"""MuJoCo and Isaac must use the same provisional servo gains (one actuator model across simulators)."""

import sys
from pathlib import Path

import mujoco
import pytest
from ament_index_python.packages import get_package_share_directory

from piper_sim import mujoco_model, robot_description

ISAAC = Path(__file__).resolve().parents[3] / "isaac"
sys.path.insert(0, str(ISAAC))
from servo_gains import gains  # noqa: E402


@pytest.mark.parametrize("camera", ["zedxm", "none"])
def test_isaac_gains_equal_mujoco(camera):
    if not ISAAC.exists():
        pytest.skip("isaac/ not next to the sources")
    cfg = Path(get_package_share_directory("piper_sim")) / "config" / "mujoco"
    scene = mujoco_model(robot_description("mujoco", camera=camera), cfg / "inputs.xml", cfg / "scene.xml")
    m = mujoco.MjModel.from_xml_path(str(scene))
    GAINS = gains(camera != "none")
    for i in range(m.nu):
        name = m.actuator(i).name
        kp, kd, _ = GAINS[name.removeprefix("piper_")]  # gains are keyed by the official joint names; the model carries the prefix
        assert m.actuator_gainprm[i, 0] == pytest.approx(kp, rel=0.02), name
        assert -m.actuator_biasprm[i, 2] == pytest.approx(kd, rel=0.03), name
