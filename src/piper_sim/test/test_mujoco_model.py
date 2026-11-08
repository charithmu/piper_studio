"""The generated MuJoCo model must reproduce the official kinematics and couple the fingers."""

import sys
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import piper_sim
from piper_sim import robot_description

sys.path.insert(0, str(Path(get_package_prefix("piper_description")) / "lib" / "piper_description"))
import model_audit  # noqa: E402

ARM = [f"joint{i}" for i in range(1, 7)]
P = "piper_"  # the model carries the default prefix; the official model does not
OFFICIAL = (Path(get_package_share_directory("agx_arm_description"))
            / "agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro")


def test_generated_mujoco_model_matches_official(tmp_path):
    """MJCF from mujoco_ros2_control's converter + our inputs: exact kinematics, coupled fingers."""
    mujoco = pytest.importorskip("mujoco")
    cfg = Path(get_package_share_directory("piper_sim")) / "config" / "mujoco"
    scene = piper_sim.mujoco_model(robot_description("mujoco"), cfg / "inputs.xml", cfg / "scene.xml",
                                           cache_dir=tmp_path)
    mj = model_audit.load_mjcf("mjcf", scene)
    ref = model_audit.load_urdf("official", OFFICIAL)
    rng = np.random.default_rng(1)
    lo = np.array([ref.joints[j].lower for j in ARM]); hi = np.array([ref.joints[j].upper for j in ARM])
    for q in [np.zeros(6)] + [rng.uniform(lo, hi) for _ in range(100)]:
        assert np.allclose(mj.fk(dict(zip([P + j for j in ARM], q))), ref.fk(dict(zip(ARM, q))), atol=1e-6)
    for j in ARM:
        assert (mj.joints[P + j].lower, mj.joints[P + j].upper) == pytest.approx(
            (ref.joints[j].lower, ref.joints[j].upper), abs=1e-5)
    m = mujoco.MjModel.from_xml_path(str(scene)); d = mujoco.MjData(m)
    d.ctrl[m.actuator(P + "gripper").id] = 0.06
    while d.time < 1.0:
        mujoco.mj_step(m, d)
    q = {n: d.qpos[m.jnt_qposadr[m.joint(n).id]] for n in (P + "gripper", P + "gripper_joint1", P + "gripper_joint2")}
    assert q[P + "gripper"] == pytest.approx(0.06, abs=1e-3)
    assert q[P + "gripper_joint1"] == pytest.approx(0.5 * q[P + "gripper"], abs=1e-4)
    assert q[P + "gripper_joint2"] == pytest.approx(-0.5 * q[P + "gripper"], abs=1e-4)
