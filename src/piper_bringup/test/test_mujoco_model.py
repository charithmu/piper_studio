"""The generated MuJoCo model must reproduce the official kinematics and couple the fingers."""

import sys
from pathlib import Path

import numpy as np
import pytest
from ament_index_python.packages import get_package_share_directory, get_package_prefix

import piper_description
from piper_description import robot_description

sys.path.insert(0, str(Path(get_package_prefix("piper_description")) / "lib" / "piper_description"))
import model_audit  # noqa: E402

ARM = [f"joint{i}" for i in range(1, 7)]
OFFICIAL = (Path(get_package_share_directory("agx_arm_description"))
            / "agx_arm_urdf/piper/urdf/piper_with_gripper_description.xacro")


def test_generated_mujoco_model_matches_official(tmp_path):
    """MJCF from mujoco_ros2_control's converter + our inputs: exact kinematics, coupled fingers."""
    mujoco = pytest.importorskip("mujoco")
    cfg = Path(get_package_share_directory("piper_bringup")) / "config" / "mujoco"
    scene = piper_description.mujoco_model(robot_description("mujoco"), cfg / "inputs.xml", cfg / "scene.xml",
                                           cache_dir=tmp_path)
    mj = model_audit.load_mjcf("mjcf", scene)
    ref = model_audit.load_urdf("official", OFFICIAL)
    rng = np.random.default_rng(1)
    lo = np.array([ref.joints[j].lower for j in ARM]); hi = np.array([ref.joints[j].upper for j in ARM])
    for q in [np.zeros(6)] + [rng.uniform(lo, hi) for _ in range(100)]:
        qd = dict(zip(ARM, q))
        assert np.allclose(mj.fk(qd), ref.fk(qd), atol=1e-6)
    for j in ARM:
        assert (mj.joints[j].lower, mj.joints[j].upper) == pytest.approx(
            (ref.joints[j].lower, ref.joints[j].upper), abs=1e-5)
    m = mujoco.MjModel.from_xml_path(str(scene)); d = mujoco.MjData(m)
    d.ctrl[m.actuator("gripper").id] = 0.06
    while d.time < 1.0:
        mujoco.mj_step(m, d)
    q = {n: d.qpos[m.jnt_qposadr[m.joint(n).id]] for n in ("gripper", "gripper_joint1", "gripper_joint2")}
    assert q["gripper"] == pytest.approx(0.06, abs=1e-3)
    assert q["gripper_joint1"] == pytest.approx(0.5 * q["gripper"], abs=1e-4)
    assert q["gripper_joint2"] == pytest.approx(-0.5 * q["gripper"], abs=1e-4)
