import sys
from pathlib import Path

import numpy as np
import pytest

from piper_core.kinematics import Kinematics

ROOT = Path(__file__).resolve().parents[3]


def test_zero_pose_and_rotation_about_joint1():
    pytest.importorskip("piper_assets")
    kin = Kinematics.from_assets("zedxm")
    p0 = kin.points([0] * 6)
    assert set(p0) == {"link1", "link2", "link3", "link4", "link5", "link6", "tcp", "camera_optical"}
    p1 = kin.points([np.pi / 2, 0, 0, 0, 0, 0])             # joint 1 turns everything about the base z axis
    for k in ("tcp", "camera_optical", "link6"):
        assert p1[k][2] == pytest.approx(p0[k][2], abs=1e-9)
        assert np.hypot(*p1[k][:2]) == pytest.approx(np.hypot(*p0[k][:2]), abs=1e-9)
        assert p1[k][:2] == pytest.approx([-p0[k][1], p0[k][0]], abs=1e-9)


def test_no_camera_description_has_no_camera_point():
    pytest.importorskip("piper_assets")
    assert "camera_optical" not in Kinematics.from_assets(None).points([0] * 6)


def test_matches_the_reference_chain_used_to_check_the_firmware():
    """tools/real_arm/fk_compare.py walks the vendor URDF (the source of the shipped one): link6 must agree."""
    pytest.importorskip("piper_assets")
    sys.path.insert(0, str(ROOT / "tools/real_arm"))
    fk = pytest.importorskip("fk_compare")
    if not fk.URDF.exists():
        pytest.skip("vendor submodule not checked out")
    chain, kin = fk.load_chain(), Kinematics.from_assets("zedxm")
    rng = np.random.default_rng(3)
    for _ in range(50):
        q = rng.uniform(-1.2, 1.2, 6)
        assert kin.pose(q, "link6") == pytest.approx(fk.urdf_fk(chain, q), abs=1e-9)


def test_dict_input_and_errors():
    pytest.importorskip("piper_assets")
    kin = Kinematics.from_assets("zedxm")
    q = [0.1, 0.5, -0.5, 0.2, 0.3, 0.0]
    assert kin.pose(dict(zip([f"joint{i}" for i in range(1, 7)], q)), "tcp_link") == pytest.approx(kin.pose(q, "tcp_link"))
    with pytest.raises(ValueError):
        kin.pose([0, 0], "tcp_link")
    with pytest.raises(KeyError):
        kin.pose([0] * 6, "nope")
