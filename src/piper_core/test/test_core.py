"""piper_core: guard rules, limits from the URDF, named poses. No ROS needed except for the URDF test."""

import math

import pytest

from piper_core import GuardConfig, check_command, limits_from_urdf, named_poses_from_srdf

ARM = tuple(f"joint{i}" for i in range(1, 7))
LIMITS = {**{j: (-1.0, 1.0) for j in ARM}, "gripper": (0.0, 0.1)}
CFG = GuardConfig(ARM, max_step=0.15, limit_margin=0.01, limit_slack=0.1)
MEASURED = {j: 0.0 for j in ARM}


def run(values, names=None, measured=MEASURED, limits=LIMITS):
    names = names or list(ARM)[: len(values)]
    return check_command(names, values, limits, measured, CFG)


def test_accepts_a_small_step_inside_the_limits():
    v = run([0.1, 0, 0, 0, 0, 0])
    assert v.ok and v.positions[0] == pytest.approx(0.1)


def test_rejects_nan_and_inf():
    assert not run([math.nan, 0, 0, 0, 0, 0]).ok
    assert not run([0, math.inf, 0, 0, 0, 0]).ok


def test_rejects_a_jump_over_max_step():
    v = run([0.2, 0, 0, 0, 0, 0])
    assert not v.ok and "max_step" in v.reason


def test_clamps_slightly_outside_but_rejects_far_outside():
    near = {**MEASURED, "joint1": 1.0}
    v = run([1.05, 0, 0, 0, 0, 0], measured=near)   # inside limit + slack: clamped to hi - margin
    assert v.ok and v.positions[0] == pytest.approx(0.99)
    assert not run([1.2, 0, 0, 0, 0, 0], measured={**MEASURED, "joint1": 1.15}).ok


def test_rejects_without_measured_state_or_limits():
    assert not run([0.0] * 6, measured={}).ok
    assert not run([0.0] * 6, limits={}).ok


def test_gripper_checked_against_its_limits_only():
    assert run([0.05], names=["gripper"]).ok
    assert not run([0.2], names=["gripper"]).ok


def test_limits_from_the_real_urdf():
    pytest.importorskip("xacro")
    pd = pytest.importorskip("piper_description")
    limits = limits_from_urdf(pd.robot_description("mock"))
    assert limits["piper_joint1"] == pytest.approx((-2.618, 2.618), abs=1e-3)
    assert limits["piper_gripper"] == pytest.approx((0.0, 0.07))  # firmware stroke
    assert limits["piper_joint3"][1] == 0.0


def test_named_poses_from_srdf():
    srdf = """<robot name="x"><group_state name="zero" group="arm"><joint name="j1" value="0"/><joint name="j2" value="0.01"/></group_state>
              <group_state name="open" group="gripper"><joint name="g" value="0.1"/></group_state></robot>"""
    assert named_poses_from_srdf(srdf) == {"zero": {"j1": 0.0, "j2": 0.01}}
    assert named_poses_from_srdf(srdf, group="gripper") == {"open": {"g": 0.1}}
