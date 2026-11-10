"""The named poses 'zero' and 'park' must agree everywhere they are written down: SRDF, startup positions, guard margin and the real-arm tool."""

import importlib.util
import re
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SRDF = ROOT / "src/piper_control/config/moveit/piper.srdf.xacro"
INITIAL = ROOT / "src/piper_description/config/initial_positions.yaml"
GUARD = ROOT / "src/piper_control/scripts/command_guard.py"
TOOL = ROOT / "tools/real_arm/arm_zero.py"
JOINTS = [f"joint{i}" for i in range(1, 7)]


def srdf_pose(name):
    text = SRDF.read_text()
    block = re.search(rf'<group_state name="{name}" group="arm">(.*?)</group_state>', text, re.S).group(1)
    vals = dict(re.findall(r'<joint name="\$\{prefix\}(\w+)" value="([-0-9.]+)"', block))
    return [float(vals[j]) for j in JOINTS]


def test_zero_is_the_startup_pose():
    initial = yaml.safe_load(INITIAL.read_text())["initial_positions"]
    assert srdf_pose("zero") == [initial[j] for j in JOINTS]


def test_zero_is_computed_from_the_limits_and_the_margin():
    """One rule: 0 for every joint, held `margin` inside a limit that is 0. The hand-written values must equal piper_core's."""
    from piper_core import limits_from_urdf, zero_pose
    from piper_description import robot_description

    limits = limits_from_urdf(robot_description("mock"))
    pose = zero_pose(limits, [f"piper_{j}" for j in JOINTS], 0.01)
    assert [pose[f"piper_{j}"] for j in JOINTS] == pytest.approx(srdf_pose("zero"))


def test_park_is_zero_with_joint5_lowered():
    zero, park = srdf_pose("zero"), srdf_pose("park")
    assert [r - z for r, z in zip(park, zero)] == pytest.approx([0, 0, 0, 0, park[4], 0])
    assert park[4] == pytest.approx(0.3840)  # 22.0 deg, chosen on the real arm 2026-11-10


def test_guard_margin_equals_the_zero_pose_margin():
    margin = float(re.search(r'declare_parameter\("limit_margin", ([0-9.]+)\)', GUARD.read_text()).group(1))
    assert margin == pytest.approx(srdf_pose("zero")[1])


def test_real_arm_tool_uses_the_same_numbers():
    pytest.importorskip("pyAgxArm")
    sys.path.insert(0, str(TOOL.parent))
    spec = importlib.util.spec_from_file_location("arm_zero", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    assert mod.ZERO_MARGIN == pytest.approx(srdf_pose("zero")[1])
    assert mod.PARK_J5 == pytest.approx(srdf_pose("park")[4])


def test_pose_table_of_arm_pose_tool_equals_the_srdf():
    pytest.importorskip("pyAgxArm")
    path = TOOL.parent / "arm_pose.py"
    sys.path.insert(0, str(TOOL.parent))
    spec = importlib.util.spec_from_file_location("arm_pose", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for name, q in mod.POSES.items():
        assert q == pytest.approx(srdf_pose(name)), name
    assert set(mod.POSES) == {"zero", "park", "ready"}


def test_console_ready_pose_equals_the_srdf():
    from piper_core.arm_cli import READY
    assert READY == pytest.approx(srdf_pose("ready"))
