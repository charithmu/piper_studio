"""The named poses 'zero' and 'rest' must agree everywhere they are written down: SRDF, startup positions, guard margin and the real-arm tool."""

import importlib.util
import re
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SRDF = ROOT / "src/piper_bringup/config/moveit/piper.srdf.xacro"
INITIAL = ROOT / "src/piper_description/config/initial_positions.yaml"
GUARD = ROOT / "src/piper_bringup/scripts/command_guard.py"
TOOL = ROOT / "tools/arm_zero.py"
JOINTS = [f"joint{i}" for i in range(1, 7)]


def srdf_pose(name):
    text = SRDF.read_text()
    block = re.search(rf'<group_state name="{name}" group="arm">(.*?)</group_state>', text, re.S).group(1)
    vals = dict(re.findall(r'<joint name="(\w+)" value="([-0-9.]+)"', block))
    return [float(vals[j]) for j in JOINTS]


def test_zero_is_the_startup_pose():
    initial = yaml.safe_load(INITIAL.read_text())["initial_positions"]
    assert srdf_pose("zero") == [initial[j] for j in JOINTS]


def test_rest_is_zero_with_joint5_lowered():
    zero, rest = srdf_pose("zero"), srdf_pose("rest")
    assert [r - z for r, z in zip(rest, zero)] == pytest.approx([0, 0, 0, 0, rest[4], 0])
    assert rest[4] == pytest.approx(0.2936)  # 16.82 deg, recorded on the real arm 2026-10-14


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
    assert mod.REST_J5 == pytest.approx(srdf_pose("rest")[4])
