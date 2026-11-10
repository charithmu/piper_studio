"""The cuRobo scenario tools without a GPU or an arm: the stored plan passes the offline checks, is slowed by the cap, and the runner's dry run and the simulator run work."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCENARIO = ROOT / "tools/real_arm/fixtures/scenario_pick_25_0_05.json"
RUNNER = ROOT / "tools/real_arm/run_scenario.py"


def test_stored_plan_is_valid_and_retiming_caps_the_speed():
    pa = pytest.importorskip("piper_assets")
    from piper_core.backend import Trajectory, check_trajectory
    from piper_core.stream_plan import peak_speed, retime
    doc = json.loads(SCENARIO.read_text())
    assert doc["schema"] == "piper.scenario/1" and [s["op"] for s in doc["steps"]] == ["gripper", "move", "move", "gripper", "move", "gripper", "move"]
    lim = pa.limits()
    for name, leg in doc["legs"].items():
        t = Trajectory.from_json(json.dumps(leg))
        slow = retime(t, 0.3)
        assert peak_speed(slow) <= 0.3 * 1.05, name
        assert check_trajectory(slow, lim.hard, margin=lim.margin, max_step=lim.max_step, max_velocity=lim.velocity) is None, name
    first = Trajectory.from_json(json.dumps(doc["legs"]["approach"]))
    last = Trajectory.from_json(json.dumps(doc["legs"]["home"]))
    assert first.q[0] == pytest.approx(pa.zero_pose(), abs=1e-5) and last.q[-1] == pytest.approx(pa.zero_pose(), abs=1e-5)       # starts and ends at zero
    for a, b in (("approach", "grasp"), ("grasp", "lift"), ("lift", "home")):                                                      # every leg starts where the previous one ended
        qa, qb = json.loads(json.dumps(doc["legs"][a]["q"]))[-1], doc["legs"][b]["q"][0]
        assert qa == pytest.approx(qb, abs=1e-3), (a, b)


def test_runner_dry_run_needs_no_backend():
    pytest.importorskip("piper_assets")
    out = subprocess.run([sys.executable, str(RUNNER), str(SCENARIO), "--dry-run"], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert "offline check: OK" in out.stdout and out.stdout.count("offline check: OK") == 4 and "dry run" in out.stdout


def test_runner_refuses_yes_for_the_real_arm():
    pytest.importorskip("piper_assets")
    out = subprocess.run([sys.executable, str(RUNNER), str(SCENARIO), "--backend", "real", "--yes"], capture_output=True, text=True, timeout=60)
    assert out.returncode != 0 and "--yes is for the simulation only" in (out.stderr + out.stdout)


@pytest.mark.skipif("mujoco" not in sys.modules and __import__("importlib").util.find_spec("mujoco") is None, reason="MuJoCo not installed")
def test_scenario_runs_in_the_simulator_and_ends_at_zero(tmp_path):
    pytest.importorskip("piper_mujoco")
    copy = tmp_path / "scenario.json"
    copy.write_text(SCENARIO.read_text())
    out = subprocess.run([sys.executable, str(RUNNER), str(copy), "--backend", "mujoco", "--yes"], capture_output=True, text=True, timeout=240,
                         env={**__import__("os").environ, "MUJOCO_GL": "egl"})
    assert out.returncode == 0, out.stderr[-500:]
    assert "back at zero: True" in out.stdout and out.stdout.count("ok=True") == 4


def _gripper_tool():
    import importlib.util
    spec = importlib.util.spec_from_file_location("gripper_test", ROOT / "tools/real_arm/gripper_test.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_gripper_times_come_from_the_trace():
    g = _gripper_tool()
    trace = [(0.02 * k, max(0.0, 0.07 - 0.0004 * k), 0.0) for k in range(220)]      # closes at 20 mm/s from 70 mm (0.4 mm per 20 ms sample)
    reach, settle = g.times_from_trace(trace, 0.0)
    assert reach == pytest.approx(3.5, abs=0.06) and settle == pytest.approx(3.5, abs=0.06)
    held = [(0.02 * k, max(0.03, 0.07 - 0.0004 * k), -1.0) for k in range(160)]    # stalls on an object at 30 mm
    reach, settle = g.times_from_trace(held, 0.0)
    assert reach is None and settle == pytest.approx(2.0, abs=0.06)
    assert g.stats([1.0, 3.0, None])["mean"] == 2.0 and g.stats([])["n"] == 0


def test_gripper_empty_and_force_stages_run_against_the_fake_gripper():
    import argparse
    from test_arm_cli import make as make_fake
    g = _gripper_tool()
    sdk, arm, _ = make_fake(started=True)
    ns = argparse.Namespace(cycles=2, effort=1.0)
    out = g.stage_empty(arm, ns, lambda t: None)
    assert len(out["open"]) == 2 and out["open"][0]["width_mm"] == pytest.approx(70.0, abs=1.0) and out["close"][0]["width_mm"] == pytest.approx(0.0, abs=1.0)
    assert out["close"][0]["t_reach_s"] == pytest.approx(3.5, abs=0.2)
    rows = g.stage_force(arm, ns, lambda t: None)
    assert len(rows) == 10 and {r["effort_n"] for r in rows} == {0.5, 1.0, 1.5, 2.0, 3.0}
    sdk.grip.width = sdk.grip.target = 0.07
    sdk.grip.obstacle = 0.03
    held = g.move(arm, 0.0, 1.0)
    assert held["reason"] == "stalled" and held["width_mm"] == pytest.approx(30.0, abs=1.0) and held["t_reach_s"] is None and held["t_settle_s"] > 1.5
