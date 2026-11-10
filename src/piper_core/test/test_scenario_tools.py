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
