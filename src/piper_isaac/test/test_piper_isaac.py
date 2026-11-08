"""PiperIsaacSim on real Isaac Sim (GPU). Opt-in: PIPER_TEST_ISAAC=1 python -m pytest src/piper_isaac/test/test_piper_isaac.py -s   (needs the USD from `scripts/isaac.sh build-usd`).
The parity numbers printed with -s: max |measured - commanded| of the same 2 s plan (25 ms points, with dq) on PiperIsaacSim, PiperSim and, for the ROS Isaac backend,
`python tools/run_trajectory.py --backend isaac --trajectory plan.json --result r.json` (reported 0.025 rad by Piper Studio on 2026-11-08)."""

import os

import numpy as np
import pytest

import piper_assets as pa
from piper_core import Trajectory
from piper_isaac import PiperIsaacSim

pytestmark = pytest.mark.skipif(os.environ.get("PIPER_TEST_ISAAC") != "1", reason="needs Isaac Sim and the GPU: set PIPER_TEST_ISAAC=1")
Z = np.array(pa.zero_pose())


def plan(dt=0.024956, dur=2.0, amp=(0.6, 0.0, 0.0, 0.4, 0.0, 0.0)):
    """The plan of Piper Studio's parity numbers (and test_piper_sim): minimum jerk, j1 +0.6 rad, j4 +0.4 rad, 25 ms points with velocities."""
    t = np.arange(0.0, dur + 1e-9, dt)
    s = t / t[-1]
    p, v = 10 * s**3 - 15 * s**4 + 6 * s**5, (30 * s**2 - 60 * s**3 + 30 * s**4) / t[-1]
    d = np.array(amp)
    return Trajectory(pa.JOINT_NAMES, t, Z + p[:, None] * d, v[:, None] * d)


def track(res, tr):
    ref = np.stack([np.interp(res.t, tr.t, tr.q[:, j]) for j in range(6)], 1)
    return float(np.abs(res.q_meas - ref).max())


@pytest.fixture(scope="module")
def sim():
    s = PiperIsaacSim("table")
    yield s
    s.close()


def test_starts_at_zero_with_the_shared_scene(sim):
    assert sim.start() and sim.state().q == pytest.approx(Z, abs=1e-4) and sim.time == 0.0
    poses = sim.object_poses()
    assert set(poses) == {"cube_red", "cube_green", "cylinder_blue"} and poses["cube_red"][0] == pytest.approx([0.30, 0.06, 0.02], abs=2e-3)


def test_the_parity_plan_runs_cleanly(sim):
    sim.reset()
    tr = plan()
    r = sim.execute(tr)
    err = track(r, tr)
    print(f"\nPiperIsaacSim: max |measured - commanded| {err:.4f} rad, end error {np.abs(r.q_meas[-1] - tr.q[-1]).max():.2e}, {len(r.t)} steps, contacts {len(r.contacts)}")
    assert r.ok and r.reason == "done" and err < 0.06 and np.abs(r.q_meas[-1] - tr.q[-1]).max() < 2e-3 and r.contacts == []


def test_refusals_move_nothing_and_leave_the_simulation_time_alone(sim):
    sim.reset()
    t0, q0 = sim.time, sim.state().q
    far = plan()
    far.q[:] += 0.3 * np.array([1, 1, -1, 1, 1, 1.0])
    for tr, reason in ((far, "start_too_far"), (Trajectory(pa.JOINT_NAMES, [0, 1.0], [np.zeros(6)] * 2), "guard:margin"),
                       (Trajectory(pa.JOINT_NAMES, [0, 5.0], [Z, Z + [0, 0, 0, 2.0, 0, 0]]), "guard:limit")):
        r = sim.execute(tr)
        assert not r.ok and r.reason == reason
    assert sim.time == t0 and sim.state().q == pytest.approx(q0)


def test_a_25ms_and_a_2ms_plan_of_one_move_reach_the_same_pose(sim):
    ends = []
    for dt in (0.024956, 0.002):
        sim.reset()
        assert sim.execute(plan(dt=dt, dur=1.5)).ok
        ends.append(sim.state().q)
    assert ends[0] == pytest.approx(ends[1], abs=1e-3)


def test_gripper_and_repeated_execute(sim):
    sim.reset()
    sim.set_gripper(0.0)
    sim.set_gripper(0.07)
    assert sim.execute(plan(dur=1.0)).ok
    back = Trajectory(pa.JOINT_NAMES, plan(dur=1.0).t, plan(dur=1.0).q[::-1].copy(), -plan(dur=1.0).dq[::-1].copy())
    assert sim.execute(back).ok and sim.state().q == pytest.approx(Z, abs=1e-3)


def test_parity_with_pipersim(sim):
    pytest.importorskip("piper_mujoco")
    from piper_mujoco import PiperSim

    tr = plan()
    sim.reset()
    ri = sim.execute(tr)
    ms = PiperSim("table")
    rm = ms.execute(tr)
    ms.close()
    print(f"\nparity: PiperIsaacSim {track(ri, tr):.4f} rad, PiperSim {track(rm, tr):.4f} rad; end pose difference {np.abs(ri.q_meas[-1] - rm.q_meas[-1]).max():.2e}")
    assert ri.ok and rm.ok and np.abs(ri.q_meas[-1] - rm.q_meas[-1]).max() < 2e-3 and abs(track(ri, tr) - track(rm, tr)) < 0.03
