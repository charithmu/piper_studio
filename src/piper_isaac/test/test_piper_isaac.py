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


# ---- the ZED X Mini wrist camera (the default) ----------------------------------------------------------------------------------------
def test_wrist_camera_is_on_by_default_and_matches_the_description(sim):
    """The optical frame Isaac renders from equals the description's: gripper-chain pose at several arm poses; size and depth are the camera model's."""
    assert sim.camera_enabled
    c = sim.camera_info()
    rgb, depth = sim.camera_image(depth=True)
    assert rgb.shape == (c["height"], c["width"], 3) and depth.shape == (c["height"], c["width"])
    assert float(rgb.std()) > 5.0, "colour image is blank"
    assert np.isfinite(depth).mean() > 0.2 and np.nanmin(depth) >= c["depth_min"] and np.nanmax(depth) <= c["depth_max"]
    t0 = sim.time
    sim.camera_image()
    assert sim.time == t0, "rendering the camera must not advance simulation time"


def test_the_camera_pose_equals_the_description_chain(sim):
    """camera_pose() against link6 x piper_assets.camera_pose('link6'): link6's own pose comes from the MuJoCo-free FK of the description (model_audit)."""
    import sys
    import xml.etree.ElementTree as ET

    from ament_index_python.packages import get_package_prefix  # noqa: F401  (needs the ROS environment; skipped otherwise)
    sys.path.insert(0, os.path.join(get_package_prefix("piper_description"), "lib", "piper_description"))
    import model_audit

    ref = model_audit.load_urdf("ref", pa.urdf_path(0.07, True))
    assert sim.start()
    for q in ([0, 0.01, -0.01, 0, 0, 0], [0.3, 0.6, -0.8, 0.2, 0.5, -0.4]):
        r = sim.move_to(q)
        assert r.ok, r.message if hasattr(r, "message") else r
        qd = dict(zip([f"joint{i}" for i in range(1, 7)], sim.state().q))
        want = ref.fk(qd, "zed_left_camera_frame_optical")
        got = sim.camera_pose()
        assert np.abs(got[:3, 3] - want[:3, 3]).max() < 2e-3, (got[:3, 3], want[:3, 3])
        assert np.abs(got[:3, :3] - want[:3, :3]).max() < 5e-3


def test_robot_only_variant_has_no_camera():
    s = PiperIsaacSim("empty", camera=False)
    try:
        assert not s.camera_enabled
        with pytest.raises(RuntimeError):
            s.camera_image()
        assert s.start()
    finally:
        s.close()
