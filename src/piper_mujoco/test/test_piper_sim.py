"""PiperSim against the backend contract (no ROS, no hardware)."""

import numpy as np
import pytest

import piper_assets as pa
from piper_core import ArmBackend, Trajectory
from piper_mujoco import PiperSim

Z = np.array(pa.zero_pose())


def plan(dt=0.024956, dur=2.0, amp=(0.6, 0.0, 0.0, 0.4, 0.0, 0.0), start=Z, end_at_zero=False):
    """A smooth rest-to-rest move away from `start` (minimum jerk), with velocities, like a cuRobo plan."""
    t = np.arange(0.0, dur + 1e-9, dt)
    s = t / t[-1]
    p, v = 10 * s**3 - 15 * s**4 + 6 * s**5, (30 * s**2 - 60 * s**3 + 30 * s**4) / t[-1]
    d = np.array(amp)
    return Trajectory(pa.JOINT_NAMES, t, start + p[:, None] * d, v[:, None] * d)


@pytest.fixture(params=["empty", "table"])
def sim(request):
    s = PiperSim(request.param)
    yield s
    s.close()


def test_is_a_backend_and_starts_at_zero(sim):
    assert isinstance(sim, ArmBackend)
    assert sim.state().q == pytest.approx(Z, abs=1e-9) and sim.start()
    assert sim.limits_info.gripper_stroke == 0.07
    assert sim.object_poses().keys() == ({"cube_red", "cube_green", "cylinder_blue"} if sim.scene == "table" else set())


def test_zero_pose_with_open_gripper_has_no_contact(sim):
    c = []
    for _ in range(200):
        sim._step(c, sim.time)
    assert c == [] and sim.state().q == pytest.approx(Z, abs=1e-6)


def test_out_and_back_ends_at_zero_without_contacts(sim):
    out = plan()
    r = sim.execute(out)
    assert r.ok and r.reason == "done" and r.contacts == []
    assert len(r.t) == len(r.q_cmd) == len(r.q_meas) and np.max(np.abs(sim.state().q - out.q[-1])) < 1e-3
    back = plan(start=sim.state().q, amp=tuple(Z - sim.state().q))               # planned from state().q, like a planner in a loop
    back.q[0] = sim.state().q
    r = sim.execute(back)
    assert r.ok and np.max(np.abs(sim.state().q - Z)) < 1e-3                       # within 1 mrad of zero


def test_25ms_and_2ms_plans_of_the_same_move_end_at_the_same_pose():
    ends = []
    for dt in (0.024956, 0.002):
        s = PiperSim()
        r = s.execute(plan(dt=dt))
        assert r.ok
        ends.append(s.state().q)
    assert np.max(np.abs(ends[0] - ends[1])) < 1e-3


def test_refusals_move_nothing(sim):
    before = sim.state().q.copy()
    t0 = sim.time
    bad_limit = plan(); bad_limit.q[5, 3] = 2.0
    exact_zero = plan(start=np.zeros(6))
    far = plan(start=Z + np.array([0.1, 0, 0, 0, 0, 0]))
    jump = Trajectory.from_pairs([(0.0, Z), (0.002, Z + np.array([0.2, 0, 0, 0, 0, 0]))])
    for traj, reason in [(bad_limit, "guard:limit"), (exact_zero, "guard:margin"), (far, "start_too_far"), (jump, "guard:step")]:
        r = sim.execute(traj)
        assert not r.ok and r.reason == reason, (reason, r.reason, r.detail)
    assert sim.time == t0 and sim.state().q == pytest.approx(before)


def test_command_goes_through_the_guard(sim):
    sim.command(Z + np.array([0.01, 0, 0, 0, 0, 0]))
    from piper_core import BackendError
    with pytest.raises(BackendError):
        sim.command([3.0, 0, 0, 0, 0, 0])
    sim.hold()
    sim.reset()
    assert sim.state().q == pytest.approx(Z, abs=1e-9)


def test_move_to_and_stop_return_to_zero(sim):
    r = sim.move_to(Z + np.array([0.5, 0.3, -0.3, 0.2, 0.2, 0.1]))
    assert r.ok and np.max(np.abs(sim.state().q - (Z + [0.5, 0.3, -0.3, 0.2, 0.2, 0.1]))) < 2e-3
    assert not sim.move_to([2.7, 0, 0, 0, 0, 0]).ok                                # outside the limits: refused
    assert sim.stop() and np.max(np.abs(sim.state().q - Z)) < 1e-3


def test_gripper_contract_matches_the_real_arm_api():
    """Same shape as RealArm: set_gripper(width, effort=None, timeout=None) -> measured width, gripper_opening(), gripper_force() (NaN here), width in state()."""
    import math
    s = PiperSim("table", gripper_width=0.07)
    assert s.gripper_opening() == pytest.approx(0.07) and s.state().gripper_width == pytest.approx(0.07) and math.isnan(s.gripper_force())
    w = s.set_gripper(0.03, effort=1.0)
    assert w == pytest.approx(0.03, abs=1e-3) and s.state().gripper_width == pytest.approx(w)
    assert s.set_gripper(0.05, 2.0, 1.5) == pytest.approx(0.05, abs=1e-3)
    with pytest.raises(Exception, match="outside"):
        s.set_gripper(0.08)


def test_gripper_and_table_scene():
    s = PiperSim("table", gripper_width=0.07)
    s.set_gripper(0.03)
    assert s.model.joint("gripper").id >= 0 and s.data.qpos[s.model.joint("gripper").qposadr[0]] == pytest.approx(0.03, abs=1e-3)
    s.reset()
    assert s.data.qpos[s.model.joint("gripper").qposadr[0]] == pytest.approx(0.07)
    with pytest.raises(Exception):
        PiperSim(gripper_width=0.08)


def test_wheel_urdf_and_mjcf_agree_on_the_zero_pose_tcp():
    """The URDF a planner reads and the MuJoCo model place link 6 at the same place at zero (kinematics from one description)."""
    import xml.etree.ElementTree as ET
    sim = PiperSim()
    root = ET.fromstring(pa.urdf_path(0.07).read_text())
    joints = {j.get("name"): j for j in root.findall("joint")}
    p = np.zeros(3)
    # forward kinematics of the URDF chain to gripper_base at zero pose
    R = np.eye(3)
    def rot(rpy):
        r, pp, y = rpy
        cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(pp), np.sin(pp), np.cos(y), np.sin(y)
        return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr], [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr], [-sp, cp*sr, cp*cr]])
    def axis_rot(ax, a):
        ax = np.array(ax) / np.linalg.norm(ax); K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    chain = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    parent = {j.find("child").get("link"): j for j in root.findall("joint")}
    link, path = "link6", []
    while link in parent:
        path.append(parent[link]); link = parent[link].find("parent").get("link")
    for j in reversed(path):
        o = j.find("origin")
        p = p + R @ np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        R = R @ rot([float(v) for v in o.get("rpy", "0 0 0").split()])
        if j.get("name") in chain:
            a = Z[chain.index(j.get("name"))]
            R = R @ axis_rot([float(v) for v in j.find("axis").get("xyz").split()], a)
    import mujoco
    mujoco.mj_forward(sim.model, sim.data)
    assert sim.data.body("link6").xpos == pytest.approx(p, abs=1e-6)
