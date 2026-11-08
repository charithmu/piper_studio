"""The backend contract without hardware: Trajectory, resampling, and the rules every backend applies to a whole trajectory."""

import numpy as np
import pytest

from piper_core.backend import ARM_JOINTS, ArmBackend, Trajectory, check_start, check_trajectory, resample

LIMITS = {"joint1": (-2.6, 2.6), "joint2": (0.0, 3.14), "joint3": (-2.97, 0.0), "joint4": (-1.7, 1.7), "joint5": (-1.2, 1.2), "joint6": (-2.9, 2.9)}
ZERO = [0.0, 0.01, -0.01, 0.0, 0.0, 0.0]


def move(dt=0.024956, n=40, amp=0.6, start=ZERO, with_dq=True):
    """A smooth rest-to-rest move of joint1 (minimum jerk), sampled with a non-uniform-looking dt like cuRobo's."""
    t = np.arange(n) * dt
    s = t / t[-1]
    pos = 10 * s**3 - 15 * s**4 + 6 * s**5
    vel = (30 * s**2 - 60 * s**3 + 30 * s**4) / t[-1]
    q = np.tile(start, (n, 1))
    q[:, 0] += amp * pos
    dq = np.zeros_like(q)
    dq[:, 0] = amp * vel
    return Trajectory(list(ARM_JOINTS), t, q, dq if with_dq else None)


def test_json_round_trip_and_schema():
    tr = move()
    tr.meta = {"planner": "curobo"}
    back = Trajectory.from_json(tr.to_json())
    assert back.joint_names == tr.joint_names and back.meta == tr.meta
    assert np.array_equal(back.q, tr.q) and np.array_equal(back.dq, tr.dq) and back.ddq is None
    with pytest.raises(ValueError):
        Trajectory.from_json('{"schema": "other/1"}')
    with pytest.raises(ValueError):
        Trajectory(list(ARM_JOINTS), [0, 1], [[0.0] * 5, [0.0] * 5])


def test_from_pairs_matches_the_old_realarm_input():
    tr = Trajectory.from_pairs([(0.0, ZERO), (0.5, ZERO)])
    assert tr.q.shape == (2, 6) and tr.dq is None


def test_hermite_reproduces_the_curve_from_a_coarse_plan():
    coarse, fine = move(0.024956, 40), move(0.002, 481)
    t, q = resample(coarse, 500)
    ref = np.interp(t / t[-1], fine.t / fine.t[-1], fine.q[:, 0])
    # same move, different sampling: the end points agree and the curve is close (cubic Hermite from position and velocity)
    assert q[0, 0] == pytest.approx(ZERO[0]) and q[-1, 0] == pytest.approx(ZERO[0] + 0.6)
    assert np.max(np.abs(q[:, 0] - ref)) < 1e-4
    assert t[-1] == pytest.approx(coarse.t[-1]) and np.all(np.diff(t) > 0)


def test_time_is_used_as_given_not_uniform_dt():
    tr = Trajectory.from_pairs([(0.0, ZERO), (0.1, [0.1] + ZERO[1:]), (0.5, [0.5] + ZERO[1:])])
    t, q = resample(tr, 100)
    assert q[np.argmin(abs(t - 0.1)), 0] == pytest.approx(0.1) and q[np.argmin(abs(t - 0.3)), 0] == pytest.approx(0.3)


def test_a_normal_plan_passes_unchanged_at_the_zero_pose():
    assert check_trajectory(move(), LIMITS) is None                    # zero is 0.01 inside the limits: passes
    assert check_trajectory(move(0.002, 481), LIMITS) is None          # 2 ms plan of the same move
    on_limit = move(start=[0.0, 0.0, 0.0, 0, 0, 0])                    # the exact-zero pose sits ON the limit: not allowed
    assert check_trajectory(on_limit, LIMITS)[0] == "guard:margin"


@pytest.mark.parametrize("edit,reason", [
    (lambda q: q.__setitem__((5, 0), 2.7), "guard:limit"),
    (lambda q: q.__setitem__((5, 1), 3.135), "guard:margin"),
    (lambda q: q.__setitem__((5, 2), float("nan")), "bad_trajectory"),
])
def test_bad_points_refuse_the_whole_trajectory(edit, reason):
    tr = move()
    before = tr.q.copy()
    edit(tr.q)
    got = check_trajectory(tr, LIMITS)
    assert got and got[0] == reason
    assert np.array_equal(np.delete(tr.q, 5, 0), np.delete(before, 5, 0))   # nothing else was touched


def test_step_velocity_and_time_rules():
    jump = Trajectory.from_pairs([(0.0, ZERO), (0.002, [0.2] + ZERO[1:])])      # 0.2 rad in one control step
    assert check_trajectory(jump, LIMITS)[0] == "guard:step"
    fast = Trajectory.from_pairs([(0.0, ZERO), (0.1, [0.5] + ZERO[1:])])       # 5 rad/s
    assert check_trajectory(fast, LIMITS)[0] == "guard:velocity"
    assert check_trajectory(Trajectory.from_pairs([(0.0, ZERO), (0.0, ZERO)]), LIMITS)[0] == "bad_trajectory"
    assert check_trajectory(Trajectory.from_pairs([(0.1, ZERO), (0.2, ZERO)]), LIMITS)[0] == "bad_trajectory"


def test_start_must_be_near_the_present_pose():
    tr = move()
    assert check_start(tr, ZERO) is None
    assert check_start(tr, [0.1] + ZERO[1:])[0] == "start_too_far"


def test_protocol_is_checkable():
    class Dummy:
        joint_names = list(ARM_JOINTS)
        start = stop = close = state = execute = command = hold = move_to = lambda self, *a, **k: None
    assert isinstance(Dummy(), ArmBackend)


def test_exec_result_json_round_trip():
    from piper_core.backend import ExecResult
    r = ExecResult(False, "guard:step", "joint1", np.array([0.0, 0.1]), np.zeros((2, 6)), np.ones((2, 6)), [(0.1, "guard:rejected", "x")], [(0.1, "a", "b", 0.001)])
    back = ExecResult.from_json(r.to_json())
    assert back.ok is False and back.reason == "guard:step" and back.message == "guard:step: joint1"
    assert np.array_equal(back.q_meas, r.q_meas) and back.guard_events == r.guard_events and back.contacts == r.contacts
    assert ExecResult(True).to_json() and ExecResult.from_json(ExecResult(True).to_json()).q_cmd.shape == (0, 6)
