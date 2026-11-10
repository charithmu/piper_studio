"""What piper_pick relies on from RealArm, as tests against the fake SDK: results, refusals that never move the arm, hold, abort, start/stop, streaming options, metrics."""

from types import SimpleNamespace as NS

import numpy as np
import pytest
from test_real_arm import SLUMPED, FakeSdk, at, make

from piper_core.backend import ExecResult, Trajectory
from piper_core.real_arm import JOINTS, RealArm
from piper_core.stream_metrics import analyze, format_metrics, repeatability


def ramp(arm, joint=0, amount=0.3, seconds=1.0, dt=0.025, pad=0.0):
    z = arm.zero()
    n = int(round(seconds / dt))
    rows = [(dt * k, [z[j] + (amount * k / n if i == joint else 0.0) for i, j in enumerate(JOINTS)]) for k in range(n + 1)]
    return Trajectory.from_pairs(rows)


def started():
    sdk, arm = make(SLUMPED)
    assert arm.start()
    return sdk, arm


def hook_sleep(arm, sdk, fn):
    """Call fn(call_number) on every sleep of the arm (the fake clock), to inject events in the middle of a run."""
    base, n = arm._sleep, [0]

    def sleep(s):
        n[0] += 1
        fn(n[0])
        base(s)
    arm._sleep = sleep


# ---------------------------------------------------------------- results
def test_execute_returns_an_exec_result_with_ok_reason_and_message():
    sdk, arm = started()
    r = arm.execute(ramp(arm))
    assert isinstance(r, ExecResult) and r.ok and r.reason == "done" and r.message.startswith("done")
    bad = arm.execute(ramp(arm, amount=9.0, seconds=60.0))
    assert isinstance(bad, ExecResult) and not bad.ok and bad.reason == "guard:limit" and "joint1" in bad.message and bad.message.startswith("guard:limit: ")


def test_every_refusal_has_a_reason_and_leaves_the_arm_where_it_was():
    sdk, arm = started()
    before, n = arm.positions(), len(sdk.sent)
    cases = {
        "guard:limit": ramp(arm, amount=9.0, seconds=60.0),
        "guard:velocity": ramp(arm, amount=0.4, seconds=0.025 * 4),
        "start_too_far": Trajectory.from_pairs([(0.0, [0.5] + list(arm.zero().values())[1:]), (0.1, [0.5] + list(arm.zero().values())[1:])]),
        "bad_trajectory": Trajectory.from_pairs([(0.0, [0.0] * 6)]),
    }
    for reason, traj in cases.items():
        r = arm.execute(traj)
        assert not r.ok and r.reason == reason, (reason, r.message)
        assert len(sdk.sent) == n, reason                              # nothing was sent to the arm
    assert arm.positions() == before


def test_a_disabled_arm_is_refused_instead_of_silently_ignored():
    sdk, arm = started()
    sdk.on = False
    sdk.q, sdk.target = list(arm.zero().values()), list(arm.zero().values())        # still at a valid pose, but the motors are off
    n = len(sdk.sent)
    r = arm.execute(ramp(arm))
    assert not r.ok and r.reason == "not_ready" and "not started" in r.message and "slump" not in r.message
    assert not arm.move_to(arm.zero()) and len(sdk.sent) == n
    with pytest.raises(Exception, match="not started"):
        arm.command(list(arm.zero().values()))
    assert len(sdk.sent) == n


def test_a_fault_stops_a_run_and_holds():
    sdk, arm = started()
    hook_sleep(arm, sdk, lambda i: setattr(sdk, "fault", True) if i == 60 else None)
    r = arm.execute(ramp(arm, seconds=2.0))
    assert not r.ok and r.reason == "fault" and "driver_overcurrent" in r.message and r.guard_events
    assert sdk.on                                                       # holding, motors still on


# ---------------------------------------------------------------- guard rejection while running never lets a bad point out
def test_guard_rejection_in_the_middle_stops_and_holds():
    sdk, arm = started()
    jumped = []

    def jump(i):
        if i == 80 and not jumped:                                       # the arm is pushed 0.5 rad away from the plan
            sdk.q[0] += 0.5
            sdk.target[0] += 0.5
            jumped.append(True)
    hook_sleep(arm, sdk, jump)
    r = arm.execute(ramp(arm, seconds=2.0))
    assert not r.ok and r.reason == "guard:rejected" and "holding" in r.detail
    sent_after = [q for q in sdk.sent if q[0] > 0.3 + 0.05]
    assert sent_after == [] or all(abs(q[0] - sdk.q[0]) < 0.2 for q in sent_after)


# ---------------------------------------------------------------- hold
def test_hold_at_a_valid_pose_sends_the_measured_pose_exactly():
    sdk, arm = started()
    assert arm.move_to({**arm.zero(), "joint1": 0.2})
    q = arm.positions()
    arm.hold()
    assert sdk.sent[-1] == pytest.approx([q[j] for j in JOINTS], abs=1e-12)        # not clamped: it is inside the limits


def test_hold_does_nothing_unless_the_arm_is_started():
    """Where a powered-off arm rests is not something downstream code can hold: hold() only acts on a started arm (the internal _hold aims at the range edge there)."""
    sdk, arm = make(SLUMPED)
    sdk.on = True
    arm.hold()
    assert sdk.sent == []
    arm._hold()
    assert sdk.sent[-1][1] == pytest.approx(arm.limits["joint2"][0]) and sdk.sent[-1][2] == pytest.approx(arm.limits["joint3"][1])


# ---------------------------------------------------------------- abort / stop
def test_abort_stops_a_running_trajectory_and_holds():
    sdk, arm = started()
    hook_sleep(arm, sdk, lambda i: arm.abort() if i == 50 else None)
    r = arm.execute(ramp(arm, seconds=2.0))
    assert not r.ok and r.reason == "stopped" and r.guard_events[-1][1] == "stopped"
    assert sdk.on and not arm._executing
    q = arm.positions()
    assert sdk.sent[-1] == pytest.approx([q[j] for j in JOINTS], abs=1e-9)         # holds where it is
    arm._sleep = lambda s: (setattr(arm, "_t", 0), FakeSdk.advance(sdk, s))        # a later run works again (the flag is cleared)
    assert arm.abort() is None                                                      # idle: harmless


def test_abort_when_idle_does_not_stop_the_next_run():
    sdk, arm = started()
    arm.abort()
    assert arm.execute(ramp(arm)).ok


def test_ctrl_c_holds_and_reraises():
    sdk, arm = started()

    def interrupt(i):
        if i == 40:
            raise KeyboardInterrupt
    hook_sleep(arm, sdk, interrupt)
    with pytest.raises(KeyboardInterrupt):
        arm.execute(ramp(arm, seconds=2.0))
    assert not arm._executing and sdk.on


def test_stop_is_refused_while_a_trajectory_runs():
    sdk, arm = started()
    results = []
    hook_sleep(arm, sdk, lambda i: results.append(arm.stop()) if i == 30 else None)
    r = arm.execute(ramp(arm, seconds=1.0))
    assert r.ok and results and not results[0].ok and "abort" in results[0].message


# ---------------------------------------------------------------- start / stop sequence
def test_stop_goes_to_zero_then_park_then_estop_then_reset():
    sdk, arm = started()
    assert arm.move_to({**arm.zero(), "joint1": 0.3})
    n = len(sdk.sent)
    r = arm.stop(settle=0.2)
    assert r and "e-stopped" in r.message
    moves = sdk.sent[n:]
    assert moves[0] == pytest.approx([arm.zero()[j] for j in JOINTS], abs=1e-9)         # zero first
    assert moves[-1] == pytest.approx([arm.park()[j] for j in JOINTS], abs=1e-9)        # then park
    assert sdk.estops == 1 and sdk.resets == 1 and not sdk.on and not sdk.estopped


def test_start_after_stop_works_at_once():
    sdk, arm = started()
    assert arm.stop(settle=0.2)
    assert arm.start() and arm._pose_state() == "zero" and sdk.on


# ---------------------------------------------------------------- streaming options
def accel_ramp(arm, seconds=2.0, dt=0.025, amount=0.4):
    """Slow start, fast middle: the segment velocities differ, so per_point has something to follow."""
    n = int(round(seconds / dt))
    s = np.linspace(0, 1, n + 1)
    pos = 10 * s**3 - 15 * s**4 + 6 * s**5
    z = arm.zero()
    return Trajectory.from_pairs([(dt * k, [z[j] + (amount * pos[k] if j == "joint1" else 0.0) for j in JOINTS]) for k in range(n + 1)])


def test_fixed_mode_uses_one_speed_and_per_point_follows_the_segments():
    sdk, arm = started()
    sdk.speeds.clear()
    assert arm.execute(accel_ramp(arm)).ok
    fixed = set(arm.last_stream["percent"])
    assert len(fixed) == 1 and len(set(sdk.speeds)) == 1
    sdk.speeds.clear()
    assert arm.move_to(arm.zero()).ok
    sdk.speeds.clear()
    assert arm.execute(accel_ramp(arm), mode="per_point").ok
    per = arm.last_stream["percent"]
    assert len(set(per)) > 5 and min(per) >= 5 and max(per) <= max(fixed)
    assert len(set(sdk.speeds)) > 5                                                  # set_speed_percent was called as the segments changed


def test_lookahead_sends_points_ahead_of_the_clock():
    sdk, arm = started()
    tr = ramp(arm, seconds=1.0)
    n = len(sdk.sent)
    assert arm.execute(tr, lookahead=3).ok
    assert sdk.sent[n] == pytest.approx(list(tr.q[1 + 3]), abs=1e-9)
    assert sdk.sent[-1] == pytest.approx(list(tr.q[-1]), abs=1e-9)
    assert not arm.execute(tr, lookahead=11).ok and not arm.execute(tr, mode="turbo").ok


def test_auto_lookahead_follows_the_measured_lags_and_bad_values_are_refused():
    assert [RealArm.auto_lookahead(v, 0.025) for v in (0.0, 0.137, 0.548, 0.822, 1.095, 3.0)] == [1, 2, 3, 4, 5, 6]     # joint 1 on the arm: lags 45 / 75 / 95 / 115 ms, capped at 6
    sdk, arm = started()
    assert arm.execute(accel_ramp(arm, seconds=3.0, amount=0.3), lookahead="auto").ok
    assert not arm.execute(accel_ramp(arm), lookahead="soon").ok and not arm.execute(accel_ramp(arm), lookahead=True).ok


def test_the_run_leaves_a_fine_log_and_metrics_find_the_lag():
    sdk, arm = started()
    tr = ramp(arm, seconds=2.0, amount=0.3)
    assert arm.execute(tr).ok
    st = arm.last_stream
    assert len(st["t"]) > 5 * len(st["t_sent"]) and len(st["t_sent"]) == len(tr.t) - 1
    m = analyze(tr, st)
    assert list(m["joints"]) == ["joint1"]
    assert 150 < m["lag_ms"] < 350                                                   # the fake is a first-order lag of 250 ms
    assert m["residual_rms_deg"] < m["rms_deg_at_zero_lag"]
    assert m["timing"]["late_max_ms"] < 10 and abs(m["timing"]["gap_mean_ms"] - 25.0) < 2.0
    assert "lag" in format_metrics(m)


def test_metrics_reject_a_plan_that_does_not_move():
    sdk, arm = started()
    still = Trajectory.from_pairs([(0.0, list(arm.zero().values())), (0.5, list(arm.zero().values()))])
    assert arm.execute(still).ok
    with pytest.raises(ValueError, match="does not move"):
        analyze(still, arm.last_stream)


def test_repeatability():
    r = repeatability([[0.1] * 6, [0.1002] * 6, [0.0998] * 6])
    assert len(r["std_deg"]) == 6 and r["std_deg"][0] == pytest.approx(0.0115, abs=0.002) and r["runs"] == 3
    with pytest.raises(ValueError):
        repeatability([[0.0] * 6])


def test_stream_plans_start_and_end_at_rest_at_q0():
    from piper_core.stream_plan import cosine, peak_speed, ramp, required_percent
    q0 = [0.0, 0.01, -0.01, 0.0, 0.0, 0.0]
    c = cosine(q0, 0, 0.2, 4.0)
    assert len(c.t) == 161 and c.t[1] - c.t[0] == pytest.approx(0.025) and c.q[0] == pytest.approx(q0) and c.q[-1] == pytest.approx(q0, abs=1e-12)
    assert c.q[:, 0].max() == pytest.approx(0.2, abs=1e-6) and peak_speed(c) == pytest.approx(0.2 * np.pi / 4.0, rel=0.02)
    assert (c.q[:, 1:] == np.array(q0[1:])).all()                               # only the chosen joint moves
    r = ramp(q0, 4, -0.3, 2.0)
    assert r.q[0] == pytest.approx(q0) and r.q[-1] == pytest.approx(q0) and r.q[:, 4].min() == pytest.approx(-0.3, abs=1e-6) and peak_speed(r) == pytest.approx(1.875 * 0.3 / 2.0, rel=0.03)
    assert required_percent(c) == int(np.ceil(peak_speed(c) * 1.2 * 100 / 3.0))


# ---------------------------------------------------------------- start / stop from any state, status, e-stop, stale feedback, bus lock
def test_start_clears_an_active_estop_and_works():
    sdk, arm = make(SLUMPED)
    sdk.estopped = True
    assert arm.status().e_stop and not arm.status().started
    r = arm.start()
    assert r and "cleared an e-stop" in r.message and sdk.resets == 1 and not sdk.estopped and arm.status().started and arm.status().at_zero


def test_start_twice_and_stop_twice_are_harmless():
    sdk, arm = make(SLUMPED)
    assert arm.start() and arm.start() and arm.status().started
    assert arm.stop(settle=0.2) and not arm.status().started
    r = arm.stop(settle=0.2)
    assert r and "already stopped" in r.message and sdk.estops == 1
    assert arm.start() and arm.status().started


def test_stop_clears_an_estop_and_stops_a_half_started_arm():
    sdk, arm = make(SLUMPED)
    sdk.estopped = True
    r = arm.stop()
    assert r and "e-stop cleared" in r.message and not sdk.estopped
    sdk, arm = make(SLUMPED)
    sdk.on = True                                              # enabled but never moved to zero (a start() that was interrupted)
    r = arm.stop(settle=0.2)
    assert r and sdk.estops == 1 and not sdk.on and arm._pose_state() == "slumped"


def test_stop_with_a_fault_commands_nothing_and_leaves_the_operator_to_estop():
    sdk, arm = make(SLUMPED)
    assert arm.start()
    n = len(sdk.sent)
    sdk.fault = True
    r = arm.stop()
    assert not r and "operator" in r.message and sdk.on and len(sdk.sent) == n and sdk.estops == 0


def test_status_says_why_and_never_mentions_the_off_pose():
    sdk, arm = make(SLUMPED)
    st = arm.status()
    assert not st.started and not st.enabled and not st.at_zero and st.faults == [] and not st.e_stop
    assert arm.start() and arm.status().started
    sdk.fault = True
    st = arm.status()
    assert not st.started and st.faults
    sdk.fault = False
    sdk.on = False
    for text in (arm.execute(ramp(arm)).message, str(arm.status()), arm.stop().message):
        assert "slump" not in text.lower()


class StampedSdk(FakeSdk):
    """get_joint_angles carries a receive stamp like the real SDK; `frozen` stops it advancing (arm silent / CAN lost)."""

    def __init__(self, q, clock):
        super().__init__(q)
        self.clock, self.frozen, self.stamp = clock, False, 0.0

    def get_joint_angles(self):
        if not self.frozen:
            self.stamp = self.clock()
        return NS(msg=list(self.q), timestamp=self.stamp + 1.7e9)         # any epoch: only the advance matters


def test_stale_feedback_is_a_fault_and_stops_a_run():
    from types import SimpleNamespace as NS_
    now = [0.0]
    sdk = StampedSdk(SLUMPED, lambda: now[0])
    sdk.estops = sdk.resets = 0
    sdk.estopped = False

    def sleep(sec):
        now[0] += sec
        sdk.advance(sec)
    arm = RealArm(sdk, clock=lambda: now[0], sleep=sleep)
    assert arm.start() and arm.problems() == []
    hook_sleep(arm, sdk, lambda i: setattr(sdk, "frozen", True) if i == 80 else None)
    r = arm.execute(ramp(arm, seconds=2.0))
    assert not r.ok and r.reason == "fault" and "stale" in r.message and sdk.on
    assert not arm.status().started and "stale" in " ".join(arm.status().faults)
    sdk.frozen = False
    assert arm.problems() == []                                              # fresh again


def test_a_can_error_in_the_middle_of_a_run_is_reported_and_holds_if_it_can():
    sdk, arm = started()
    boom = {"n": 0}
    real_move = sdk.move_j

    def flaky(q):
        boom["n"] += 1
        if boom["n"] == 10:
            raise OSError("can0: network is down")
        real_move(q)
    sdk.move_j = flaky
    r = arm.execute(ramp(arm, seconds=2.0))
    assert not r.ok and r.reason == "fault" and "CAN/SDK error" in r.message and not arm._executing


def test_bus_lock_allows_one_owner(tmp_path, monkeypatch):
    from piper_core.bus_lock import BusBusyError, BusLock
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path))
    a, b = BusLock("can9"), BusLock("can9")
    a.acquire()
    with pytest.raises(BusBusyError, match="in use by pid"):
        b.acquire()
    BusLock("can8").acquire().release()                                      # another bus is independent
    a.release()
    b.acquire()
    b.release()
