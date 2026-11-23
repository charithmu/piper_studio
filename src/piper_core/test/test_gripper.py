"""The gripper contract shared by the real arm and the simulators (piper_core.gripper)."""

import pytest

from piper_core.gripper import (GRIPPER_DEFAULT_EFFORT, GRIPPER_MAX_EFFORT, GripperWatch, check_gripper_command, grip_force_limit, ramp_target, run_gripper_move)


def test_the_force_is_the_effort_up_to_the_measured_plateau():
    assert grip_force_limit(1.0, 2.09) == 1.0 and grip_force_limit(2.0, 2.09) == 2.0 and grip_force_limit(3.0, 2.09) == 2.09


def test_ramp_moves_at_the_speed_and_lands_on_the_goal():
    t, seen = 0.07, []
    while t != 0.0:
        t = ramp_target(t, 0.0, 0.25, 0.002)
        seen.append(t)
    assert len(seen) == pytest.approx(0.07 / (0.25 * 0.002), abs=2) and all(a >= b for a, b in zip(seen, seen[1:]))
    assert ramp_target(0.0, 0.07, 0.25, 0.002) == pytest.approx(0.0005) and ramp_target(0.0699, 0.07, 0.25, 0.002) == 0.07


def test_the_command_rules_are_the_real_arms():
    assert check_gripper_command(0.03, None, 0.07, 3.0, 1.0) == (1.0, None)
    for width, effort in ((0.08, 1.0), (-0.01, 1.0), (0.03, 3.1), (0.03, 0.0), (float("nan"), 1.0), (0.03, float("inf"))):
        eff, err = check_gripper_command(width, effort, 0.07, 3.0, 1.0)
        assert eff is None and "outside" in err


def test_the_real_arm_and_the_simulators_share_the_constants():
    import piper_core.real_arm as ra
    assert ra.GRIPPER_STROKE == 0.07 and ra.GRIPPER_DEFAULT_EFFORT == GRIPPER_DEFAULT_EFFORT == 1.0 and ra.GRIPPER_MAX_EFFORT == GRIPPER_MAX_EFFORT == 3.0
    assert ra.GripperWatch is GripperWatch


class FakeGripper:
    """A one-dimensional gripper with a hard object at `block` (m): the opening cannot go below it."""
    def __init__(self, w0=0.07, block=0.0):
        self.w, self.t, self.block, self.f, self.limit = w0, 0.0, block, 0.0, 0.0

    def step(self, target):
        self.w = max(target, self.block)
        self.f = -self.limit if target < self.block else 0.0
        self.t += 0.002

    def run(self, width, effort=1.0, timeout=5.0):
        return run_gripper_move(width=width, effort=effort, timeout=timeout, speed=0.25, plateau=2.09, dt=0.002, step=self.step, opening=lambda: self.w,
                                force=lambda: self.f, time_now=lambda: self.t, set_force_limit=lambda n: setattr(self, "limit", n), current_target=self.w)


def test_a_move_without_an_object_is_reached_with_the_real_duration():
    g = FakeGripper()
    r, target = g.run(0.0)
    assert r.reason == "reached" and target == 0.0 and 0.55 < r.duration < 0.70


def test_a_move_onto_an_object_ends_stalled_with_the_limited_force():
    g = FakeGripper(block=0.054)
    r, _ = g.run(0.0, effort=3.0)
    assert r.reason == "stalled" and r.width == pytest.approx(0.054) and r.force == pytest.approx(-2.09)


def test_a_move_that_cannot_start_is_no_motion_and_one_that_never_ends_times_out():
    g = FakeGripper(w0=0.05, block=0.05)
    r, _ = g.run(0.0)
    assert r.reason in ("no_motion", "stalled")
    g = FakeGripper()
    r, _ = g.run(0.0, timeout=0.1)
    assert r.reason == "timeout"
