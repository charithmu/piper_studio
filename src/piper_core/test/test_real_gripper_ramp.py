"""RealArm.set_gripper(speed=) against a fake gripper: the target ramps like the simulators', the end is decided by GripperWatch."""

from types import SimpleNamespace as NS

import pytest

from piper_core.real_arm import RealArm
from test_real_arm import FakeSdk, SLUMPED


class FakeGripper:
    """Follows the last target at 0.25 m/s but cannot close below `object_width` (an object between the fingers)."""

    def __init__(self, width, object_width=0.0):
        self.width, self.target, self.object_width, self.sent = width, width, object_width, []

    def advance(self, dt):
        goal = max(self.target, self.object_width)
        step = 0.25 * dt
        self.width += max(-step, min(step, goal - self.width))

    def move_gripper_m(self, value, force):
        self.sent.append((value, force))
        self.target = value

    def get_gripper_status(self):
        return NS(msg=NS(value=self.width, force=-1.0 if self.width <= self.object_width + 1e-6 and self.object_width else 0.0))


def make(object_width):
    sdk, grip, now = FakeSdk(SLUMPED), FakeGripper(0.07, object_width), [0.0]
    sdk.on = True

    def sleep(s):
        now[0] += s
        grip.advance(s)
    arm = RealArm(sdk, clock=lambda: now[0], sleep=sleep)
    arm.gripper = grip
    return arm, grip


def test_a_ramped_close_steps_the_target_down_at_the_speed():
    arm, grip = make(object_width=0.0)
    arm.set_gripper(0.0, effort=1.0, timeout=10.0, speed=0.05)
    targets = [v for v, _ in grip.sent]
    assert targets[0] == pytest.approx(0.07 - 0.05 * 0.02)
    assert all(a >= b for a, b in zip(targets, targets[1:])) and targets[-1] == 0.0
    assert arm.last_gripper.reason == "reached"
    assert 1.2 < arm.last_gripper.duration < 2.2           # 70 mm at 50 mm/s plus the 0.3 s hold


def test_a_ramped_close_on_an_object_stalls_and_aims_at_where_it_is():
    arm, grip = make(object_width=0.04)
    width = arm.set_gripper(0.0, effort=1.0, timeout=10.0, speed=0.05)
    assert arm.last_gripper.reason == "stalled"
    assert width == pytest.approx(0.04, abs=0.002)
    assert grip.sent[-1][0] == pytest.approx(width, abs=1e-6)


def test_without_speed_one_target_is_sent():
    arm, grip = make(object_width=0.0)
    arm.set_gripper(0.0, effort=1.0, timeout=5.0)
    assert grip.sent[0][0] == 0.0
