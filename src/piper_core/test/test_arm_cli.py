"""The piper-arm console driven key by key against a fake SDK arm: what each key does, what it refuses, and that e / SPACE work mid-move."""

import math
from types import SimpleNamespace as NS

import pytest
from test_real_arm import SLUMPED, FakeSdk

from piper_core.arm_cli import DEG, Console, _keys
from piper_core.real_arm import JOINTS, RealArm


class Fake(FakeSdk):
    def __init__(self, q):
        super().__init__(q)
        self.estops, self.resets, self.estopped = 0, 0, False

    def get_arm_status(self):
        st = super().get_arm_status()
        st.msg.arm_status = 1 if self.estopped else st.msg.arm_status
        return st

    def electronic_emergency_stop(self):
        self.estops, self.estopped, self.on = self.estops + 1, True, False
        self.q, self.target = list(SLUMPED), list(SLUMPED)

    def reset(self):
        self.resets, self.estopped = self.resets + 1, False


def make(q=SLUMPED):
    sdk = Fake(q)
    now = [0.0]

    def sleep(s):
        now[0] += s
        sdk.advance(s)
    arm = RealArm(sdk, clock=lambda: now[0], sleep=sleep)
    return sdk, arm, Console(arm, speed=10)


def run(c, seconds=1.0):
    for _ in range(int(round(seconds / 0.1))):
        c.arm._sleep(0.1)
        c.tick()


def until(c, what, limit=40.0):
    t = 0.0
    while t < limit and not what():
        run(c, 0.1)
        t += 0.1
    assert what(), c.render()


def started():
    sdk, arm, c = make()
    c.key("s")
    c.key("y")
    until(c, lambda: c.task is None)
    assert sdk.on and arm.pose_state() == "zero"
    return sdk, arm, c


def test_start_waits_for_y_then_enables_and_goes_to_zero():
    sdk, arm, c = make()
    c.key("s")
    assert c.prompt and "START" in c.prompt
    assert not sdk.on and not sdk.sent                       # nothing happened before y
    c.key("y")
    until(c, lambda: c.task is None)
    assert sdk.on and arm.pose_state() == "zero"
    assert sdk.sent[0] == pytest.approx([arm.zero()[j] for j in JOINTS])
    assert any("READY" in x for x in c.log_lines)


def test_any_other_key_cancels_a_prompt():
    sdk, arm, c = make()
    c.key("s")
    c.key("n")
    assert c.prompt is None and c.task is None
    assert not sdk.on and not sdk.sent


def test_poses_are_refused_while_the_motors_are_off():
    sdk, arm, c = make()
    for k in "zpr":
        c.key(k)
        assert c.prompt is None and c.task is None
    assert not sdk.sent and any("refused" in x for x in c.log_lines)


def test_park_and_ready_after_start():
    sdk, arm, c = started()
    c.key("p")
    c.key("y")
    until(c, lambda: c.task is None)
    assert c.snap["near"] == "park" and arm.positions()["joint5"] == pytest.approx(arm.park()["joint5"], abs=0.02)
    c.key("r")
    assert "BIG MOVE" in c.prompt
    c.key("y")
    until(c, lambda: c.task is None)
    c.refresh()
    assert c.snap["near"] == "ready"
    c.key("z")
    c.key("y")
    until(c, lambda: c.task is None)
    assert c.snap["near"] == "zero"


def test_e_stops_at_once_in_the_middle_of_a_move():
    sdk, arm, c = started()
    c.key("r")
    c.key("y")
    run(c, 0.3)
    assert c.task is not None and sdk.estops == 0
    c.key("e")
    assert sdk.estops == 1 and c.task is None and not sdk.on
    c.refresh()
    assert "E-STOP" in c.render()


def test_e_needs_no_prompt_even_when_idle():
    sdk, arm, c = make()
    c.key("e")
    assert sdk.estops == 1


def test_space_holds_the_pose_and_keeps_the_motors_on():
    sdk, arm, c = started()
    c.key("r")
    c.key("y")
    run(c, 0.3)
    n = len(sdk.sent)
    c.key(" ")
    assert c.task is None and sdk.on and sdk.estops == 0
    assert len(sdk.sent) == n + 1                              # one hold command at the present pose
    q = arm.positions()
    assert sdk.sent[-1] == pytest.approx([min(max(q[j], arm.limits[j][0]), arm.limits[j][1]) for j in JOINTS])


def test_end_session_goes_zero_park_then_asks_for_estop_then_for_reset():
    sdk, arm, c = started()
    c.key("p")
    c.key("y")
    until(c, lambda: c.task is None)
    c.key("t")
    assert "END SESSION" in c.prompt
    c.key("y")
    until(c, lambda: c.prompt and "At PARK" in c.prompt)
    assert sdk.estops == 0 and sdk.on                          # parked, motors still on, waiting for the operator
    assert c.snap["near"] == "park"
    c.key("y")
    until(c, lambda: c.prompt and "at rest" in c.prompt.lower())
    assert sdk.estops == 1 and sdk.resets == 0
    c.key("y")
    until(c, lambda: c.task is None)
    assert sdk.resets == 1 and not sdk.on and not sdk.estopped


def test_clear_estop_asks_first():
    sdk, arm, c = make()
    c.key("e")
    c.key("c")
    assert c.prompt and sdk.resets == 0
    c.key("y")
    until(c, lambda: c.task is None)
    assert sdk.resets == 1 and not sdk.estopped


def test_a_fault_blocks_start_and_aborts_a_move():
    sdk, arm, c = make()
    sdk.fault = True
    c.key("s")
    assert c.prompt is None and not sdk.sent and any("not starting" in x for x in c.log_lines)
    sdk.fault = False
    sdk2, arm2, c2 = started()
    c2.key("r")
    c2.key("y")
    run(c2, 0.3)
    sdk2.fault = True
    run(c2, 0.3)
    assert c2.task is None and any("ABORT" in x for x in c2.log_lines) and sdk2.on


def test_jog_steps_one_joint_and_respects_the_limits():
    sdk, arm, c = started()
    c.key("j")
    assert c.jog_joint == -1
    c.key("5")
    c.key("+")
    assert sdk.sent[-1][4] == pytest.approx(0.5 / DEG, abs=1e-3)
    assert sdk.sent[-1][:4] == pytest.approx([arm.zero()[j] for j in JOINTS[:4]], abs=1e-3)       # the other joints stay where they are
    c.key("RIGHT")
    assert sdk.sent[-1][4] == pytest.approx(1.0 / DEG, abs=1e-3)
    c.key("-")
    c.key("-")
    c.key("-")
    assert sdk.sent[-1][4] == pytest.approx(-0.5 / DEG, abs=1e-3)
    n = len(sdk.sent)
    for _ in range(10):
        c.key("+")
        run(c, 0.3)
    assert len(sdk.sent) > n
    c.key("ESC")
    assert c.jog_joint is None


def test_jog_refuses_to_run_away_from_a_lagging_arm_and_when_off():
    sdk, arm, c = make()
    c.key("j")
    assert c.jog_joint is None and not sdk.sent
    sdk, arm, c = started()
    c.key("j")
    c.key("2")
    c.jog_step = 2.0
    sent = len(sdk.sent)
    for _ in range(4):                                         # presses without waiting for the arm: target runs away from the measured pose
        c.key("+")
    assert len(sdk.sent) - sent < 4 and any("behind the target" in x for x in c.log_lines)


def test_jog_stays_inside_the_stored_limits():
    sdk, arm, c = started()
    lo = arm.limits["joint2"][0]
    c.key("j")
    c.key("2")
    c.jog_target[1] = lo + 0.01
    c.key("-")
    assert any("would leave" in x for x in c.log_lines)


def test_speed_keys_and_quit():
    sdk, arm, c = make()
    c.key("]")
    assert c.speed == 15
    for _ in range(10):
        c.key("]")
    assert c.speed == 50
    for _ in range(10):
        c.key("[")
    assert c.speed == 3
    c.key("q")
    assert c.quit                                               # motors off: no question
    sdk, arm, c = started()
    c.key("q")
    assert c.prompt and not c.quit                              # motors on: asks
    c.key("y")
    assert c.quit and sdk.on


def test_screen_shows_status_and_keys():
    sdk, arm, c = make()
    text = c.render()
    assert "PIPER ARM" in text and "MOTORS OFF" in text and "slumped" in text and "EMERGENCY STOP" in text
    assert "gripper" in text
    assert f"{math.degrees(SLUMPED[4]):.2f}" in text


def test_key_parser():
    assert _keys(b"\x1b[C\x1b[Dab") == ["RIGHT", "LEFT", "a", "b"]
    assert _keys(b"\x1b") == ["ESC"]


def test_gripper_reads():
    sdk, arm, c = make()
    assert arm.gripper_opening() is None and arm.gripper_force() is None
    arm.gripper = NS(get_gripper_status=lambda: NS(msg=NS(value=0.0123, force=1.5)))
    assert arm.gripper_opening() == pytest.approx(0.0123) and arm.gripper_force() == pytest.approx(1.5)
