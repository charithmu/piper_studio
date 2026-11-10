"""The piper-arm console driven key by key against a fake SDK arm: what each key does, what it refuses, and that e / SPACE work mid-move."""

import math
from types import SimpleNamespace as NS

import pytest
from test_real_arm import SLUMPED, FakeSdk

from piper_core.arm_cli import DEG, Console, _keys
from piper_core.real_arm import JOINTS, RealArm


class FakeGripper:
    """Width follows the target at 20 mm/s; an `obstacle` width stops a closing move (a held object); `frozen` ignores commands (not responding)."""

    def __init__(self):
        self.width, self.target, self.force, self.obstacle, self.frozen, self.sent = 0.0, 0.0, 0.0, None, False, []

    def advance(self, dt):
        step = 0.02 * dt
        w = self.width + max(-step, min(step, self.target - self.width))
        if self.obstacle is not None and self.target < self.obstacle <= self.width:
            w = max(w, self.obstacle)
        self.force = 0.0 if w == self.target else (1.0 if self.obstacle is not None and w == self.obstacle else 0.0)
        self.width = w

    def move_gripper_m(self, value=0.0, force=1.0):
        self.sent.append((value, force))
        if not self.frozen:
            self.target = value

    def get_gripper_status(self):
        return NS(msg=NS(value=self.width, force=self.force))


class Fake(FakeSdk):
    def __init__(self, q):
        super().__init__(q)
        self.estops, self.resets, self.estopped = 0, 0, False
        self.grip = FakeGripper()

    def advance(self, dt):
        super().advance(dt)
        self.grip.advance(dt)

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
    arm.gripper = sdk.grip
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
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert sdk.on and arm.pose_state() == "zero"
    return sdk, arm, c


def test_start_waits_for_enter_then_enables_and_goes_to_zero():
    sdk, arm, c = make()
    c.key("s")
    assert c.prompt and "START" in c.prompt
    assert not sdk.on and not sdk.sent                       # nothing happened before Enter
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert sdk.on and arm.pose_state() == "zero"
    assert sdk.sent[0] == pytest.approx([arm.zero()[j] for j in JOINTS])
    assert any("READY" in x for x in c.log_lines)


def test_any_other_key_cancels_a_prompt():
    sdk, arm, c = make()
    c.key("s")
    c.key("y")                                               # y no longer confirms
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
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert c.snap["near"] == "park" and arm.positions()["joint5"] == pytest.approx(arm.park()["joint5"], abs=0.02)
    c.key("r")
    assert "BIG MOVE" in c.prompt
    c.key("ENTER")
    until(c, lambda: c.task is None)
    c.refresh()
    assert c.snap["near"] == "ready"
    c.key("z")
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert c.snap["near"] == "zero"


def test_e_stops_at_once_in_the_middle_of_a_move():
    sdk, arm, c = started()
    c.key("r")
    c.key("ENTER")
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
    c.key("ENTER")
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
    c.key("ENTER")
    until(c, lambda: c.task is None)
    c.key("t")
    assert "END SESSION" in c.prompt
    c.key("ENTER")
    until(c, lambda: c.prompt and "At PARK" in c.prompt)
    assert sdk.estops == 0 and sdk.on                          # parked, motors still on, waiting for the operator
    assert c.snap["near"] == "park"
    c.key("ENTER")
    until(c, lambda: c.prompt and "at rest" in c.prompt.lower())
    assert sdk.estops == 1 and sdk.resets == 0
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert sdk.resets == 1 and not sdk.on and not sdk.estopped


def test_clear_estop_asks_first():
    sdk, arm, c = make()
    c.key("e")
    c.key("c")
    assert c.prompt and sdk.resets == 0
    c.key("ENTER")
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
    c2.key("ENTER")
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
    c.key("ENTER")
    assert c.quit and sdk.on


def test_screen_shows_status_and_keys():
    sdk, arm, c = make()
    text = c.render()
    assert "PIPER ARM" in text and "MOTORS OFF" in text and "slumped" in text and "EMERGENCY STOP" in text
    assert "gripper" in text
    assert f"{math.degrees(SLUMPED[4]):.2f}" in text


def test_key_parser():
    assert _keys(b"\x1b[C\x1b[Dab") == ["RIGHT", "LEFT", "a", "b"]
    assert _keys(b"\r\n") == ["ENTER", "ENTER"]
    assert _keys(b"\x1b") == ["ESC"]


def test_gripper_reads():
    sdk, arm, c = make()
    arm.gripper = None
    assert arm.gripper_opening() is None and arm.gripper_force() is None
    arm.gripper = NS(get_gripper_status=lambda: NS(msg=NS(value=0.0123, force=1.5)))
    assert arm.gripper_opening() == pytest.approx(0.0123) and arm.gripper_force() == pytest.approx(1.5)


# ---------------------------------------------------------------------- gripper
def test_open_and_close_ask_first_then_reach_the_width():
    sdk, arm, c = make()
    c.key("o")
    assert "OPEN" in c.prompt and sdk.grip.sent == []
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert sdk.grip.sent[0] == pytest.approx((0.07, 1.0)) and sdk.grip.width == pytest.approx(0.07, abs=1e-3)
    assert any("reached" in x for x in c.log_lines)
    c.key("l")
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert sdk.grip.width == pytest.approx(0.0, abs=1e-3) and sdk.grip.sent[-1][0] == 0.0


def test_closing_on_an_object_ends_stalled_with_width_and_force():
    sdk, arm, c = make()
    sdk.grip.width = sdk.grip.target = 0.07
    sdk.grip.obstacle = 0.03
    c.key("l")
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert any("stalled" in x and "30.0 mm" in x and "1.00 N" in x for x in c.log_lines), list(c.log_lines)
    assert sdk.grip.sent[-1][0] == pytest.approx(0.03, abs=1e-3)         # stopped pushing: aimed at where it stopped


def test_a_gripper_that_does_not_move_is_reported():
    sdk, arm, c = make()
    sdk.grip.frozen = True
    c.key("o")
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert any("no_motion" in x for x in c.log_lines)


def test_gripper_commands_are_guarded():
    sdk, arm, c = make()
    for width, effort in ((0.08, 1.0), (-0.01, 1.0), (0.03, 0.0), (0.03, 3.5), (float("nan"), 1.0), (0.03, float("nan"))):
        assert not arm.gripper_command(width, effort)
    assert sdk.grip.sent == []
    assert arm.gripper_command(0.03, 3.0) and sdk.grip.sent[-1] == (0.03, 3.0)
    arm.gripper_max_effort = 5.0                                              # raising the cap is a deliberate act
    assert arm.gripper_command(0.03, 4.0)
    sdk.estopped = True                                                       # e-stop: no gripper commands
    n = len(sdk.grip.sent)
    assert not arm.gripper_command(0.03, 1.0) and len(sdk.grip.sent) == n


def test_space_stops_the_gripper_where_it_is():
    sdk, arm, c = make()
    c.key("o")
    c.key("ENTER")
    run(c, 0.8)
    assert c.task is not None
    w = sdk.grip.width
    c.key(" ")
    assert c.task is None and sdk.grip.sent[-1][0] == pytest.approx(w, abs=2e-3)
    run(c, 1.0)
    assert sdk.grip.width == pytest.approx(w, abs=2e-3)


def test_force_key_cycles_and_never_exceeds_the_cap():
    sdk, arm, c = make()
    seen = []
    for _ in range(7):
        c.key("f")
        seen.append(c.grip_force)
    assert seen[:5] == [1.5, 2.0, 3.0, 0.5, 1.0] and max(seen) <= arm.gripper_max_effort
    c.key("l")
    c.key("ENTER")
    until(c, lambda: c.task is None)
    assert sdk.grip.sent[-1][1] <= arm.gripper_max_effort


def test_gripper_step_mode():
    sdk, arm, c = make()
    c.key("g")
    assert c.grip_mode
    c.key("+")
    assert sdk.grip.sent[-1][0] == pytest.approx(0.005)
    run(c, 0.5)
    c.key("+")
    assert sdk.grip.sent[-1][0] == pytest.approx(sdk.grip.width + 0.005, abs=1e-3)
    c.key(",")
    assert c.grip_step == 2.0
    c.key("-")
    for _ in range(5):
        c.key("-")
    assert sdk.grip.sent[-1][0] >= 0.0
    c.key("ESC")
    assert not c.grip_mode


def test_set_gripper_blocks_and_reports_how_it_ended():
    sdk, arm, c = make()
    w = arm.set_gripper(0.05, effort=1.5)
    assert w == pytest.approx(0.05, abs=1e-3) and arm.last_gripper.reason == "reached"
    sdk.grip.obstacle = 0.02
    w = arm.set_gripper(0.0)
    assert w == pytest.approx(0.02, abs=1e-3) and arm.last_gripper.reason == "stalled" and arm.last_gripper.force == pytest.approx(1.0)
    with pytest.raises(Exception, match="outside"):
        arm.set_gripper(0.2)


def test_state_carries_the_gripper_reading():
    sdk, arm, c = make()
    sdk.grip.width = 0.0123
    st = arm.state()
    assert st.gripper_width == pytest.approx(0.0123) and st.gripper_force == pytest.approx(0.0)
    arm.gripper = None
    assert math.isnan(arm.state().gripper_width)
