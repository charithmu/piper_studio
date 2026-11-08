"""RealArm against a fake SDK arm: the start/stop/execute rules without hardware (no ROS, no pyAgxArm needed)."""

from types import SimpleNamespace as NS

import pytest

from piper_core.real_arm import JOINTS, RealArm

LIMITS = {"joint1": (-2.6, 2.6), "joint2": (0.0, 3.14), "joint3": (-2.97, 0.0), "joint4": (-1.7, 1.7), "joint5": (-1.2, 1.2), "joint6": (-2.9, 2.9)}
SLUMPED = [0.004, -0.0382, 0.0427, -0.0073, 0.3859, 0.0]


class FakeSdk:
    """Just enough of pyAgxArm's arm: first-order tracking of the last move_j target, sags into SLUMPED when disabled."""

    OPTIONS = NS(MOTION_MODE=NS(J="J"))

    def __init__(self, q):
        self.q, self.target, self.mode, self.speed, self.on = list(q), list(q), 0, 0, False
        self.fault = False
        self.sent = []

    def advance(self, dt):
        if self.on:
            k = min(1.0, 4.0 * dt)
            self.q = [a + k * (b - a) for a, b in zip(self.q, self.target)]

    def get_joint_angle_vel_limits(self, i, timeout=0):
        lo, hi = LIMITS[f"joint{i}"]
        return NS(msg=NS(min_angle_limit=lo, max_angle_limit=hi))

    def get_joint_angles(self):
        return NS(msg=list(self.q))

    def get_arm_status(self):
        return NS(msg=NS(ctrl_mode=1, teach_status=0, arm_status=7 if self.fault else 0, err_code=0, mode_feedback=1 if self.mode else 0))

    def get_driver_states(self, i):
        return NS(msg=NS(foc_status=NS(driver_overcurrent=self.fault and i == 3)))

    def set_speed_percent(self, p):
        self.speed = p

    def set_motion_mode(self, m):
        self.mode = 1

    def get_joint_enable_status(self, i):
        return self.on

    def enable(self):
        self.on = True
        return True

    def disable(self):
        self.on = False
        self.q = list(SLUMPED)
        self.target = list(SLUMPED)

    def move_j(self, q):
        assert self.on, "move_j while disabled"
        self.sent.append(list(q))
        self.target = list(q)

    def disconnect(self):
        pass


def make(q):
    sdk = FakeSdk(q)
    now = [0.0]

    def sleep(s):
        now[0] += s
        sdk.advance(s)
    return sdk, RealArm(sdk, clock=lambda: now[0], sleep=sleep)


def at(arm, goal):
    q = arm.positions()
    return max(abs(q[j] - goal[j]) for j in JOINTS) < 0.02


def test_start_from_the_slumped_pose_ends_enabled_at_zero():
    sdk, arm = make(SLUMPED)
    assert arm.pose_state() == "slumped"
    r = arm.start()
    assert r, r
    assert sdk.on and arm.pose_state() == "zero" and at(arm, arm.zero())
    assert sdk.sent[0] == pytest.approx([arm.zero()[j] for j in JOINTS])         # the one move out of the slump aims at zero
    assert arm.zero()["joint2"] == pytest.approx(0.01) and arm.zero()["joint3"] == pytest.approx(-0.01)


def test_start_refuses_on_a_fault_and_sends_nothing():
    sdk, arm = make(SLUMPED)
    sdk.fault = True
    r = arm.start()
    assert not r and "joint3:driver_overcurrent" in r.message
    assert not sdk.on and not sdk.sent


def test_nothing_moves_from_the_slumped_pose_except_start():
    sdk, arm = make(SLUMPED)
    assert not arm.move_to(arm.park())
    assert not arm.execute([(0.0, SLUMPED), (0.5, [0.0] * 6)])
    assert not sdk.sent


def test_move_to_a_valid_pose_and_back_to_zero_with_the_guard():
    sdk, arm = make(SLUMPED)
    assert arm.start()
    r = arm.move_to({**arm.zero(), "joint1": 0.5, "joint4": 0.3})
    assert r and at(arm, {**arm.zero(), "joint1": 0.5, "joint4": 0.3})
    assert not arm.move_to({**arm.zero(), "joint1": 2.7})                          # outside the limit: refused, not clamped
    assert sdk.sent[-1][0] == pytest.approx(0.5)


def test_execute_streams_a_trajectory_through_the_guard():
    sdk, arm = make(SLUMPED)
    assert arm.start()
    z = arm.zero()
    traj = [(0.025 * k, [z[j] + (0.3 * k / 20 if j == "joint1" else 0.0) for j in JOINTS]) for k in range(21)]
    r = arm.execute(traj)
    assert r, r
    assert len(sdk.sent) >= 21 and sdk.speed >= 1
    assert at(arm, dict(zip(JOINTS, traj[-1][1])))


def test_execute_refuses_a_bad_trajectory_and_aborts_on_a_guard_rejection():
    sdk, arm = make(SLUMPED)
    assert arm.start()
    z = [arm.zero()[j] for j in JOINTS]
    assert not arm.execute([(0.0, z), (0.01, [z[0] + 1.0] + z[1:])])               # 100 rad/s
    assert not arm.execute([(0.0, [z[0] + 0.5] + z[1:]), (1.0, z)])                # does not start at the present pose
    sent = len(sdk.sent)
    r = arm.execute([(0.0, z), (0.1, [z[0] + 0.1] + z[1:]), (2.6, [2.7] + z[1:])])  # last point outside joint1's limit
    assert not r and r.reason == "guard:limit"
    assert len(sdk.sent) == sent                                                   # refused whole before anything moved


def test_stop_parks_and_powers_off_into_the_slump():
    sdk, arm = make(SLUMPED)
    assert arm.start()
    assert arm.move_to({**arm.zero(), "joint1": 0.4})
    r = arm.stop(settle=0.2)
    assert r, r
    assert not sdk.on and arm.pose_state() == "slumped"
    assert sdk.sent[-1][4] == pytest.approx(0.2936)                                # the last pose before power-off was park
    assert arm.stop()                                                              # a slumped arm is left alone


def test_execute_is_an_arm_backend_and_logs_the_run():
    from piper_core.backend import ArmBackend, Trajectory
    sdk, arm = make(SLUMPED)
    assert isinstance(arm, ArmBackend) and arm.joint_names == list(JOINTS)
    assert arm.start()
    z = arm.zero()
    tr = Trajectory.from_pairs([(0.0249 * k, [z[j] + (0.3 * k / 20 if j == "joint1" else 0.0) for j in JOINTS]) for k in range(21)])
    r = arm.execute(tr)
    assert r.ok and r.reason == "done" and len(r.t) == len(r.q_cmd) == len(r.q_meas) == 20 and r.contacts == []
    assert arm.state().q[0] == pytest.approx(0.3, abs=0.02)
    assert arm.execute(tr).reason == "start_too_far"                              # the arm is at 0.3, the plan starts at zero
    assert sdk.sent[-1][0] == pytest.approx(0.3)                                  # and nothing moved
    with pytest.raises(Exception):
        arm.command([2.7, 0, 0, 0, 0, 0])
    arm.command([0.31, *[arm.state().q[i] for i in range(1, 6)]])
