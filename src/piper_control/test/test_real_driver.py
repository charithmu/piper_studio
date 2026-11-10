"""piper_real_driver against a fake SDK arm: topics, services, setpoints and the safety rules, without a ROS graph and without the arm (the node's methods are called directly)."""

import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

rclpy = pytest.importorskip("rclpy")
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "piper_core/test"))
from sensor_msgs.msg import JointState  # noqa: E402
from std_srvs.srv import SetBool, Trigger  # noqa: E402
from test_arm_cli import Fake  # noqa: E402  (FakeSdk with e-stop, reset and a fake gripper)
from test_real_arm import SLUMPED  # noqa: E402

from piper_control.real_driver import RealDriver  # noqa: E402
from piper_core.real_arm import JOINTS, RealArm  # noqa: E402


class MotorFake(Fake):
    def get_motor_states(self, i):
        return NS(msg=NS(velocity=0.01 * i, torque=0.1 * i, current=0.0, position=0.0))

    def get_driver_states(self, i):
        return NS(msg=NS(vol=24.0, foc_temp=30.0, motor_temp=28.0, bus_current=0.1,
                         foc_status=NS(driver_overcurrent=self.fault and i == 3, voltage_too_low=False, motor_overheating=False, driver_overheating=False,
                                       collision_status=False, driver_error_status=False, stall_status=False)))

    def get_arm_status(self):
        st = super().get_arm_status()
        st.msg.motion_status, st.msg.trajectory_num = 0, 0
        return st


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def drv():
    sdk = MotorFake(SLUMPED)
    now = [0.0]

    def sleep(s):
        now[0] += s
        sdk.advance(s)
    arm = RealArm(sdk, clock=lambda: now[0], sleep=sleep)
    arm.gripper = sdk.grip
    node = RealDriver(arm=arm)
    published = {}
    for name in ("pub", "arm_status_pub", "motor_status_pub", "gripper_status_pub"):
        setattr(node, name, NS(publish=lambda m, n=name: published.__setitem__(n, m)))
    node.published, node.sdk, node.fake_arm = published, sdk, arm
    yield node
    node.destroy_node()


def call(fn, **kw):
    resp = SetBool.Response() if kw else Trigger.Response()
    return fn(SetBool.Request(**kw) if kw else Trigger.Request(), resp)


def cmd(drv, q, grip=None, effort=None):
    names, pos = list(JOINTS), list(q)
    msg = JointState(name=names, position=pos)
    if grip is not None:
        msg.name.append("gripper")
        msg.position.append(grip)
        msg.effort = [float("nan")] * 6 + [effort if effort is not None else 0.0]
    drv.on_command(msg)


def test_feedback_has_official_names_gripper_velocity_and_effort(drv):
    drv.publish_feedback()
    m = drv.published["pub"]
    assert list(m.name) == list(JOINTS) + ["gripper"]
    assert list(m.position[:6]) == pytest.approx(SLUMPED) and m.position[6] == pytest.approx(0.0)
    assert m.velocity[2] == pytest.approx(0.03) and m.effort[2] == pytest.approx(0.3)


def test_no_feedback_means_nothing_is_published(drv):
    drv.sdk.get_joint_angles = lambda: None
    drv.publish_feedback()
    assert "pub" not in drv.published


def test_status_topics_match_the_vendor_layout(drv):
    drv.publish_status()
    arm = drv.published["arm_status_pub"].status[0]
    assert arm.name == "arm" and arm.level == bytes([0])
    values = {kv.key for kv in arm.values}
    assert {"ctrl_mode", "arm_status", "mode_feedback", "teach_status", "motion_status", "trajectory_num", "err_status"} <= values
    motors = drv.published["motor_status_pub"].status
    assert [s.name for s in motors] == list(JOINTS) and motors[0].level == bytes([0])
    assert {kv.key for kv in motors[0].values} >= {"enabled", "undervoltage", "driver_overcurrent", "bus_voltage_v", "driver_temp_deg", "motor_temp_deg"}
    assert drv.published["gripper_status_pub"].status[0].name == "gripper"


def test_a_fault_or_estop_is_reported_as_error_level(drv):
    drv.sdk.fault = True
    drv.publish_status()
    assert drv.published["arm_status_pub"].status[0].level == bytes([2])
    assert drv.published["motor_status_pub"].status[2].level == bytes([2]) and "driver_overcurrent" in drv.published["motor_status_pub"].status[2].message
    drv.sdk.fault = False
    drv.sdk.estopped = True
    drv.publish_status()
    arm = drv.published["arm_status_pub"].status[0]
    assert arm.level == bytes([2]) and "arm_status=1" in arm.message


def test_start_service_goes_to_zero_and_stop_service_ends_the_session(drv):
    r = call(drv.on_start)
    assert r.success and "ready" in r.message and drv.fake_arm.status().started and drv.fake_arm.status().at_zero
    assert not drv.busy.is_set()
    r = call(drv.on_stop)
    assert r.success and drv.sdk.estops == 1 and drv.sdk.resets == 1 and not drv.sdk.on
    r = call(drv.on_stop)
    assert r.success and "already stopped" in r.message


def test_start_clears_an_estop_and_a_second_start_is_harmless(drv):
    drv.sdk.estopped = True
    r = call(drv.on_start)
    assert r.success and "cleared an e-stop" in r.message
    assert call(drv.on_start).success


def test_setpoints_are_dropped_until_started_and_after_that_reach_the_arm(drv):
    z = [drv.fake_arm.zero()[j] for j in JOINTS]
    cmd(drv, z)
    assert drv.sdk.sent == [] and drv.rejected == 1                      # not started: refused, nothing sent
    assert call(drv.on_start).success
    n = len(drv.sdk.sent)
    q = list(z)
    q[0] += 0.05
    cmd(drv, q)
    assert len(drv.sdk.sent) == n + 1 and drv.sdk.sent[-1][0] == pytest.approx(q[0], abs=1e-6)


def test_nan_setpoints_are_ignored_and_setpoints_are_dropped_while_busy(drv):
    assert call(drv.on_start).success
    n = len(drv.sdk.sent)
    cmd(drv, [float("nan")] * 6)
    assert len(drv.sdk.sent) == n
    drv.busy.set()
    cmd(drv, [drv.fake_arm.zero()[j] for j in JOINTS])
    assert len(drv.sdk.sent) == n
    drv.busy.clear()


def test_gripper_setpoints_use_the_effort_and_are_sent_only_when_they_change(drv):
    assert call(drv.on_start).success                                       # the gripper moves only while the arm is started
    z = [drv.fake_arm.zero()[j] for j in JOINTS]
    cmd(drv, z, grip=0.03, effort=1.5)
    assert drv.sdk.grip.sent == [(0.03, 1.5)]
    for _ in range(5):                                                      # ros2_control repeats the same target every cycle
        cmd(drv, z, grip=0.03, effort=1.5)
    assert len(drv.sdk.grip.sent) == 1
    cmd(drv, z, grip=0.01)                                                  # no effort in the message: the default 1.0 N
    assert drv.sdk.grip.sent[-1] == (0.01, 1.0)
    cmd(drv, z, grip=0.02, effort=9.0)                                      # over the 3 N cap: refused, nothing sent
    assert len(drv.sdk.grip.sent) == 2 and drv.rejected >= 1


def test_estop_service_works_at_once_and_clear_estop_resets(drv):
    assert call(drv.on_start).success
    r = call(drv.on_estop)
    assert r.success and drv.sdk.estopped and not drv.sdk.on
    r = call(drv.on_clear_estop)
    assert r.success and not drv.sdk.estopped and drv.sdk.resets == 1
    assert call(drv.on_clear_estop).message == "no e-stop was active"


def test_estop_goes_through_while_a_start_is_running(drv):
    drv.busy.set()                                                          # a start / stop is running in another thread
    assert call(drv.on_estop).success and drv.sdk.estops == 1
    drv.busy.clear()


def test_enable_service_only_disables(drv):
    assert call(drv.on_start).success
    r = call(drv.on_enable, data=True)
    assert not r.success and "start" in r.message and drv.sdk.on
    r = call(drv.on_enable, data=False)
    assert r.success and not drv.sdk.on


def test_start_while_another_lifecycle_call_runs_is_refused(drv):
    drv.busy.set()
    r = call(drv.on_start)
    assert not r.success and "another" in r.message
    drv.busy.clear()


def test_stop_with_a_fault_commands_nothing_and_says_the_operator_estops(drv):
    assert call(drv.on_start).success
    n = len(drv.sdk.sent)
    drv.sdk.fault = True
    r = call(drv.on_stop)
    assert not r.success and "operator" in r.message and len(drv.sdk.sent) == n and drv.sdk.on
