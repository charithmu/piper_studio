"""backend:=real wiring without an arm: ros2_control <-> topic driver interface, with fake_driver.

Checks the activation safety rule: the arm must hold its *measured* pose when controllers are
activated, never jump to the URDF initial positions.
"""

import os
import subprocess
import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from piper_py import Piper

START = [0.3, 0.5, -0.4, 0.0, 0.2, 0.0]  # deliberately not the URDF initial pose


@pytest.mark.launch_test
def generate_test_description():
    bringup = os.path.join(get_package_share_directory("piper_sim"), "launch", "piper.launch.py")
    return launch.LaunchDescription([
        launch_ros.actions.Node(package="piper_py", executable="fake_driver",
                                parameters=[{"initial_positions": START + [0.02]}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup), launch_arguments={
            "backend": "real", "driver": "false", "moveit": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def set_fault(value: str, tries: int = 8):
    """`ros2 param set` on the fake driver. The CLI finds nodes through the ros2 daemon, which may still be starting (a cold daemon made the
    first call fail with `Node not found`, the flaky plumbing failure seen on 2026-11-08), so retry."""
    for _ in range(tries):
        r = subprocess.run(["ros2", "param", "set", "--spin-time", "3", "/fake_agx_arm_driver", "fault", value], capture_output=True)
        if r.returncode == 0:
            return
        time.sleep(0.5)
    raise AssertionError(f"could not set the fake driver's fault parameter: {r.stderr.decode()[-200:]}")


class TestRealPlumbing(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = Piper(wait=0)
        cls.arm.wait_ready(60.0)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:  # spawners finish asynchronously
            if len(cls.arm.controllers()) == 4:
                break
            time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.arm.close()

    def assert_near(self, a, b, tol):
        for i, (x, y) in enumerate(zip(a, b)):
            self.assertAlmostEqual(x, y, delta=tol, msg=f"joint{i + 1}")

    def wait_near(self, target, tol, timeout=10.0):
        """Poll until the measured joints are within `tol` of `target` (a motion needs time that depends on the machine), then assert it. Waiting is for the arrival only:
        the checks that something must NOT move stay strict (`wait_settled` first, then compare)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(abs(x - y) <= tol for x, y in zip(self.arm.joint_positions(), target)):
                break
            time.sleep(0.1)
        self.assert_near(self.arm.joint_positions(), target, tol)

    def wait_settled(self, timeout=10.0, window=0.5, eps=1e-4):
        """Poll until no joint has moved more than `eps` over `window` seconds; returns the settled joints."""
        deadline, prev, t_prev = time.monotonic() + timeout, self.arm.joint_positions(), time.monotonic()
        while time.monotonic() < deadline:
            time.sleep(0.1)
            if time.monotonic() - t_prev >= window:
                cur = self.arm.joint_positions()
                if max(abs(x - y) for x, y in zip(cur, prev)) < eps:
                    return cur
                prev, t_prev = cur, time.monotonic()
        return self.arm.joint_positions()

    def test_1_commanders_start_inactive(self):
        states = self.arm.controllers()
        self.assertEqual(states["joint_state_broadcaster"], "active")
        for c in ("arm_controller", "gripper_controller", "arm_position_controller"):
            self.assertEqual(states[c], "inactive", c)

    def test_2_state_is_measured_feedback(self):
        time.sleep(0.5)
        self.assert_near(self.arm.joint_positions(), START, 1e-3)

    def test_2a_velocity_and_effort_reach_the_client(self):
        """The vendor driver's per-joint velocity and torque estimate pass through ros2_control to the client."""
        eff = self.arm.joint_efforts()
        self.assertAlmostEqual(eff[self.arm.arm_joints[0]], 0.1, places=6)
        self.assertAlmostEqual(eff[self.arm.arm_joints[5]], 0.6, places=6)
        self.assertIn(self.arm.arm_joints[0], self.arm.joint_velocities())

    def test_2b_driver_status_is_read_and_a_fault_blocks_activation(self):
        """The vendor driver's diagnostics (arm, motors, gripper) are readable, and a reported fault refuses activation."""
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not (self.arm.status()["arm"] and len(self.arm.status()["motors"]) == 6):
            time.sleep(0.2)
        st = self.arm.status()
        self.assertEqual(st["arm"]["level"], "OK")
        self.assertEqual(st["arm"]["arm_status_name"], "normal")
        self.assertEqual([m["name"] for m in st["motors"]], [f"joint{i}" for i in range(1, 7)])
        self.assertEqual(self.arm.faults(), [])
        set_fault("joint3")
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and not self.arm.faults():
            time.sleep(0.2)
        self.assertEqual(self.arm.faults(), ["joint3: driver_overcurrent"])
        r = self.arm.activate()
        self.assertFalse(r)
        self.assertEqual(r.code, "ARM_FAULT")
        self.assertEqual(self.arm.controllers()["arm_controller"], "inactive")
        set_fault("")
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline and self.arm.faults():
            time.sleep(0.2)
        self.assertEqual(self.arm.faults(), [])

    def test_2c_a_driver_fault_stops_commands_at_the_guard(self):
        """With a reported driver fault nothing reaches the driver (the guard drops it), and commands flow again when the fault clears."""
        self.assertTrue(self.arm.activate())
        before = self.wait_settled()
        target = [q + (0.05 if i == 0 else 0.0) for i, q in enumerate(before)]
        set_fault("joint3")
        time.sleep(1.5)
        self.arm.move_joints(target, 1.0)                                   # result ignored: the goal cannot be met
        self.assert_near(self.arm.joint_positions(), before, 2e-3)           # the guard dropped the commands
        set_fault("")
        time.sleep(1.5)
        r = self.arm.move_joints(target, 1.0)
        self.assertTrue(r, r)                                               # commands flow again
        self.wait_near(target, 5e-3)
        r = self.arm.move_joints(before, 1.0)
        self.assertTrue(r, r)
        self.wait_near(before, 5e-3)
        self.arm.deactivate()

    def test_3_activation_holds_measured_pose(self):
        measured = self.wait_settled()
        r = self.arm.activate()
        self.assertTrue(r, r)
        time.sleep(1.0)
        self.assert_near(self.arm.joint_positions(), measured, 1e-3)         # holds the MEASURED pose; never jumps to the URDF initial pose

    def test_4_trajectory_reaches_target_on_driver(self):
        target = [0.0, 0.8, -0.8, 0.1, 0.3, -0.2]
        r = self.arm.move_joints(target, duration=1.5)
        self.assertTrue(r, r)
        self.wait_near(target, 0.02)

    def test_5_gripper(self):
        r = self.arm.gripper(0.05)
        self.assertTrue(r, r)

    def test_6_streaming_handover(self):
        self.assertTrue(self.arm.use_streaming())
        states = self.arm.controllers()
        self.assertEqual((states["arm_controller"], states["arm_position_controller"]), ("inactive", "active"))
        q = self.arm.joint_positions()
        for _ in range(100):  # 1 s at 100 Hz, joint1 +0.2 rad
            q[0] += 0.002
            self.assertTrue(self.arm.stream_joints(q))
            time.sleep(0.01)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and abs(self.arm.joint_positions()[0] - q[0]) > 0.01:      # the stream and the controller need a moment to catch up
            time.sleep(0.1)
        self.assertAlmostEqual(self.arm.joint_positions()[0], q[0], delta=0.01)
        self.assertFalse(self.arm.stream_joints([0, 0, 1.0, 0, 0, 0]))  # joint3 out of limits
        self.assertTrue(self.arm.use_trajectories())

    def test_7_deactivate(self):
        self.assertTrue(self.arm.deactivate())
        states = self.arm.controllers()
        self.assertTrue(all(states[c] == "inactive"
                            for c in ("arm_controller", "gripper_controller", "arm_position_controller")))

    def test_8_no_motion_while_inactive(self):
        """Inactive controllers make ros2_control publish NaN; the guard must keep it from the driver."""
        before = self.wait_settled()                     # let the previous test's motion finish; from here on NOTHING may move it
        time.sleep(1.5)
        self.assert_near(self.arm.joint_positions(), before, 1e-4)
