"""backend:=real wiring without an arm: ros2_control <-> topic driver interface, with fake_driver.

Checks the activation safety rule: the arm must hold its *measured* pose when controllers are
activated, never jump to the URDF initial positions.
"""

import os
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
    bringup = os.path.join(get_package_share_directory("piper_bringup"), "launch", "piper.launch.py")
    return launch.LaunchDescription([
        launch_ros.actions.Node(package="piper_py", executable="fake_driver",
                                parameters=[{"initial_positions": START + [0.02]}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup), launch_arguments={
            "backend": "real", "driver": "false", "moveit": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


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

    def test_1_commanders_start_inactive(self):
        states = self.arm.controllers()
        self.assertEqual(states["joint_state_broadcaster"], "active")
        for c in ("arm_controller", "gripper_controller", "arm_position_controller"):
            self.assertEqual(states[c], "inactive", c)

    def test_2_state_is_measured_feedback(self):
        time.sleep(0.5)
        self.assert_near(self.arm.joint_positions(), START, 1e-3)

    def test_3_activation_holds_measured_pose(self):
        r = self.arm.activate()
        self.assertTrue(r, r)
        time.sleep(1.0)
        self.assert_near(self.arm.joint_positions(), START, 1e-3)

    def test_4_trajectory_reaches_target_on_driver(self):
        target = [0.0, 0.8, -0.8, 0.1, 0.3, -0.2]
        r = self.arm.move_joints(target, duration=1.5)
        self.assertTrue(r, r)
        self.assert_near(self.arm.joint_positions(), target, 0.02)

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
        time.sleep(0.5)
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
        before = self.arm.joint_positions()
        time.sleep(1.5)
        self.assert_near(self.arm.joint_positions(), before, 1e-4)
