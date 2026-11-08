"""backend:=real without an arm: the start and stop sequences, from and to the slumped (powered-off) pose.

fake_driver starts where the real arm sits when powered off (joints 2 and 3 about 0.04 rad outside their limits, joint 5 at 0.39 rad) and sags
back into that pose whenever its motors are disabled. Nothing may be commanded from that pose; start() leaves it with one checked move to
zero (command_guard go_zero); stop() goes to zero, parks, and powers off, which puts the arm back into the slumped pose.
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

SLUMPED = [0.004, -0.0382, 0.0427, -0.0073, 0.3859, 0.0]  # the pose a powered-off real arm sags into, measured 2026-10-14


@pytest.mark.launch_test
def generate_test_description():
    bringup = os.path.join(get_package_share_directory("piper_sim"), "launch", "piper.launch.py")
    return launch.LaunchDescription([
        launch_ros.actions.Node(package="piper_py", executable="fake_driver",
                                parameters=[{"initial_positions": SLUMPED + [0.0], "slump_positions": SLUMPED + [0.0], "time_constant": 0.01}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup), launch_arguments={
            "backend": "real", "driver": "false", "servo": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestRealStartStop(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = Piper(wait=0)
        cls.arm.wait_ready(90.0, moveit=True)
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            if len(cls.arm.controllers()) == 4:
                break
            time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.arm.close()

    def assert_near(self, a, b, tol):
        for i, (x, y) in enumerate(zip(a, b)):
            self.assertAlmostEqual(x, y, delta=tol, msg=f"joint{i + 1}")

    def test_1_nothing_is_commanded_from_the_slumped_pose(self):
        self.assertTrue(self.arm._outside_limits())
        deadline = time.monotonic() + 15
        r = self.arm.activate()
        while r.code == "STALE_FEEDBACK" and time.monotonic() < deadline:     # discovery of the driver's feedback can take a moment
            time.sleep(0.3)
            r = self.arm.activate()
        self.assertFalse(r)
        self.assertEqual(r.code, "NOT_STARTED", r)
        self.assertNotIn("park", self.arm.named_poses())      # internal poses are not offered downstream

    def test_2_start_leaves_the_slumped_pose_for_zero(self):
        r = self.arm.start(1.0)
        self.assertTrue(r, r)
        self.assert_near(self.arm.joint_positions(), [0.0, 0.01, -0.01, 0.0, 0.0, 0.0], 0.02)
        self.assertFalse(self.arm._outside_limits())
        self.assertEqual(self.arm.controllers()["arm_controller"], "active")

    def test_3_move_anywhere_valid_and_back_to_zero(self):
        r = self.arm.move_to_joints([0.2, 0.3, -0.3, 0.0, 0.3, 0.0], 0.3)
        self.assertTrue(r, r)
        r = self.arm.move_named("zero", 0.3)
        self.assertTrue(r, r)

    def test_4_stop_parks_and_powers_off_into_the_slump(self):
        r = self.arm.stop(0.3, settle=0.5)
        self.assertTrue(r, r)
        self.assertEqual(r.message, "parked, motors off")
        self.assertEqual(self.arm.controllers()["arm_controller"], "inactive")   # nothing commands a limp arm
        time.sleep(0.5)
        self.assertTrue(self.arm._outside_limits())                               # the arm sagged into the slumped pose
        self.assertTrue(self.arm.stop(1.0))                                       # a second stop leaves it alone

    def test_5_the_cycle_repeats(self):
        r = self.arm.start(1.0)
        self.assertTrue(r, r)
        self.assert_near(self.arm.joint_positions(), [0.0, 0.01, -0.01, 0.0, 0.0, 0.0], 0.02)
