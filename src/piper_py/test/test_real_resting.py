"""backend:=real with a Piper resting on its stops, without an arm: fake_driver starts where the real arm sits when disabled.

A real Piper rests with joints 2 and 3 about 0.04 rad past their limits (-0.038 / +0.043 rad) and joint 5 at about 0.39 rad. The guard must not refuse
that pose (activation), must clamp commands to just inside the limits, and MoveIt must plan from it to 'zero' and 'rest'. Also covers piper enable/disable.
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

RESTING = [0.004, -0.0382, 0.0427, -0.0073, 0.3859, 0.0]  # measured on the real arm, motors off


@pytest.mark.launch_test
def generate_test_description():
    bringup = os.path.join(get_package_share_directory("piper_bringup"), "launch", "piper.launch.py")
    return launch.LaunchDescription([
        launch_ros.actions.Node(package="piper_py", executable="fake_driver",
                                parameters=[{"initial_positions": RESTING + [0.0]}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup), launch_arguments={
            "backend": "real", "driver": "false", "servo": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestRealResting(unittest.TestCase):
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

    def test_1_activation_from_the_stops_is_accepted(self):
        r = self.arm.activate()
        self.assertTrue(r, r)
        time.sleep(1.0)
        q = self.arm.joint_positions()
        self.assertGreaterEqual(q[1], 0.0)   # joint 2 was pulled just inside its limit
        self.assertLessEqual(q[2], 0.0)      # joint 3 too
        self.assert_near(q, [RESTING[0], 0.01, -0.01, RESTING[3], RESTING[4], RESTING[5]], 0.01)

    def test_2_zero_then_rest_through_moveit(self):
        r = self.arm.move_named("zero", 1.0)
        self.assertTrue(r, r)
        self.assert_near(self.arm.joint_positions(), [0.0, 0.01, -0.01, 0.0, 0.0, 0.0], 0.02)
        r = self.arm.move_rest(1.0)
        self.assertTrue(r, r)
        self.assert_near(self.arm.joint_positions(), [0.0, 0.01, -0.01, 0.0, 0.2936, 0.0], 0.02)

    def test_3_disable_and_enable(self):
        before = self.arm.joint_positions()
        r = self.arm.set_motors(False)
        self.assertTrue(r, r)
        self.assertEqual(self.arm.controllers()["arm_controller"], "inactive")  # nothing commands a limp arm
        r = self.arm.set_motors(True)
        self.assertTrue(r, r)
        self.assert_near(self.arm.joint_positions(), before, 0.01)
