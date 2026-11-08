"""The carrier example works: the Piper, mounted with a non-default prefix on a carrier that owns its controller manager, is driven
through piper_py and ends up where the mount transform says (proves docs/INTEGRATION.md)."""

import math
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from piper_py import Piper


@pytest.mark.launch_test
def generate_test_description():
    demo = f"{get_package_share_directory('piper_examples')}/launch/carrier_demo.launch.py"
    return launch.LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(demo)), launch_testing.actions.ReadyToTest()])


class TestCarrier(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = Piper(prefix="arm_", wait=0)
        cls.arm.wait_ready(90.0, moveit=True)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:  # spawners finish asynchronously
            if cls.arm.controllers().get("gripper_controller") == "active":
                break
            time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.arm.close()

    def test_names_carry_the_prefix(self):
        self.assertTrue(all(j.startswith("arm_") for j in self.arm.joint_state()))

    def test_moves_through_the_control_stack(self):
        self.assertTrue(self.arm.move_named("ready", velocity_scaling=0.6))
        self.assertTrue(self.arm.move_pose([0.25, 0.0, 0.10], [0, 1, 0, 0]))

    def test_tcp_is_where_the_mount_puts_it(self):
        # arm base is at (0.12, 0, 0.08) in world, turned 90 deg about z: the arm's x (forward) is world +y, its y is world -x
        self.assertTrue(self.arm.move_pose([0.25, 0.0, 0.10], [0, 1, 0, 0]))
        (x, y, z), _ = self.arm.tcp_pose(frame="world")
        self.assertAlmostEqual(x, 0.12, delta=0.01)
        self.assertAlmostEqual(y, 0.25, delta=0.01)
        self.assertAlmostEqual(z, 0.08 + 0.10, delta=0.01)
