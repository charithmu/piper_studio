"""End-to-end check of piper_bringup + MoveIt + piper_py, identical for every backend.

Backends: PIPER_TEST_BACKENDS (comma separated, default "mock,gazebo,mujoco"). "isaac" is opt-in because it
uses the shared GPU: PIPER_TEST_BACKENDS=isaac (needs scripts/isaac.sh build-usd first).
"""

import math
import os
import time
import unittest

import launch
import launch_testing.actions
import pytest
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from piper_py import Piper

DOWN = [0.0, 1.0, 0.0, 0.0]  # tcp z axis pointing down (180 deg about base y)
# Joint 5 (+/-70 deg) limits vertical approach to TCP heights below ~0.12 m, 0.17-0.30 m forward.


BACKENDS = os.environ.get("PIPER_TEST_BACKENDS", "mock,gazebo,mujoco").split(",")


@pytest.mark.launch_test
@launch_testing.parametrize("backend", BACKENDS)
def generate_test_description(backend):
    bringup = os.path.join(get_package_share_directory("piper_bringup"), "launch", "piper.launch.py")
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup),
                                 launch_arguments={"backend": backend, "rviz": "false"}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {"backend": backend}


class TestBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.arm = Piper(wait=0)
        cls.arm.wait_ready(90.0, moveit=True)
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:  # spawners finish asynchronously
            if cls.arm.controllers().get("gripper_controller") == "active":
                break
            time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.arm.close()

    def assert_joints(self, target, tol=0.02):
        for name, (a, b) in zip(("j1", "j2", "j3", "j4", "j5", "j6"),
                                zip(self.arm.joint_positions(), target)):
            self.assertAlmostEqual(a, b, delta=tol, msg=name)

    def test_0_backend_is_running(self, backend):
        topics = dict(self.arm.node.get_topic_names_and_types())
        nodes = self.arm.node.get_node_names()
        print(f"[backend under test] {backend}")
        if backend == "gazebo":
            self.assertIn("/clock", topics)
            self.assertIn("gz_ros_control", nodes)
        elif backend == "isaac":
            self.assertIn("/clock", topics)
            self.assertIn("/isaac/joint_states", topics)
        elif backend == "mujoco":
            self.assertIn("/clock", topics)
            self.assertNotIn("gz_ros_control", nodes)
        else:
            self.assertNotIn("/clock", topics)

    def test_1_direct_trajectory(self):
        target = [0.2, 0.8, -0.8, 0.1, 0.5, -0.2]
        r = self.arm.move_joints(target, duration=1.0)
        self.assertTrue(r, r)
        self.assert_joints(target)

    def test_2_out_of_limits_is_rejected_without_moving(self):
        before = self.arm.joint_positions()
        r = self.arm.move_joints([0, 0.5, 0.5, 0, 0, 0], duration=1.0)  # joint3 limit is [-2.967, 0]
        self.assertFalse(r)
        self.assertEqual(r.code, "OUT_OF_LIMITS")
        self.assert_joints(before, tol=1e-6)

    def test_3_named_poses(self):
        self.assertIn("ready", self.arm.named_poses())
        r = self.arm.move_named("ready", velocity_scaling=1.0)
        self.assertTrue(r, r)
        q = self.arm.named_poses()["ready"]
        self.assert_joints([q[f"joint{i}"] for i in range(1, 7)])

    def test_4_pose_goal(self):
        r = self.arm.move_pose([0.25, 0.0, 0.10], DOWN, velocity_scaling=1.0)
        self.assertTrue(r, r)

    def test_5_linear_pose_goal(self):
        self.assertTrue(self.arm.move_pose([0.25, 0.0, 0.10], DOWN, velocity_scaling=1.0))
        r = self.arm.move_pose([0.25, 0.0, 0.06], DOWN, linear=True, velocity_scaling=0.5)
        self.assertTrue(r, r)

    def test_6_unreachable_pose_fails(self):
        r = self.arm.move_pose([2.0, 0.0, 0.2], DOWN)
        self.assertFalse(r)
        self.assertNotEqual(r.code, "SUCCESS")

    def test_7_gripper(self):
        # Close fully onto the limit and reopen: a simulator joint must not stick at its hard stop.
        for width in (0.06, 0.0, 0.05, 0.0):
            r = self.arm.gripper(width)
            self.assertTrue(r, r)
            self.assertTrue(math.isclose(self.arm.joint_state()["gripper"], width, abs_tol=0.003))
