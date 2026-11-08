#!/usr/bin/env python3
"""Safety relay between ros2_control (JointStateTopicSystem) and the agx_arm_ctrl driver.

ros2_control publishes NaN commands while no controller claims the interfaces, and agx_arm_ctrl
turns NaN into 0.0 (the folded rest pose). This node forwards a command to the driver only if:
  * every value is finite,
  * every value is within the URDF position limits (arm joints: kept `limit_margin` inside them; a value up to `limit_slack`
    outside is clamped to that range instead of rejected, because a real Piper rests with joints 2 and 3 about 0.04 rad past
    their limits, so "hold the measured pose" and "go to zero" would otherwise be refused or sit exactly on the arm's stored limit),
  * driver feedback is fresh, and
  * every arm joint target is within `max_step` rad of the measured position.
Rejected commands are dropped (the driver keeps its last valid target) and logged.

Names: ros2_control and the URDF use prefixed joint names (`prefix`, default piper_), the AgileX driver uses the official unprefixed
ones. This node is the one place that translates: driver feedback (feedback/joint_states, official names) is republished prefixed on
hw/joint_states for the hardware interface, and accepted commands (hw/joint_commands, prefixed) are sent to the driver
(control/joint_states) with the prefix removed.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from piper_core import GuardConfig, check_command, limits_from_urdf


class CommandGuard(Node):
    def __init__(self):
        super().__init__("command_guard")
        self.max_step = self.declare_parameter("max_step", 0.15).value  # rad per command
        self.max_feedback_age = self.declare_parameter("max_feedback_age", 0.1).value  # s
        self.limit_margin = self.declare_parameter("limit_margin", 0.01).value  # rad kept inside the URDF limits (arm joints)
        self.limit_slack = self.declare_parameter("limit_slack", 0.1).value  # rad outside the limits that is clamped, not rejected
        self.prefix = self.declare_parameter("prefix", "piper_").value
        self.arm_joints = list(self.declare_parameter(
            "arm_joints", [f"{self.prefix}joint{i}" for i in range(1, 7)]).value)
        self.limits: dict[str, tuple[float, float]] = {}
        self.feedback: tuple[float, dict[str, float]] | None = None
        self.rejected = 0
        self.pub = self.create_publisher(JointState, "control/joint_states", 1)
        self.feedback_pub = self.create_publisher(JointState, "hw/joint_states", 1)
        self.create_subscription(JointState, "hw/joint_commands", self.on_command, 1)
        self.create_subscription(JointState, "feedback/joint_states", self.on_feedback, 1)
        self.create_subscription(String, "robot_description", self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_description(self, msg: String):
        self.limits = limits_from_urdf(msg.data)
        self.get_logger().info(f"limits loaded for {len(self.limits)} joints")

    def on_feedback(self, msg: JointState):
        """Driver feedback uses the official joint names; everything downstream uses the prefixed ones."""
        msg.name = [self.prefix + n for n in msg.name]
        self.feedback = (time.monotonic(), dict(zip(msg.name, msg.position)))
        self.feedback_pub.publish(msg)

    def reject(self, reason: str):
        self.rejected += 1
        self.get_logger().warn(f"command rejected: {reason}", throttle_duration_sec=1.0)

    def on_command(self, msg: JointState):
        if not self.limits:
            return self.reject("no robot_description yet")
        if self.feedback is None or time.monotonic() - self.feedback[0] > self.max_feedback_age:
            return self.reject("driver feedback missing or stale")
        verdict = check_command(msg.name, msg.position, self.limits, self.feedback[1], self.cfg())  # piper_core: the one rule set
        if not verdict.ok:
            return self.reject(verdict.reason)
        msg.position = verdict.positions
        msg.name = [n.removeprefix(self.prefix) for n in msg.name]  # the driver speaks the official names
        self.pub.publish(msg)

    def cfg(self) -> GuardConfig:
        return GuardConfig(tuple(self.arm_joints), self.max_step, self.limit_margin, self.limit_slack)


def main():
    rclpy.init()
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
