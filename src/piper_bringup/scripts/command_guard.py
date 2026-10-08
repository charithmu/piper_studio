#!/usr/bin/env python3
"""Safety relay between ros2_control (JointStateTopicSystem) and the agx_arm_ctrl driver.

ros2_control publishes NaN commands while no controller claims the interfaces, and agx_arm_ctrl
turns NaN into 0.0 (the folded rest pose). This node forwards a command to the driver only if:
  * every value is finite,
  * every value is within the URDF position limits,
  * driver feedback is fresh, and
  * every arm joint target is within `max_step` rad of the measured position.
Rejected commands are dropped (the driver keeps its last valid target) and logged.
"""

import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class CommandGuard(Node):
    def __init__(self):
        super().__init__("command_guard")
        self.max_step = self.declare_parameter("max_step", 0.15).value  # rad per command
        self.max_feedback_age = self.declare_parameter("max_feedback_age", 0.1).value  # s
        self.arm_joints = list(self.declare_parameter(
            "arm_joints", [f"joint{i}" for i in range(1, 7)]).value)
        self.limits: dict[str, tuple[float, float]] = {}
        self.feedback: tuple[float, dict[str, float]] | None = None
        self.rejected = 0
        self.pub = self.create_publisher(JointState, "control/joint_states", 1)
        self.create_subscription(JointState, "hw/joint_commands", self.on_command, 1)
        self.create_subscription(JointState, "feedback/joint_states", self.on_feedback, 1)
        self.create_subscription(String, "robot_description", self.on_description,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_description(self, msg: String):
        root = ET.fromstring(msg.data)
        self.limits = {
            j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
            for j in root.findall("joint")
            if j.get("type") in ("revolute", "prismatic") and j.find("limit") is not None}
        self.get_logger().info(f"limits loaded for {len(self.limits)} joints")

    def on_feedback(self, msg: JointState):
        self.feedback = (time.monotonic(), dict(zip(msg.name, msg.position)))

    def reject(self, reason: str):
        self.rejected += 1
        self.get_logger().warn(f"command rejected: {reason}", throttle_duration_sec=1.0)

    def on_command(self, msg: JointState):
        if not self.limits:
            return self.reject("no robot_description yet")
        if self.feedback is None or time.monotonic() - self.feedback[0] > self.max_feedback_age:
            return self.reject("driver feedback missing or stale")
        measured = self.feedback[1]
        for name, pos in zip(msg.name, msg.position):
            if not math.isfinite(pos):
                return self.reject(f"{name} is not finite (no active controller?)")
            lo, hi = self.limits.get(name, (-math.inf, math.inf))
            if not lo - 1e-6 <= pos <= hi + 1e-6:
                return self.reject(f"{name}={pos:.4f} outside [{lo:.4f}, {hi:.4f}]")
            if name in self.arm_joints:
                if name not in measured or not math.isfinite(measured[name]):
                    return self.reject(f"no measured state for {name}")
                if abs(pos - measured[name]) > self.max_step:
                    return self.reject(
                        f"{name} step {pos - measured[name]:+.3f} rad exceeds max_step {self.max_step}")
        self.pub.publish(msg)


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
