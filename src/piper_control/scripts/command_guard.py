#!/usr/bin/env python3
"""Safety relay between ros2_control (JointStateTopicSystem) and the agx_arm_ctrl driver.

ros2_control publishes NaN commands while no controller claims the interfaces, and agx_arm_ctrl
turns NaN into 0.0 (the vendor's exact-zero pose). This node forwards a command to the driver only if:
  * every value is finite,
  * every value is within the URDF position limits, never outside (arm joints: a value closer than `limit_margin` to a limit is
    clamped to the margin; a powered-off arm's slumped pose is outside the limits, so nothing can be commanded from it),
  * driver feedback is fresh, and
  * every arm joint target is within `max_step` rad of the measured position.
Rejected commands are dropped (the driver keeps its last valid target) and logged.

The one exception is the service `go_zero` (std_srvs/Trigger): the start sequence. It is the only way out of the slumped pose. It accepts
only a slumped or already-zero arm (piper_core.classify_state), sends the zero pose to the driver once (the driver moves with its own speed
setting), waits until the measured state is at zero and stays there, and answers success only then. Controllers must be inactive.

Names: ros2_control and the URDF use prefixed joint names (`prefix`, default piper_), the AgileX driver uses the official unprefixed
ones. This node is the one place that translates: driver feedback (feedback/joint_states, official names) is republished prefixed on
hw/joint_states for the hardware interface, and accepted commands (hw/joint_commands, prefixed) are sent to the driver
(control/joint_states) with the prefix removed.
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from piper_core import GuardConfig, check_command, classify_state, limits_from_urdf, zero_pose


class CommandGuard(Node):
    def __init__(self):
        super().__init__("command_guard")
        self.max_step = self.declare_parameter("max_step", 0.15).value  # rad per command
        self.max_feedback_age = self.declare_parameter("max_feedback_age", 0.1).value  # s
        self.limit_margin = self.declare_parameter("limit_margin", 0.01).value  # rad kept inside the URDF limits (arm joints)
        self.zero_tolerance = self.declare_parameter("zero_tolerance", 0.01).value  # rad: "at zero" for go_zero
        self.go_zero_timeout = self.declare_parameter("go_zero_timeout", 30.0).value  # s
        self.go_zero_hold = self.declare_parameter("go_zero_hold", 1.0).value  # s the arm must stay at zero
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
        self.create_service(Trigger, "go_zero", self.on_go_zero, callback_group=ReentrantCallbackGroup())
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
        return GuardConfig(tuple(self.arm_joints), self.max_step, self.limit_margin)

    def _fresh_feedback(self, max_age=None):
        if self.feedback is None or time.monotonic() - self.feedback[0] > (max_age or self.max_feedback_age):
            return None
        return self.feedback[1]

    def on_go_zero(self, request, response):
        """Leave the slumped pose (or confirm zero): one move to the zero pose, then check it was reached and held."""
        def fail(message):
            self.get_logger().error(f"go_zero: {message}")
            response.success, response.message = False, message
            return response

        if not self.limits:
            return fail("no robot_description yet")
        q = self._fresh_feedback(max_age=1.0)
        if q is None:
            return fail("driver feedback missing or stale")
        state = classify_state(q, self.limits, self.arm_joints, self.limit_margin, self.zero_tolerance)
        if state not in ("slumped", "zero"):
            return fail(f"the arm is {state}, not slumped or at zero: go_zero only leaves the powered-off pose; "
                        "plan to zero with the controllers")
        zero = zero_pose(self.limits, self.arm_joints, self.limit_margin)
        msg = JointState(name=[n.removeprefix(self.prefix) for n in self.arm_joints], position=[zero[n] for n in self.arm_joints])
        deadline, at_zero_since = time.monotonic() + self.go_zero_timeout, None
        while time.monotonic() < deadline:
            self.pub.publish(msg)    # the driver keeps its last target; resending is harmless and survives a lost message
            time.sleep(0.1)
            q = self._fresh_feedback(max_age=1.0)     # looser than for commands: this loop polls at 10 Hz, the check must not race it
            if q is None:
                return fail("driver feedback lost during the move")
            if classify_state(q, self.limits, self.arm_joints, self.limit_margin, self.zero_tolerance) == "zero":
                at_zero_since = at_zero_since or time.monotonic()
                if time.monotonic() - at_zero_since >= self.go_zero_hold:
                    response.success, response.message = True, "at zero"
                    return response
            else:
                at_zero_since = None
        return fail(f"zero not reached within {self.go_zero_timeout:.0f} s (state: {q})")


def main():
    rclpy.init()
    node = CommandGuard()
    executor = MultiThreadedExecutor()    # go_zero waits for feedback, which arrives on another thread
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
