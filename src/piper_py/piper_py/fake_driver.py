"""Stand-in for agx_arm_ctrl's topic interface, for testing backend:=real without an arm.

Publishes feedback/joint_states at `rate` Hz starting from `initial_positions`, and moves each
joint toward the latest control/joint_states target with a first-order lag (`time_constant`).
Like the real driver, it ignores commands while `enabled` is false and turns NaN into 0.0
(agx_arm_ctrl's _safe_get_value), so tests can prove that NaN never reaches it.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"joint{i}" for i in range(1, 7)] + ["gripper"]


class FakeDriver(Node):
    def __init__(self):
        super().__init__("fake_agx_arm_driver")
        self.declare_parameter("initial_positions", [0.3, 0.5, -0.4, 0.0, 0.2, 0.0, 0.02])
        self.declare_parameter("rate", 200.0)
        self.declare_parameter("time_constant", 0.05)
        self.declare_parameter("enabled", True)
        self.q = list(self.get_parameter("initial_positions").value)
        self.target = list(self.q)
        self.dt = 1.0 / self.get_parameter("rate").value
        self.alpha = min(1.0, self.dt / self.get_parameter("time_constant").value)
        self.pub = self.create_publisher(JointState, "feedback/joint_states", 1)
        self.create_subscription(JointState, "control/joint_states", self.on_command, 1)
        self.create_timer(self.dt, self.step)

    def on_command(self, msg: JointState):
        if not self.get_parameter("enabled").value:
            return
        for name, pos in zip(msg.name, msg.position):
            if name in JOINTS:
                self.target[JOINTS.index(name)] = 0.0 if math.isnan(pos) else pos

    def step(self):
        prev = list(self.q)
        self.q = [q + self.alpha * (t - q) for q, t in zip(self.q, self.target)]
        msg = JointState(name=JOINTS, position=self.q,
                         velocity=[(q - p) / self.dt for q, p in zip(self.q, prev)])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
