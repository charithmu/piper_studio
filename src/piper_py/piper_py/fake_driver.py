"""Stand-in for agx_arm_ctrl's topic interface, for testing backend:=real without an arm.

Publishes feedback/joint_states at `rate` Hz starting from `initial_positions`, and moves each
joint toward the latest control/joint_states target with a first-order lag (`time_constant`).
It also publishes the vendor driver's diagnostics (feedback/arm_status, motor_status, gripper_status at 2 Hz, same keys); the
parameter `fault` (a joint name such as "joint3", or "arm") makes the matching entry report an ERROR, so tests can prove that faults are seen.
Like the real driver, it ignores commands while `enabled` is false and turns NaN into 0.0
(agx_arm_ctrl's _safe_get_value), so tests can prove that NaN never reaches it.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import json

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

JOINTS = [f"joint{i}" for i in range(1, 7)] + ["gripper"]


class FakeDriver(Node):
    def __init__(self):
        super().__init__("fake_agx_arm_driver")
        self.declare_parameter("initial_positions", [0.3, 0.5, -0.4, 0.0, 0.2, 0.0, 0.02])
        self.declare_parameter("rate", 200.0)
        self.declare_parameter("time_constant", 0.05)
        self.declare_parameter("enabled", True)
        self.declare_parameter("fault", "")
        self.q = list(self.get_parameter("initial_positions").value)
        self.target = list(self.q)
        self.dt = 1.0 / self.get_parameter("rate").value
        self.alpha = min(1.0, self.dt / self.get_parameter("time_constant").value)
        self.pub = self.create_publisher(JointState, "feedback/joint_states", 1)
        self.create_subscription(JointState, "control/joint_states", self.on_command, 1)
        self.create_service(SetBool, "enable_agx_arm", self.on_enable)  # like the vendor driver
        self.arm_status_pub = self.create_publisher(DiagnosticArray, "feedback/arm_status", 1)
        self.motor_status_pub = self.create_publisher(DiagnosticArray, "feedback/motor_status", 10)
        self.gripper_status_pub = self.create_publisher(DiagnosticArray, "feedback/gripper_status", 1)
        self.create_timer(self.dt, self.step)
        self.create_timer(0.5, self.publish_status)

    def _status(self, name, level, message, values):
        st = DiagnosticStatus(name=name, hardware_id=f"fake:{name}", level=bytes([level]), message=message)
        st.values = [KeyValue(key=k, value=v if isinstance(v, str) else json.dumps(v)) for k, v in values.items()]
        return st

    def publish_status(self):
        fault = self.get_parameter("fault").value
        now = self.get_clock().now().to_msg()
        arm_bad = fault == "arm"
        arm = DiagnosticArray()
        arm.header.stamp = now
        arm.status = [self._status("arm", 2 if arm_bad else 0, "arm_status=7, err_status=0" if arm_bad else "arm_status=0, err_status=0", {
            "ctrl_mode": 1, "arm_status": 7 if arm_bad else 0, "mode_feedback": 1, "teach_status": 0, "motion_status": 0, "trajectory_num": 0,
            "err_status": 0, "joint_angle_limit": [False] * 6, "communication_status_joint": [False] * 6})]
        self.arm_status_pub.publish(arm)
        motors = DiagnosticArray()
        motors.header.stamp = now
        for name in JOINTS[:6]:
            bad = fault == name
            motors.status.append(self._status(name, 2 if bad else 0, "driver_overcurrent" if bad else "enabled", {
                "enabled": "true", "undervoltage": "false", "motor_overtemp": "false", "driver_overcurrent": "true" if bad else "false",
                "driver_overtemp": "false", "collision_tripped": "false", "driver_error": "false", "stall_tripped": "false",
                "bus_voltage_v": "24.0", "driver_temp_deg": "32", "motor_temp_deg": "30", "bus_current": "0.1", "status_raw": "0"}))
        self.motor_status_pub.publish(motors)
        grip = DiagnosticArray()
        grip.header.stamp = now
        grip.status = [self._status("gripper", 0, "enabled", {"driver_enable_status": True, "homing_status": True})]
        self.gripper_status_pub.publish(grip)

    def on_enable(self, request, response):
        self.set_parameters([Parameter("enabled", Parameter.Type.BOOL, bool(request.data))])
        response.success, response.message = True, "enabled" if request.data else "disabled"
        return response

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
