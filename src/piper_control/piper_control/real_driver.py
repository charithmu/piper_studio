"""ROS driver for the real Piper, built on piper_core.RealArm. Replaces the vendor node (agx_arm_ctrl) for backend:=real.

Why our own: one owner of the SDK and the CAN bus (bus lock), the same start / stop as RealArm (clear an e-stop, motors on, one checked move to zero; zero, park, e-stop, clear), the electronic
e-stop and its clear, stale-feedback detection and the gripper stall logic, all verified on the arm through RealArm and `piper-arm`. The vendor node stays in external/ unmodified and unused.

Same topic contract as the vendor node, so command_guard, ros2_control (topic-based hardware), the controllers and piper_py need no change:
  publishes  feedback/joint_states      JointState, official names joint1..6 + gripper (width, m); velocity and effort (motor torque estimate, N.m)      pub_rate Hz
             feedback/arm_status        DiagnosticArray, one entry `arm` (ctrl_mode, arm_status, mode_feedback, teach_status, motion_status, trajectory_num, err_status, ...)  2 Hz
             feedback/motor_status      DiagnosticArray, one entry per joint (enabled, bus_voltage_v, temperatures, fault flags)                                          2 Hz
             feedback/gripper_status    DiagnosticArray, entry `gripper` (width, force, driver flags)                                                                       2 Hz
  subscribes control/joint_states       JointState, official names: the streamed setpoints (already through command_guard). Arm joints -> RealArm.command (guard, started, fresh feedback);
                                        `gripper` width (+ effort in N) -> RealArm.gripper_command, only when it changes
  services   start (Trigger)            clear an e-stop, motors on, one checked move to zero. Blocks until done. Controllers must be inactive.
             stop (Trigger)             from any state: zero, park, e-stop, clear (nothing commanded if a fault is reported). Controllers must be inactive.
             estop (Trigger)            the electronic e-stop, immediately, whatever else is running
             clear_estop (Trigger)      reset after an e-stop (motors stay off, the arm stays powered)
             enable_agx_arm (SetBool)   false: plain motor-off (the arm sags: only from park or with the arm held); true is refused: use `start`
Parameters: canbus (can0), speed_percent (20: the speed of `start` and of streamed setpoints), gripper (true), pub_rate (200), diag_rate (2).
Starting the node moves nothing and enables nothing. If the node dies the arm keeps holding its last target (the operator e-stops: `piper estop`, `piper-arm` key e, tools/real_arm/arm_stop.py).
"""

from __future__ import annotations

import json
import math
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from piper_core.real_arm import JOINTS, RealArm, RealArmError

FAULT_FLAGS = (("undervoltage", "voltage_too_low"), ("motor_overtemp", "motor_overheating"), ("driver_overcurrent", "driver_overcurrent"),
               ("driver_overtemp", "driver_overheating"), ("collision_tripped", "collision_status"), ("driver_error", "driver_error_status"),
               ("stall_tripped", "stall_status"))
ARM_STATUS_NAMES = {0: "normal", 1: "emergency stop", 2: "no solution", 3: "singularity", 4: "target angle out of range", 5: "joint communication", 6: "brake not released",
                    7: "collision", 8: "teach drag overspeed", 9: "joint status", 10: "other", 14: "overtemperature", 15: "overtemperature"}
ERROR_ARM_STATUS = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 14, 15)


def kv(values: dict) -> list:
    return [KeyValue(key=k, value=v if isinstance(v, str) else json.dumps(v)) for k, v in values.items()]


class RealDriver(Node):
    def __init__(self, arm: RealArm | None = None):
        super().__init__("piper_real_driver")
        self.canbus = self.declare_parameter("canbus", "can0").value
        self.speed_percent = int(self.declare_parameter("speed_percent", 20).value)
        use_gripper = bool(self.declare_parameter("gripper", True).value)
        pub_rate = float(self.declare_parameter("pub_rate", 200.0).value)
        diag_rate = float(self.declare_parameter("diag_rate", 2.0).value)
        self.arm = arm or RealArm.connect(self.canbus, gripper=use_gripper, speed_percent=self.speed_percent)
        self.busy = threading.Event()                    # a start / stop is running: streamed setpoints are dropped meanwhile
        self._last_grip: tuple[float, float] | None = None
        self.rejected = 0
        self.pub = self.create_publisher(JointState, "feedback/joint_states", 1)
        self.arm_status_pub = self.create_publisher(DiagnosticArray, "feedback/arm_status", 1)
        self.motor_status_pub = self.create_publisher(DiagnosticArray, "feedback/motor_status", 10)
        self.gripper_status_pub = self.create_publisher(DiagnosticArray, "feedback/gripper_status", 1)
        io = MutuallyExclusiveCallbackGroup()
        self.create_subscription(JointState, "control/joint_states", self.on_command, 1, callback_group=io)
        self.create_timer(1.0 / pub_rate, self.publish_feedback, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(1.0 / diag_rate, self.publish_status, callback_group=MutuallyExclusiveCallbackGroup())
        services = ReentrantCallbackGroup()               # start / stop block for seconds; estop must still get through
        for name, cb in (("start", self.on_start), ("stop", self.on_stop), ("estop", self.on_estop), ("clear_estop", self.on_clear_estop)):
            self.create_service(Trigger, name, cb, callback_group=services)
        self.create_service(SetBool, "enable_agx_arm", self.on_enable, callback_group=services)
        self.get_logger().info(f"piper_real_driver on {self.canbus} (speed {self.speed_percent} %); nothing is moved or enabled until the start service is called")

    # ------------------------------------------------------------------ feedback
    def publish_feedback(self):
        pos = self.arm.positions()
        if pos is None:
            return                                       # no feedback: command_guard sees the silence and blocks commands
        vel, eff = [], []
        for i in range(1, 7):
            ms = self.arm.sdk.get_motor_states(i)
            vel.append(float(ms.msg.velocity) if ms is not None else math.nan)
            eff.append(float(ms.msg.torque) if ms is not None else math.nan)
        names, position = list(JOINTS), [pos[j] for j in JOINTS]
        w = self.arm.gripper_opening()
        if w is not None:
            names.append("gripper")
            position.append(w)
            vel.append(0.0)
            eff.append(self.arm.gripper_force() or 0.0)
        msg = JointState(name=names, position=position, velocity=vel, effort=eff)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def _status(self, name: str, level: int, message: str, values: dict) -> DiagnosticStatus:
        return DiagnosticStatus(name=name, hardware_id=f"piper:{self.canbus}", level=bytes([level]), message=message, values=kv(values))

    def publish_status(self):
        now = self.get_clock().now().to_msg()
        sdk = self.arm.sdk
        st = sdk.get_arm_status()
        problems = self.arm.problems()
        a = st.msg if st is not None else None
        code = int(a.arm_status) if a is not None else -1
        level = 2 if (problems or code in ERROR_ARM_STATUS) else 0
        message = "; ".join(problems) if problems else f"arm_status={code} ({ARM_STATUS_NAMES.get(code, code)})"
        arm = DiagnosticArray()
        arm.header.stamp = now
        arm.status = [self._status("arm", level, message, {
            "ctrl_mode": int(a.ctrl_mode) if a else -1, "arm_status": code, "mode_feedback": int(a.mode_feedback) if a else -1,
            "teach_status": int(a.teach_status) if a else -1, "motion_status": int(a.motion_status) if a else -1,
            "trajectory_num": int(a.trajectory_num) if a else -1, "err_status": int(a.err_code) if a else -1,
            "joint_angle_limit": [False] * 6, "communication_status_joint": [False] * 6, "feedback_age_s": round(self.arm.feedback_age(), 3)})]
        self.arm_status_pub.publish(arm)
        motors = DiagnosticArray()
        motors.header.stamp = now
        for i, name in enumerate(JOINTS, 1):
            d = sdk.get_driver_states(i)
            flags = {k: bool(getattr(d.msg.foc_status, attr, False)) for k, attr in FAULT_FLAGS} if d is not None else {}
            bad = [k for k, v in flags.items() if v]
            motors.status.append(self._status(name, 2 if bad else 0, ", ".join(bad) if bad else "enabled" if sdk.get_joint_enable_status(i) else "disabled", {
                "enabled": "true" if sdk.get_joint_enable_status(i) else "false", **{k: "true" if v else "false" for k, v in flags.items()},
                "bus_voltage_v": str(getattr(d.msg, "vol", "")) if d else "", "driver_temp_deg": str(getattr(d.msg, "foc_temp", "")) if d else "",
                "motor_temp_deg": str(getattr(d.msg, "motor_temp", "")) if d else "", "bus_current": str(getattr(d.msg, "bus_current", "")) if d else "", "status_raw": "0"}))
        self.motor_status_pub.publish(motors)
        grip = DiagnosticArray()
        grip.header.stamp = now
        w, f = self.arm.gripper_opening(), self.arm.gripper_force()
        grip.status = [self._status("gripper", 0 if w is not None else 1, "ok" if w is not None else "no gripper feedback",
                                    {"width_m": w if w is not None else "", "force_n": f if f is not None else "", "driver_enable_status": w is not None, "homing_status": True})]
        self.gripper_status_pub.publish(grip)

    # ------------------------------------------------------------------ setpoints
    def on_command(self, msg: JointState):
        """Streamed setpoints (already through command_guard). Anything refused is dropped and logged; the arm keeps its last target."""
        if self.busy.is_set():
            return
        by_name = {n: i for i, n in enumerate(msg.name)}
        if all(j in by_name for j in JOINTS):
            q = [msg.position[by_name[j]] for j in JOINTS]
            if all(math.isfinite(x) for x in q):
                try:
                    self.arm.command(q)
                except RealArmError as e:
                    self._reject(str(e))
        if "gripper" in by_name and self.arm.gripper is not None:      # independent of the arm part: the gripper works whether or not the arm is started
            width = msg.position[by_name["gripper"]]
            effort = msg.effort[by_name["gripper"]] if len(msg.effort) > by_name["gripper"] else 0.0
            effort = effort if math.isfinite(effort) and effort > 0.0 else self.arm.gripper_effort
            if math.isfinite(width) and (self._last_grip is None or abs(width - self._last_grip[0]) > 5e-4 or effort != self._last_grip[1]):
                out = self.arm.gripper_command(width, effort)
                if out:
                    self._last_grip = (width, effort)
                else:
                    self._reject(out.message)

    def _reject(self, why: str):
        self.rejected += 1
        self.get_logger().warn(f"command refused: {why}", throttle_duration_sec=1.0)

    # ------------------------------------------------------------------ services
    def _run(self, op, response, label: str):
        if self.busy.is_set():
            response.success, response.message = False, "another start / stop is running"
            return response
        self.busy.set()
        try:
            out = op()
            response.success, response.message = bool(out.ok), out.message
        except Exception as e:                           # the SDK or the bus failed in the middle: say so, leave the arm as it is
            response.success, response.message = False, f"{label} failed: {e!r}"
        finally:
            self.busy.clear()
            self._last_grip = None
        if response.success:
            self.get_logger().info(f"{label}: {response.message}")
        else:
            self.get_logger().error(f"{label}: {response.message}")
        return response

    def on_start(self, request, response):
        return self._run(lambda: self.arm.start(), response, "start")

    def on_stop(self, request, response):
        return self._run(lambda: self.arm.stop(), response, "stop")

    def on_estop(self, request, response):
        """Immediately, whatever else is running: the lifecycle thread sees the e-stop as a fault and holds."""
        try:
            self.arm.abort()
            self.arm.sdk.electronic_emergency_stop()
            response.success, response.message = True, "electronic e-stop sent: the arm lowers under damping; clear_estop, then start"
        except Exception as e:
            response.success, response.message = False, f"e-stop could not be sent: {e!r} (power switch!)"
        self.get_logger().error(response.message)
        return response

    def on_clear_estop(self, request, response):
        if self.busy.is_set():
            response.success, response.message = False, "a start / stop is running"
            return response
        out = self.arm._clear_estop()
        response.success, response.message = (True, "no e-stop was active") if out is None else (out.ok, out.message)
        return response

    def on_enable(self, request, response):
        if request.data:
            response.success, response.message = False, "refused: use the start service (it enables the motors and moves to zero)"
            return response
        return self._run(lambda: self.arm.disable(), response, "disable")

    def close(self):
        self.arm.close()                                 # frees the bus; the motors are NOT touched
