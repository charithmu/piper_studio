"""Python client for a Piper arm brought up with piper_bringup (any backend).

Every motion call blocks until the controller reports the outcome and returns a Result.
A Result is successful only when execution succeeded, never merely because planning did.

    from piper_py import Piper
    with Piper() as arm:
        arm.move_named("ready")
        arm.move_pose([0.25, 0.0, 0.20], [0, 1, 0, 0])   # xyz, quaternion xyzw of tcp_link
        arm.gripper(0.05)
"""

from __future__ import annotations

import math
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, ParallelGripperCommand
from controller_manager_msgs.srv import ListControllers, SwitchController
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint, ServoStatus)
from moveit_msgs.srv import ServoCommandType
from tf2_ros import Buffer, TransformException, TransformListener
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]


def _code_names(cls) -> dict[int, str]:
    return {v: k for k in dir(cls) if k.isupper() and isinstance(v := getattr(cls, k), int)}


_MOVEIT_CODES = _code_names(MoveItErrorCodes)
_FJT_CODES = _code_names(FollowJointTrajectory.Result)


@dataclass
class Result:
    success: bool
    code: str
    message: str = ""
    details: dict = field(default_factory=dict)

    def __bool__(self):
        return self.success


def _duration(seconds: float) -> Duration:
    return Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))


class Piper:
    """Blocking client; owns a node spun on a background thread."""

    def __init__(self, namespace: str = "", node_name: str = "piper_py",
                 group: str = "arm", tcp_link: str = "tcp_link", base_frame: str = "base_link",
                 wait: float = 30.0, use_sim_time: bool = False):
        if not rclpy.ok():
            rclpy.init()
            self._owns_context = True
        else:
            self._owns_context = False
        self.group, self.tcp_link, self.base_frame = group, tcp_link, base_frame
        ns = namespace.strip("/")
        self._prefix = f"/{ns}" if ns else ""
        # use_sim_time: node.get_clock() follows /clock, so client-side pacing (streaming loops) is in
        # simulation time and independent of the simulator's real-time factor.
        self.node = Node(node_name, namespace=ns or None,
                         parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=use_sim_time)]
                         if use_sim_time else [])
        self._lock = threading.Lock()
        self._joint_state: JointState | None = None
        self._joint_state_time = 0.0
        # State monitors take the newest sample only (depth 1) in their own callback group, so a
        # backlog or a long-running action callback can never hand the caller a stale state.
        state_group = ReentrantCallbackGroup()
        self.node.create_subscription(JointState, f"{self._prefix}/joint_states", self._on_joints,
                                      qos_profile_sensor_data, callback_group=state_group)
        # Raw driver feedback (backend:=real only); used to verify state before activating controllers.
        self._feedback: tuple[float, JointState] | None = None
        self.node.create_subscription(JointState, f"{self._prefix}/feedback/joint_states", self._on_feedback,
                                      qos_profile_sensor_data, callback_group=state_group)
        self._stream = self.node.create_publisher(
            Float64MultiArray, f"{self._prefix}/arm_position_controller/commands", 10)
        # MoveIt Servo (TCP-frame jogging through the streaming controller)
        self._twist = self.node.create_publisher(TwistStamped, f"{self._prefix}/servo_node/delta_twist_cmds", 10)
        self._servo_type = self.node.create_client(ServoCommandType, f"{self._prefix}/servo_node/switch_command_type")
        self._servo_code = None
        self.node.create_subscription(ServoStatus, f"{self._prefix}/servo_node/status", self._on_servo_status, 10,
                                      callback_group=state_group)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self.node, spin_thread=False)
        self._list_controllers = self.node.create_client(
            ListControllers, f"{self._prefix}/controller_manager/list_controllers")
        self._switch_controllers = self.node.create_client(
            SwitchController, f"{self._prefix}/controller_manager/switch_controller")
        self._trajectory = ActionClient(self.node, FollowJointTrajectory,
                                        f"{self._prefix}/arm_controller/follow_joint_trajectory")
        self._gripper = ActionClient(self.node, ParallelGripperCommand,
                                     f"{self._prefix}/gripper_controller/gripper_cmd")
        self._move_group = ActionClient(self.node, MoveGroup, f"{self._prefix}/move_action")
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin.start()
        self._named: dict[str, dict[str, float]] | None = None
        self._limits: dict[str, tuple[float, float]] | None = None
        if wait:
            self.wait_ready(wait)

    # ------------------------------------------------------------------ lifecycle
    def close(self):
        self._executor.shutdown(timeout_sec=2.0)
        self._spin.join(timeout=2.0)
        self.node.destroy_node()
        if self._owns_context and rclpy.ok():
            rclpy.shutdown()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def wait_ready(self, timeout: float = 30.0, moveit: bool = False) -> bool:
        """Wait for joint states and the controller manager (and move_group if moveit=True)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._joint_state is not None and self._list_controllers.service_is_ready() \
                    and (not moveit or self._move_group.server_is_ready()):
                return True
            time.sleep(0.05)
        raise TimeoutError("Piper not ready: no joint states or controller manager")

    # ------------------------------------------------------------------ state
    def _on_joints(self, msg: JointState):
        with self._lock:
            self._joint_state = msg
            self._joint_state_time = time.monotonic()

    def state_age(self) -> float:
        """Seconds since the last joint state arrived (inf if none yet)."""
        with self._lock:
            return time.monotonic() - self._joint_state_time if self._joint_state else math.inf

    def joint_state(self) -> dict[str, float]:
        """Latest measured positions by joint name (arm joints plus gripper width)."""
        with self._lock:
            msg = self._joint_state
        if msg is None:
            raise RuntimeError("no joint state received yet")
        return dict(zip(msg.name, msg.position))

    def joint_positions(self) -> list[float]:
        js = self.joint_state()
        return [js[j] for j in ARM_JOINTS]

    def _on_servo_status(self, msg: ServoStatus):
        self._servo_code = msg.code

    def _on_feedback(self, msg: JointState):
        with self._lock:
            self._feedback = (time.monotonic(), msg)

    # ------------------------------------------------------------------ controllers (command ownership)
    def _call(self, client, request, timeout: float = 5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{client.srv_name} unavailable")
        fut = client.call_async(request)
        if not self._wait(fut, timeout):
            raise RuntimeError(f"{client.srv_name} timed out")
        return fut.result()

    def controllers(self) -> dict[str, str]:
        """Controller name -> state (active / inactive / ...)."""
        res = self._call(self._list_controllers, ListControllers.Request())
        return {c.name: c.state for c in res.controller}

    def switch(self, activate=(), deactivate=()) -> Result:
        """Atomically activate/deactivate controllers (STRICT: all or nothing)."""
        req = SwitchController.Request(activate_controllers=list(activate),
                                       deactivate_controllers=list(deactivate),
                                       strictness=SwitchController.Request.STRICT, activate_asap=True)
        req.timeout.sec = 5
        res = self._call(self._switch_controllers, req, 10.0)
        return Result(res.ok, "SUCCESS" if res.ok else "SWITCH_FAILED", res.message)

    def activate(self, max_feedback_age: float = 0.2, tolerance: float = 0.01) -> Result:
        """Activate arm_controller and gripper_controller once measured state is trustworthy.

        On backend:=real the controllers start inactive. A trajectory controller holds whatever
        position it reads at activation, so activate only when driver feedback is fresh and
        ros2_control reports the same positions; otherwise the arm could be commanded to a stale pose.
        """
        states = self.controllers()
        wanted = [c for c in ("arm_controller", "gripper_controller") if c in states]
        real = "command_guard" in self.node.get_node_names() or \
            self.node.count_publishers(f"{self._prefix}/feedback/joint_states") > 0
        if real:
            with self._lock:
                fb = self._feedback
            if fb is None or time.monotonic() - fb[0] > max_feedback_age:
                return Result(False, "STALE_FEEDBACK", "no fresh feedback/joint_states from the driver")
            measured = dict(zip(fb[1].name, fb[1].position))
            reported = self.joint_state()
            diff = {j: abs(measured[j] - reported[j]) for j in ARM_JOINTS if j in measured}
            # NaN compares false with everything, so test for agreement rather than disagreement.
            if len(diff) != len(ARM_JOINTS) or not all(math.isfinite(d) and d <= tolerance
                                                       for d in diff.values()):
                return Result(False, "STATE_MISMATCH", "ros2_control state differs from driver feedback",
                              {"diff": diff})
        todo = [c for c in wanted if states[c] != "active"]
        if states.get("arm_position_controller") == "active" and "arm_controller" in todo:
            return self.switch(todo, ["arm_position_controller"])
        return self.switch(todo) if todo else Result(True, "SUCCESS", "already active")

    def deactivate(self) -> Result:
        """Stop all command controllers; the arm holds through its own firmware/servo."""
        states = self.controllers()
        active = [c for c in ("arm_controller", "arm_position_controller", "gripper_controller")
                  if states.get(c) == "active"]
        return self.switch(deactivate=active) if active else Result(True, "SUCCESS", "nothing active")

    def use_streaming(self) -> Result:
        """Hand the arm to arm_position_controller for stream_joints() (teleop, Servo, policies). Idempotent."""
        st = self.controllers()
        if st.get("arm_position_controller") == "active" and st.get("arm_controller") != "active":
            return Result(True, "SUCCESS", "already streaming")
        return self.switch(["arm_position_controller"], [c for c in ["arm_controller"] if st.get(c) == "active"])

    def use_trajectories(self) -> Result:
        """Hand the arm back to arm_controller (move_joints, MoveIt). Idempotent."""
        st = self.controllers()
        if st.get("arm_controller") == "active" and st.get("arm_position_controller") != "active":
            return Result(True, "SUCCESS", "already in trajectory mode")
        return self.switch(["arm_controller"], [c for c in ["arm_position_controller"] if st.get(c) == "active"])

    def stream_joints(self, positions: list[float]) -> Result:
        """Publish one position target to arm_position_controller (call at a steady rate).

        No interpolation is done here: targets must be close to the current state.
        """
        bad = self._out_of_limits(positions)
        if bad:
            return Result(False, "OUT_OF_LIMITS", f"outside URDF limits: {bad}")
        self._stream.publish(Float64MultiArray(data=list(map(float, positions))))
        return Result(True, "SENT")

    def _out_of_limits(self, positions) -> dict[str, float]:
        if len(positions) != len(ARM_JOINTS):
            raise ValueError(f"expected {len(ARM_JOINTS)} joint values")
        limits = self.joint_limits()
        return {j: p for j, p in zip(ARM_JOINTS, positions) if not limits[j][0] <= p <= limits[j][1]}

    # ------------------------------------------------------------------ actions
    def _run(self, client: ActionClient, goal, timeout: float):
        if not client.wait_for_server(timeout_sec=5.0):
            return None, Result(False, "NO_SERVER", f"{client._action_name} unavailable")
        send = client.send_goal_async(goal)
        if not self._wait(send, 5.0):
            return None, Result(False, "TIMED_OUT", "goal not acknowledged")
        handle = send.result()
        if not handle.accepted:
            return None, Result(False, "REJECTED", "goal rejected by server")
        done = handle.get_result_async()
        if not self._wait(done, timeout):
            handle.cancel_goal_async()
            return None, Result(False, "TIMED_OUT", f"no result within {timeout} s; goal canceled")
        return done.result(), None

    @staticmethod
    def _wait(future, timeout: float) -> bool:
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        return event.wait(timeout)

    def move_joints(self, positions: list[float], duration: float = 3.0) -> Result:
        """Execute a single-segment joint trajectory directly on arm_controller (no MoveIt)."""
        # joint_trajectory_controller does not enforce URDF limits; never send a target past a hard stop.
        bad = self._out_of_limits(positions)
        if bad:
            limits = self.joint_limits()
            return Result(False, "OUT_OF_LIMITS", f"outside URDF limits: {bad}",
                          {j: limits[j] for j in bad})
        traj = JointTrajectory(joint_names=ARM_JOINTS, points=[
            JointTrajectoryPoint(positions=list(map(float, positions)),
                                 velocities=[0.0] * len(ARM_JOINTS),
                                 time_from_start=_duration(duration))])
        res, err = self._run(self._trajectory, FollowJointTrajectory.Goal(trajectory=traj), duration + 10.0)
        if err:
            return err
        code = res.result.error_code
        return Result(code == FollowJointTrajectory.Result.SUCCESSFUL,
                      _FJT_CODES.get(code, str(code)), res.result.error_string)

    def gripper(self, width: float, max_effort: float = 0.0, timeout: float = 10.0) -> Result:
        """Command the gripper opening width in metres.

        Succeeds if the width is reached, or if the fingers stall while *closing* (code GRASPED:
        something is between them). A stall while opening, or without moving, is a failure.
        """
        start = self.joint_state().get("gripper")
        cmd = JointState(name=["gripper"], position=[float(width)])
        if max_effort > 0:
            cmd.effort = [float(max_effort)]
        res, err = self._run(self._gripper, ParallelGripperCommand.Goal(command=cmd), timeout)
        if err:
            return err
        r = res.result
        final = r.state.position[0] if r.state.position else None
        details = {"width": final, "start": start, "goal": width}
        if r.reached_goal:
            return Result(True, "SUCCESS", details=details)
        closed = start is not None and final is not None and final < start - 0.002
        if r.stalled and width < (start or 0.0) and closed:
            return Result(True, "GRASPED", "stalled while closing", details)
        return Result(False, "STALLED" if r.stalled else "FAILED",
                      "gripper did not reach the goal", details)

    # ------------------------------------------------------------------ MoveIt
    def _plan_and_execute(self, constraints: Constraints, velocity_scaling: float,
                          pipeline: str = "ompl", planner: str = "", timeout: float = 60.0) -> Result:
        req = MotionPlanRequest(group_name=self.group, num_planning_attempts=5, allowed_planning_time=5.0,
                                max_velocity_scaling_factor=velocity_scaling,
                                max_acceleration_scaling_factor=velocity_scaling,
                                pipeline_id=pipeline, planner_id=planner,
                                goal_constraints=[constraints])
        req.start_state.is_diff = True
        goal = MoveGroup.Goal(request=req)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        res, err = self._run(self._move_group, goal, timeout)
        if err:
            return err
        code = res.result.error_code.val
        return Result(code == MoveItErrorCodes.SUCCESS, _MOVEIT_CODES.get(code, str(code)),
                      details={"planning_time": res.result.planning_time})

    def move_to_joints(self, positions: list[float], velocity_scaling: float = 0.2) -> Result:
        """Collision-aware plan to a joint configuration, then execute."""
        c = Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=float(p), tolerance_above=1e-3,
                            tolerance_below=1e-3, weight=1.0) for j, p in zip(ARM_JOINTS, positions)])
        return self._plan_and_execute(c, velocity_scaling)

    def _get_string_param(self, node: str, name: str) -> str:
        client = self.node.create_client(GetParameters, f"{self._prefix}/{node}/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"{node} parameter service unavailable")
            fut = client.call_async(GetParameters.Request(names=[name]))
            if not self._wait(fut, 5.0):
                raise RuntimeError(f"timed out reading {node}/{name}")
            return fut.result().values[0].string_value
        finally:
            self.node.destroy_client(client)

    def joint_limits(self) -> dict[str, tuple[float, float]]:
        """Position limits of revolute/prismatic joints from the URDF in robot_state_publisher."""
        if self._limits is None:
            urdf = ET.fromstring(self._get_string_param("robot_state_publisher", "robot_description"))
            self._limits = {
                j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
                for j in urdf.findall("joint")
                if j.get("type") in ("revolute", "prismatic") and j.find("limit") is not None}
        return self._limits

    def named_poses(self) -> dict[str, dict[str, float]]:
        """Arm group states from the SRDF loaded in move_group."""
        if self._named is None:
            srdf = ET.fromstring(self._get_string_param("move_group", "robot_description_semantic"))
            self._named = {
                s.get("name"): {j.get("name"): float(j.get("value")) for j in s.findall("joint")}
                for s in srdf.findall("group_state") if s.get("group") == self.group}
        return self._named

    def move_named(self, name: str, velocity_scaling: float = 0.2) -> Result:
        poses = self.named_poses()
        if name not in poses:
            return Result(False, "UNKNOWN_NAME", f"{name!r} not in {sorted(poses)}")
        return self.move_to_joints([poses[name][j] for j in ARM_JOINTS], velocity_scaling)

    def move_pose(self, position, orientation_xyzw, frame: str | None = None, linear: bool = False,
                  velocity_scaling: float = 0.2, position_tolerance: float = 0.001,
                  orientation_tolerance: float = 0.01) -> Result:
        """Move tcp_link to a pose. linear=True uses the Pilz LIN planner (straight Cartesian path)."""
        ps = PoseStamped()
        ps.header.frame_id = frame or self.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, position)
        o = ps.pose.orientation
        o.x, o.y, o.z, o.w = map(float, orientation_xyzw)
        region = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[position_tolerance])
        pc = PositionConstraint(header=ps.header, link_name=self.tcp_link, weight=1.0)
        pc.constraint_region.primitives.append(region)
        pc.constraint_region.primitive_poses.append(ps.pose)
        oc = OrientationConstraint(header=ps.header, link_name=self.tcp_link, orientation=o, weight=1.0,
                                   absolute_x_axis_tolerance=orientation_tolerance,
                                   absolute_y_axis_tolerance=orientation_tolerance,
                                   absolute_z_axis_tolerance=orientation_tolerance)
        c = Constraints(position_constraints=[pc], orientation_constraints=[oc])
        if linear:
            return self._plan_and_execute(c, velocity_scaling, "pilz_industrial_motion_planner", "LIN")
        return self._plan_and_execute(c, velocity_scaling)

    # ------------------------------------------------------------------ wrist camera
    COLOR_TOPIC = "/camera/color/image_raw"
    DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"
    INFO_TOPIC = "/camera/color/camera_info"

    def image(self, kind: str = "color", timeout: float = 5.0):
        """Wait for the next camera frame. Returns (array, header): color -> HxWx3 uint8 (RGB), depth -> HxW float32
        in metres (NaN where invalid). Same topics on every backend (and the RealSense driver)."""
        topic, msg_type = (self.COLOR_TOPIC, Image) if kind == "color" else (self.DEPTH_TOPIC, Image)
        box: list = []
        done = threading.Event()
        sub = self.node.create_subscription(msg_type, f"{self._prefix}{topic}",
                                            lambda m: (box.append(m), done.set()), qos_profile_sensor_data)
        try:
            if not done.wait(timeout):
                raise TimeoutError(f"no image on {topic} within {timeout} s")
        finally:
            self.node.destroy_subscription(sub)
        return _image_to_array(box[0]), box[0].header

    def camera_info(self, timeout: float = 5.0) -> CameraInfo:
        box: list = []
        done = threading.Event()
        sub = self.node.create_subscription(CameraInfo, f"{self._prefix}{self.INFO_TOPIC}",
                                            lambda m: (box.append(m), done.set()), qos_profile_sensor_data)
        try:
            if not done.wait(timeout):
                raise TimeoutError(f"no camera_info within {timeout} s")
        finally:
            self.node.destroy_subscription(sub)
        return box[0]

    # ------------------------------------------------------------------ TCP pose and Servo (Cartesian jogging)
    def tcp_pose(self, frame: str | None = None) -> tuple[list[float], list[float]]:
        """Current tcp_link pose in `frame` (default base_link): ([x, y, z], [qx, qy, qz, qw])."""
        try:
            t = self._tf.lookup_transform(frame or self.base_frame, self.tcp_link, rclpy.time.Time()).transform
        except TransformException as e:
            raise RuntimeError(f"no TF {frame or self.base_frame} -> {self.tcp_link}: {e}") from e
        return ([t.translation.x, t.translation.y, t.translation.z],
                [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    def servo_start(self) -> Result:
        """Hand the arm to the streaming controller and put MoveIt Servo in Cartesian (twist) mode."""
        r = self.use_streaming()
        if not r:
            return r
        req = ServoCommandType.Request(command_type=ServoCommandType.Request.TWIST)
        res = self._call(self._servo_type, req, 10.0)
        return Result(bool(res.success), "SUCCESS" if res.success else "SERVO_REFUSED",
                      "" if res.success else "servo_node refused TWIST mode")

    def servo_twist(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0), frame: str | None = None):
        """Publish one velocity command (m/s, rad/s) expressed in `frame` (default tcp_link). Repeat at >= 20 Hz."""
        m = TwistStamped()
        m.header.frame_id = frame or self.tcp_link
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = map(float, linear)
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = map(float, angular)
        self._twist.publish(m)

    def servo_to_pose(self, position, orientation_xyzw=None, tol_pos: float = 0.003, tol_rot: float = 0.03,
                      max_linear: float = 0.10, max_angular: float = 0.5, gain: float = 3.0,
                      timeout: float = 30.0, rate: float = 50.0) -> Result:
        """Closed-loop move of tcp_link to a pose in base_link by Servo twists (collision/singularity aware).

        Success only when the measured TCP is within tol_pos (m) and tol_rot (rad) of the target and has stopped
        commanding; Servo halts (collision, singularity, joint limit) end it with the Servo status code.
        """
        started = self.servo_start()
        if not started:
            return started
        target_p = np.asarray(position, float)
        target_q = None if orientation_xyzw is None else np.asarray(orientation_xyzw, float) / np.linalg.norm(orientation_xyzw)
        deadline = time.monotonic() + timeout
        period = 1.0 / rate
        result = Result(False, "TIMED_OUT", f"not within tolerance after {timeout} s")
        while time.monotonic() < deadline:
            cur_p, cur_q = self.tcp_pose()
            ep = target_p - np.asarray(cur_p)
            lin = gain * ep
            if np.linalg.norm(lin) > max_linear:
                lin *= max_linear / np.linalg.norm(lin)
            ang, rot_err = np.zeros(3), 0.0
            if target_q is not None:
                qe = _quat_mul(target_q, _quat_conj(np.asarray(cur_q)))
                if qe[3] < 0:
                    qe = -qe
                rot_err = 2 * math.atan2(np.linalg.norm(qe[:3]), qe[3])
                axis = qe[:3] / (np.linalg.norm(qe[:3]) or 1.0)
                ang = gain * rot_err * axis
                if np.linalg.norm(ang) > max_angular:
                    ang *= max_angular / np.linalg.norm(ang)
            if np.linalg.norm(ep) <= tol_pos and rot_err <= tol_rot:
                result = Result(True, "SUCCESS", details={"pos_err_mm": float(np.linalg.norm(ep) * 1000), "rot_err_rad": rot_err})
                break
            if self._servo_code in _SERVO_HALTS and np.linalg.norm(lin) > 0.02:
                names = {v: k for k, v in vars(ServoStatus).items() if k.isupper() and isinstance(v, int)}
                result = Result(False, names.get(self._servo_code, str(self._servo_code)),
                                "Servo stopped the motion", {"pos_err_mm": float(np.linalg.norm(ep) * 1000)})
                break
            self.servo_twist(lin, ang, frame=self.base_frame)
            time.sleep(period)
        for _ in range(5):  # explicit zero command so Servo halts promptly
            self.servo_twist()
            time.sleep(period)
        return result


# Servo halts (not mere decelerations, which Servo handles itself): collision, singularity, joint limit.
_SERVO_HALTS = (ServoStatus.HALT_FOR_COLLISION, ServoStatus.HALT_FOR_SINGULARITY, ServoStatus.JOINT_BOUND)


def _image_to_array(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> numpy without cv_bridge/OpenCV (rgb8, bgr8, rgba8, mono8, 16UC1 [mm], 32FC1 [m])."""
    enc, h, w = msg.encoding, msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        a = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
        return a if enc == "rgb8" else a[:, :, ::-1].copy()
    if enc in ("rgba8", "bgra8"):
        a = buf.reshape(h, msg.step)[:, : w * 4].reshape(h, w, 4)[:, :, :3]
        return a if enc == "rgba8" else a[:, :, ::-1].copy()
    if enc == "mono8":
        return buf.reshape(h, msg.step)[:, :w].copy()
    if enc == "16UC1":
        a = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(h, msg.step // 2)[:, :w].astype(np.float32) * 1e-3
        a[a == 0] = np.nan
        return a
    if enc == "32FC1":
        return np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(h, msg.step // 4)[:, :w].copy()
    raise ValueError(f"unsupported image encoding {enc!r}")


def _quat_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])
