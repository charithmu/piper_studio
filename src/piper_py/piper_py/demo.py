"""Scripted demo that exercises every control level and records what happened.

    piper demo --out run.json

Runs the same sequence on any backend and writes: per-step outcome (Result code, timing) and the joint
state stream, so runs on different backends can be compared (scripts/build_demo_page.py).
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from piper_py.robot import ARM_JOINTS, Piper, Result

DOWN = [0.0, 1.0, 0.0, 0.0]  # tool z axis pointing down


class Recorder:
    """Stores (stamp, positions by name) for every /joint_states message.

    Runs on its own node and thread with a deep queue so neither the client's streaming loop nor other
    callbacks can starve it (a simulator publishes 200 states per simulated second; a shallow queue in a busy
    client silently loses samples).
    """

    def __init__(self, arm: Piper):
        self.samples: list[tuple[float, dict]] = []
        self.last = 0.0  # stamp of the newest sample (O(1) to read)
        self._lock = threading.Lock()
        self.node = Node("piper_demo_recorder", namespace=arm._prefix.strip("/") or None)
        qos = QoSProfile(depth=10000, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.node.create_subscription(JointState, f"{arm._prefix}/joint_states", self._cb, qos)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def close(self):
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self.node.destroy_node()

    def _cb(self, m: JointState):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        with self._lock:
            self.samples.append((t, dict(zip(m.name, m.position))))
            self.last = t

    def snapshot(self):
        with self._lock:
            return list(self.samples)


def stamp_now(rec: Recorder) -> float:
    """Time of the latest joint state, in the backend's own time base (simulated or wall clock)."""
    return rec.last


def run(arm: Piper, backend: str) -> dict:
    rec = Recorder(arm)
    arm.wait_ready(120.0, moveit=True)
    states = {}
    for _ in range(100):  # controllers are spawned asynchronously
        states = arm.controllers()
        if states.get("arm_controller") == "active" and states.get("gripper_controller") == "active":
            break
        time.sleep(0.2)
    steps: list[dict] = []

    def step(name: str, level: str, fn):
        t0 = stamp_now(rec)
        w0 = time.monotonic()
        r = fn()
        steps.append({"name": name, "level": level, "t_start": t0, "t_end": stamp_now(rec),
                      "wall_s": round(time.monotonic() - w0, 3), "success": bool(r), "code": r.code,
                      "message": r.message})
        print(f"[demo:{backend}] {name:34s} {'OK ' if r else 'FAIL'} {r.code} ({time.monotonic() - w0:.1f}s wall)", flush=True)
        return r

    step("move to named pose 'rest'", "planning", lambda: arm.move_named("rest", 1.0))
    step("move to named pose 'ready'", "planning", lambda: arm.move_named("ready", 1.0))
    step("open gripper to 80 mm", "gripper", lambda: arm.gripper(0.08))
    step("pose goal above target (z=0.10)", "planning", lambda: arm.move_pose([0.25, 0.0, 0.10], DOWN, velocity_scaling=1.0))
    step("straight-line down to z=0.06", "planning (Pilz LIN)",
         lambda: arm.move_pose([0.25, 0.0, 0.06], DOWN, linear=True, velocity_scaling=0.5))
    step("close gripper to 20 mm", "gripper", lambda: arm.gripper(0.02))
    step("straight-line up to z=0.10", "planning (Pilz LIN)",
         lambda: arm.move_pose([0.25, 0.0, 0.10], DOWN, linear=True, velocity_scaling=0.5))
    step("open gripper to 80 mm", "gripper", lambda: arm.gripper(0.08))
    step("direct trajectory to joint goal", "trajectory (no MoveIt)",
         lambda: arm.move_joints([0.5, 1.0, -1.1, 0.0, 0.8, 0.0], duration=2.5))

    def streaming() -> Result:
        r = arm.use_streaming()
        if not r:
            return r
        q0 = arm.joint_positions()
        rate, seconds = 50.0, 6.0
        t_start = stamp_now(rec)  # paced on the joint-state stamps: simulated time in simulators, wall time otherwise
        for k in range(int(rate * seconds)):
            ph = 2 * math.pi * k / (rate * 3.0)  # one slow circle in joint space per 3 s
            q = list(q0)
            q[0] = q0[0] + 0.35 * math.sin(ph)
            q[1] = q0[1] + 0.10 * (1 - math.cos(ph))
            q[5] = q0[5] + 0.50 * math.sin(2 * ph)
            s = arm.stream_joints(q)
            if not s:
                arm.use_trajectories()
                return s
            while stamp_now(rec) < t_start + (k + 1) / rate:
                time.sleep(0.001)
        time.sleep(0.3)
        back = arm.use_trajectories()
        return Result(bool(back), "SUCCESS" if back else back.code, "streamed 6 s then handed back")

    step("stream joint targets at 50 Hz (6 s)", "streaming", streaming)
    step("return to 'rest'", "planning", lambda: arm.move_named("rest", 1.0))

    samples = rec.snapshot()
    rec.close()
    return {
        "backend": backend,
        "steps": steps,
        "joints": [*ARM_JOINTS, "gripper"],
        "t": [s[0] for s in samples],
        "q": [[s[1].get(j) for j in [*ARM_JOINTS, "gripper"]] for s in samples],
    }


def main(namespace: str, backend: str, out: Path) -> int:
    with Piper(namespace=namespace, wait=0) as arm:
        result = run(arm, backend)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result))
    ok = all(s["success"] for s in result["steps"])
    print(f"[demo:{backend}] {sum(s['success'] for s in result['steps'])}/{len(result['steps'])} steps succeeded -> {out}")
    return 0 if ok else 1
