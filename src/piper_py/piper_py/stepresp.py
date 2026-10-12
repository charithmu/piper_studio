"""Step-response characterisation of a backend's joint position servo, through the streaming controller.

    piper stepresp --backend NAME --out steps.json

For each tested joint: hold the current pose, step the position target by `delta`, hold, step back.
The same code runs on every backend, so differences are the backend's actuator/solver behaviour, not
the description or the controllers (identical everywhere). Analysis: tools/analyze_step_response.py.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from piper_py.demo import Recorder, stamp_now
from piper_py.robot import ARM_JOINTS, Piper

# (joint index, step in rad). Moderate steps from the 'ready' pose, inside limits.
TESTS = [(0, 0.20), (1, -0.20), (2, 0.20), (3, 0.30), (4, -0.30), (5, 0.40)]
READY = [0.0, 1.0, -1.0, 0.0, 1.0, 0.0]
RATE = 100.0
HOLD = 1.5


def _hold(arm: Piper, rec: Recorder, q: list[float], seconds: float):
    t0 = stamp_now(rec)
    k = 0
    while stamp_now(rec) < t0 + seconds:
        arm.stream_joints(q)
        k += 1
        t_next = t0 + k / RATE
        while stamp_now(rec) < t_next and stamp_now(rec) < t0 + seconds:
            time.sleep(0.0005)


def main(namespace: str, backend: str, out: Path) -> int:
    with Piper(namespace=namespace, wait=0) as arm:
        rec = Recorder(arm)
        arm.wait_ready(120.0, moveit=True)
        for _ in range(100):
            if arm.controllers().get("arm_controller") == "active":
                break
            time.sleep(0.2)
        r = arm.move_joints(READY, duration=3.0)
        assert r, r
        time.sleep(0.5)
        assert arm.use_streaming()
        events = []
        for j, delta in TESTS:
            q0 = list(READY)
            q1 = list(READY)
            q1[j] += delta
            _hold(arm, rec, q0, 0.6)
            t_up = stamp_now(rec)
            _hold(arm, rec, q1, HOLD)
            t_down = stamp_now(rec)
            _hold(arm, rec, q0, HOLD)
            events.append({"joint": ARM_JOINTS[j], "index": j, "delta": delta, "t_up": t_up, "t_down": t_down,
                           "t_end": stamp_now(rec)})
            print(f"[stepresp:{backend}] {ARM_JOINTS[j]} {delta:+.2f} rad", flush=True)
        arm.use_trajectories()
        samples = rec.snapshot()
        rec.close()
    result = {"backend": backend, "ready": READY, "events": events,
              "t": [s[0] for s in samples], "q": [[s[1].get(n) for n in ARM_JOINTS] for s in samples]}
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result))
    print(f"[stepresp:{backend}] wrote {out}")
    return 0
