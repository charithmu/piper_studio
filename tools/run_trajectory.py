#!/usr/bin/env python3
"""Run one planner trajectory on a simulation backend of this workspace and write what happened: start the stack, execute, stop it.

    source scripts/session.sh 105
    python tools/run_trajectory.py --backend isaac --trajectory plan.json --result result.json     # mock | gazebo | mujoco | isaac

`plan.json` is a `piper.trajectory/1` file (piper_core.backend.Trajectory.to_json()); `result.json` is a `piper.exec_result/1` file. The trajectory
runs on the backend's own `arm_controller` exactly as given (`Piper.execute_trajectory`), so the ROS backends and the ROS-free `piper_mujoco.PiperSim`
take the same file. Isaac needs the GPU (it starts through gpu-run inside the launch) and its USD (`scripts/isaac.sh build-usd`).
The stack runs in its own process group and is stopped by that group (never pkill -f); the ROS_DOMAIN_ID of the shell is used.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", required=True, choices=["mock", "gazebo", "mujoco", "isaac"])
    ap.add_argument("--trajectory", type=Path, required=True)
    ap.add_argument("--result", type=Path, required=True)
    ap.add_argument("--startup-timeout", type=float, default=180.0)
    a = ap.parse_args()
    if "ROS_DOMAIN_ID" not in os.environ:
        sys.exit("source scripts/session.sh <free ROS_DOMAIN_ID> first (Piper range 70-161)")

    from piper_core.backend import Trajectory
    from piper_py import Piper
    traj = Trajectory.from_json(a.trajectory.read_text())

    log = a.result.with_suffix(".stack.log")
    with open(log, "w") as f:
        stack = subprocess.Popen(["ros2", "launch", "piper_sim", "piper.launch.py", f"backend:={a.backend}", "rviz:=false", "camera:=none"],
                                 stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
    code = 1
    try:
        with Piper(wait=0) as arm:
            if not arm.wait_ready(a.startup_timeout, moveit=False):
                sys.exit(f"the stack did not come up in {a.startup_timeout:.0f} s (see {log})")
            deadline = time.monotonic() + 60
            while time.monotonic() < deadline and arm.controllers().get("arm_controller") != "active":
                time.sleep(0.2)
            time.sleep(1.0)
            res = arm.execute_trajectory(traj)
        a.result.write_text(res.to_json())
        print(f"{'OK' if res else 'FAILED'} {res.reason} {res.detail}".strip(), f"({len(res.t)} samples) -> {a.result}")
        code = 0 if res else 1
    finally:
        os.killpg(stack.pid, signal.SIGINT)
        try:
            stack.wait(60)
        except subprocess.TimeoutExpired:
            os.killpg(stack.pid, signal.SIGTERM)
            stack.wait(30)
        print(f"stack stopped (log: {log})")
    return code


if __name__ == "__main__":
    sys.exit(main())
