#!/usr/bin/env python3
"""Plan the test scenario with cuRobo: approach -> grasp -> (close gripper) -> lift -> (open gripper) -> back to zero. Writes one scenario.json that run_scenario.py runs on the real arm,
on the ROS path or on MuJoCo. No arm, no ROS: it only needs the GPU.

Runs in the cuRobo study's own environment (cuRobo and torch live there; we only import its planner, read-only), through gpu-run:

    gpu-run ~/projects/ml_tests/experiments/curobo_piper_study/.venv/bin/python tools/real_arm/curobo_plan_scenario.py [--xyz 0.25 0 0.05] [--yaw 0]

The target is a TCP position (metres, base frame) with the tool pointing straight down; cuRobo plans the approach to 5 cm above it (--approach), the descent, and the lift; the way home is a joint-space
plan back to zero. Every leg starts where the previous one ended (planned from the arm's zero pose, as piper_pick does from the measured pose). The cuRobo scene is the study's default: a table plane at the
base (z = -3 mm) and nothing else. Check that the real workspace matches before running it on the arm.
Output: ~/data/ml/isaac/piper_studio/scenario/<time>/scenario.json (schema piper.scenario/1: steps, trajectories as piper.trajectory/1).
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

STUDY = Path.home() / "projects/ml_tests/experiments/curobo_piper_study"
sys.path.insert(0, str(STUDY / "src"))
from curobo_piper.motion import ZERO, tool_quat  # noqa: E402
from curobo_piper.planner import PiperPlanner  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xyz", type=float, nargs=3, default=[0.25, 0.0, 0.05], help="TCP target at the grasp, base frame, metres (tool-down reach is limited to about z < 0.11 m incl. the approach: results/07_piper/reach_grid.json)")
    ap.add_argument("--yaw", type=float, default=0.0, help="gripper yaw about the tool z axis, degrees (yaw and yaw+-180 are tried)")
    ap.add_argument("--approach", type=float, default=0.05, help="start of the descent this far above the target, metres")
    ap.add_argument("--open", type=float, default=0.07, help="gripper width at the start and after the lift, metres")
    ap.add_argument("--close", type=float, default=0.0, help="gripper width commanded at the grasp, metres (0: closes until it stalls on an object)")
    ap.add_argument("--camera", choices=["none", "zedxm"], default=None, help="the arm to plan for: zedxm = the ZED X Mini on its bracket, none = the robot only. Default: what docs/real_arm/arms/<arm>.yaml says is mounted on the real arm")
    ap.add_argument("--arm", default="piper-01", help="the real arm whose mounted hardware is the default (docs/real_arm/arms/<arm>.yaml)")
    ap.add_argument("--out", default=str(Path.home() / "data/ml/isaac/piper_studio/scenario"))
    args = ap.parse_args()

    import arm_config
    cam = arm_config.camera(args.arm) if args.camera is None else (None if args.camera == "none" else "zedxm")
    planner = PiperPlanner.from_assets(cam, gripper_width=0.07)      # the model Piper Studio ships (piper_assets.curobo_config); refuses a stale one
    q0 = np.asarray(ZERO, dtype=float)
    legs, used_yaw = None, None
    for yaw in (args.yaw, args.yaw + 180 if args.yaw <= 0 else args.yaw - 180):
        res, legs = planner.plan_grasp(q0, [args.xyz], [tool_quat(0, yaw)], approach=-args.approach, lift=-args.approach)
        if legs is not None:
            used_yaw = yaw
            break
    if legs is None:
        raise SystemExit(f"no plan for TCP {args.xyz} with the tool pointing down: it may be out of reach (the study's reach scan: results/07_piper/reach_grid.json) or in collision with the table")
    home = planner.plan_joints(legs["lift"].q[-1], q0)
    if home is None:
        raise SystemExit("no plan back to zero from the lift pose")
    steps = [{"op": "gripper", "width": args.open, "label": "open"},
             {"op": "move", "leg": "approach"}, {"op": "move", "leg": "grasp"},
             {"op": "gripper", "width": args.close, "label": "close"},
             {"op": "move", "leg": "lift"},
             {"op": "gripper", "width": args.open, "label": "open (release)"},
             {"op": "move", "leg": "home"}]
    trajs = {"approach": legs["approach"], "grasp": legs["grasp"], "lift": legs["lift"], "home": home}
    doc = {"schema": "piper.scenario/1", "target_xyz": args.xyz, "yaw_deg": used_yaw, "approach_m": args.approach, "steps": steps,
           "legs": {k: json.loads(t.to_json()) for k, t in trajs.items()}, "planner": "curobo (curobo_piper_study PiperPlanner)", "camera": cam,
           "scene": "table plane at base z = -3 mm only", "created": time.strftime("%Y-%m-%d %H:%M:%S")}
    out = Path(args.out) / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    (out / "scenario.json").write_text(json.dumps(doc))
    for k, t in trajs.items():
        v = float(np.abs(np.diff(t.q, axis=0)).max(axis=1).max() / np.diff(t.t).mean())
        print(f"{k:9s} {len(t.t):4d} points, {t.t[-1]:5.2f} s, peak {v:.2f} rad/s, solve {t.meta.get('solve_ms', float('nan')):.0f} ms")
    print("saved", out / "scenario.json")


if __name__ == "__main__":
    main()
