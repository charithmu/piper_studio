#!/usr/bin/env python3
"""Real Piper: measure how the arm follows a trajectory of 25 ms setpoints through RealArm.execute (tracking error, lag, overshoot, timing, speed percent vs real speed, repeatability).

    source scripts/env.sh
    python tools/real_arm/arm_stream_test.py --offline                       # no arm: print the plan, the speed it needs and the numbers to expect
    python tools/real_arm/arm_stream_test.py                                 # joint 1, 10 deg cosine over 4 s, mode fixed, at most 10 % speed
    python tools/real_arm/arm_stream_test.py --mode per_point --lookahead 2 --repeat 3
    python tools/real_arm/arm_stream_test.py --joint 5 --amplitude 15 --period 2 --max-percent 20      # faster: you raise --max-percent on purpose, one step at a time

The arm must be ENABLED and at a valid pose (piper-arm: `s`). One joint moves out and back; the others stay put. It refuses a plan that needs more speed than --max-percent (default 10) or leaves the
stored limits (0.01 rad margin). Nothing is sent before you press Enter; Ctrl-C holds the pose (motors stay on). The user must be present, hand on the power switch.
Every run is saved (plan, commanded, fine measured log, metrics) in ~/data/ml/isaac/piper_studio/stream/ and the metrics are printed (piper_core.stream_metrics). The arm is left enabled where it ended.
Ladder used for the report (docs/real_arm/CHECKLIST.md): 10 % -> 20 -> 30 -> 50, each step approved by the user, fixed vs per_point vs lookahead compared at the same plan.
"""
import argparse
import json
import math
import time
from pathlib import Path

import numpy as np

from piper_core import RealArm
from piper_core.real_arm import JOINTS
from piper_core.stream_metrics import analyze, format_metrics, repeatability
from piper_core.stream_plan import cosine, peak_speed, ramp, required_percent

OUT = Path.home() / "data/ml/isaac/piper_studio/stream"
MARGIN = 0.01


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b", default="can0")
    ap.add_argument("--joint", type=int, default=1, choices=range(1, 7), metavar="1..6")
    ap.add_argument("--amplitude", type=float, default=10.0, help="degrees (negative: the other way)")
    ap.add_argument("--period", type=float, default=4.0, help="seconds (cosine: one cycle; ramp: one way)")
    ap.add_argument("--cycles", type=int, default=1, help="cosine cycles per run")
    ap.add_argument("--kind", choices=("cosine", "ramp"), default="cosine")
    ap.add_argument("--dt", type=float, default=0.025, help="setpoint spacing, seconds (cuRobo: 0.025)")
    ap.add_argument("--mode", choices=("fixed", "per_point"), default="fixed")
    ap.add_argument("--lookahead", default="0", help="0..10 setpoints ahead of the clock, or 'auto' (follows the speed: 35 ms + 73 ms per rad/s)")
    ap.add_argument("--margin", type=float, default=20.0, help="speed headroom percent")
    ap.add_argument("--max-percent", type=int, default=10, help="refuse a plan that needs more speed percent than this (raise it on purpose)")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--offline", action="store_true", help="no arm: print the plan only")
    args = ap.parse_args()
    args.lookahead = "auto" if str(args.lookahead) == "auto" else int(args.lookahead)
    j = args.joint - 1
    amp = math.radians(args.amplitude)

    arm = None
    if args.offline:
        q0 = [0.0, 0.01, -0.01, 0.0, 0.0, 0.0]
    else:
        arm = RealArm.connect(args.canbus)
        q = arm.positions()
        q0 = [min(max(q[n], arm.limits[n][0] + MARGIN), arm.limits[n][1] - MARGIN) for n in JOINTS]
    try:
        traj = (cosine(q0, j, amp, args.period, args.cycles, args.dt) if args.kind == "cosine" else ramp(q0, j, amp, args.period, args.dt))
        pct = required_percent(traj, args.margin)
        print(f"plan: joint {args.joint} {args.kind}, {args.amplitude:+.1f} deg, {len(traj.t)} points every {args.dt * 1000:.0f} ms, {traj.t[-1]:.1f} s, "
              f"peak {peak_speed(traj):.3f} rad/s ({math.degrees(peak_speed(traj)):.1f} deg/s) -> needs {pct} % (+{args.margin:.0f} %), mode {args.mode}, lookahead {args.lookahead}")
        if pct > args.max_percent:
            raise SystemExit(f"refused: needs {pct} % but --max-percent is {args.max_percent}. Slow it down (--period) or raise --max-percent deliberately.")
        if arm is None:
            return
        lo, hi = arm.limits[JOINTS[j]]
        if not (lo + MARGIN <= q0[j] + min(0.0, amp) and q0[j] + max(0.0, amp) <= hi - MARGIN):
            raise SystemExit(f"refused: joint {args.joint} would leave [{math.degrees(lo + MARGIN):.1f}, {math.degrees(hi - MARGIN):.1f}] deg: use --amplitude with the other sign or a smaller value")
        if arm._not_started():
            raise SystemExit(f"refused: {arm._not_started()}. Start it first (piper-arm: s).")
        if max(abs(q[n] - q0[i]) for i, n in enumerate(JOINTS)) > 0.04:
            raise SystemExit("refused: the arm is more than 0.04 rad from the plan's start pose; move it near zero or the pose you want first")
        input("Arm clear, hand on the power switch? Enter to RUN (Ctrl-C holds)... ")
        Path(args.out).mkdir(parents=True, exist_ok=True)
        finals = []
        for run in range(1, args.repeat + 1):
            r = arm.execute(traj, margin_percent=args.margin, mode=args.mode, lookahead=args.lookahead)
            print(f"run {run}/{args.repeat}: ok={r.ok} {r.message}")
            if arm.last_stream and len(arm.last_stream["t"]) > 5 and r.q_meas.size:
                m = analyze(traj, arm.last_stream)
                print(format_metrics(m))
                name = f"{time.strftime('%Y%m%d_%H%M%S')}_j{args.joint}_{args.kind}_{args.mode}_la{args.lookahead}_run{run}.json"
                (Path(args.out) / name).write_text(json.dumps({"args": vars(args), "traj": json.loads(traj.to_json()), "result": json.loads(r.to_json()),
                                                               "stream": arm.last_stream, "metrics": m}, default=float))
                print("saved", Path(args.out) / name)
                finals.append([arm.positions()[n] for n in JOINTS])
            if not r.ok:
                break
            time.sleep(1.0)
        if len(finals) > 1:
            print("repeatability of the final pose:", {k: [round(x, 3) for x in v] if isinstance(v, list) else v for k, v in repeatability(finals).items()})
        print("done: the arm is left enabled where it ended (piper-arm to park / end the session)")
    except KeyboardInterrupt:
        print("\nCtrl-C: holding the current position; the motors stay ENABLED.")
        if arm is not None:
            arm.hold()
    finally:
        if arm is not None:
            arm.close()


if __name__ == "__main__":
    main()
