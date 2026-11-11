#!/usr/bin/env python3
"""Real gripper measurements (R2 in docs/real_arm/CHECKLIST.md), with an Enter before every stage that moves the gripper. User present, fingers out of the closing path.

    source scripts/env.sh                         # close piper-arm and any other program first (one program on the bus)
    python tools/real_arm/gripper_test.py empty                     # A: open / close 5 times, nothing in the gripper: stroke, empty-closed width, times, repeatability, force
    python tools/real_arm/gripper_test.py force                     # B: closing and opening time at 0.5 1 1.5 2 3 N (nothing in the gripper)
    python tools/real_arm/gripper_test.py object --name box         # C: an object: stalled width, force, creep over 3 s at 1 2 3 N, 3 repeats each
    python tools/real_arm/gripper_test.py slip --name box           # D: an object held at one force while the arm moves (joint 1, 20 deg over 2 s): the width during the move

A, B and C only talk to the gripper, so they also show whether it works with the arm's motors off (run them that way first). D needs the arm started (`piper-arm` `s`, then close it) and moves joint 1;
the arm is left where it ended. Results are printed and saved in <data dir>/gripper/. Measured, not assumed: the times come from a 50 Hz trace of the width.
"""
import argparse
import json
import math
import threading
import time
from pathlib import Path

import numpy as np

from piper_tools.paths import data_dir

OUT = data_dir("gripper")
REACH_TOL, SETTLE_TOL = 0.001, 0.0003          # m: "reached the target" and "stopped changing"


def times_from_trace(trace, target, w_final=None):
    """(t_reach, t_settle) in seconds from a (t, width, force) trace: first time within 1 mm of the target (None if never), and first time within 0.3 mm of the final width and staying there."""
    if not trace:
        return None, None
    t = np.array([x[0] for x in trace])
    w = np.array([x[1] for x in trace])
    wf = w[-1] if w_final is None else w_final
    hit = np.nonzero(np.abs(w - target) <= REACH_TOL)[0]
    near = np.abs(w - wf) <= SETTLE_TOL
    settle = next((float(t[i]) for i in range(len(t)) if near[i:].all()), None)
    return (float(t[hit[0]]) if len(hit) else None), settle


def stats(values):
    v = np.array([x for x in values if x is not None], dtype=float)
    return {"n": int(len(v)), "mean": float(v.mean()) if len(v) else None, "std": float(v.std()) if len(v) else None, "min": float(v.min()) if len(v) else None, "max": float(v.max()) if len(v) else None}


def move(arm, width, effort):
    """One gripper move; returns the result row (width in mm, times in s, force)."""
    arm.set_gripper(width, effort=effort)
    g = arm.last_gripper
    t_reach, t_settle = times_from_trace(g.trace, width, g.width)
    forces = [f for _, _, f in g.trace if f is not None]
    return {"target_mm": width * 1000, "width_mm": g.width * 1000, "reason": g.reason, "t_reach_s": t_reach, "t_settle_s": t_settle, "force_n": g.force,
            "force_peak_n": float(max(forces, key=abs)) if forces else None, "effort_n": effort}


def mm(x):
    return "   --" if x is None else f"{x * 1000:7.2f}"


def line(label, rows, key, scale=1.0, unit=""):
    s = stats([None if r[key] is None else r[key] * scale for r in rows])
    return f"  {label:28s} mean {s['mean']:8.3f}{unit}  std {s['std']:6.3f}  min {s['min']:8.3f}  max {s['max']:8.3f}  (n={s['n']})" if s["n"] else f"  {label:28s} none"


def stage_empty(arm, args, ask):
    ask(f"A: {args.cycles} x (open, close) with nothing in the gripper at {args.effort} N")
    opens, closes = [], []
    for i in range(args.cycles):
        opens.append(move(arm, arm.gripper_stroke, args.effort))
        closes.append(move(arm, 0.0, args.effort))
        print(f"  cycle {i + 1}: open {opens[-1]['width_mm']:.2f} mm in {opens[-1]['t_reach_s']} s, close {closes[-1]['width_mm']:.2f} mm in {closes[-1]['t_reach_s']} s, force {opens[-1]['force_n']} / {closes[-1]['force_n']} N")
    print("\nOPEN  (command 70 mm)")
    print(line("width, mm", opens, "width_mm"))
    print(line("time to within 1 mm, s", opens, "t_reach_s"))
    print(line("force at rest, N", opens, "force_n"))
    print("CLOSE (command 0 mm)")
    print(line("width, mm", closes, "width_mm"))
    print(line("time to within 1 mm, s", closes, "t_reach_s"))
    print(line("force at rest, N", closes, "force_n"))
    return {"open": opens, "close": closes}


def stage_force(arm, args, ask):
    ask("B: close and open at 0.5, 1, 1.5, 2, 3 N, twice each, nothing in the gripper")
    rows = []
    for effort in (0.5, 1.0, 1.5, 2.0, 3.0):
        for _ in range(2):
            o = move(arm, arm.gripper_stroke, effort)
            c = move(arm, 0.0, effort)
            rows.append({"effort_n": effort, "open": o, "close": c})
        sel = [r for r in rows if r["effort_n"] == effort]
        print(f"  {effort:3.1f} N: close {np.mean([r['close']['t_reach_s'] or math.nan for r in sel]):.2f} s, open {np.mean([r['open']['t_reach_s'] or math.nan for r in sel]):.2f} s, "
              f"closed width {np.mean([r['close']['width_mm'] for r in sel]):.2f} mm, force closed {np.mean([r['close']['force_n'] or math.nan for r in sel]):.3f} N")
    return rows


def stage_object(arm, args, ask):
    out = {}
    for effort in args.efforts:
        rows = []
        for rep in range(args.repeats):
            arm.set_gripper(arm.gripper_stroke, effort=effort)
            ask(f"C: object '{args.name}' between the open fingers (repeat {rep + 1}/{args.repeats}, force {effort} N); Enter closes on it")
            c = move(arm, 0.0, effort)
            t0, creep = time.time(), []
            while time.time() - t0 < 3.0:                          # hold and watch the width for 3 s: does the object creep?
                creep.append(arm.gripper_opening())
                time.sleep(0.1)
            c["creep_mm"] = (creep[-1] - creep[0]) * 1000
            c["force_after_3s_n"] = arm.gripper_force()
            rows.append(c)
            print(f"  {effort} N #{rep + 1}: {c['reason']} at {c['width_mm']:.2f} mm, force {c['force_n']} N (peak {c['force_peak_n']}), after 3 s {creep[-1] * 1000:.2f} mm ({c['creep_mm']:+.2f} mm), force {c['force_after_3s_n']}")
            arm.set_gripper(arm.gripper_stroke, effort=effort)
        out[str(effort)] = rows
        print(line(f"stalled width @ {effort} N, mm", rows, "width_mm"))
        print(line(f"force @ {effort} N, N", rows, "force_n"))
    return out


def stage_slip(arm, args, ask):
    from piper_core.stream_plan import cosine
    st = arm.status()
    if not st.started:
        raise SystemExit("D needs the arm started (piper-arm `s`, then close it): it moves joint 1")
    arm.set_gripper(arm.gripper_stroke, effort=args.effort)
    ask(f"D: object '{args.name}' between the open fingers; Enter closes on it at {args.effort} N, then joint 1 moves {args.amplitude} deg over {args.period} s and back")
    c = move(arm, 0.0, args.effort)
    print(f"  holding: {c['reason']} at {c['width_mm']:.2f} mm, force {c['force_n']} N")
    if c["reason"] != "stalled":
        print("  the gripper did not stall on an object: nothing to test.")
        arm.set_gripper(arm.gripper_stroke, effort=args.effort)
        return {"held": c}
    q0 = arm.positions()
    traj = cosine([q0[j] if j not in ("joint2", "joint3") else arm.zero()[j] for j in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")], 0,
                  math.radians(args.amplitude), args.period)
    samples, stop = [], threading.Event()

    def watch():
        t0 = time.time()
        while not stop.is_set():
            samples.append((time.time() - t0, arm.gripper_opening(), arm.gripper_force()))
            time.sleep(0.02)
    th = threading.Thread(target=watch, daemon=True)
    th.start()
    r = arm.execute(traj)
    stop.set()
    th.join()
    w = np.array([s[1] for s in samples if s[1] is not None]) * 1000
    f = [s[2] for s in samples if s[2] is not None]
    res = {"held": c, "move_ok": bool(r.ok), "move": r.message, "width_before_mm": c["width_mm"], "width_min_mm": float(w.min()), "width_max_mm": float(w.max()),
           "width_after_mm": float(w[-1]), "force_min_n": float(min(f)) if f else None, "force_max_n": float(max(f)) if f else None, "samples": len(w)}
    print(f"  move: ok={r.ok} {r.message}; width during the move {res['width_min_mm']:.2f} .. {res['width_max_mm']:.2f} mm (before {c['width_mm']:.2f}, after {res['width_after_mm']:.2f}); "
          f"force {res['force_min_n']} .. {res['force_max_n']} N")
    print("  SLIP: " + ("none (width within 0.5 mm)" if res["width_max_mm"] - res["width_min_mm"] < 0.5 else "the width changed by more than 0.5 mm: the object moved in the gripper (or the force read-back is noisy)"))
    ask("release the object: Enter opens the gripper")
    arm.set_gripper(arm.gripper_stroke, effort=args.effort)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stage", choices=("empty", "force", "object", "slip"))
    ap.add_argument("--name", default="object", help="what is in the gripper (written to the result)")
    ap.add_argument("--effort", type=float, default=1.0, help="N (empty, slip)")
    ap.add_argument("--efforts", type=float, nargs="+", default=[1.0, 2.0, 3.0], help="N, stage object")
    ap.add_argument("--cycles", type=int, default=5)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--amplitude", type=float, default=20.0, help="deg, stage slip")
    ap.add_argument("--period", type=float, default=2.0, help="s, stage slip (joint 1 cosine)")
    ap.add_argument("--canbus", "-b", default="can0")
    args = ap.parse_args()

    from piper_core import RealArm
    arm = RealArm.connect(args.canbus)
    print("arm status:", arm.status())

    def ask(text):
        input(f"\n>>> {text}  [Enter = go, Ctrl-C = exit] ")
    try:
        res = {"empty": stage_empty, "force": stage_force, "object": stage_object, "slip": stage_slip}[args.stage](arm, args, ask)
        OUT.mkdir(parents=True, exist_ok=True)
        path = OUT / f"{time.strftime('%Y%m%d_%H%M%S')}_{args.stage}_{args.name if args.stage in ('object', 'slip') else 'empty'}.json"
        path.write_text(json.dumps({"args": vars(args), "arm_status_started": arm.status().started, "result": res}, default=float))
        print("\nsaved", path)
    except KeyboardInterrupt:
        print("\nCtrl-C: stopping where it is.")
        arm.abort()
        if arm.status().started:
            arm.hold()
    finally:
        arm.close()


if __name__ == "__main__":
    main()
