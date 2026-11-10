#!/usr/bin/env python3
"""RealArm smoke test, stage by stage, on the real arm. The user is present, hand on the power switch, and presses Enter before every stage that moves anything.

    source scripts/env.sh
    python tools/real_arm/realarm_smoke.py [--speed 10]

Stages (each prints what it did and what it measured; Ctrl-C holds the arm where it is and exits):
  0  connect, status(): read only                                    4  hold(), then a 5 deg move of joint 1 with move_to() and back
  1  start(): clear an e-stop if any, motors on, move to zero       5  execute() of a 10 deg cosine on joint 1 over 4 s at 25 ms
  2  state(): joints, gripper width and force                       6  gripper: open, close, with set_gripper() and its result
  3  guard: a command outside the limits and a bad trajectory      7  abort(): start a slow execute and abort it from a timer thread
     are refused and nothing moves                                  8  stop(): zero, park, e-stop, reset; status() after
The arm is the one piper_core.RealArm drives for piper_pick, so this is the first time these calls run on hardware. Results to the hardware log (docs/real_arm/arms/piper-01.md).
"""
import argparse
import math
import threading
import time

import numpy as np

from piper_core import RealArm
from piper_core.backend import Trajectory
from piper_core.real_arm import JOINTS
from piper_core.stream_metrics import analyze, format_metrics
from piper_core.stream_plan import cosine


def ask(text):
    input(f"\n>>> {text}  [Enter = go, Ctrl-C = hold and exit] ")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b", default="can0")
    ap.add_argument("--speed", type=int, default=10)
    args = ap.parse_args()
    arm = RealArm.connect(args.canbus)
    arm.speed_percent = args.speed
    try:
        print("0. status:", arm.status())
        ask(f"1. start() at {args.speed} %: the arm goes to zero")
        r = arm.start(args.speed)
        print("   ", r.ok, r.message)
        if not r:
            return
        print("2. state:", np.round(np.degrees(arm.state().q), 2).tolist(), "deg · gripper", arm.state().gripper_width, "m", arm.state().gripper_force, "N")

        print("3. guard refusals (nothing should move):")
        before = arm.positions()
        bad = {j: v for j, v in arm.zero().items()}
        bad["joint2"] = -0.5
        r = arm.move_to(bad)
        print("    move_to outside the limits ->", r.ok, r.message)
        r = arm.execute(Trajectory.from_pairs([(0.0, list(arm.zero().values())), (0.025, [9.0] + list(arm.zero().values())[1:])]))
        print("    execute with a bad point ->", r.ok, r.message)
        moved = max(abs(arm.positions()[j] - before[j]) for j in JOINTS)
        print(f"    max joint change {math.degrees(moved):.3f} deg (must be ~0)")

        ask("4. hold(), then joint 1 +5 deg with move_to() and back")
        arm.hold()
        z = arm.zero()
        for target in ({**z, "joint1": math.radians(5)}, z):
            r = arm.move_to(target)
            print("   ", r.ok, r.message, "-> joint1", round(math.degrees(arm.positions()["joint1"]), 2), "deg")

        ask("5. execute(): joint 1, 10 deg cosine over 4 s, 25 ms setpoints")
        traj = cosine([arm.positions()[j] if j != "joint2" and j != "joint3" else arm.zero()[j] for j in JOINTS], 0, math.radians(10), 4.0)
        r = arm.execute(traj, mode="fixed")
        print("   ", r.ok, r.message)
        if arm.last_stream and len(arm.last_stream["t"]) > 5:
            print(format_metrics(analyze(traj, arm.last_stream)))

        ask("6. gripper: open, then close (fingers clear)")
        for w in (arm.gripper_stroke, 0.0):
            got = arm.set_gripper(w)
            print(f"    target {w * 1000:.0f} mm -> {got * 1000:.1f} mm, {arm.last_gripper.reason}, force {arm.last_gripper.force}")

        ask("7. abort(): a slow 20 deg cosine over 10 s, aborted after 2 s")
        slow = cosine([arm.positions()[j] if j not in ("joint2", "joint3") else arm.zero()[j] for j in JOINTS], 0, math.radians(20), 10.0)
        threading.Timer(2.0, arm.abort).start()
        r = arm.execute(slow)
        print("   ", r.ok, r.reason, "(want False, stopped) · the arm holds at", round(math.degrees(arm.positions()["joint1"]), 2), "deg")
        arm.move_to(arm.zero())

        ask("8. stop(): zero, park, e-stop (damped lowering), reset. Support the arm")
        r = arm.stop(args.speed)
        print("   ", r.ok, r.message)
        print("    status:", arm.status())
    except KeyboardInterrupt:
        print("\nCtrl-C: holding where the arm is (motors stay on).")
        arm.abort()
        arm.hold()
    finally:
        arm.close()


if __name__ == "__main__":
    main()
