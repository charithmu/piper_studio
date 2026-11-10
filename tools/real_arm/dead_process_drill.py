#!/usr/bin/env python3
"""Dead-process drill (R1.7): what the arm does when the program driving it dies. The user kills it from another terminal, hand on the power switch.

    source scripts/env.sh
    python tools/real_arm/dead_process_drill.py hold        # starts the arm, holds at zero, prints its pid; you run `kill -9 <pid>`
    python tools/real_arm/dead_process_drill.py moving      # starts the arm, then runs a slow 30 deg / 12 s cosine on joint 1; you run `kill -9 <pid>` in the middle

Expected (vendor firmware behaviour, not yet confirmed on this arm): the motors stay enabled and the arm holds the last target it was given. After the kill, look at the arm and run
    python tools/real_arm/arm_joints.py        # passive: does the pose stay put? press q
    python tools/real_arm/arm_session.py status   # reads only: motors enabled?
then end the session by hand: `piper-arm` key t (or key e for the emergency stop, d to disable). In `moving` the arm should stop within a fraction of a second at (or a few degrees ahead of) where it was.
If the arm keeps moving, or goes limp, or drifts: cut the power and tell me exactly what it did. -9 is the worst case (no cleanup code runs); also try plain `kill <pid>` (SIGTERM) and Ctrl-C once.
"""
import math
import os
import sys
import time

from piper_core import RealArm
from piper_core.stream_plan import cosine

mode = sys.argv[1] if len(sys.argv) > 1 else "hold"
if mode not in ("hold", "moving"):
    raise SystemExit(__doc__)
arm = RealArm.connect("can0")
input(f"Arm clear, hand on the power switch? Enter to START the arm (to zero) and run the '{mode}' drill... ")
r = arm.start(10)
print("start:", r.ok, r.message, flush=True)
if not r:
    raise SystemExit(1)
print(f"\nPID {os.getpid()}  <- in another terminal: kill -9 {os.getpid()}   (also try: kill {os.getpid()}, and Ctrl-C)\n", flush=True)
if mode == "hold":
    while True:
        q = arm.positions()
        print(f"holding at zero, joint1 {math.degrees(q['joint1']):7.3f} deg, motors on: {arm.status().enabled}", flush=True)
        time.sleep(1.0)
else:
    q0 = arm.positions()
    traj = cosine([q0["joint1"]] + [arm.zero()[j] for j in ("joint2", "joint3", "joint4", "joint5", "joint6")], 0, math.radians(30), 12.0)
    print("moving: joint 1 +30 deg over 12 s, then back (second half). Kill it in the middle.", flush=True)
    res = arm.execute(traj)
    print("finished without being killed:", res.ok, res.message, flush=True)
