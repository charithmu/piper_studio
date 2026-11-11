#!/usr/bin/env python3
"""Real Piper: move between the hard-coded poses (zero, park, ready) with move_j, on an arm that is already ENABLED. The pose table is POSES below.

    source scripts/env.sh
    python tools/real_arm/arm_pose.py --list
    python tools/real_arm/arm_pose.py zero  [--speed 10]       # shows the move, asks for Enter, moves, watches, holds
    python tools/real_arm/arm_pose.py park  [--dry-run]
    python tools/real_arm/arm_pose.py ready --speed 10          # a big move (joints 2, 3, 5 by about 57 deg): look at the printed table first

It does not enable or disable the motors and does not leave the slumped pose: tools/real_arm/arm_session.py start does that (the only way out of the slumped pose is its one move
to zero). Refuses when: motors are not all enabled, any joint is outside its stored limits, a fault is reported, a target is outside the limits (or nearer than 0.01 rad to one).
Stops and HOLDS the present pose on a real fault (status, error code, driver bits), a timeout or Ctrl-C; the motors stay enabled. Speed is 10 % by default (about 17 deg/s on one joint),
at most 50. The pose values equal the SRDF group states (test_named_poses); `park` is only ever the step before disabling.
This arm has no e-stop button: keep a hand on the power switch (or use tools/real_arm/arm_stop.py).
"""
import argparse
import math
import time

from piper_tools.arm_common import angles, deg, open_arm, resolve_canbus
from piper_tools.arm_zero import MODE_J, drv_faults, hold, status_problem, wait_mode

MARGIN = 0.01                      # rad: the guard's limit_margin
DONE_DEG, DONE_HOLD_S = 0.5, 1.0
PARK_J5 = 0.3840                   # rad (22.0 deg), see docs/real_arm/arms/piper-01.md "Park pose"
POSES = {                          # joint1..joint6, rad. Must equal the SRDF group states (test_named_poses)
    "zero": [0.0, 0.01, -0.01, 0.0, 0.0, 0.0],
    "park": [0.0, 0.01, -0.01, 0.0, PARK_J5, 0.0],
    "ready": [0.0, 1.0, -1.0, 0.0, 1.0, 0.0],
}


def read_state(arm):
    """(joints, limits, enabled list, problems). Reads only. Raises SystemExit if there is no feedback."""
    cur = st = None
    for _ in range(50):                      # up to 5 s for BOTH the joint angles and the status frame
        cur, st = angles(arm), arm.get_arm_status()
        if cur is not None and st is not None:
            break
        time.sleep(0.1)
    if cur is None or st is None:
        missing = " and ".join(n for n, v in (("joint angles", cur), ("arm status", st)) if v is None)
        raise SystemExit(f"no feedback: {missing} not received in 5 s. Is the arm powered and not in an e-stop, and is another program using the bus? Nothing was sent.")
    problems = []
    if int(st.msg.ctrl_mode) not in (0, 1):
        problems.append(f"control mode {int(st.msg.ctrl_mode)} (not standby or CAN control)")
    if int(st.msg.teach_status) != 0:
        problems.append("teach mode is on")
    if status_problem(arm):
        problems.append(status_problem(arm))
    if drv_faults(arm):
        problems.append("driver faults " + ", ".join(drv_faults(arm)))
    lim = {}
    for i in range(1, 7):
        r = arm.get_joint_angle_vel_limits(i, timeout=2.0)
        if r is None:
            problems.append(f"stored limits of joint {i} unreadable")
        else:
            lim[i] = (r.msg.min_angle_limit, r.msg.max_angle_limit)
    enabled = [bool(arm.get_joint_enable_status(i)) for i in range(1, 7)]
    return cur, lim, enabled, problems


def outside_limits(cur, lim):
    return [i + 1 for i, a in enumerate(cur) if i + 1 in lim and not lim[i + 1][0] - 1e-3 <= a <= lim[i + 1][1] + 1e-3]


def target_problems(goal, lim):
    return [f"joint{i + 1} target {deg(a)} deg is outside [{deg(lim[i + 1][0] + MARGIN)}, {deg(lim[i + 1][1] - MARGIN)}] (limits +- margin)"
            for i, a in enumerate(goal) if not lim[i + 1][0] + MARGIN - 1e-9 <= a <= lim[i + 1][1] - MARGIN + 1e-9]


def configure(arm, speed):
    """Speed first (every later mode frame carries it), then CAN control + MOVE_J, read back."""
    arm.set_speed_percent(int(speed))
    arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.J)
    return wait_mode(arm, MODE_J)


def go(arm, lim, goal, label, speed, say=print):
    """One move_j to `goal`, watched at 10 Hz. Returns None when reached (all joints within DONE_DEG for DONE_HOLD_S), else the reason (the arm is then HOLDING)."""
    cur = angles(arm)
    dist = max(abs(a - b) for a, b in zip(cur, goal))
    timeout = 8.0 + 2.0 * dist / (speed / 100.0 * 3.0)      # twice the nominal time (100 % = 3.0 rad/s, measured at 10 %) plus 8 s
    arm.move_j(goal)
    say(f"move_j sent: {label}  (largest move {math.degrees(dist):.1f} deg, speed {speed} %, timeout {timeout:.0f} s)")
    t0, ok_since = time.monotonic(), None
    while True:
        time.sleep(0.1)
        now, t = angles(arm), time.monotonic()
        err = max(abs(deg(a - b)) for a, b in zip(now, goal))
        faults = drv_faults(arm)
        reason = status_problem(arm) or (f"driver faults {faults}" if faults else None) or ("timeout" if t - t0 > timeout else None)
        say(f"  t={t - t0:4.1f}s  max error {err:5.2f} deg  joints(deg) {[round(deg(a), 1) for a in now]}")
        if reason:
            say(f"ABORT: {reason} -> holding the current position; the motors stay enabled")
            hold(arm, lim)
            return reason
        ok_since = (ok_since or t) if err < DONE_DEG else None
        if ok_since and t - ok_since >= DONE_HOLD_S:
            say(f"{label} reached")
            return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pose", nargs="?", choices=sorted(POSES))
    ap.add_argument("--list", action="store_true", help="print the pose table and exit (no arm needed)")
    ap.add_argument("--canbus", "-b")
    ap.add_argument("--speed", type=int, default=10, help="speed percent 1..50 (default 10)")
    ap.add_argument("--dry-run", action="store_true", help="preflight and plan only: reads, sends nothing")
    args = ap.parse_args()
    if args.list or not args.pose:
        for name, q in POSES.items():
            print(f"{name:6s} " + "  ".join(f"{deg(a):7.2f}" for a in q) + "  deg (joint1..6)")
        return
    if not 1 <= args.speed <= 50:
        raise SystemExit("--speed must be 1..50")
    goal = POSES[args.pose]
    try:
        arm, _, fw, profile = open_arm(resolve_canbus(args.canbus))
    except RuntimeError as e:
        raise SystemExit(f"{e}\nNothing was changed.")
    lim, running = {}, False
    try:
        cur, lim, enabled, problems = read_state(arm)
        problems += target_problems(goal, lim) if lim else []
        if outside_limits(cur, lim):
            problems.append(f"joint(s) {outside_limits(cur, lim)} outside their stored limits (slumped pose): run arm_session.py start first")
        if not all(enabled):
            problems.append(f"motors not all enabled {enabled}: run arm_session.py start first (this tool never enables)")
        print(f"firmware {fw['software_version']} · profile {profile}")
        print("joint   now(deg)  " + f"{args.pose}(deg)  move(deg)")
        for i in range(6):
            print(f"  {i + 1}    {deg(cur[i]):8.2f}   {deg(goal[i]):8.2f}   {deg(goal[i] - cur[i]):8.2f}")
        if problems:
            raise SystemExit("not moving: " + "; ".join(problems))
        dist = max(abs(deg(a - b)) for a, b in zip(cur, goal))
        print(f"largest move {dist:.1f} deg at {args.speed} % speed")
        if dist < DONE_DEG:
            print(f"already at {args.pose}: nothing to do.")
            return
        if args.dry_run:
            print("dry run: only read requests were sent.")
            return
        input(f"Arm clear, hand on the power switch? Enter to MOVE to {args.pose} (Ctrl-C cancels)... ")
        if not configure(arm, args.speed):
            raise SystemExit("the arm did not report CAN control + MOVE_J: not moving")
        running = True
        go(arm, lim, goal, f"{args.pose} pose", args.speed)
        running = False
        fin = angles(arm)
        print("final (deg):", [round(deg(a), 2) for a in fin], f"· error {max(abs(deg(a - b)) for a, b in zip(fin, goal)):.2f} deg · motors still ENABLED")
    except KeyboardInterrupt:
        if running:
            print("\nCtrl-C: holding the current position; the motors stay ENABLED.")
            hold(arm, lim)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
