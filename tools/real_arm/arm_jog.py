#!/usr/bin/env python3
"""Real Piper: move ONE joint (default joint 5) in small steps that you type, very slowly. Everything else is held where it is.

    source scripts/env.sh
    python tools/real_arm/arm_jog.py --dry-run          # preflight only: reads, sends nothing
    python tools/real_arm/arm_jog.py                    # joint 5, speed 3 %, step 0.5 deg, range 10..23 deg

At the prompt:   Enter or +   one step up (larger angle)    -   one step down    <number>   absolute angle in degrees (at most 3 deg from here)
                 s            status (angles, faults)         q   quit: the motors STAY ON, the arm keeps holding

It never disables the motors (use tools/real_arm/arm_stop.py, with the arm supported by hand). Every move is one move_j with the other joints at their
present angle; joints resting outside their stored range (2 and 3 in the slumped pose) are aimed 0.01 rad inside it, which moves them about 2.7 deg once,
at the first command: you are asked before that. A real fault (status, error code, driver bits) or Ctrl-C holds the present pose.
Stored limits and the 10..23 deg range (--min/--max, the slumped j5 is 22.7-24 deg: do not push past the stop) are checked before every command.
This arm has no e-stop button: keep a hand on the power switch.
"""
import argparse
import math
import time

from arm_common import angles, deg, open_arm, resolve_canbus
from arm_zero import MODE_J, drv_faults, hold, status_problem, wait_mode

MARGIN = 0.01          # rad, same as the guard's limit_margin
SETTLE_DEG, SETTLE_S, MOVE_TIMEOUT = 0.3, 0.5, 15.0
MAX_STEP_DEG = 3.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b")
    ap.add_argument("--joint", type=int, default=5, choices=range(1, 7), metavar="1..6")
    ap.add_argument("--speed", type=int, default=3, help="speed percent 1..20 (default 3; 10 %% is about 17 deg/s)")
    ap.add_argument("--step", type=float, default=0.5, help="degrees per + / - (default 0.5, max 3)")
    ap.add_argument("--min", type=float, default=10.0, dest="lo", help="lowest angle allowed for the joint, degrees (default 10)")
    ap.add_argument("--max", type=float, default=23.0, dest="hi", help="highest angle allowed for the joint, degrees (default 23)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    if not 1 <= args.speed <= 20:
        raise SystemExit("--speed must be 1..20")
    if not 0 < args.step <= MAX_STEP_DEG:
        raise SystemExit(f"--step must be 0..{MAX_STEP_DEG}")
    j = args.joint - 1

    try:
        arm, _, fw, profile = open_arm(resolve_canbus(args.canbus))
    except RuntimeError as e:
        raise SystemExit(f"{e}\nNothing was changed.")
    enabled, lim = False, {}
    try:
        cur = None
        for _ in range(30):
            cur = angles(arm)
            if cur is not None:
                break
            time.sleep(0.1)
        st = arm.get_arm_status()
        if cur is None or st is None:
            raise SystemExit("no joint/status feedback: not starting")
        problems = []
        if int(st.msg.ctrl_mode) not in (0, 1):
            problems.append(f"control mode {int(st.msg.ctrl_mode)}")
        if int(st.msg.teach_status) != 0:
            problems.append("teach mode is on")
        if status_problem(arm):
            problems.append(status_problem(arm))
        if drv_faults(arm):
            problems.append("driver faults " + ", ".join(drv_faults(arm)))
        for i in range(1, 7):
            r = arm.get_joint_angle_vel_limits(i, timeout=2.0)
            if r is None:
                problems.append(f"stored limits of joint {i} unreadable")
            else:
                lim[i] = (r.msg.min_angle_limit, r.msg.max_angle_limit)
        if problems:
            raise SystemExit("not starting: " + "; ".join(problems))

        # fixed targets for the other joints: present angle, inside the stored range by the margin
        base = [min(max(a, lim[i + 1][0] + MARGIN), lim[i + 1][1] - MARGIN) for i, a in enumerate(cur)]
        jlo = max(math.radians(args.lo), lim[j + 1][0] + MARGIN)
        jhi = min(math.radians(args.hi), lim[j + 1][1] - MARGIN)
        shifts = [i + 1 for i in range(6) if i != j and abs(base[i] - cur[i]) > 1e-4]
        enabled_now = [bool(arm.get_joint_enable_status(i)) for i in range(1, 7)]
        print(f"firmware {fw['software_version']} · profile {profile} · motors enabled {enabled_now}")
        print("joint (deg): now " + str([round(deg(a), 2) for a in cur]))
        print(f"jogging joint {args.joint}: {deg(cur[j]):.2f} deg now, allowed {deg(jlo):.1f}..{deg(jhi):.1f}, step {args.step} deg, speed {args.speed} %")
        if shifts:
            print("first command also moves joint(s) " + ", ".join(map(str, shifts)) + " by "
                  + ", ".join(f"{deg(base[i - 1] - cur[i - 1]):+.2f} deg" for i in shifts) + " (into the stored range)")
        if not jlo <= cur[j] <= jhi:
            raise SystemExit(f"joint {args.joint} is at {deg(cur[j]):.2f} deg, outside the allowed {deg(jlo):.1f}..{deg(jhi):.1f}: change --min/--max deliberately")
        if args.dry_run:
            print("dry run: only read requests were sent.")
            return

        input("Arm clear, hand on the power switch? Enter to CONFIGURE and ENABLE (Ctrl-C cancels)... ")
        arm.set_speed_percent(args.speed)
        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.J)
        if not wait_mode(arm, MODE_J):
            raise SystemExit("the arm did not report CAN control + MOVE_J: nothing enabled")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and not arm.enable():
            time.sleep(0.05)
        enabled = any(arm.get_joint_enable_status(i) for i in range(1, 7))
        if not all(arm.get_joint_enable_status(i) for i in range(1, 7)):
            raise SystemExit("could not enable all six joints: not moving (use arm_stop.py to release)")
        target = list(base)
        target[j] = cur[j]
        first = True
        print("enabled. Commands: Enter/+ up, - down, <deg> absolute, s status, q quit (motors stay on)")
        while True:
            now = angles(arm)
            ans = input(f"j{args.joint} = {deg(now[j]):.2f} deg > ").strip().lower()
            if ans == "q":
                break
            if ans == "s":
                print("joints (deg)", [round(deg(a), 2) for a in now], "· faults:", status_problem(arm) or drv_faults(arm) or "none")
                continue
            try:
                new = now[j] + math.radians(args.step) if ans in ("", "+") else now[j] - math.radians(args.step) if ans == "-" else math.radians(float(ans))
            except ValueError:
                print("not understood")
                continue
            if abs(new - now[j]) > math.radians(MAX_STEP_DEG) + 1e-9:
                print(f"refused: more than {MAX_STEP_DEG} deg from here; do it in steps")
                continue
            if not jlo - 1e-9 <= new <= jhi + 1e-9:
                print(f"refused: {deg(new):.2f} deg is outside {deg(jlo):.1f}..{deg(jhi):.1f}")
                continue
            if first and shifts and input("this also moves the other joint(s) listed above. Enter to go, anything else cancels: ").strip():
                continue
            first = False
            target[j] = new
            arm.move_j(target)
            t0, ok_since, reason = time.monotonic(), None, None
            while True:
                time.sleep(0.1)
                pos, t = angles(arm), time.monotonic()
                reason = status_problem(arm) or (f"driver faults {drv_faults(arm)}" if drv_faults(arm) else None)
                if reason or t - t0 > MOVE_TIMEOUT:
                    reason = reason or "timeout"
                    break
                err = max(abs(deg(a - b)) for a, b in zip(pos, target))
                ok_since = (ok_since or t) if err < SETTLE_DEG else None
                if ok_since and t - ok_since >= SETTLE_S:
                    break
            if reason:
                print("ABORT:", reason, "-> holding; motors stay on")
                hold(arm, lim)
                break
            print(f"  reached: j{args.joint} {deg(pos[j]):.2f} deg (asked {deg(new):.2f}), others {[round(deg(a), 2) for i, a in enumerate(pos) if i != j]}")
        print("leaving the motors ENABLED and holding. Release: tools/real_arm/arm_stop.py (support the arm).")
    except KeyboardInterrupt:
        if enabled:
            print("\nCtrl-C: holding the current position; the motors stay ENABLED.")
            hold(arm, lim)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
