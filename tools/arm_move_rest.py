#!/usr/bin/env python3
"""Real Piper: move to the folded rest pose with ONE slow move_j. First motion test; user present. This arm has NO e-stop button: keep a hand on the power switch/plug (cutting power drops a raised arm, fine at the rest pose).

    source scripts/env.sh
    python tools/arm_move_rest.py --dry-run            # reads only: shows start, target and the plan
    python tools/arm_move_rest.py [--speed 10]         # asks for Enter, then enables, moves, holds

Target = the 'rest' pose of piper.srdf (joint2 +0.01, joint3 -0.01 rad, others 0), clamped to the limits stored in the arm.
Uses move_j only (firmware-smoothed position-velocity mode). Speed is set explicitly (default 10 %).
Aborts, and holds the current position with move_j (never disables a raised arm), when: arm status != normal, an error code appears,
a joint leaves its stored limits by more than 2 deg, the distance to the target grows, or the move takes longer than the timeout.
Ctrl-C does the same. At the end it asks before disabling; disable only when the arm rests folded (disable drops a raised arm).
"""
import argparse
import math
import time

from arm_common import angles, deg, open_arm, resolve_channel

REST = [0.0, 0.01, -0.01, 0.0, 0.0, 0.0]
TOL_DEG = 0.5
MAX_START_DISTANCE_DEG = 30.0
LIMIT_SLACK_DEG = 2.0


def check_status(arm):
    st = arm.get_arm_status()
    if st is None:
        return "no arm status"
    if int(st.msg.arm_status) != 0 or int(st.msg.err_code) != 0:
        return f"arm_status={int(st.msg.arm_status)} err_code={int(st.msg.err_code)}"
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", "-i", help="CAN interface name (default: auto-detect)")
    ap.add_argument("--speed", type=int, default=10, help="speed percent, 1..30 (default 10)")
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--dry-run", action="store_true", help="only read and print the plan")
    args = ap.parse_args()
    if not 1 <= args.speed <= 30:
        raise SystemExit("--speed must be 1..30 for this test")

    arm, _, fw, profile = open_arm(resolve_channel(args.channel))
    enabled = False
    try:
        for _ in range(30):
            if arm.get_fps() > 0 and angles(arm) is not None:
                break
            time.sleep(0.1)
        start = angles(arm)
        if start is None:
            raise SystemExit("no joint feedback")
        print("firmware:", fw["software_version"], " hardware:", fw["hardware_version"], " SDK profile:", profile)
        bad = check_status(arm)
        if bad:
            raise SystemExit(f"not starting: {bad}")

        lim = {}
        for i in range(1, 7):
            m = arm.get_joint_angle_vel_limits(i, timeout=2.0)
            if m is None:
                raise SystemExit(f"could not read the stored limits of joint {i}")
            lim[i] = (m.msg.min_angle_limit, m.msg.max_angle_limit)
        target = [min(max(t, lim[i + 1][0] + 0.01), lim[i + 1][1] - 0.01) for i, t in enumerate(REST)]  # 0.57 deg margin
        dist = max(abs(deg(a - b)) for a, b in zip(start, target))
        print("joint  start(deg)  target(deg)  delta(deg)  stored limits(deg)")
        for i in range(6):
            print(f"  {i + 1}    {deg(start[i]):8.2f}   {deg(target[i]):8.2f}   {deg(target[i] - start[i]):8.2f}    "
                  f"[{deg(lim[i + 1][0]):7.1f}, {deg(lim[i + 1][1]):6.1f}]")
        print(f"largest joint move {dist:.1f} deg at {args.speed} % speed")
        if dist > MAX_START_DISTANCE_DEG:
            raise SystemExit(f"start is {dist:.1f} deg from the rest pose (> {MAX_START_DISTANCE_DEG}): not a small move, aborting")
        if args.dry_run:
            print("dry run: nothing was sent except read requests.")
            return

        input("Arm clear, hand on the power switch (no e-stop button), you are watching? Press Enter to ENABLE and MOVE (Ctrl-C cancels)... ")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and not arm.enable():
            time.sleep(0.05)
        enabled = all(arm.get_joint_enable_status(i) for i in range(1, 7))
        print("enabled:", enabled)
        if not enabled:
            raise SystemExit("could not enable all joints")
        arm.set_speed_percent(args.speed)
        time.sleep(0.3)
        arm.move_j(target)
        print("move_j sent")

        t0, best, hold_reason = time.monotonic(), None, None
        while True:
            time.sleep(0.1)
            cur = angles(arm)
            bad = check_status(arm)
            err = max(abs(deg(a - b)) for a, b in zip(cur, target))
            if bad:
                hold_reason = bad
            elif any(deg(c - lim[i + 1][0]) < -LIMIT_SLACK_DEG - 3.0 or deg(c - lim[i + 1][1]) > LIMIT_SLACK_DEG + 3.0
                     for i, c in enumerate(cur)):
                hold_reason = "a joint is outside its stored limits"
            elif best is not None and err > best + 1.0:
                hold_reason = f"moving away from the target ({err:.1f} deg, best {best:.1f})"
            elif time.monotonic() - t0 > args.timeout:
                hold_reason = "timeout"
            best = err if best is None else min(best, err)
            print(f"  t={time.monotonic() - t0:4.1f}s  max error {err:5.2f} deg   joints(deg) {[round(deg(c), 1) for c in cur]}")
            if hold_reason:
                break
            if err < TOL_DEG:
                print("target reached")
                break
        if hold_reason:
            print("ABORT:", hold_reason, "-> holding the current position with move_j")
            arm.move_j(angles(arm))
        time.sleep(1.0)
        print("final (deg):", [round(deg(a), 2) for a in angles(arm)])
        input("Press Enter to DISABLE (only if the arm rests folded; Ctrl-C leaves it enabled and holding)... ")
        for _ in range(10):
            arm.disable()
            if not any(arm.get_joint_enable_status(i) for i in range(1, 7)):
                break
            time.sleep(0.1)
        enabled = any(arm.get_joint_enable_status(i) for i in range(1, 7))
        print("disabled:", not enabled)
    except KeyboardInterrupt:
        if enabled:
            print("\nCtrl-C: holding the current position with move_j; the arm stays ENABLED (run arm_enable_test.py or the GUI to disable).")
            cur = angles(arm)
            if cur:
                arm.move_j(cur)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
