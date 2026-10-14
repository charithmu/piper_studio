#!/usr/bin/env python3
"""Real Piper: put the arm in a known state, then move it to the zero pose with ONE slow move_j. Replaces arm_move_rest.py.

    source scripts/env.sh
    python tools/arm_zero.py --dry-run         # preflight and plan only: reads, nothing is changed
    python tools/arm_zero.py [--speed 10]      # asks for Enter, then configures, enables, moves, holds, asks before disabling

Zero pose = all joints 0 (what the vendor's go-zero demo and the GUI's "Go Zero" use). By default joints whose stored limit IS 0 (joint 2 min, joint 3 max)
are aimed 0.5 deg inside that limit so the target never sits on the limit; --exact sends pure zeros like the vendor demo.

Sequence (each step verified before the next; nothing is sent until you press Enter):
  0. preflight, reads only: arm is a slave, status normal, no error, teach off, not e-stopped, stored limits readable, start within --max-move-deg of the target
  1. known configuration: SDK joint-limit clamp on (local), speed percent set FIRST (the SDK's mode frame carries a speed, default 50 %), then CAN control + MOVE_J;
     read back that the arm reports MOVE_J
  2. enable all six joints; latch the target at the CURRENT position (move_j(current)) so a stale target cannot pull the arm; require 1 s of no drift
  3. one move_j to the target at the chosen speed; watched at 10 Hz
  4. hold; then ask before disabling (disable drops a raised arm: the zero pose is the folded rest pose, so it is safe there)
Aborts, and HOLDS the current position with move_j (it never disables a raised arm), on: status != normal, error code, driver fault bits, a joint rate above
1.2 rad/s, a joint more than 5 deg outside its stored limits, moving away from the target, or timeout. Ctrl-C does the same.
Not used on purpose: reset() (powers the arm off), move_js / MIT / CPV, firmware Cartesian moves, any set_* write to stored parameters.
This arm has no e-stop button: keep a hand on the power switch.
"""
import argparse
import math
import time

from arm_common import angles, deg, open_arm, resolve_canbus

MODE_J = 1
RATE_LIMIT = 1.2          # rad/s between two 0.1 s samples: about four times the expected rate at 10 % speed, so something is running away
OUTSIDE_LIMIT_DEG = 5.0
DONE_DEG, DONE_HOLD_S = 0.5, 1.0
MARGIN_DEG = 0.5


def drv_faults(arm):
    bad = []
    for i in range(1, 7):
        d = arm.get_driver_states(i)
        if d is None:
            continue
        for k in ("voltage_too_low", "motor_overheating", "driver_overcurrent", "driver_overheating", "collision_status", "driver_error_status", "stall_status"):
            if getattr(d.msg.foc_status, k, False):
                bad.append(f"j{i}:{k}")
    return bad


def status_problem(arm):
    st = arm.get_arm_status()
    if st is None:
        return "no arm status"
    m = st.msg
    if int(m.arm_status) != 0 or int(m.err_code) != 0:
        return f"arm_status={int(m.arm_status)} err_code={int(m.err_code)}"
    return None


def wait_mode(arm, want, secs=2.5):
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        st = arm.get_arm_status()
        if st is not None and int(st.msg.mode_feedback) == want and int(st.msg.ctrl_mode) == 1:
            return True
        time.sleep(0.05)
    return False


def hold(arm):
    cur = angles(arm)
    if cur:
        arm.move_j(cur)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b", help="CAN bus name, e.g. can0 (default: auto-detect)")
    ap.add_argument("--speed", type=int, default=10, help="speed percent, 1..30 (default 10)")
    ap.add_argument("--timeout", type=float, default=30.0, help="seconds allowed for the move")
    ap.add_argument("--max-move-deg", type=float, default=35.0, help="refuse if any joint must move more than this")
    ap.add_argument("--exact", action="store_true", help="send pure zeros (no margin inside the stored limits)")
    ap.add_argument("--stay", action="store_true", help="leave the motors enabled at the end (no disable prompt)")
    ap.add_argument("--yes", action="store_true", help="skip the Enter prompts")
    ap.add_argument("--dry-run", action="store_true", help="preflight and plan only")
    args = ap.parse_args()
    if not 1 <= args.speed <= 30:
        raise SystemExit("--speed must be 1..30 for this tool")

    try:
        arm, _, fw, profile = open_arm(resolve_canbus(args.canbus))
    except RuntimeError as e:
        raise SystemExit(f"{e}\nNothing was changed.")
    enabled = False
    try:
        # ---- 0. preflight (reads only)
        print(f"firmware {fw['software_version']} · hardware {fw['hardware_version']} · SDK profile {profile}")
        cur = None
        for _ in range(30):
            cur = angles(arm)
            if cur is not None:
                break
            time.sleep(0.1)
        st = arm.get_arm_status()
        problems = []
        if cur is None or st is None:
            raise SystemExit("no joint/status feedback: not starting")
        m = st.msg
        if int(m.ctrl_mode) not in (0, 1):
            problems.append(f"control mode {int(m.ctrl_mode)} (not standby/CAN control: teach, master or other)")
        if int(m.teach_status) != 0:
            problems.append(f"teach state {int(m.teach_status)} (stop teach mode first)")
        bad = status_problem(arm)
        if bad:
            problems.append(bad)
        if drv_faults(arm):
            problems.append("driver fault bits: " + ", ".join(drv_faults(arm)))
        lim = {}
        for i in range(1, 7):
            r = arm.get_joint_angle_vel_limits(i, timeout=2.0)
            if r is None:
                problems.append(f"stored limits of joint {i} unreadable")
            else:
                lim[i] = (r.msg.min_angle_limit, r.msg.max_angle_limit)
        if problems:
            raise SystemExit("not starting: " + "; ".join(problems))

        margin = 0.0 if args.exact else math.radians(MARGIN_DEG)
        target = [min(max(0.0, lim[i][0] + (margin if lim[i][0] == 0.0 else 0.0)), lim[i][1] - (margin if lim[i][1] == 0.0 else 0.0)) for i in range(1, 7)]
        dist = max(abs(deg(a - b)) for a, b in zip(cur, target))
        print("joint   now(deg)  target(deg)  move(deg)   stored limits(deg)")
        for i in range(6):
            print(f"  {i + 1}    {deg(cur[i]):8.2f}   {deg(target[i]):8.2f}   {deg(target[i] - cur[i]):8.2f}    [{deg(lim[i + 1][0]):7.1f}, {deg(lim[i + 1][1]):6.1f}]")
        print(f"largest move {dist:.1f} deg at {args.speed} % speed · now: mode_feedback {int(m.mode_feedback)}, "
              f"enabled {[bool(arm.get_joint_enable_status(i)) for i in range(1, 7)]}")
        if dist > args.max_move_deg:
            raise SystemExit(f"largest move {dist:.1f} deg exceeds --max-move-deg {args.max_move_deg}: not a small move, aborting")
        if dist < DONE_DEG:
            print("already at the zero pose: nothing to do (nothing was changed).")
            return
        if args.dry_run:
            print("dry run: only read requests were sent.")
            return

        if not args.yes:
            input("Arm clear, hand on the power switch, you are watching? Press Enter to CONFIGURE, ENABLE and MOVE (Ctrl-C cancels)... ")

        # ---- 1. known configuration
        arm.set_joint_limits_enabled(True)               # SDK-side clamp to the model limits (local, not stored in the arm)
        arm.set_speed_percent(args.speed)                # first: it updates the speed carried by every later mode frame
        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.J)   # CAN control + MOVE_J
        if not wait_mode(arm, MODE_J):
            raise SystemExit("the arm did not report CAN control + MOVE_J: stopping before enabling (nothing is enabled)")
        print(f"configured: CAN control, MOVE_J (read back), speed {args.speed} %, SDK joint-limit clamp on")

        # ---- 2. enable and latch
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and not arm.enable():
            time.sleep(0.05)
        enabled = any(arm.get_joint_enable_status(i) for i in range(1, 7))
        if not all(arm.get_joint_enable_status(i) for i in range(1, 7)):
            raise SystemExit("could not enable all six joints: holding, not moving")
        hold(arm)                                          # latch the target at the present position
        base = angles(arm)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            now = angles(arm)
            drift = max(abs(deg(a - b)) for a, b in zip(now, base))
            bad = status_problem(arm) or ("driver faults " + ",".join(drv_faults(arm)) if drv_faults(arm) else None)
            if bad or drift > 0.5:
                hold(arm)
                raise SystemExit(f"after enabling: {bad or f'drift {drift:.2f} deg'}: holding, not moving (the arm stays enabled)")
            time.sleep(0.1)
        print("enabled and latched: no drift for 1 s")

        # ---- 3. the move
        arm.move_j(target)
        print("move_j sent")
        t0, best, prev, prev_t, ok_since, reason = time.monotonic(), None, cur, time.monotonic(), None, None
        while True:
            time.sleep(0.1)
            now, t = angles(arm), time.monotonic()
            err = max(abs(deg(a - b)) for a, b in zip(now, target))
            rate = max(abs(a - b) for a, b in zip(now, prev)) / max(t - prev_t, 1e-3)
            prev, prev_t = now, t
            faults = drv_faults(arm)
            reason = (status_problem(arm) or (f"driver faults {faults}" if faults else None)
                      or (f"joint rate {rate:.2f} rad/s" if rate > RATE_LIMIT else None)
                      or ("a joint is outside its stored limits" if any(deg(a - lim[i + 1][0]) < -OUTSIDE_LIMIT_DEG or deg(a - lim[i + 1][1]) > OUTSIDE_LIMIT_DEG for i, a in enumerate(now)) else None)
                      or (f"moving away from the target ({err:.1f} deg, best {best:.1f})" if best is not None and err > best + 1.0 else None)
                      or ("timeout" if t - t0 > args.timeout else None))
            best = err if best is None else min(best, err)
            print(f"  t={t - t0:4.1f}s  max error {err:5.2f} deg  rate {rate:4.2f} rad/s  joints(deg) {[round(deg(a), 1) for a in now]}")
            if reason:
                break
            ok_since = (ok_since or t) if err < DONE_DEG else None
            if ok_since and t - ok_since >= DONE_HOLD_S:
                print("zero pose reached")
                break
        if reason:
            print("ABORT:", reason, "-> holding the current position with move_j; the arm stays enabled")
            hold(arm)
        time.sleep(1.0)
        fin = angles(arm)
        print("final (deg):", [round(deg(a), 2) for a in fin], "· error to target", f"{max(abs(deg(a - b)) for a, b in zip(fin, target)):.2f} deg")

        # ---- 4. end
        if reason or args.stay:
            print("leaving the motors enabled." + ("" if args.stay else " Disable only when the arm rests folded (arm_enable_test.py or the GUI)."))
            return
        if not args.yes:
            input("Press Enter to DISABLE (the arm is at the folded zero pose); Ctrl-C leaves it enabled and holding... ")
        for _ in range(10):
            arm.disable()
            if not any(arm.get_joint_enable_status(i) for i in range(1, 7)):
                break
            time.sleep(0.1)
        enabled = any(arm.get_joint_enable_status(i) for i in range(1, 7))
        print("disabled:", not enabled)
    except KeyboardInterrupt:
        if enabled:
            print("\nCtrl-C: holding the current position; the motors stay ENABLED.")
            hold(arm)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
