#!/usr/bin/env python3
"""Real Piper: put the arm in a known state, then move it to the zero pose with ONE move_j. Replaces arm_move_rest.py.

    source scripts/env.sh
    python tools/arm_zero.py --dry-run         # preflight and plan only: reads, nothing is changed
    python tools/arm_zero.py [--speed 50]      # asks for Enter, then configures, enables, moves, holds, asks before disabling

Zero pose = all joints 0 (what the vendor's go-zero demo and the GUI's "Go Zero" use). By default joints whose stored limit IS 0 (joint 2 min, joint 3 max)
are aimed 0.5 deg inside that limit so the target never sits on the limit; --exact sends pure zeros like the vendor demo.

Sequence (nothing is sent until you press Enter):
  0. preflight, reads only: arm is a slave, status normal, no error, teach off, driver bits clean, stored limits readable
  1. known configuration: speed percent set FIRST (the SDK's mode frame carries a speed, default 50 %), then CAN control + MOVE_J; read back that the arm reports MOVE_J
  2. enable all six joints (enabling alone does not move the arm)
  3. one move_j to the zero pose, watched at 10 Hz; done when all joints are within 0.5 deg for 1 s. No time limit unless --timeout is given
  4. hold; then ask before disabling (disable drops a raised arm: the zero pose is the folded rest pose, so it is safe there)
Only real faults stop it: arm status != normal, an error code, or a driver fault bit (it then HOLDS the pose with move_j, joints resting past a stored limit are pulled to its edge, and leaves the motors enabled).
Ctrl-C does the same. To release the arm afterwards: tools/arm_stop.py. Not used: reset() (powers the arm off), move_js / MIT / CPV, any set_* write to stored parameters.
This arm has no e-stop button: keep a hand on the power switch.
"""
import argparse
import math
import time

from arm_common import angles, deg, open_arm, resolve_canbus

MODE_J = 1
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


def hold(arm, lim):
    """Hold the present pose with move_j; joints resting outside their stored range (joint 2/3 at the stops) are pulled to the range edge (about 2 deg)."""
    cur = angles(arm)
    if cur:
        arm.move_j([min(max(a, lim[i + 1][0]), lim[i + 1][1]) if lim else a for i, a in enumerate(cur)])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b", help="CAN bus name, e.g. can0 (default: auto-detect)")
    ap.add_argument("--speed", type=int, default=50, help="speed percent, 1..100 (default 50)")
    ap.add_argument("--timeout", type=float, default=0.0, help="seconds allowed for the move (0 = no limit)")
    ap.add_argument("--exact", action="store_true", help="send pure zeros (no margin inside the stored limits)")
    ap.add_argument("--stay", action="store_true", help="leave the motors enabled at the end (no disable prompt)")
    ap.add_argument("--yes", action="store_true", help="skip the Enter prompts")
    ap.add_argument("--dry-run", action="store_true", help="preflight and plan only")
    args = ap.parse_args()
    if not 1 <= args.speed <= 100:
        raise SystemExit("--speed must be 1..100")

    try:
        arm, _, fw, profile = open_arm(resolve_canbus(args.canbus))
    except RuntimeError as e:
        raise SystemExit(f"{e}\nNothing was changed.")
    enabled, lim = False, {}
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
        if dist < DONE_DEG:
            print("already at the zero pose: nothing to do (nothing was changed).")
            return
        if args.dry_run:
            print("dry run: only read requests were sent.")
            return

        if not args.yes:
            input("Arm clear, hand on the power switch, you are watching? Press Enter to CONFIGURE, ENABLE and MOVE (Ctrl-C cancels)... ")

        # ---- 1. known configuration
        arm.set_speed_percent(args.speed)                # first: it updates the speed carried by every later mode frame
        arm.set_motion_mode(arm.OPTIONS.MOTION_MODE.J)   # CAN control + MOVE_J
        if not wait_mode(arm, MODE_J):
            raise SystemExit("the arm did not report CAN control + MOVE_J: stopping before enabling (nothing is enabled)")
        print(f"configured: CAN control, MOVE_J (read back), speed {args.speed} %")

        # ---- 2. enable and latch
        t0 = time.monotonic()
        while time.monotonic() - t0 < 5.0 and not arm.enable():
            time.sleep(0.05)
        enabled = any(arm.get_joint_enable_status(i) for i in range(1, 7))
        if not all(arm.get_joint_enable_status(i) for i in range(1, 7)):
            raise SystemExit("could not enable all six joints: holding, not moving")
        print("all six joints enabled")

        # ---- 3. the move
        arm.move_j(target)
        print("move_j sent")
        t0, ok_since, reason = time.monotonic(), None, None
        while True:
            time.sleep(0.1)
            now, t = angles(arm), time.monotonic()
            err = max(abs(deg(a - b)) for a, b in zip(now, target))
            faults = drv_faults(arm)
            reason = (status_problem(arm) or (f"driver faults {faults}" if faults else None)
                      or ("timeout" if args.timeout and t - t0 > args.timeout else None))
            print(f"  t={t - t0:4.1f}s  max error {err:5.2f} deg  joints(deg) {[round(deg(a), 1) for a in now]}")
            if reason:
                break
            ok_since = (ok_since or t) if err < DONE_DEG else None
            if ok_since and t - ok_since >= DONE_HOLD_S:
                print("zero pose reached")
                break
        if reason:
            print("ABORT:", reason, "-> holding the current position with move_j; the arm stays enabled")
            hold(arm, lim)
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
            hold(arm, lim)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
