#!/usr/bin/env python3
"""Real Piper: start and end a session. The operator confirms enabling and disabling; the moves are the checked ones (move_j, low speed).

    source scripts/env.sh
    python tools/real_arm/arm_session.py status             # reads only
    python tools/real_arm/arm_session.py start [--speed 10] # preflight, Enter to ENABLE, then one move to ZERO (from the slumped pose or from anywhere valid)
    python tools/real_arm/arm_session.py stop  [--speed 10] # zero, park, Enter: e-stop (damped lowering), Enter: reset (clear). --plain-disable: disable instead

start:  1 preflight (slave, status normal, teach off, no driver fault, limits readable)  2 shows the plan, waits for Enter  3 speed first, then CAN control + MOVE_J, read back
        4 enable all six joints (read back)  5 one move_j to zero; success = zero within 0.5 deg for 1 s. Leaves the motors ON and holding at zero.
stop:   needs enabled motors and a valid pose. zero, 1 s, park (joint 5 lowered, see arm_pose.py), Enter, then the electronic e-stop (damped lowering), waits until the arm is still,
        reports the slumped pose, Enter, then reset (clears the e-stop; the arm stays powered and `start` works at once). --plain-disable ends with a plain disable instead. Any fault or Ctrl-C
        holds the pose with the motors ON (use arm_stop.py to release).
The only way out of the slumped pose is start's single move to zero. Between start and stop use arm_pose.py (known poses), arm_jog.py (one joint) or RealArm.
Needs no sudo, but CAN must be up (scripts/can_up.sh). Only one program may use the bus at a time. This arm has no e-stop button: hand on the power switch, or arm_stop.py.
"""
import argparse
import time

from piper_tools.arm_common import angles, deg, open_arm, resolve_canbus
from piper_tools.arm_pose import POSES, configure, go, outside_limits, read_state
from piper_tools.arm_zero import hold


def describe(cur, lim, enabled):
    if outside_limits(cur, lim):
        return "slumped (powered-off sag, outside the stored limits)"
    near = [n for n, q in POSES.items() if max(abs(a - b) for a, b in zip(cur, q)) < 0.01]
    return f"valid, at {near[0]}" if near else "valid"


def table(cur, lim, enabled, goal=None):
    print("joint   now(deg)" + ("  goal(deg)  move(deg)" if goal else "") + "   stored limits(deg)")
    for i in range(6):
        g = f"   {deg(goal[i]):8.2f}   {deg(goal[i] - cur[i]):8.2f}" if goal else ""
        print(f"  {i + 1}    {deg(cur[i]):8.2f}{g}    [{deg(lim[i + 1][0]):7.1f}, {deg(lim[i + 1][1]):6.1f}]")
    print(f"motors enabled {enabled} · pose: {describe(cur, lim, enabled)}")


def disable(arm):
    for _ in range(10):
        arm.disable()
        if not any(arm.get_joint_enable_status(i) for i in range(1, 7)):
            return True
        time.sleep(0.1)
    return not any(arm.get_joint_enable_status(i) for i in range(1, 7))


def status_text(arm):
    st = arm.get_arm_status()
    return "no status" if st is None else f"arm_status {int(st.msg.arm_status)}, err {int(st.msg.err_code)}"


def settle(arm, timeout=15.0):
    """Wait until no joint has moved more than 0.2 deg in the last second (the damped lowering is over), or the timeout."""
    t0, prev = time.monotonic(), angles(arm)
    while time.monotonic() - t0 < timeout:
        time.sleep(1.0)
        now = angles(arm)
        if prev and now and max(abs(deg(a - b)) for a, b in zip(now, prev)) < 0.2:
            return True
        prev = now
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=("status", "start", "stop"))
    ap.add_argument("--canbus", "-b")
    ap.add_argument("--speed", type=int, default=10, help="speed percent 1..50 (default 10)")
    ap.add_argument("--plain-disable", action="store_true", help="stop: end with a plain disable at park instead of e-stop + reset")
    args = ap.parse_args()
    if not 1 <= args.speed <= 50:
        raise SystemExit("--speed must be 1..50")
    try:
        arm, _, fw, profile = open_arm(resolve_canbus(args.canbus))
    except RuntimeError as e:
        raise SystemExit(f"{e}\nNothing was changed.")
    lim, running = {}, False
    try:
        cur, lim, enabled, problems = read_state(arm)
        print(f"firmware {fw['software_version']} · profile {profile}")
        table(cur, lim, enabled, POSES["zero"] if args.action == "start" else None)
        if args.action == "status":
            print("problems: " + ("; ".join(problems) if problems else "none"))
            return
        if problems:
            raise SystemExit("not continuing: " + "; ".join(problems))
        slumped = bool(outside_limits(cur, lim))

        if args.action == "start":
            if all(enabled) and not slumped and max(abs(a - b) for a, b in zip(cur, POSES["zero"])) < 0.01:
                print("already enabled and at zero: nothing to do.")
                return
            input("Arm clear, hand on the power switch? Enter to CONFIGURE, ENABLE and move to ZERO (Ctrl-C cancels)... ")
            if not configure(arm, args.speed):
                raise SystemExit("the arm did not report CAN control + MOVE_J: nothing enabled")
            print(f"configured: CAN control, MOVE_J (read back), speed {args.speed} %")
            t0 = time.monotonic()
            while time.monotonic() - t0 < 5.0 and not arm.enable():
                time.sleep(0.05)
            if not all(arm.get_joint_enable_status(i) for i in range(1, 7)):
                raise SystemExit("could not enable all six joints: not moving (arm_stop.py --disable releases it)")
            print("all six joints enabled")
            running = True
            why = go(arm, lim, POSES["zero"], "zero pose", args.speed)
            running = False
            fin = angles(arm)
            print("final (deg):", [round(deg(a), 2) for a in fin])
            print("READY: enabled and at zero." if not why else f"NOT READY: {why}. The motors are ON and the arm is holding.")
            return

        # stop
        if slumped and not any(enabled):
            print("already powered off and slumped: nothing to do.")
            return
        if slumped:
            raise SystemExit("the arm is outside its limits: run `arm_session.py start` first (its move to zero is the way out), then stop")
        if not all(enabled):
            raise SystemExit(f"motors only partly enabled {enabled}: run `arm_session.py start` first")
        input("Arm clear, hand on the power switch? Enter to go to ZERO, then PARK (Ctrl-C cancels)... ")
        if not configure(arm, args.speed):
            raise SystemExit("the arm did not report CAN control + MOVE_J: not moving")
        running = True
        for label, goal in (("zero pose", POSES["zero"]), ("park pose", POSES["park"])):
            if max(abs(a - b) for a, b in zip(angles(arm), goal)) < 0.0087:
                print(f"already at the {label}")
            else:
                why = go(arm, lim, goal, label, args.speed)
                if why:
                    print(f"NOT PARKED: {why}. The motors are ON and the arm is holding (arm_stop.py to release).")
                    return
            time.sleep(1.0)
        running = False
        if args.plain_disable:
            input("At PARK. Support the arm with a hand; Enter to DISABLE the motors (Ctrl-C leaves them on, holding)... ")
            off = disable(arm)
            time.sleep(1.0)
            print("motors disabled:", off, "· slumped pose (deg):", [round(deg(a), 2) for a in angles(arm)])
            return
        input("At PARK. Enter to EMERGENCY-STOP: the arm lowers slowly under damping and the motors go limp (Ctrl-C leaves them on, holding)... ")
        arm.electronic_emergency_stop()
        print("electronic emergency stop sent")
        settle(arm)
        print("slumped pose (deg):", [round(deg(a), 2) for a in angles(arm)], "· status:", status_text(arm))
        input("Arm at rest. Enter to CLEAR the e-stop (reset; Ctrl-C leaves it in e-stop, arm_stop.py --reset does it later)... ")
        arm.reset()
        time.sleep(1.5)
        print("reset sent · status:", status_text(arm), "· motors enabled:", [bool(arm.get_joint_enable_status(i)) for i in range(1, 7)])
        print("Done: e-stop cleared, motors off, the arm stays powered. `start` can follow at once (verified 2026-11-10).")
    except KeyboardInterrupt:
        if running:
            print("\nCtrl-C: holding the current position; the motors stay ENABLED (arm_stop.py to release).")
            hold(arm, lim)
        else:
            print("\ncancelled: motors left as they are.")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
