#!/usr/bin/env python3
"""Real Piper: release or stop the arm. No questions asked, so it works in a hurry.

    source scripts/env.sh
    python tools/arm_stop.py            # motors OFF (disable): the arm goes limp. A raised arm FALLS: hold it by hand first.
    python tools/arm_stop.py --estop    # electronic emergency stop: a raised arm LOWERS SLOWLY with constant damping (vendor docs)
    python tools/arm_stop.py --reset    # vendor recovery order after an e-stop: e-stop, 1 s, reset(). reset() powers the arm off.

Plain disable is what arm_enable_test.py does at its end. The e-stop is the closest thing to a stop button this arm has (there is no hardware one);
after it the arm reports EMERGENCY STOP and needs --reset (or a power cycle) before it takes commands again.
Sends: the firmware query, then the chosen command(s). Prints the state before and after.
"""
import argparse
import time

from arm_common import angles, deg, open_arm, resolve_canbus

ARM_STATUS = {0: "normal", 1: "EMERGENCY STOP"}


def state(arm):
    st = arm.get_arm_status()
    en = [bool(arm.get_joint_enable_status(i)) for i in range(1, 7)]
    ang = angles(arm)
    if st is None:
        return "no status", en, ang
    m = st.msg
    return f"arm_status {ARM_STATUS.get(int(m.arm_status), int(m.arm_status))}, err {int(m.err_code)}, mode_feedback {int(m.mode_feedback)}", en, ang


def show(tag, arm):
    s, en, ang = state(arm)
    print(f"{tag}: {s} · enabled {en} · joints (deg) {[round(deg(a), 1) for a in ang] if ang else None}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b", help="CAN bus name, e.g. can0 (default: auto-detect)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--estop", action="store_true", help="electronic emergency stop (slow damped lowering)")
    g.add_argument("--reset", action="store_true", help="e-stop, 1 s, reset (powers the arm off)")
    args = ap.parse_args()
    try:
        arm, _, fw, profile = open_arm(resolve_canbus(args.canbus))
    except RuntimeError as e:
        raise SystemExit(f"{e}\nNothing was sent to the arm except the firmware query.")
    try:
        show("before", arm)
        if args.estop or args.reset:
            arm.electronic_emergency_stop()
            print("electronic emergency stop sent: the arm lowers slowly under damping")
            time.sleep(1.0)
            if args.reset:
                arm.reset()
                print("reset sent: the arm powers off")
                time.sleep(1.0)
        else:
            for _ in range(20):
                arm.disable()
                if not any(arm.get_joint_enable_status(i) for i in range(1, 7)):
                    break
                time.sleep(0.1)
            print("disable sent: motors off")
        time.sleep(0.5)
        show("after ", arm)
        if args.estop:
            print("the arm is in EMERGENCY STOP: run again with --reset (or power-cycle) before using it.")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
