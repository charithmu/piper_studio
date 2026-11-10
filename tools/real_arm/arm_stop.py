#!/usr/bin/env python3
"""Real Piper: EMERGENCY STOP. No questions asked, so it works in a hurry. Run it from a second terminal while another tool is moving the arm.

    source scripts/env.sh
    python tools/real_arm/arm_stop.py             # electronic e-stop (default): a raised arm LOWERS SLOWLY with constant damping (vendor docs)
    python tools/real_arm/arm_stop.py --disable   # plain motors OFF: the arm goes limp. A raised arm FALLS: hold it by hand first.
    python tools/real_arm/arm_stop.py --reset     # e-stop, 1 s, reset(): the vendor recovery order. Clears the e-stop and disables the motors; the arm stays powered.

The e-stop is the closest thing to a stop button this arm has (there is no hardware one; cutting power drops it). After it the arm reports EMERGENCY STOP and takes no commands until
`arm_stop.py --reset` (no power cycle needed, verified 2026-11-10 in arm_session.py stop); then start again with arm_session.py start. Another program that is still sending (arm_pose.py, arm_session.py, a RealArm in Python) may fight the
stop or keep running: stop it with Ctrl-C too. The bus is shared for reading, so this works beside a running tool, but only one writer should be active afterwards.
Sends: the firmware query, then the chosen command(s). Prints the state before and after. Drilled on the real arm 2026-11-10 (from park: damped lowering, then reset).
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
    g.add_argument("--disable", action="store_true", help="plain motors off (the arm falls if raised)")
    g.add_argument("--estop", action="store_true", help="electronic emergency stop (the default; kept for old command lines)")
    g.add_argument("--reset", action="store_true", help="e-stop, 1 s, reset (powers the arm off)")
    args = ap.parse_args()
    try:
        arm, _, fw, profile = open_arm(resolve_canbus(args.canbus), lock=False)
    except RuntimeError as e:
        raise SystemExit(f"{e}\nNothing was sent to the arm except the firmware query.")
    try:
        show("before", arm)
        if not args.disable:
            arm.electronic_emergency_stop()
            print("electronic emergency stop sent: the arm lowers slowly under damping")
            time.sleep(1.0)
            if args.reset:
                arm.reset()
                print("reset sent: e-stop cleared, motors off")
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
        if not args.disable and not args.reset:
            print("the arm is in EMERGENCY STOP: run again with --reset (or power-cycle) before using it.")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
