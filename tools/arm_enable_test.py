#!/usr/bin/env python3
"""Real Piper: enable the motors, hold still for a few seconds, disable. NO motion command is sent.

    source scripts/env.sh && python tools/arm_enable_test.py [--channel can0] [--hold 3]

Needs the user at the arm with a hand on the power switch (this arm has no e-stop button), the arm in its folded rest pose and clear of obstacles.
What it sends: motor enable frames, then (at the end, or on any error / Ctrl-C) motor disable frames. The arm should
only hold where it is. It aborts and disables if a joint moves more than 1 degree, or the arm status leaves 'normal'.
Disabling releases the motors: only run it with the arm resting in the folded pose, or it can drop under gravity.
"""
import argparse
import math
import time

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config


def angles(arm):
    ja = arm.get_joint_angles()
    return list(ja.msg) if ja is not None else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds to stay enabled")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation prompt")
    args = ap.parse_args()
    if not args.yes:
        input("Arm folded at rest, clear of obstacles, hand on the power switch? Press Enter to enable (Ctrl-C to cancel)... ")

    cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel=args.channel)
    arm = AgxArmFactory.create_arm(cfg)
    arm.connect()
    try:
        for _ in range(30):
            if arm.get_fps() > 0 and angles(arm) is not None:
                break
            time.sleep(0.1)
        start = angles(arm)
        st = arm.get_arm_status()
        if start is None or st is None or int(st.msg.arm_status) != 0 or int(st.msg.err_code) != 0:
            print("Not starting: no feedback or arm status not normal.", start, st and st.msg)
            return
        print("start (deg):", [round(math.degrees(a), 2) for a in start])

        t0, ok = time.monotonic(), False
        while time.monotonic() - t0 < 5.0:       # enable() sends the enable frame and reports whether all joints are enabled
            if arm.enable():
                ok = True
                break
            time.sleep(0.05)
        print("all six joints enabled:", ok, " per joint:", [bool(arm.get_joint_enable_status(i)) for i in range(1, 7)])
        if not ok:
            return

        t0 = time.monotonic()
        while time.monotonic() - t0 < args.hold:
            cur, st = angles(arm), arm.get_arm_status()
            drift = max(abs(math.degrees(a - b)) for a, b in zip(cur, start))
            print(f"  t={time.monotonic() - t0:3.1f}s  max drift {drift:5.2f} deg  arm_status={int(st.msg.arm_status)} err={int(st.msg.err_code)}"
                  f"  currents(A)={[round(arm.get_motor_states(i).msg.current, 2) for i in range(1, 7)]}")
            if drift > 1.0 or int(st.msg.arm_status) != 0:
                print("ABORT: joint moved or status changed")
                break
            time.sleep(0.5)
    finally:
        for _ in range(10):                      # disable until every joint reports disabled
            arm.disable()
            if not any(arm.get_joint_enable_status(i) for i in range(1, 7)):
                break
            time.sleep(0.1)
        print("disabled:", [not arm.get_joint_enable_status(i) for i in range(1, 7)], " end (deg):",
              [round(math.degrees(a), 2) for a in (angles(arm) or [])])
        arm.disconnect()


if __name__ == "__main__":
    main()
