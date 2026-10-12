#!/usr/bin/env python3
"""Read-only probe of a real Piper over CAN: firmware, status, joints, motors, drivers.

    source scripts/env.sh && python tools/arm_probe.py [--channel can0] [--json out.json]

Safety: this script only reads. Without --limits it sends exactly one frame, the firmware query inside get_firmware().
With --limits it also sends read requests for the stored joint angle/speed/acceleration and flange limits (queries, no writes).
It never calls enable(), disable(), reset(), set_*() or any move_*(). Run it with the arm powered and CAN up.
"""
import argparse
import json
import time

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config

ARM_STATUS = {0: "normal", 1: "EMERGENCY STOP", 2: "no solution", 3: "singularity", 4: "target exceeds limit",
              5: "joint communication error", 6: "joint brake not released", 7: "collision", 8: "overspeed in teaching",
              9: "joint status error", 10: "other error", 11: "teaching record", 12: "teaching execution",
              13: "teaching pause", 14: "main controller over-temperature", 15: "release resistor over-temperature"}
CTRL_MODE = {0: "standby", 1: "CAN control", 2: "teaching", 3: "ethernet", 4: "wifi", 5: "remote", 6: "linkage teaching", 7: "offline trajectory"}


def fields(m):
    """Plain dict of a message's public fields (robust to SDK version differences)."""
    msg = getattr(m, "msg", m)
    return {k: v for k, v in vars(msg).items() if not k.startswith("_")} if hasattr(msg, "__dict__") else str(msg)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--limits", action="store_true", help="also query the limits stored in the arm (read requests)")
    ap.add_argument("--json", help="also write the result to this file")
    args = ap.parse_args()

    # DEFAULT profile = oldest message layout; the firmware query is identical on every profile.
    cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel=args.channel)
    arm = AgxArmFactory.create_arm(cfg)
    arm.connect()
    out = {}
    try:
        for _ in range(30):                      # up to 3 s for feedback to arrive
            if arm.get_fps() > 0:
                break
            time.sleep(0.1)
        out["feedback_fps"] = arm.get_fps()
        out["comm_ok"] = arm.is_ok()
        print(f"feedback rate: {out['feedback_fps']} Hz   communication ok: {out['comm_ok']}")

        fw = arm.get_firmware(timeout=2.0)
        out["firmware"] = fw
        print("firmware:", fw if fw else "NO ANSWER (older firmware, or the query was not received)")
        if fw:
            v = fw.get("software_version", "")
            print("  -> SDK profile hint: " + ("v189 (>= S-V1.8-9)" if v >= "S-V1.8-9" else "v188" if v == "S-V1.8-8"
                  else "v183" if "S-V1.8-3" <= v <= "S-V1.8-7" else "default (<= S-V1.8-2)"))

        st = arm.get_arm_status()
        if st is not None:
            m = st.msg
            out["arm_status"] = fields(st)
            print(f"status: ctrl_mode={CTRL_MODE.get(int(m.ctrl_mode), m.ctrl_mode)}  arm_status={ARM_STATUS.get(int(m.arm_status), m.arm_status)}"
                  f"  mode_feedback={m.mode_feedback}  teach={m.teach_status}  motion={m.motion_status}  err_code={m.err_code}")
        ja = arm.get_joint_angles()
        if ja is not None:
            out["joint_angles_rad"] = list(ja.msg)
            print("joint angles (deg):", [round(x * 57.29578, 2) for x in ja.msg])
        out["enabled"] = [bool(arm.get_joint_enable_status(i)) for i in range(1, 7)]
        print("joint enabled:", out["enabled"])
        time.sleep(0.5)                          # driver frames arrive at 40 Hz; make sure each joint has been heard
        out["motors"], out["drivers"] = {}, {}
        for i in range(1, 7):
            ms, ds = arm.get_motor_states(i), arm.get_driver_states(i)
            out["motors"][i], out["drivers"][i] = fields(ms) if ms else None, fields(ds) if ds else None
            print(f"joint {i}: motor {out['motors'][i]}\n         driver {out['drivers'][i]}")
        try:
            out["tcp_pose"] = list(arm.get_tcp_pose().msg)
            print("tcp pose [m, rad]:", [round(x, 4) for x in out["tcp_pose"]])
        except Exception as e:  # noqa: BLE001 - optional
            print("tcp pose: not available:", e)
        if args.limits:
            out["limits"] = {}
            for i in range(1, 7):
                a, c = arm.get_joint_angle_vel_limits(i, timeout=2.0), arm.get_joint_acc_limits(i, timeout=2.0)
                out["limits"][i] = {"angle_vel": fields(a) if a else None, "acc": fields(c) if c else None}
                print(f"joint {i} limits: {out['limits'][i]}")
            f = arm.get_flange_vel_acc_limits(timeout=2.0)
            out["limits"]["flange"] = fields(f) if f else None
            print("flange limits:", out["limits"]["flange"])
    finally:
        arm.disconnect()
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2, default=str)
        print("wrote", args.json)


if __name__ == "__main__":
    main()
