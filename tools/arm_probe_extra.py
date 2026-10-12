#!/usr/bin/env python3
"""Extra READ-ONLY reads of a real Piper (kept outside the repo on purpose; see README.md in this folder).

    cd ~/projects/ros/piper_studio && source scripts/env.sh && python <this file> [--channel can0] [--json out.json]

Reads: gripper status (value/force/mode/driver bits), gripper control echo, gripper max-range setting (stroke),
crash-protection rating per joint, joint assistance rating, error flags of the arm status, and the SDK's local
settings. Frames sent: enquiry frames only (CAN 0x477 with param_enquiry = 2 / 4 / 5 and every setting field 0,
CAN 0x477 can never write in this form). Never calls enable/disable/reset/calibrate/set_*/move_*.
Expected on S-V1.7-3: the assistance rating needs firmware >= S-V1.8-1 per the vendor protocol notes, so None is normal.
"""
import argparse
import json
import time

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config


def fields(m):
    msg = getattr(m, "msg", m)
    if isinstance(msg, (list, tuple)):
        return list(msg)
    return {k: (fields(v) if hasattr(v, "__dict__") and not isinstance(v, (int, float, str)) else v)
            for k, v in vars(msg).items() if not k.startswith("_")} if hasattr(msg, "__dict__") else str(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--json")
    args = ap.parse_args()
    cfg = create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel=args.channel)
    arm = AgxArmFactory.create_arm(cfg)
    grip = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER)   # before connect(), as the docs recommend; sends nothing
    arm.connect()
    out = {}
    try:
        for _ in range(30):
            if arm.get_fps() > 0:
                break
            time.sleep(0.1)
        def wait(fn, secs=3.0):
            t0 = time.monotonic()
            while time.monotonic() - t0 < secs:
                v = fn()
                if v is not None:
                    return v
                time.sleep(0.05)
            return None

        st = wait(arm.get_arm_status)
        print("arm status received:", st is not None)
        if st is not None:
            out["arm_status_flags"] = fields(st)
            print("arm status:", out["arm_status_flags"])

        gs = wait(grip.get_gripper_status)
        out["gripper_status"] = fields(gs) if gs else None
        print("gripper status:", out["gripper_status"], "  effector fps:", grip.get_fps(), " is_ok:", grip.is_ok())
        gc = wait(grip.get_gripper_ctrl_states, 1.0)
        out["gripper_ctrl_states"] = fields(gc) if gc else None
        print("gripper control echo:", out["gripper_ctrl_states"])
        gp = grip.get_gripper_teaching_pendant_param(timeout=2.0)
        out["gripper_teaching_pendant_param"] = fields(gp) if gp else None
        print("gripper teaching/range params (max_range_config 0 = unset, 0.07 / 0.1 = stroke in m):", out["gripper_teaching_pendant_param"])

        cr = arm.get_crash_protection_rating(timeout=2.0)
        out["crash_protection_rating"] = fields(cr) if cr else None
        print("crash protection rating per joint (0 = off .. 8 = most sensitive):", out["crash_protection_rating"])
        ar = arm.get_joint_assistance_rating(timeout=2.0)
        out["joint_assistance_rating"] = fields(ar) if ar else None
        print("joint assistance rating (0..10):", out["joint_assistance_rating"], "" if ar else "  (None is expected below S-V1.8-1)")

        out["sdk_joint_limits_enabled"] = arm.get_joint_limits_enabled()
        out["sdk_auto_motion_mode_switching"] = arm.get_auto_set_motion_mode_enabled()
        print("SDK local settings: joint limit clamp =", out["sdk_joint_limits_enabled"],
              " auto motion-mode switching =", out["sdk_auto_motion_mode_switching"])
    finally:
        arm.disconnect()
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2, default=str)
        print("wrote", args.json)


if __name__ == "__main__":
    main()
