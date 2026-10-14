#!/usr/bin/env python3
"""Read everything readable from a real Piper, without moving it. Reusable: run it before and after any firmware change.

    source scripts/env.sh
    python tools/arm_info.py                    # all stages, saves a JSON next to the others
    python tools/arm_info.py --census-only      # stage 1 only: is the arm there, what is it sending (passive)
    python tools/arm_info.py --basic            # stages 1-2: plus firmware and status
    python tools/arm_info.py --cpv              # also read CPV drive parameters (SEE WARNING)

Stages: 1 census (passive listen, nothing sent) -> 2 basic info (one firmware query; the SDK profile is then chosen from the firmware
version and the connection is reopened with it) -> 3 everything else (enquiry frames only).
It never calls enable/disable/reset/calibrate/set_*/move_*. Requires can0 (or --channel) to be UP: sudo ip link set can0 up type can bitrate 1000000.
WARNING: the SDK's get_cpv_* reads first switch the arm's motion mode to CPV ("_maybe_set_motion_mode('cpv')"), a state change. They are
therefore off by default (--cpv to opt in, arm at rest, and expect a mode change).
"""
import argparse
import collections
import datetime
import json
import math
import sys
import time
from pathlib import Path

from arm_common import deg, fields, open_arm, wait_for

CTRL_MODE = {0: "standby", 1: "CAN control", 2: "teaching", 3: "ethernet", 4: "wifi", 5: "remote", 6: "linkage teaching input (master)", 7: "offline trajectory"}
ARM_STATUS = {0: "normal", 1: "EMERGENCY STOP", 2: "no solution", 3: "singularity", 4: "target exceeds limit", 5: "joint communication error",
              6: "joint brake not released", 7: "collision", 8: "overspeed in teaching", 9: "joint status error", 10: "other error",
              11: "teaching record", 12: "teaching execution", 13: "teaching pause", 14: "main controller over-temperature",
              15: "release resistor over-temperature"}
MODE_FB = {0: "MOVE_P", 1: "MOVE_J", 2: "MOVE_L", 3: "MOVE_C", 4: "MOVE_MIT (old)", 5: "MOVE_CPV", 6: "MOVE_MIT (v188+)"}
TEACH = {0: "off", 1: "recording", 2: "stop recording", 3: "playing back", 4: "paused", 5: "resumed", 6: "terminated", 7: "moving to start"}
URDF_LIMITS_DEG = {1: (-150.0, 150.0), 2: (0.0, 180.0), 3: (-170.0, 0.0), 4: (-100.0, 100.0), 5: (-70.0, 70.0), 6: (-120.0, 120.0)}  # official URDF, gripper arm
ROLE_IDS = {"slave/feedback (normal)": {0x2A1, 0x2A5, 0x2A8, 0x251, 0x261}, "master/leader control frames": {0x155, 0x156, 0x157, 0x159, 0x151}}


def census(channel, seconds):
    import can
    bus = can.Bus(channel=channel, interface="socketcan")
    counts, t0 = collections.Counter(), time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            m = bus.recv(timeout=0.2)
            if m is not None:
                counts[m.arbitration_id] += 1
    finally:
        bus.shutdown()
    span = max(time.monotonic() - t0, 1e-6)
    return {hex(i): {"frames": n, "hz": round(n / span, 1)} for i, n in sorted(counts.items())}, set(counts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--census-only", action="store_true")
    ap.add_argument("--basic", action="store_true", help="stop after the basic info (stages 1-2)")
    ap.add_argument("--cpv", action="store_true", help="also read CPV parameters (switches the arm to CPV mode!)")
    ap.add_argument("--seconds", type=float, default=2.0, help="census duration")
    ap.add_argument("--arm-id", help="your label for this physical arm (e.g. piper-01); stored in the JSON and the file name")
    ap.add_argument("--out", help="JSON output path (default: ~/data/ml/isaac/piper_studio/arm_info/arm_info_<fw>_<time>.json)")
    args = ap.parse_args()
    out = {"time": datetime.datetime.now().isoformat(timespec="seconds"), "channel": args.channel, "arm_id": args.arm_id}

    # ---- stage 1: can the host see the arm? (passive)
    print("== 1. census (passive) ==")
    try:
        state = Path(f"/sys/class/net/{args.channel}/operstate").read_text().strip()
    except OSError:
        sys.exit(f"{args.channel} does not exist: the USB-CAN adapter is not seen (check the USB plug; lsusb shows 1d50:606f when present).")
    if state != "up":
        sys.exit(f"{args.channel} is {state}: bring it up first (needs sudo):  sudo ip link set {args.channel} up type can bitrate 1000000")
    ids, idset = census(args.channel, args.seconds)
    out["census"] = ids
    for k, v in ids.items():
        print(f"  {k:>6}  {v['frames']:5d} frames  {v['hz']:6.1f} Hz")
    if not ids:
        print("  NO FRAMES: the arm is not transmitting (power, cable/plug, teach-playback state?). Stopping; nothing was sent.")
        out["role"] = "silent"
    else:
        roles = [r for r, s in ROLE_IDS.items() if s & idset]
        out["role"] = roles or ["unknown frames"]
        print("  role guess:", out["role"])
    if args.census_only or not ids:
        _save(out, args, None)
        return

    # ---- stage 2: basic info. Open with the oldest profile, read the firmware, reopen with the right profile.
    print("== 2. basic info ==")
    try:
        arm, grip, fw, profile = open_arm(args.channel, gripper=True)
    except RuntimeError as e:
        _save(out, args, None)
        sys.exit(str(e))
    out["firmware"], out["sdk_profile"] = fw, profile
    print("  firmware:", fw, "\n  SDK profile:", profile)
    try:
        wait = wait_for

        st = wait(arm.get_arm_status)
        if st is not None:
            m = st.msg
            out["arm_status"] = fields(st)
            print(f"  status: ctrl_mode={CTRL_MODE.get(int(m.ctrl_mode), m.ctrl_mode)}  arm_status={ARM_STATUS.get(int(m.arm_status), m.arm_status)}"
                  f"  mode={MODE_FB.get(int(m.mode_feedback), m.mode_feedback)}  teach={TEACH.get(int(m.teach_status), m.teach_status)}"
                  f"  motion_status={m.motion_status}  err_code={m.err_code}")
        else:
            print("  status: none received")
        out["enabled"] = [bool(arm.get_joint_enable_status(i)) for i in range(1, 7)]
        print("  joints enabled:", out["enabled"])
        if args.basic:
            _save(out, args, fw)
            return

        # ---- stage 3: everything else
        print("== 3. everything readable ==")
        ja = wait(arm.get_joint_angles)
        if ja is not None:
            out["joint_angles_rad"] = list(ja.msg)
            out["joint_angles_deg"] = [deg(x) for x in ja.msg]
            print("  joint angles (deg):", out["joint_angles_deg"])
        for name, fn in (("flange_pose", arm.get_flange_pose), ("tcp_pose", arm.get_tcp_pose)):
            v = wait(fn, 1.5)
            out[name] = list(v.msg) if v else None
            print(f"  {name} [m, rad]:", [round(x, 4) for x in v.msg] if v else None)
        out["motors"], out["drivers"] = {}, {}
        for i in range(1, 7):
            ms, ds = arm.get_motor_states(i), arm.get_driver_states(i)
            out["motors"][i], out["drivers"][i] = fields(ms) if ms else None, fields(ds) if ds else None
            d = out["drivers"][i] or {}
            print(f"  joint {i}: motor {out['motors'][i]}  driver V={d.get('vol')} T_drv={d.get('foc_temp')} T_mot={d.get('motor_temp')} status={d.get('foc_status')}")
        out["limits"] = {}
        for i in range(1, 7):
            a, c = arm.get_joint_angle_vel_limits(i, timeout=2.0), arm.get_joint_acc_limits(i, timeout=2.0)
            ent = {"angle_vel": fields(a) if a else None, "acc": fields(c) if c else None}
            if a:
                lo, hi = deg(a.msg.min_angle_limit), deg(a.msg.max_angle_limit)
                ref = URDF_LIMITS_DEG[i]
                ent["deg"] = [lo, hi]
                ent["vs_official_urdf"] = "same" if abs(lo - ref[0]) < 0.1 and abs(hi - ref[1]) < 0.1 else f"DIFFERS (URDF {ref})"
            out["limits"][i] = ent
            print(f"  joint {i} limits: {ent.get('deg')} deg, vmax {ent['angle_vel'] and ent['angle_vel'].get('max_joint_spd')} rad/s, "
                  f"amax {ent['acc'] and ent['acc'].get('max_joint_acc')} rad/s^2  [{ent.get('vs_official_urdf')}]")
        f = arm.get_flange_vel_acc_limits(timeout=2.0)
        out["flange_limits"] = fields(f) if f else None
        print("  flange limits:", out["flange_limits"])
        for name, fn in (("crash_protection_rating", arm.get_crash_protection_rating), ("joint_assistance_rating", arm.get_joint_assistance_rating)):
            v = fn(timeout=2.0)
            out[name] = list(v.msg) if v else None
            print(f"  {name}:", out[name])
        if profile in ("v188", "v189"):
            v = arm.get_ik_joint_angles()
            out["ik_joint_angles"] = list(v.msg) if v else None
            print("  ik_joint_angles (only after a move_p):", out["ik_joint_angles"])
        if "master/leader control frames" in (out["role"] if isinstance(out["role"], list) else []):
            v = arm.get_leader_joint_angles()
            out["leader_joint_angles_rad"] = list(v.msg) if v else None
            print("  leader joint angles:", out["leader_joint_angles_rad"])
        gs = wait(grip.get_gripper_status, 2.0)
        out["gripper_status"] = fields(gs) if gs else None
        gc = wait(grip.get_gripper_ctrl_states, 1.0)
        out["gripper_ctrl_states"] = fields(gc) if gc else None
        gp = grip.get_gripper_teaching_pendant_param(timeout=2.0)
        out["gripper_params"] = fields(gp) if gp else None
        print("  gripper status:", out["gripper_status"], "\n  gripper ctrl echo:", out["gripper_ctrl_states"], "\n  gripper params:", out["gripper_params"])
        out["sdk"] = {"joint_limits_clamp": arm.get_joint_limits_enabled(), "auto_motion_mode_switching": arm.get_auto_set_motion_mode_enabled(),
                      "feedback_fps": arm.get_fps(), "is_ok": arm.is_ok(), "comm_error": str(arm.get_comm_error())}
        print("  sdk:", out["sdk"])
        if args.cpv:
            print("  --cpv: reading CPV parameters (the arm is switched to CPV mode by the SDK)")
            out["cpv"] = {}
            for i in range(1, 7):
                out["cpv"][i] = {k: getattr(arm, f"get_cpv_{k}")(i, timeout=2.0) for k in ("pos", "vel", "acc", "dcc", "cv", "pp", "kp", "ki")}
                print(f"  cpv joint {i}:", out["cpv"][i])
    finally:
        arm.disconnect()
    _save(out, args, fw)


def _save(out, args, fw):
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    ver = (fw or {}).get("software_version", "nofw")
    ident = f"{args.arm_id}_" if args.arm_id else ""
    path = Path(args.out) if args.out else Path.home() / "data/ml/isaac/piper_studio/arm_info" / f"arm_info_{ident}{ver}_{stamp}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=2, default=str))
    print("saved", path)


if __name__ == "__main__":
    main()
