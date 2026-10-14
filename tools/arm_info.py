#!/usr/bin/env python3
"""Read everything readable from a real Piper, without moving it, and show a short coloured summary.

    source scripts/env.sh
    python tools/arm_info.py                  # summary (colour in a terminal)
    python tools/arm_info.py -r               # the long raw form: bus table and every value
    python tools/arm_info.py -c               # also read the CPV drive parameters (SEE WARNING)
    python tools/arm_info.py -b can1          # use this CAN bus (default: auto-detect)
    python tools/arm_info.py --arm-id piper-01   # label for the saved JSON

The CAN bus is the Linux network interface of the USB-CAN adapter (can0, can1, ...), not something the arm reports. It is
auto-detected: the only CAN bus, else the only one that is up; name it with -b/--canbus when there are several. If it is down:
scripts/can_up.sh (asks for your sudo password).

What it does: listens for a fixed 1.5 s to see whether the arm is sending (and in which role); stops there if the bus is silent, nothing sent.
Then one firmware query (the SDK profile is chosen from the version) and enquiry frames. It never calls enable/disable/reset/calibrate/set_*/move_*.
The full result is always saved as JSON (~/data/ml/isaac/piper_studio/arm_info/, or --out).
WARNING: the SDK's get_cpv_* reads first switch the arm's motion mode to CPV (a state change; the SDK's mode frame also carries its default speed of 50 %).
That is why -c/--cpv-check is opt-in. Afterwards the tool puts the mode back (MOVE_P/J/L/C, as read at the start) and says whether it read back.
"""
import argparse
import collections
import datetime
import json
import os
import shutil
import sys
import time
from pathlib import Path

from arm_common import deg, fields, open_arm, resolve_canbus, wait_for

CENSUS_SECONDS = 1.5  # fixed: just long enough to see 200 Hz and 20 Hz frames
CTRL_MODE = {0: "standby", 1: "CAN control", 2: "teaching", 3: "ethernet", 4: "wifi", 5: "remote", 6: "linkage teaching input (master)", 7: "offline trajectory"}
ARM_STATUS = {0: "normal", 1: "EMERGENCY STOP", 2: "no solution", 3: "singularity", 4: "target exceeds limit", 5: "joint communication error",
              6: "joint brake not released", 7: "collision", 8: "overspeed in teaching", 9: "joint status error", 10: "other error",
              11: "teaching record", 12: "teaching execution", 13: "teaching pause", 14: "main controller over-temperature",
              15: "release resistor over-temperature"}
MODE_FB = {0: "MOVE_P", 1: "MOVE_J", 2: "MOVE_L", 3: "MOVE_C", 4: "MOVE_MIT (old)", 5: "MOVE_CPV", 6: "MOVE_MIT"}
TEACH = {0: "off", 1: "recording", 2: "stop recording", 3: "playing back", 4: "paused", 5: "resumed", 6: "terminated", 7: "moving to start"}
URDF_LIMITS_DEG = {1: (-150.0, 150.0), 2: (0.0, 180.0), 3: (-170.0, 0.0), 4: (-100.0, 100.0), 5: (-70.0, 70.0), 6: (-120.0, 120.0)}  # official URDF
ROLE_IDS = {"slave (normal feedback)": {0x2A1, 0x2A5, 0x2A8, 0x251, 0x261}, "master/leader (control frames)": {0x155, 0x156, 0x157, 0x159, 0x151}}


# --------------------------------------------------------------------------------------------------------------- colour
class Style:
    def __init__(self, on):
        self.on = on
        c = (lambda n: f"\033[{n}m") if on else (lambda n: "")
        self.reset, self.bold, self.dim = c(0), c(1), c(2)
        self.red, self.green, self.yellow, self.cyan = c(31), c(32), c(33), c(36)
        self.mark = {"ok": f"{self.green}✓{self.reset}", "warn": f"{self.yellow}!{self.reset}", "bad": f"{self.red}✗{self.reset}",
                     "info": f"{self.dim}·{self.reset}"}

    def paint(self, level, text):
        return {"ok": self.green, "warn": self.yellow, "bad": self.red, "info": self.dim}[level] + text + self.reset

    def pill(self, level, text):
        """Coloured block like  SLAVE  (plain text when colour is off: [SLAVE])."""
        if not self.on:
            return f"[{text}]"
        bg = {"ok": "42;30", "warn": "43;30", "bad": "41;97", "info": "100;97"}[level]
        return f"\033[{bg};1m {text} \033[0m"

    def rule(self, title, width):
        return f"{self.dim}──{self.reset} {self.bold}{title}{self.reset} {self.dim}{'─' * max(width - len(title) - 5, 4)}{self.reset}"


# -------------------------------------------------------------------------------------------------------------- reading
def census(canbus):
    """Passive listen for a fixed period. Returns ({hex id: {frames, hz}}, set of ids)."""
    import can
    bus = can.Bus(channel=canbus, interface="socketcan")  # python-can's own keyword
    counts, t0 = collections.Counter(), time.monotonic()
    try:
        while time.monotonic() - t0 < CENSUS_SECONDS:
            m = bus.recv(timeout=0.2)
            if m is not None:
                counts[m.arbitration_id] += 1
    finally:
        bus.shutdown()
    span = max(time.monotonic() - t0, 1e-6)
    return {hex(i): {"frames": n, "hz": round(n / span, 1)} for i, n in sorted(counts.items())}, set(counts)


def _restore_mode(arm, prev):
    """Put the motion mode back after the CPV reads (the SDK switched it to CPV); verifies on the next status frames.

    prev = the mode read before the CPV reads: P/J/L/C are restored as they were. If the arm was already in CPV (an earlier -c run, or CPV use),
    the normal resting mode MOVE_J is set. MIT modes are never restored automatically.
    """
    names = {0: "P", 1: "J", 2: "L", 3: "C", 5: "J"}
    if prev not in names:
        return {"restored": False, "note": f"previous mode {prev} is not restored automatically"}
    target = {"P": 0, "J": 1, "L": 2, "C": 3}[names[prev]]
    arm.set_motion_mode(getattr(arm.OPTIONS.MOTION_MODE, names[prev]))
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.5:
        st = arm.get_arm_status()
        if st is not None and int(st.msg.mode_feedback) == target:
            return {"restored": True, "to": MODE_FB[target], "note": "it was already in CPV before; set to MOVE_J" if prev == 5 else ""}
        time.sleep(0.05)
    return {"restored": False, "note": "mode did not read back as " + MODE_FB[target]}


def collect(args, canbus):
    out = {"time": datetime.datetime.now().isoformat(timespec="seconds"), "canbus": canbus, "arm_id": args.arm_id}
    out["census"], idset = census(canbus)
    roles = [r for r, s in ROLE_IDS.items() if s & idset]
    out["role"] = roles or (["unknown frames"] if idset else ["silent"])
    if not idset:
        return out
    try:
        arm, grip, fw, profile = open_arm(canbus, gripper=True)
    except RuntimeError as e:  # frames are on the bus but the arm does not answer (e.g. master mode, teach playback)
        out["error"] = str(e)
        return out
    out["firmware"], out["sdk_profile"] = fw, profile
    try:
        st = wait_for(arm.get_arm_status)
        out["arm_status"] = ({k: int(getattr(st.msg, k)) for k in ("ctrl_mode", "arm_status", "mode_feedback", "teach_status", "motion_status",
                                                                   "trajectory_num", "err_code")}
                             | {"err_status": fields(st).get("err_status")}) if st else None
        out["enabled"] = [bool(arm.get_joint_enable_status(i)) for i in range(1, 7)]
        ja = wait_for(arm.get_joint_angles)
        out["joint_angles_rad"] = list(ja.msg) if ja else None
        out["joint_angles_deg"] = [deg(x) for x in ja.msg] if ja else None
        for name, fn in (("flange_pose", arm.get_flange_pose), ("tcp_pose", arm.get_tcp_pose)):
            v = wait_for(fn, 1.5)
            out[name] = list(v.msg) if v else None
        out["motors"], out["drivers"] = {}, {}
        for i in range(1, 7):
            ms, ds = arm.get_motor_states(i), arm.get_driver_states(i)
            out["motors"][i], out["drivers"][i] = fields(ms) if ms else None, fields(ds) if ds else None
        out["limits"] = {}
        for i in range(1, 7):
            a, c = arm.get_joint_angle_vel_limits(i, timeout=2.0), arm.get_joint_acc_limits(i, timeout=2.0)
            ent = {"angle_vel": fields(a) if a else None, "acc": fields(c) if c else None}
            if a:
                lo, hi = deg(a.msg.min_angle_limit), deg(a.msg.max_angle_limit)
                ref = URDF_LIMITS_DEG[i]
                ent["deg"] = [lo, hi]
                ent["same_as_urdf"] = abs(lo - ref[0]) < 0.1 and abs(hi - ref[1]) < 0.1
            out["limits"][i] = ent
        f = arm.get_flange_vel_acc_limits(timeout=2.0)
        out["flange_limits"] = fields(f) if f else None
        for name, fn in (("crash_protection_rating", arm.get_crash_protection_rating), ("joint_assistance_rating", arm.get_joint_assistance_rating)):
            v = fn(timeout=2.0)
            out[name] = list(v.msg) if v else None
        if profile in ("v188", "v189"):
            v = arm.get_ik_joint_angles()
            out["ik_joint_angles"] = list(v.msg) if v else None
        if roles and roles[0].startswith("master"):
            v = arm.get_leader_joint_angles()
            out["leader_joint_angles_rad"] = list(v.msg) if v else None
        gs = wait_for(grip.get_gripper_status, 2.0)
        out["gripper_status"] = fields(gs) if gs else None
        gc = wait_for(grip.get_gripper_ctrl_states, 1.0)
        out["gripper_ctrl_states"] = fields(gc) if gc else None
        gp = grip.get_gripper_teaching_pendant_param(timeout=2.0)
        out["gripper_params"] = fields(gp) if gp else None
        out["sdk"] = {"joint_limits_clamp": arm.get_joint_limits_enabled(), "auto_motion_mode_switching": arm.get_auto_set_motion_mode_enabled(),
                      "feedback_fps": arm.get_fps(), "is_ok": arm.is_ok(), "comm_error": str(arm.get_comm_error())}
        if args.cpv_check:
            out["cpv"] = {}
            for i in range(1, 7):
                row = {}
                for k in ("pos", "vel", "acc", "dcc", "cv", "pp", "kp", "ki"):
                    try:
                        row[k] = getattr(arm, f"get_cpv_{k}")(i, timeout=2.0)
                    except Exception as e:  # noqa: BLE001 - report, keep going
                        row[k] = f"error: {e}"
                out["cpv"][i] = row
            out["cpv_restore"] = _restore_mode(arm, (out.get("arm_status") or {}).get("mode_feedback"))
    finally:
        arm.disconnect()
    return out


# ------------------------------------------------------------------------------------------------------------- summary
def _faults(foc, skip=("driver_enable_status",)):
    return [k for k, v in (foc or {}).items() if v and k not in skip]


def idset_ok(out):
    return bool(out["census"])


def summarize(out):
    """Rows for the summary: [(label, level, text)] and the list of notes [(level, text)]."""
    rows, notes = [], []
    cen = out["census"]
    st = (out.get("arm_status") or {})
    hz = cen.get("0x2a1", {}).get("hz")
    role = ", ".join(out["role"])
    if out["role"] == ["silent"]:
        return [("Link", "bad", "arm is NOT sending: nothing to read (power, plug, teach playback?)")], notes
    rows.append(("Link", "ok" if idset_ok(out) else "warn", f"{len(cen)} CAN ids on {out['canbus']}" + (f", status {hz:.0f} Hz" if hz else "")))
    st_ctrl = int(st["ctrl_mode"]) if st else None
    is_master = role.startswith("master") or st_ctrl == 6
    rows.append(("Role", "warn" if is_master else ("ok" if role.startswith("slave") else "warn"),
                 "MASTER (leader): zero-force drag, sends control frames only, does not hold or take commands" if is_master else
                 "SLAVE (follower): sends feedback and takes commands" if role.startswith("slave") else f"unclear ({role})"))
    if out.get("error"):
        rows.append(("Arm", "bad", out["error"]))
        return rows, notes
    fw = out["firmware"]
    rows.append(("Identity", "info", f"{fw['software_version']} · hw {fw['hardware_version']} · build {fw['production_date']} · node {fw['node_number']} · "
                 f"batch {fw['motor_ratio_and_batch']} · SDK profile {out['sdk_profile']}"))

    if st:
        ctrl, arm_s, teach, mode = int(st["ctrl_mode"]), int(st["arm_status"]), int(st["teach_status"]), int(st["mode_feedback"])
        mode_lvl = {0: "ok", 1: "ok", 2: "ok", 3: "ok", 5: "warn"}.get(mode, "bad")
        mode_note = {5: "  (not the normal mode: any move_j switches back)", 4: "  (MIT: dangerous)", 6: "  (MIT: dangerous)"}.get(mode, "")
        rows.append(("Motion mode", mode_lvl, f"{MODE_FB.get(mode, mode)}   ({CTRL_MODE.get(ctrl, ctrl)}){mode_note}"))
        en = out["enabled"]
        n = sum(en)
        rows.append(("Motors", "ok" if n == 0 else "warn",
                     "DISABLED (0/6 enabled): no holding torque" if n == 0 else
                     f"ENABLED ({n}/6): holding position, will move on command" if n == 6 else f"PARTLY ENABLED: joints {[i + 1 for i, e in enumerate(en) if e]}"))
        rows.append(("E-stop", "bad" if arm_s == 1 else "ok",
                     "ACTIVE: the arm is in electronic emergency stop (reset needed)" if arm_s == 1 else "not active (software e-stop; this arm has no e-stop button)"))
        rows.append(("Teach mode", "ok" if teach == 0 else "warn", "off" if teach == 0 else f"{TEACH.get(teach, teach)}: the arm may move by itself or ignore commands"))
        flags = [k for k, v in ((st.get("err_status") or {}).items()) if v is True]
        problem = (arm_s not in (0, 1)) or int(st["err_code"]) != 0 or flags
        rows.append(("Faults", "bad" if problem else "ok",
                     "none" if not problem else " · ".join(x for x in ((ARM_STATUS.get(arm_s, arm_s) if arm_s not in (0, 1) else ""), f"err_code {st['err_code']}" if int(st["err_code"]) else "",
                                                              ", ".join(flags)) if x)))
        if int(st["motion_status"]) == 1 and not problem:
            notes.append(("info", "last-motion flag reads 'reach target failed' (no error code): probably a leftover flag"))
    else:
        rows.append(("Motion mode", "bad", "no status frame received: mode, enable and e-stop state unknown"))

    ja = out.get("joint_angles_deg")
    if ja:
        outside = []
        for i, a in enumerate(ja, 1):
            lo, hi = (out["limits"].get(i, {}).get("deg") or URDF_LIMITS_DEG[i])
            if a < lo - 0.5 or a > hi + 0.5:
                outside.append(f"j{i} {a:.1f}° vs [{lo:.0f}, {hi:.0f}]")
        rows.append(("Pose", "warn" if outside else "ok", "joints (deg) " + "  ".join(f"{a:7.2f}" for a in ja)))
        if outside:
            notes.append(("warn", "outside the stored limits at rest: " + "; ".join(outside)))
    tcp = out.get("tcp_pose")
    if tcp:
        rows.append(("TCP", "info", f"x {tcp[0] * 1000:.1f}  y {tcp[1] * 1000:.1f}  z {tcp[2] * 1000:.1f} mm · rpy {deg(tcp[3]):.1f} {deg(tcp[4]):.1f} {deg(tcp[5]):.1f}°"))

    gs = out.get("gripper_status")
    gp = out.get("gripper_params") or {}
    if gs:
        w = gs["value"] * 1000
        stroke = (gp.get("max_range_config") or 0) * 1000
        g_faults = _faults(gs.get("foc_status"), skip=("driver_enable_status", "homing_status", "sensor_status"))
        lvl = "bad" if g_faults else ("warn" if w < -1 or (stroke and w > stroke + 1) else "ok")
        rows.append(("Gripper", lvl, f"width {w:.1f} mm" + (f" · stroke {stroke:.0f} mm" if stroke else "") + (f" · faults {g_faults}" if g_faults else " · driver ok")))
        if lvl == "warn":
            notes.append(("warn", f"gripper width {w:.1f} mm is outside its range: zero offset?"))
    else:
        rows.append(("Gripper", "warn", "no gripper status received"))

    drv = [d for d in out["drivers"].values() if d]
    if drv:
        v = [d["vol"] for d in drv]
        td, tm = [d["foc_temp"] for d in drv], [d["motor_temp"] for d in drv]
        fl = sorted({f for d in drv for f in _faults(d.get("foc_status"))})
        lvl = "bad" if fl else ("warn" if min(v) < 20 or max(td) > 60 or max(tm) > 60 else "ok")
        rows.append(("Power", lvl, f"{min(v):.0f}–{max(v):.0f} V · driver {min(td):.0f}–{max(td):.0f} °C · motor {min(tm):.0f}–{max(tm):.0f} °C · "
                     + (f"faults: {fl}" if fl else "no fault bits")))

    lim = out["limits"]
    if all(lim[i].get("deg") for i in lim):
        diff = [f"j{i} ±{lim[i]['deg'][1]:.0f}° (URDF ±{URDF_LIMITS_DEG[i][1]:.0f}°)" for i in lim if not lim[i]["same_as_urdf"]]
        vmax = sorted({lim[i]["angle_vel"]["max_joint_spd"] for i in lim})
        amax = sorted({lim[i]["acc"]["max_joint_acc"] for i in lim})
        fl = out.get("flange_limits") or {}
        rows.append(("Limits", "info" if diff else "ok", ("angles = official URDF" if not diff else "angles differ from URDF: " + ", ".join(diff))
                     + f" · speed {'/'.join(f'{x:g}' for x in vmax)} rad/s · accel {'/'.join(f'{x:g}' for x in amax)} rad/s²"
                     + (f" · flange {fl.get('end_max_linear_vel')} m/s {fl.get('end_max_angular_vel')} rad/s" if fl else "")))
    cr, ar = out.get("crash_protection_rating"), out.get("joint_assistance_rating")
    if cr is not None:
        off = not any(cr)
        rows.append(("Protection", "warn" if off else "ok", f"crash {'OFF' if off else 'level'} {' '.join(map(str, cr))} · assistance {' '.join(map(str, ar)) if ar else 'n/a'}"))
        if off:
            notes.append(("warn", "crash protection is 0 on every joint: the firmware will not detect a collision"))
    if not out["sdk"]["is_ok"]:
        notes.append(("warn", "SDK reports the link as not ok"))
    return rows, notes


def pills_of(out):
    """The state at a glance: [(level, text)]."""
    if out["role"] == ["silent"]:
        return [("bad", "NO DATA")]
    st = out.get("arm_status") or {}
    role = ", ".join(out["role"])
    master = role.startswith("master") or (bool(st) and int(st["ctrl_mode"]) == 6)
    pills = [("warn", "MASTER") if master else ("ok", "SLAVE") if role.startswith("slave") else ("warn", "ROLE ?")]
    if out.get("error"):
        return pills + [("bad", "NO REPLY")]
    if not st:
        return pills + [("bad", "STATUS ?")]
    mode, arm_s, teach = int(st["mode_feedback"]), int(st["arm_status"]), int(st["teach_status"])
    pills.append(({0: "ok", 1: "ok", 2: "ok", 3: "ok", 5: "warn"}.get(mode, "bad"), MODE_FB.get(mode, str(mode)).replace(" (old)", "")))
    n = sum(out["enabled"])
    pills.append(("ok", "MOTORS OFF") if n == 0 else ("warn", f"MOTORS ON {n}/6"))
    pills.append(("bad", "E-STOP ON") if arm_s == 1 else ("ok", "E-STOP OFF"))
    pills.append(("ok", "TEACH OFF") if teach == 0 else ("warn", "TEACH " + TEACH.get(teach, str(teach)).upper()))
    flags = [k for k, v in (st.get("err_status") or {}).items() if v is True]
    pills.append(("bad", "FAULT") if (arm_s not in (0, 1) or int(st["err_code"]) or flags) else ("ok", "NO FAULTS"))
    return pills


def gauge(S, v, lo, hi, width=28, slack=0.5):
    """├──────●──────┤ : where v sits between lo and hi; a red diamond on the edge when it is outside (more than slack)."""
    pos = (v - lo) / (hi - lo) if hi > lo else 0.0
    outside = v < lo - slack or v > hi + slack
    i = min(max(round(pos * (width - 1)), 0), width - 1)
    mark = f"{S.red}◆{S.reset}" if outside else f"{S.green}●{S.reset}"
    return f"{S.dim}├{'─' * i}{S.reset}{mark}{S.dim}{'─' * (width - 1 - i)}┤{S.reset}", outside


def _temp(S, t):
    c = S.green if t < 45 else S.yellow if t < 60 else S.red
    return f"{c}{t:.0f}{S.reset}"


def render_summary(out, S, saved):
    width = min(shutil.get_terminal_size((100, 20)).columns, 100)
    rows, notes = summarize(out)
    R = {label: (lvl, text) for label, lvl, text in rows}
    fw = out.get("firmware") or {}
    left = f"{S.bold}{S.cyan}PIPER{S.reset}  {out.get('arm_id') or 'unlabelled'} {S.dim}· {out['canbus']} · {out['time'].replace('T', ' ')[:16]}{S.reset}"
    right = f"{fw.get('software_version', '')} · {out.get('sdk_profile', '')}".strip(" ·")
    print(f"\n  {left}   {S.dim}{right}{S.reset}\n")
    print("  " + " ".join(S.pill(l, t) for l, t in pills_of(out)) + "\n")

    def line(label, key=None):
        lvl, text = R[key or label]
        print(f"  {S.mark[lvl]} {S.dim}{label:<13}{S.reset}{S.paint(lvl, text) if lvl in ('bad', 'warn') else text}")

    print("  " + S.rule("State", width - 2))
    for key in ("Link", "Role", "Arm", "Motion mode", "Motors", "E-stop", "Teach mode", "Faults"):
        if key in R:
            line(key)
    if "Identity" in R:
        print("\n  " + S.rule("Arm", width - 2))
        line("Identity")
        ja, lim = out.get("joint_angles_deg"), out.get("limits") or {}
        for i, a in enumerate(ja or [], 1):
            lo, hi = (lim.get(i, {}).get("deg") or URDF_LIMITS_DEG[i])
            g, outside = gauge(S, a, lo, hi)
            col = S.yellow if outside else ""
            print(f"    {S.dim}j{i}{S.reset}  {col}{a:8.2f}°{S.reset}  {g}  {S.dim}[{lo:.0f}, {hi:.0f}]{S.reset}")
        if "TCP" in R:
            line("TCP")
        gs, gp = out.get("gripper_status"), out.get("gripper_params") or {}
        if gs and (gp.get("max_range_config") or 0) > 0:
            w, stroke = gs["value"] * 1000, gp["max_range_config"] * 1000
            g, _ = gauge(S, w, 0, stroke, 28, slack=1.0)
            print(f"  {S.mark[R['Gripper'][0]]} {S.dim}{'Gripper':<13}{S.reset}{w:6.1f} mm  {g}  {S.dim}[0, {stroke:.0f}] · driver {'ok' if R['Gripper'][0] == 'ok' else 'check'}{S.reset}")
        else:
            line("Gripper")
        print("\n  " + S.rule("Health", width - 2))
        drv = [d for d in out["drivers"].values() if d]
        if drv:
            lvl = R["Power"][0]
            v = [d["vol"] for d in drv]
            td, tm = [d["foc_temp"] for d in drv], [d["motor_temp"] for d in drv]
            body = (f"{min(v):.0f}–{max(v):.0f} V · driver {_temp(S, min(td))}–{_temp(S, max(td))} °C · motor {_temp(S, min(tm))}–{_temp(S, max(tm))} °C · "
                    + ("no fault bits" if lvl == "ok" else "TOO HOT or low voltage" if lvl == "warn" else R["Power"][1].split("·")[-1].strip()))
            print(f"  {S.mark[lvl]} {S.dim}{'Power':<13}{S.reset}{body}")
        print("\n  " + S.rule("Settings stored in the arm", width - 2))
        for key in ("Limits", "Protection"):
            if key in R:
                line(key)
    if notes:
        print("\n  " + S.rule("Notes", width - 2))
        for lvl, text in notes:
            print(f"  {S.mark[lvl]} {text}")
    if out.get("cpv"):
        rs = out.get("cpv_restore") or {}
        note = (f"{S.green}motion mode restored to {rs['to']}{S.reset}{(' (' + rs['note'] + ')') if rs.get('note') else ''}" if rs.get("restored") else f"{S.yellow}mode NOT restored: {rs.get('note')}{S.reset}")
        print("\n  " + S.rule("CPV drive parameters", width - 2))
        print(f"  {S.dim}the SDK switched the arm to CPV mode to read these;{S.reset} {note}")
        keys = ("pos", "vel", "acc", "dcc", "cv", "pp", "kp", "ki")
        print(f"  {S.dim}joint " + " ".join(f"{k:>9}" for k in keys) + S.reset)
        for i, row in out["cpv"].items():
            print(f"    {i}   " + " ".join(f"{row[k]:9.3f}" if isinstance(row[k], (int, float)) else f"{str(row[k])[:9]:>9}" for k in keys))
    print(f"\n  {S.dim}saved {saved}  ·  -r raw form{S.reset}\n")


# ----------------------------------------------------------------------------------------------------------------- raw
def render_raw(out, S, saved):
    print("== 1. bus census (passive, %.1f s) ==" % CENSUS_SECONDS)
    for k, v in out["census"].items():
        print(f"  {k:>6}  {v['frames']:5d} frames  {v['hz']:6.1f} Hz")
    print("  role:", out["role"])
    if out["role"] == ["silent"]:
        print("  NO FRAMES: the arm is not transmitting. Nothing was sent.")
        return
    print("== 2. basic info ==\n  firmware:", out["firmware"], "\n  SDK profile:", out["sdk_profile"])
    st = out.get("arm_status")
    if st:
        print(f"  status: ctrl_mode={CTRL_MODE.get(int(st['ctrl_mode']), st['ctrl_mode'])}  arm_status={ARM_STATUS.get(int(st['arm_status']), st['arm_status'])}"
              f"  mode={MODE_FB.get(int(st['mode_feedback']), st['mode_feedback'])}  teach={TEACH.get(int(st['teach_status']), st['teach_status'])}"
              f"  motion_status={st['motion_status']}  err_code={st['err_code']}")
    print("  joints enabled:", out["enabled"])
    print("== 3. everything readable ==")
    print("  joint angles (deg):", out.get("joint_angles_deg"))
    for name in ("flange_pose", "tcp_pose"):
        v = out.get(name)
        print(f"  {name} [m, rad]:", [round(x, 4) for x in v] if v else None)
    for i in range(1, 7):
        d = out["drivers"][i] or {}
        print(f"  joint {i}: motor {out['motors'][i]}  driver V={d.get('vol')} T_drv={d.get('foc_temp')} T_mot={d.get('motor_temp')} status={d.get('foc_status')}")
    for i, ent in out["limits"].items():
        print(f"  joint {i} limits: {ent.get('deg')} deg, vmax {ent['angle_vel'] and ent['angle_vel'].get('max_joint_spd')} rad/s, "
              f"amax {ent['acc'] and ent['acc'].get('max_joint_acc')} rad/s^2  [{'same as URDF' if ent.get('same_as_urdf') else 'DIFFERS from URDF ' + str(URDF_LIMITS_DEG[i])}]")
    print("  flange limits:", out.get("flange_limits"))
    print("  crash_protection_rating:", out.get("crash_protection_rating"), "\n  joint_assistance_rating:", out.get("joint_assistance_rating"))
    if "ik_joint_angles" in out:
        print("  ik_joint_angles (only after a move_p):", out["ik_joint_angles"])
    if "leader_joint_angles_rad" in out:
        print("  leader joint angles:", out["leader_joint_angles_rad"])
    print("  gripper status:", out.get("gripper_status"), "\n  gripper ctrl echo:", out.get("gripper_ctrl_states"), "\n  gripper params:", out.get("gripper_params"))
    print("  sdk:", out["sdk"])
    if out.get("cpv"):
        print("  CPV parameters (the arm was switched to CPV mode to read them); mode restore:", out.get("cpv_restore"))
        for i, row in out["cpv"].items():
            print(f"    joint {i}:", row)
    print("saved", saved)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b", help="CAN bus name, e.g. can0 (default: auto-detect)")
    ap.add_argument("--raw", "-r", action="store_true", help="long raw form instead of the summary")
    ap.add_argument("--cpv-check", "-c", action="store_true", help="also read CPV parameters (switches the arm to CPV mode!)")
    ap.add_argument("--arm-id", help="your label for this physical arm (e.g. piper-01); stored in the JSON and its file name")
    ap.add_argument("--out", help="JSON output path (default: ~/data/ml/isaac/piper_studio/arm_info/arm_info_<id>_<fw>_<time>.json)")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args()
    S = Style(sys.stdout.isatty() and not args.no_color and "NO_COLOR" not in os.environ or bool(os.environ.get("FORCE_COLOR")))

    canbus = resolve_canbus(args.canbus)
    try:
        out = collect(args, canbus)
    except RuntimeError as e:  # e.g. no firmware answer
        sys.exit(f"{S.red}{e}{S.reset}")
    ver = (out.get("firmware") or {}).get("software_version", "nofw")
    ident = f"{args.arm_id}_" if args.arm_id else ""
    path = Path(args.out) if args.out else Path.home() / "data/ml/isaac/piper_studio/arm_info" / f"arm_info_{ident}{ver}_{datetime.datetime.now():%Y%m%d_%H%M%S}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=2, default=str))
    (render_raw if args.raw else render_summary)(out, S, path)
    if out["role"] == ["silent"]:
        sys.exit(2)


if __name__ == "__main__":
    main()
