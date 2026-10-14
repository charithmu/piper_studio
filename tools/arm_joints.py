#!/usr/bin/env python3
"""Live joint readout of a real Piper, updated in place. Purely passive: it only listens (sends nothing), so it can run next to any other program.

    source scripts/env.sh
    python tools/arm_joints.py            # live table, degrees; Enter saves the pose, q (or Ctrl-C) quits
    python tools/arm_joints.py --rad      # radians instead of degrees
    python tools/arm_joints.py --once     # one snapshot (also used when the output is not a terminal)
    python tools/arm_joints.py -b can1    # a specific CAN bus (default: auto-detect)

Move the arm by hand (motors disabled) and read the numbers to find poses. Press Enter to print the current pose as a copy-paste line
(radians, ready for move_j / SRDF) and append it with a timestamp to ~/data/ml/isaac/piper_studio/arm_poses.log.
Reads the status feedback frames 0x2A5-0x2A7 (joints, 0.001 deg) and 0x2A8 (gripper width) that the arm broadcasts at 200 Hz in its normal slave role.
A leader/master arm or a silent bus shows NO DATA.
"""
import argparse
import datetime
import math
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import can

from arm_common import resolve_canbus

JOINT_FRAMES = {0x2A5: (1, 2), 0x2A6: (3, 4), 0x2A7: (5, 6)}
STALE_S = 0.5
POSE_LOG = Path.home() / "data/ml/isaac/piper_studio/arm_poses.log"


def s32(d, o):
    return int.from_bytes(d[o:o + 4], "big", signed=True)


class Style:
    def __init__(self, on):
        c = (lambda n: f"\033[{n}m") if on else (lambda n: "")
        self.reset, self.bold, self.dim, self.red, self.cyan, self.green = c(0), c(1), c(2), c(31), c(36), c(32)


def render(S, joints, grip_mm, changed, age, rad):
    names = [f"j{i}" for i in range(1, 7)] + ["gripper"]
    head = "".join(f"{n:>10}" for n in names)
    if joints is None:
        vals = f"{S.red}{'NO DATA: no joint frames from the arm (power? leader mode? bus?)':>70}{S.reset}"
    else:
        cells = []
        for i, v in enumerate(joints):
            txt = f"{math.radians(v):10.4f}" if rad else f"{v:10.2f}"
            cells.append(f"{S.bold}{S.cyan}{txt}{S.reset}" if changed.get(i) else txt)
        g = f"{grip_mm:7.1f} mm" if grip_mm is not None else f"{'-':>10}"
        vals = "".join(cells) + (g if grip_mm is not None else g)
    stale = age is None or age > STALE_S
    status = (f"{S.red}stale: last frame {age:.1f} s ago{S.reset}" if (age is not None and stale)
              else f"{S.green}live{S.reset}" if age is not None else "")
    unit = "rad" if rad else "deg"
    return [f"{S.dim}{head}{S.reset}", vals, f"{S.dim}units: {unit} (gripper mm) · {S.reset}{status}{S.dim} · Enter saves the pose · q quits{S.reset}"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b", help="CAN bus name, e.g. can0 (default: auto-detect)")
    ap.add_argument("--rad", action="store_true", help="show radians instead of degrees")
    ap.add_argument("--once", "-1", action="store_true", help="one snapshot and exit")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args()
    canbus = resolve_canbus(args.canbus)
    tty_out = sys.stdout.isatty()
    S = Style(tty_out and not args.no_color and "NO_COLOR" not in os.environ)
    live = tty_out and sys.stdin.isatty() and not args.once

    bus = can.Bus(channel=canbus, interface="socketcan")
    joints, grip, last_frame, prev, last_change = [None] * 6, None, None, [None] * 6, [0.0] * 6
    saved = 0
    fd = sys.stdin.fileno() if live else None
    old = termios.tcgetattr(fd) if live else None
    shown = False

    def snapshot_line(tag):
        rad = [round(math.radians(v), 4) for v in joints]
        deg = [round(v, 2) for v in joints]
        return f"{tag} joints rad {rad} · deg {deg}" + (f" · gripper {grip / 1000:.4f} m" if grip is not None else "")

    try:
        if live:
            tty.setcbreak(fd)
        t_draw, t_start = 0.0, time.monotonic()
        deadline = t_start + 1.0
        while True:
            msg = bus.recv(timeout=0.02)
            now = time.monotonic()
            while msg is not None:                                 # drain what is queued
                i = msg.arbitration_id
                if i in JOINT_FRAMES:
                    a, b = JOINT_FRAMES[i]
                    joints[a - 1], joints[b - 1] = s32(msg.data, 0) / 1000.0, s32(msg.data, 4) / 1000.0
                    last_frame = now
                elif i == 0x2A8:
                    grip = s32(msg.data, 0) * 1e-3                  # 1e-6 m units -> mm
                msg = bus.recv(timeout=0)
            for k, v in enumerate(joints):
                if v is not None and prev[k] is not None and abs(v - prev[k]) > 0.05:
                    last_change[k] = now
                prev[k] = v
            if args.once or not live:
                if now > deadline or (all(v is not None for v in joints) and grip is not None):
                    break
                continue
            if live and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key in ("q", "Q", "\x03"):
                    break
                if key in ("\n", "\r") and all(v is not None for v in joints):
                    saved += 1
                    line = snapshot_line(f"#{saved} {datetime.datetime.now():%H:%M:%S}")
                    POSE_LOG.parent.mkdir(parents=True, exist_ok=True)
                    with POSE_LOG.open("a") as f:
                        f.write(f"{datetime.datetime.now().isoformat(timespec='seconds')} {canbus} {line}\n")
                    if shown:
                        sys.stdout.write("\033[3A")
                    sys.stdout.write(f"\033[2K\r{line}\n")
                    shown = False
            if now - t_draw >= 0.05 and (last_frame or now - t_start > 0.6):   # no NO DATA flash while the first frames arrive
                t_draw = now
                changed = {k: (now - last_change[k]) < 0.4 for k in range(6)}
                lines = render(S, joints if all(v is not None for v in joints) else None, grip, changed,
                               (now - last_frame) if last_frame else None, args.rad)
                if shown:
                    sys.stdout.write("\033[3A")
                sys.stdout.write("".join(f"\033[2K\r{ln}\n" for ln in lines))
                sys.stdout.flush()
                shown = True
    except KeyboardInterrupt:
        pass
    finally:
        if live:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        bus.shutdown()
    if args.once or not live:
        lines = render(S, joints if all(v is not None for v in joints) else None, grip, {}, 0.0 if last_frame else None, args.rad)
        print("\n".join(lines[:2]))
        if all(v is not None for v in joints):
            print(snapshot_line("pose"))
        elif last_frame is None:
            sys.exit(2)
    else:
        print(f"\n{saved} pose(s) saved to {POSE_LOG}" if saved else "")


if __name__ == "__main__":
    main()
