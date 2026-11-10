"""piper-arm: one terminal console for the real Piper. Live status on top, one key per action below. No ROS.

    piper-arm [-b can0] [--speed 10]       # needs the CAN bus up (scripts/can_up.sh) and pyAgxArm (the workspace .venv has it)

It is the operator's cockpit for what was proven on the arm with tools/real_arm/ (2026-11-10): start, known poses, one-joint jog, end of session, e-stop.
Motion needs the operator: every action that moves or switches the motors waits for `y` (anything else cancels). Exceptions, always immediate:
`e` EMERGENCY STOP (electronic e-stop, damped lowering) and SPACE (stop where you are, keep holding). Both work while a move is running.

Keys:  s start (enable, go to zero)   z zero   p park   r ready (big move)   j jog a joint (1-6, then + / - or arrows, j or Esc leaves)
       t end session (zero, park, e-stop, reset)   d disable motors (the arm sags)   c clear the e-stop (reset)   [ ] speed   , . jog step   q quit
The guard is the one in piper_core: nothing is commanded outside the stored limits (0.01 rad margin), nothing while a fault is reported, nothing from the slumped pose except
the single move to zero in `s`. Speed starts at 10 % (about 17 deg/s on one joint); `]` raises it in steps up to 50 %. Quitting leaves the motors as they are.
This arm has no e-stop button: keep a hand on the power switch.
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import termios
import time
import tty
from collections import deque

from piper_core.real_arm import DONE_TOLERANCE, JOINTS, RealArm, RealArmError

MARGIN = 0.01
SPEEDS = (3, 5, 10, 15, 20, 30, 40, 50)
JOG_STEPS = (0.25, 0.5, 1.0, 2.0)               # degrees
JOG_SPEED_MAX = 5
READY = [0.0, 1.0, -1.0, 0.0, 1.0, 0.0]         # SRDF group state `ready` (rad)
POSE_KEYS = {"z": "zero", "p": "park", "r": "ready"}
DEG = 180.0 / math.pi
B, DIM, RED, GRN, YEL, CYN, REV, RST = "\x1b[1m", "\x1b[2m", "\x1b[31m", "\x1b[32m", "\x1b[33m", "\x1b[36m", "\x1b[7m", "\x1b[0m"


class Console:
    """The state machine behind the screen. `key(k)` handles one key, `tick()` runs about every 0.1 s, `render()` returns the screen text."""

    def __init__(self, arm: RealArm, speed: int = 10, label: str = ""):
        self.arm, self.speed, self.label = arm, speed, label
        self.now = arm._now
        self.task = None            # a generator: yields None (next tick) or ("confirm", text)
        self.prompt = None
        self.progress = ""
        self.log_lines = deque(maxlen=7)
        self.jog_joint = None       # None: jog mode off; else 0..5 once chosen; -1 while waiting for the joint digit
        self.jog_target = None
        self.jog_step = 0.5
        self.quit = False
        self.snap = {}
        self.refresh()

    # ------------------------------------------------------------------ helpers
    def log(self, text: str):
        self.log_lines.append(text)

    def targets(self, name):
        if name == "zero":
            return [self.arm.zero()[j] for j in JOINTS]
        if name == "park":
            return [self.arm.park()[j] for j in JOINTS]
        return list(READY)

    def refresh(self):
        a, sdk = self.arm, self.arm.sdk
        pos = a.positions()
        q = None if pos is None else [pos[j] for j in JOINTS]          # a list in joint order
        st = sdk.get_arm_status()
        en = [bool(sdk.get_joint_enable_status(i)) for i in range(1, 7)]
        near = None
        if q is not None:
            for name in ("zero", "park", "ready"):
                if max(abs(x - g) for x, g in zip(q, self.targets(name))) < 0.015:
                    near = name
        self.snap = {"q": q, "enabled": en, "problems": a.problems(), "pose": a.pose_state() if q is not None else "no feedback", "near": near,
                     "arm_status": None if st is None else int(st.msg.arm_status), "width": a.gripper_opening(), "force": a.gripper_force()}

    def moves_allowed(self, goal, allow_slumped=False):
        """None, or the reason a move to `goal` is refused."""
        a, s = self.arm, self.snap
        if s["q"] is None:
            return "no joint feedback"
        if s["problems"]:
            return "; ".join(s["problems"])
        if not all(s["enabled"]):
            return "motors are not all enabled: press s"
        if s["pose"] not in ("zero", "valid") and not allow_slumped:
            return f"the arm is {s['pose']}: press s (its move to zero is the way out)"
        for j, g in zip(JOINTS, goal):
            lo, hi = a.limits[j]
            if not lo + MARGIN - 1e-9 <= g <= hi - MARGIN + 1e-9:
                return f"{j} target {g * DEG:.2f} deg is outside [{(lo + MARGIN) * DEG:.2f}, {(hi - MARGIN) * DEG:.2f}]"
        return None

    # ------------------------------------------------------------------ tasks (generators)
    def _wait(self, seconds):
        t0 = self.now()
        while self.now() - t0 < seconds:
            yield

    def _move(self, goal, label, allow_slumped=False):
        """Generator: one watched move_j. Returns True when reached; on a fault or timeout it holds and returns False."""
        a = self.arm
        self.refresh()
        why = self.moves_allowed(goal, allow_slumped)
        if why:
            self.log(f"refused: {why}")
            return False
        cfg = a._configure(self.speed)
        if not cfg:
            self.log(f"not moving: {cfg.message}")
            return False
        q = self.snap["q"]
        dist = max(abs(x - g) for x, g in zip(q, goal))
        timeout = 8.0 + 2.0 * dist / (self.speed / 100.0 * 3.0)
        a.sdk.move_j(list(goal))
        self.log(f"move_j sent: {label} ({dist * DEG:.1f} deg at {self.speed} %)")
        t0, ok_since = self.now(), None
        while True:
            yield
            self.refresh()
            s = self.snap
            if s["problems"]:
                a._hold()
                self.log("ABORT, holding: " + "; ".join(s["problems"]))
                return False
            err = max(abs(s["q"][k] - g) for k, g in enumerate(goal))
            self.progress = f"moving to {label}: {err * DEG:.1f} deg to go"
            if self.now() - t0 > timeout:
                a._hold()
                self.log(f"timeout after {timeout:.0f} s, holding")
                return False
            t = self.now()
            ok_since = (ok_since or t) if err < DONE_TOLERANCE else None
            if ok_since and t - ok_since >= 1.0:
                self.progress = ""
                self.log(f"{label} reached (error {err * DEG:.2f} deg)")
                return True

    def _t_pose(self, name):
        goal = self.targets(name)
        self.refresh()
        why = self.moves_allowed(goal)
        if why:
            self.log(f"refused: {why}")
            return
        big = max(abs(a - g) for a, g in zip(self.snap["q"], goal)) * DEG
        extra = "  BIG MOVE: check the space around the arm." if name == "ready" or big > 40 else ""
        yield ("confirm", f"move to {name.upper()} ({big:.1f} deg, speed {self.speed} %).{extra} y = go")
        yield from self._move(goal, f"{name} pose")

    def _t_start(self):
        a = self.arm
        self.refresh()
        s = self.snap
        if s["problems"]:
            self.log("not starting: " + "; ".join(s["problems"]))
            return
        if s["pose"] == "invalid":
            self.log("not starting: the arm is outside its limits by more than the slump, or has no feedback")
            return
        if all(s["enabled"]) and s["near"] == "zero":
            self.log("already enabled and at zero")
            return
        goal = self.targets("zero")
        big = max(abs(x - g) for x, g in zip(s["q"], goal)) * DEG
        what = "enable the motors and " if not all(s["enabled"]) else ""
        yield ("confirm", f"START: {what}move to ZERO ({big:.1f} deg, speed {self.speed} %). Arm clear, hand on the power switch. y = go")
        cfg = a._configure(self.speed)
        if not cfg:
            self.log(f"not starting: {cfg.message}")
            return
        if not all(s["enabled"]):
            en = a.enable()
            if not en:
                self.log(f"not moving: {en.message} (d disables, e stops)")
                return
            self.log("all six joints enabled")
        ok = yield from self._move(goal, "zero pose", allow_slumped=True)
        if ok:
            self.log("READY: enabled and at zero")

    def _t_stop(self):
        self.refresh()
        s = self.snap
        if s["pose"] == "slumped" and not any(s["enabled"]):
            self.log("already powered off and slumped")
            return
        why = self.moves_allowed(self.targets("zero"))
        if why:
            self.log(f"refused: {why}")
            return
        yield ("confirm", f"END SESSION: zero, then park, then you confirm the e-stop (damped lowering) and the reset. Speed {self.speed} %. y = go")
        for name in ("zero", "park"):
            if not (yield from self._move(self.targets(name), f"{name} pose")):
                return
            yield from self._wait(1.0)
        yield ("confirm", "At PARK. y = EMERGENCY-STOP now (the arm lowers slowly under damping, motors go limp). Support it with a hand")
        self.arm.sdk.electronic_emergency_stop()
        self.log("electronic e-stop sent")
        t0, prev, t_prev = self.now(), self.arm.positions(), self.now()     # dicts joint -> rad
        while self.now() - t0 < 15.0:
            yield
            if self.now() - t_prev >= 1.0:
                cur = self.arm.positions()
                if prev and cur and max(abs(cur[j] - prev[j]) for j in JOINTS) * DEG < 0.2:
                    break
                prev, t_prev = cur, self.now()
        self.refresh()
        self.log("at rest: " + ", ".join(f"{x * DEG:.1f}" for x in (self.snap["q"] or [])) + " deg")
        yield ("confirm", "Arm at rest. y = CLEAR the e-stop (reset: motors stay off, the arm stays powered)")
        yield from self._do_reset()

    def _do_reset(self):
        self.arm.sdk.reset()
        yield from self._wait(1.5)
        self.refresh()
        self.log(f"reset sent: arm_status {self.snap['arm_status']}, motors enabled {sum(self.snap['enabled'])}/6")

    def _t_clear(self):
        yield ("confirm", "CLEAR the e-stop: reset (motors off, the arm stays powered). y = go")
        yield from self._do_reset()

    def _t_disable(self):
        yield ("confirm", "DISABLE the motors: the arm goes limp and sags. Best from PARK, support it with a hand. y = disable")
        out = self.arm.disable()
        self.log(out.message)

    def _t_quit(self):
        if any(self.snap["enabled"]):
            yield ("confirm", "The motors are ON; quitting leaves them enabled and holding (nothing stops them). y = quit")
        self.quit = True

    # ------------------------------------------------------------------ task engine
    def _start(self, gen):
        self.task, self.prompt = gen, None
        self._advance()

    def _advance(self):
        while self.task is not None and self.prompt is None:
            try:
                item = next(self.task)
            except StopIteration:
                self.task, self.progress = None, ""
                return
            if isinstance(item, tuple):
                self.prompt = item[1]
                return
            return                      # None: wait for the next tick

    def _cancel(self, text="cancelled"):
        if self.task is not None:
            self.task.close()
        self.task, self.prompt, self.progress = None, None, ""
        self.log(text)

    # ------------------------------------------------------------------ keys
    def estop(self):
        """Immediate: electronic e-stop, whatever is going on."""
        self._cancel("EMERGENCY STOP")
        self.jog_joint = self.jog_target = None
        self.arm.sdk.electronic_emergency_stop()
        self.log("electronic e-stop sent: the arm lowers under damping. c clears it")

    def hold(self):
        was = self.task is not None or self.jog_joint is not None
        self._cancel("HOLD: stopped where it is, motors still on")
        self.jog_joint = self.jog_target = None
        if was and any(self.snap.get("enabled", [])):
            self.arm._hold()

    def key(self, k: str):
        if k in ("e", "E"):
            return self.estop()
        if k == " ":
            return self.hold()
        if self.prompt is not None:
            if k in ("y", "Y"):
                self.prompt = None
                self._advance()
            else:
                self._cancel()
            return
        if self.task is not None:
            self.log("busy: SPACE holds here, e = emergency stop")
            return
        if self.jog_joint is not None:
            return self._jog_key(k)
        self.refresh()
        if k in ("s", "S"):
            self._start(self._t_start())
        elif k in POSE_KEYS:
            self._start(self._t_pose(POSE_KEYS[k]))
        elif k in ("t", "T"):
            self._start(self._t_stop())
        elif k in ("c", "C"):
            self._start(self._t_clear())
        elif k in ("d", "D"):
            self._start(self._t_disable())
        elif k in ("q", "Q"):
            self._start(self._t_quit())
        elif k in ("j", "J"):
            self._jog_enter()
        elif k == "[":
            self.speed = SPEEDS[max(SPEEDS.index(self.speed) - 1, 0)] if self.speed in SPEEDS else 10
        elif k == "]":
            self.speed = SPEEDS[min(SPEEDS.index(self.speed) + 1, len(SPEEDS) - 1)] if self.speed in SPEEDS else 10
        elif k in (",", "."):
            i = JOG_STEPS.index(self.jog_step)
            self.jog_step = JOG_STEPS[max(i - 1, 0)] if k == "," else JOG_STEPS[min(i + 1, len(JOG_STEPS) - 1)]

    # ------------------------------------------------------------------ jog
    def _jog_enter(self):
        q = self.snap["q"]
        if q is None:
            self.log("refused: no joint feedback")
            return
        target = [min(max(a, self.arm.limits[j][0] + MARGIN), self.arm.limits[j][1] - MARGIN) for a, j in zip(q, JOINTS)]
        why = self.moves_allowed(target)           # the clamped present pose: the arm rests 0.5 deg short of a limit at zero and park
        if why:
            self.log(f"refused: {why}")
            return
        cfg = self.arm._configure(min(self.speed, JOG_SPEED_MAX))
        if not cfg:
            self.log(f"not jogging: {cfg.message}")
            return
        self.jog_joint, self.jog_target = -1, target
        self.log(f"JOG at {min(self.speed, JOG_SPEED_MAX)} %: choose a joint 1-6")

    def _jog_key(self, k):
        if k in ("j", "J", "ESC", "q"):
            self.jog_joint = self.jog_target = None
            self.log("jog off (the arm holds its last target)")
        elif k in "123456" and len(k) == 1:
            self.jog_joint = int(k) - 1
            self.log(f"jogging joint {k}: + / - or right / left, step {self.jog_step} deg")
        elif k in (",", "."):
            i = JOG_STEPS.index(self.jog_step)
            self.jog_step = JOG_STEPS[max(i - 1, 0)] if k == "," else JOG_STEPS[min(i + 1, len(JOG_STEPS) - 1)]
        elif k in ("+", "=", "RIGHT", "UP", "-", "_", "LEFT", "DOWN"):
            if self.jog_joint < 0:
                self.log("choose a joint first: 1-6")
                return
            self._jog_press(1 if k in ("+", "=", "RIGHT", "UP") else -1)

    def _jog_press(self, sign):
        self.refresh()
        s = self.snap
        if s["problems"] or not all(s["enabled"]) or s["q"] is None:
            self.log("jog refused: " + ("; ".join(s["problems"]) or "motors off or no feedback"))
            return
        j = self.jog_joint
        new = self.jog_target[j] + sign * self.jog_step / DEG
        lo, hi = self.arm.limits[JOINTS[j]]
        if not lo + MARGIN <= new <= hi - MARGIN:
            self.log(f"jog refused: joint {j + 1} would leave [{(lo + MARGIN) * DEG:.1f}, {(hi - MARGIN) * DEG:.1f}] deg")
            return
        if abs(new - s["q"][j]) > 3.0 / DEG:
            self.log("jog refused: the arm is more than 3 deg behind the target, wait for it")
            return
        self.jog_target[j] = new
        self.arm.sdk.move_j(list(self.jog_target))
        self.log(f"joint {j + 1} -> {new * DEG:.2f} deg")

    # ------------------------------------------------------------------ loop and screen
    def tick(self):
        self.refresh()
        if self.task is not None and self.prompt is None:
            self._advance()

    def render(self) -> str:
        s = self.snap
        en = s["enabled"]
        motors = f"{GRN}MOTORS ON{RST}" if all(en) else (f"{YEL}MOTORS PARTLY ON{RST}" if any(en) else f"{DIM}MOTORS OFF{RST}")
        estop = f"{RED}{B}E-STOP{RST}" if s["arm_status"] == 1 else f"{DIM}no e-stop{RST}"
        faults = f"{RED}{'; '.join(s['problems'])}{RST}" if s["problems"] else f"{GRN}no faults{RST}"
        pose = s["pose"] + (f" · at {s['near']}" if s["near"] else "")
        out = [f"{B}PIPER ARM{RST} {self.label}   speed {B}{self.speed} %{RST}   jog step {self.jog_step}°",
               f"  {motors}   {estop}   {faults}",
               f"  pose: {B}{pose}{RST}", ""]
        q = s["q"]
        for i, j in enumerate(JOINTS):
            lo, hi = self.arm.limits[j]
            v = "   ----" if q is None else f"{q[i] * DEG:8.2f}"
            mark = f"{CYN}<{RST}" if self.jog_joint == i else " "
            out.append(f"  j{i + 1} {v}°  {DIM}[{lo * DEG:7.1f}, {hi * DEG:6.1f}]{RST} {mark}")
        w, f = s["width"], s["force"]
        out.append(f"  gripper {'---' if w is None else f'{w * 1000:.1f} mm'}   force {'---' if f is None else f'{f:.2f} N'}")
        out.append("")
        if self.prompt:
            out.append(f"{REV} {self.prompt} {RST}   (anything else cancels)")
        elif self.task is not None:
            out.append(f"{YEL}{self.progress or 'working...'}{RST}   SPACE = hold here · e = EMERGENCY STOP")
        elif self.jog_joint is not None:
            out.append(f"{CYN}JOG: " + ("choose a joint 1-6" if self.jog_joint < 0 else f"joint {self.jog_joint + 1}: + / - (or arrows)") + f"   , . step   j / Esc leave{RST}")
        else:
            out.append("")
        out += [f"  {DIM}{x}{RST}" for x in self.log_lines]
        out += ["", f"{B}s{RST} start(enable+zero)  {B}z{RST} zero  {B}p{RST} park  {B}r{RST} ready  {B}j{RST} jog  {B}t{RST} end session  {B}d{RST} disable  {B}c{RST} clear e-stop",
                f"{B}[ ]{RST} speed   {B}SPACE{RST} hold   {RED}{B}e EMERGENCY STOP{RST}   {B}q{RST} quit"]
        return "\n".join(out)


# ---------------------------------------------------------------------- terminal
def _keys(data: bytes):
    """Split raw terminal input into key names (arrows as LEFT/RIGHT/UP/DOWN, a lone Esc as ESC)."""
    out, i = [], 0
    names = {b"[A": "UP", b"[B": "DOWN", b"[C": "RIGHT", b"[D": "LEFT"}
    while i < len(data):
        if data[i] == 0x1B:
            tail = data[i + 1:i + 3]
            if tail in names:
                out.append(names[tail])
                i += 3
                continue
            out.append("ESC")
            i += 1
            continue
        out.append(chr(data[i]))
        i += 1
    return out


def run(console: Console, fd=None, period=0.1):
    fd = sys.stdin.fileno() if fd is None else fd
    old = termios.tcgetattr(fd)
    sys.stdout.write("\x1b[?1049h\x1b[?25l")
    try:
        tty.setcbreak(fd)
        while not console.quit:
            screen = console.render().replace("\n", "\x1b[K\n")
            sys.stdout.write("\x1b[H" + screen + "\x1b[K\x1b[J")
            sys.stdout.flush()
            if select.select([fd], [], [], period)[0]:
                for k in _keys(os.read(fd, 32)):
                    console.key(k)
            console.tick()
    except KeyboardInterrupt:
        console.hold()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write("\x1b[?25h\x1b[?1049l")
        sys.stdout.flush()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--canbus", "-b", default=None, help="CAN bus name (default: the only CAN bus there is, else can0)")
    ap.add_argument("--speed", type=int, default=10, choices=SPEEDS, help="starting speed percent (default 10)")
    args = ap.parse_args(argv)
    canbus = args.canbus
    if canbus is None:
        found = sorted(d for d in os.listdir("/sys/class/net") if open(f"/sys/class/net/{d}/type").read().strip() == "280") if os.path.isdir("/sys/class/net") else []
        canbus = found[0] if len(found) == 1 else "can0"
    if not sys.stdin.isatty():
        raise SystemExit("piper-arm needs a terminal (keys are read one at a time)")
    try:
        arm = RealArm.connect(canbus)
    except RealArmError as e:
        raise SystemExit(f"{e}\nNothing was sent to the arm.")
    try:
        console = Console(arm, speed=args.speed, label=f"· {canbus}")
        run(console)
        print("piper-arm closed. Motors:", "ON (holding)" if any(console.snap["enabled"]) else "off", "· pose:", console.snap["pose"])
    finally:
        arm.close()


if __name__ == "__main__":
    main()
