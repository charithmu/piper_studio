"""piper-arm: one terminal console for the real Piper. Live status on top, one key per action below. No ROS.

    piper-arm [-b can0] [--speed 10]       # needs the CAN bus up (scripts/can_up.sh) and pyAgxArm (the workspace .venv has it)

It is the operator's cockpit for what was proven on the arm with tools/real_arm/ (2026-11-10): start, known poses, one-joint jog, end of session, e-stop.
Motion needs the operator: every action that moves or switches the motors waits for Enter (anything else cancels). Exceptions, always immediate:
`e` EMERGENCY STOP (electronic e-stop, damped lowering) and SPACE (stop where you are, keep holding). Both work while a move is running.

Keys:  s start (enable, go to zero)   z zero   p park   r ready (big move)   j jog a joint (1-6, then + / - or arrows, j or Esc leaves)
       t end session (zero, park, e-stop, reset)   d disable motors (the arm sags)   c clear the e-stop (reset)   [ ] speed   , . jog step   q quit
       o open the gripper   l close it (Enter confirms; a close on an object ends 'stalled' with the width and force)   f gripper force   g gripper step mode (+ / - 5 mm)
Gripper force starts at 1.0 N (the vendor default) and is capped at 3.0 N by RealArm.gripper_max_effort; `f` cycles 0.5 1 1.5 2 3 N.
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

from piper_core.real_arm import DONE_TOLERANCE, JOINTS, MODE_J, GripperWatch, RealArm, RealArmError

SPEEDS = (3, 5, 10, 15, 20, 30, 40, 50)
JOG_STEPS = (0.25, 0.5, 1.0, 2.0)               # degrees
JOG_SPEED_MAX = 5
GRIP_FORCES = (0.5, 1.0, 1.5, 2.0, 3.0)         # N, `f` cycles; RealArm.gripper_max_effort (3 N) is the hard cap
GRIP_STEPS = (1.0, 2.0, 5.0, 10.0)              # mm per + / - in gripper step mode
GRIP_TIMEOUT = 6.0
READY = [0.0, 1.0, -1.0, 0.0, 1.0, 0.0]         # SRDF group state `ready` (rad)
POSE_KEYS = {"z": "zero", "p": "park", "r": "ready"}
DEG = 180.0 / math.pi
B, DIM, RED, GRN, YEL, CYN, REV, RST = "\x1b[1m", "\x1b[2m", "\x1b[31m", "\x1b[32m", "\x1b[33m", "\x1b[36m", "\x1b[7m", "\x1b[0m"


def mm(v):
    return "---" if v is None else f"{v * 1000:.1f}"


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
        self.grip_force, self.grip_step, self.grip_mode, self.grip_active = arm.gripper_effort, 5.0, False, False
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
        self.snap = {"q": q, "enabled": en, "problems": a.problems(), "pose": a._pose_state() if q is not None else "no feedback", "near": near,
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
        if not all(math.isfinite(g) for g in goal):
            return "a target is not a number"
        if not allow_slumped:
            verdict = a._guard(dict(zip(JOINTS, map(float, goal))))        # the one guard of piper_core (limits, margin, finite, faults), as RealArm uses it
            return None if verdict else verdict.message
        for j, g in zip(JOINTS, goal):                                       # start's single move out of the powered-off pose: only the target is checked
            lo, hi = a.limits[j]
            if not lo + a.margin - 1e-9 <= g <= hi - a.margin + 1e-9:
                return f"{j} target {g * DEG:.2f} deg is outside [{(lo + a.margin) * DEG:.2f}, {(hi - a.margin) * DEG:.2f}]"
        return None

    # ------------------------------------------------------------------ tasks (generators)
    # The arm's blocking helpers (configure up to 2.5 s, enable up to 5 s, clear-estop 1.5 s) are re-done here as generators that yield between polls,
    # so `e` (emergency stop) and SPACE are read while they run instead of after them.
    def _g_configure(self, speed):
        a = self.arm
        a.sdk.set_speed_percent(int(min(max(speed, 1), 100)))
        a.sdk.set_motion_mode(a.sdk.OPTIONS.MOTION_MODE.J)
        t0 = self.now()
        while self.now() - t0 < 2.5:
            st = a.sdk.get_arm_status()
            if st is not None and int(st.msg.mode_feedback) == MODE_J and int(st.msg.ctrl_mode) == 1:
                return True
            yield
        return False

    def _g_enable(self):
        a = self.arm
        t0 = self.now()
        while self.now() - t0 < 5.0 and not a.sdk.enable():
            yield
        return all(a.sdk.get_joint_enable_status(i) for i in range(1, 7))

    def _g_clear_estop(self):
        """True if cleared, False if not, None if no e-stop was active."""
        a = self.arm
        st = a.sdk.get_arm_status()
        if st is None or int(st.msg.arm_status) != 1:
            return None
        a.sdk.reset()
        yield from self._wait(1.5)
        st = a.sdk.get_arm_status()
        return st is not None and int(st.msg.arm_status) == 0

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
        if not (yield from self._g_configure(self.speed)):
            self.log("not moving: the arm did not report CAN control + MOVE_J")
            return False
        self.refresh()
        why = self.moves_allowed(goal, allow_slumped)                       # re-checked after the wait: anything may have happened meanwhile
        if why:
            self.log(f"refused: {why}")
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
            ok_since = (t if ok_since is None else ok_since) if err < DONE_TOLERANCE else None
            if ok_since is not None and t - ok_since >= 1.0:
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
        yield ("confirm", f"move to {name.upper()} ({big:.1f} deg, speed {self.speed} %).{extra} Enter = go")
        yield from self._move(goal, f"{name} pose")

    def _t_start(self):
        a = self.arm
        self.refresh()
        s = self.snap
        estop = s["arm_status"] == 1
        if s["problems"] and not estop:
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
        what = ("clear the e-stop, " if estop else "") + ("enable the motors and " if not all(s["enabled"]) else "")
        yield ("confirm", f"START: {what}move to ZERO ({big:.1f} deg, speed {self.speed} %). Arm clear, hand on the power switch. Enter = go")
        if estop:
            cleared = yield from self._g_clear_estop()
            if not cleared:
                self.log("not starting: could not clear the e-stop (power-cycle the arm)")
                return
            self.log("e-stop cleared")
            self.refresh()
            if self.snap["problems"]:
                self.log("not starting: " + "; ".join(self.snap["problems"]))
                return
        if not (yield from self._g_configure(self.speed)):
            self.log("not starting: the arm did not report CAN control + MOVE_J")
            return
        if not all(s["enabled"]) or estop:
            if not (yield from self._g_enable()):
                self.log("not moving: could not enable all six joints (d disables, e stops)")
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
        yield ("confirm", f"END SESSION: zero, then park, then you confirm the e-stop (damped lowering) and the reset. Speed {self.speed} %. Enter = go")
        for name in ("zero", "park"):
            if not (yield from self._move(self.targets(name), f"{name} pose")):
                return
            yield from self._wait(1.0)
        yield ("confirm", "At PARK. Enter = EMERGENCY-STOP now (the arm lowers slowly under damping, motors go limp). Support it with a hand")
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
        yield ("confirm", "Arm at rest. Enter = CLEAR the e-stop (reset: motors stay off, the arm stays powered)")
        yield from self._do_reset()

    def _do_reset(self):
        self.arm.sdk.reset()
        yield from self._wait(1.5)
        self.refresh()
        self.log(f"reset sent: arm_status {self.snap['arm_status']}, motors enabled {sum(self.snap['enabled'])}/6")

    def _t_clear(self):
        yield ("confirm", "CLEAR the e-stop: reset (motors off, the arm stays powered). Enter = go")
        yield from self._do_reset()

    def _t_disable(self):
        yield ("confirm", "DISABLE the motors: the arm goes limp and sags. Best from PARK, support it with a hand. Enter = disable")
        out = self.arm.disable()
        self.log(out.message)

    def _t_gripper(self, width, label):
        a = self.arm
        self.refresh()
        s = self.snap
        if a.gripper is None:
            self.log("refused: no gripper driver")
            return
        if s["problems"]:
            self.log("refused: " + "; ".join(s["problems"]))
            return
        yield ("confirm", f"GRIPPER: {label} to {width * 1000:.0f} mm at {self.grip_force} N (now {mm(s['width'])} mm). Enter = go")
        out = a.gripper_command(width, self.grip_force)
        if not out:
            self.log(f"refused: {out.message}")
            return
        self.grip_active = True
        try:
            watch, t0 = GripperWatch(width, self.now, self.snap["width"]), self.now()
            while True:
                yield
                self.refresh()
                w = self.snap["width"]
                reason = watch.update(w) if w is not None else None
                self.progress = f"gripper {mm(w)} mm -> {width * 1000:.0f} mm"
                if reason is None and self.now() - t0 > GRIP_TIMEOUT:
                    reason = "timeout"
                if reason:
                    if reason != "reached" and w is not None:
                        a.gripper_command(w, self.grip_force)               # stop pushing: aim at where it is
                    f = self.snap["force"]
                    self.log(f"gripper {reason}: {mm(w)} mm, force {'---' if f is None else format(f, '.2f')} N"
                             + (" (stopped short: object or end stop)" if reason == "stalled" else " (no movement: is it powered and enabled?)" if reason == "no_motion" else ""))
                    return
        finally:
            self.grip_active = False

    def _grip_key(self, k):
        if k in ("g", "G", "ESC", "q"):
            self.grip_mode = False
            self.log("gripper step mode off")
        elif k in (",", "."):
            i = GRIP_STEPS.index(self.grip_step)
            self.grip_step = GRIP_STEPS[max(i - 1, 0)] if k == "," else GRIP_STEPS[min(i + 1, len(GRIP_STEPS) - 1)]
        elif k in ("+", "=", "RIGHT", "UP", "-", "_", "LEFT", "DOWN"):
            self.refresh()
            w = self.snap["width"]
            if w is None:
                self.log("gripper step refused: no width reading")
                return
            new = min(max(w + (1 if k in ("+", "=", "RIGHT", "UP") else -1) * self.grip_step / 1000.0, 0.0), self.arm.gripper_stroke)
            out = self.arm.gripper_command(new, self.grip_force)
            self.log(f"gripper -> {new * 1000:.1f} mm at {self.grip_force} N" if out else f"gripper step refused: {out.message}")

    def _t_quit(self):
        if any(self.snap["enabled"]):
            yield ("confirm", "The motors are ON; quitting leaves them enabled and holding (nothing stops them). Enter = quit")
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
        grip = self.grip_active
        was = (self.task is not None and self.prompt is None) or self.jog_joint is not None      # something is moving; a confirm prompt moves nothing, so there is nothing to hold
        self._cancel("HOLD: stopped where it is, motors still on")
        self.jog_joint = self.jog_target = None
        if grip and self.snap.get("width") is not None:
            self.arm.gripper_command(self.snap["width"], self.grip_force)       # gripper: aim at where it is, stops pushing
        if was and any(self.snap.get("enabled", [])):
            self.arm._hold()

    def key(self, k: str):
        if k in ("e", "E"):
            return self.estop()
        if k == " ":
            return self.hold()
        if self.prompt is not None:
            if k == "ENTER":
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
        if self.grip_mode:
            return self._grip_key(k)
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
        elif k in ("o", "O"):
            self._start(self._t_gripper(self.arm.gripper_stroke, "OPEN"))
        elif k in ("l", "L"):
            self._start(self._t_gripper(0.0, "CLOSE"))
        elif k in ("g", "G"):
            if self.arm.gripper is None or self.snap["problems"]:
                self.log("refused: " + ("no gripper driver" if self.arm.gripper is None else "; ".join(self.snap["problems"])))
            else:
                self.grip_mode = True
                self.log(f"GRIPPER STEP at {self.grip_force} N: + / - move {self.grip_step} mm, , . change the step, g / Esc leave")
        elif k in ("f", "F"):
            i = GRIP_FORCES.index(self.grip_force) if self.grip_force in GRIP_FORCES else 1
            self.grip_force = GRIP_FORCES[(i + 1) % len(GRIP_FORCES)]
            self.grip_force = min(self.grip_force, self.arm.gripper_max_effort)
            self.log(f"gripper force {self.grip_force} N (cap {self.arm.gripper_max_effort} N)")
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
        target = [min(max(a, self.arm.limits[j][0] + self.arm.margin), self.arm.limits[j][1] - self.arm.margin) for a, j in zip(q, JOINTS)]
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
        m = self.arm.margin
        if not lo + m <= new <= hi - m:
            self.log(f"jog refused: joint {j + 1} would leave [{(lo + m) * DEG:.1f}, {(hi - m) * DEG:.1f}] deg")
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
        out.append(f"  gripper {mm(w)} mm   force {'---' if f is None else format(f, '.2f')} N   {DIM}command force {self.grip_force} N{RST}")
        out.append("")
        if self.prompt:
            out.append(f"{REV} {self.prompt} {RST}   (anything else cancels)")
        elif self.task is not None:
            out.append(f"{YEL}{self.progress or 'working...'}{RST}   SPACE = hold here · e = EMERGENCY STOP")
        elif self.grip_mode:
            out.append(f"{CYN}GRIPPER STEP: + / - {self.grip_step} mm at {self.grip_force} N   , . step   g / Esc leave{RST}")
        elif self.jog_joint is not None:
            out.append(f"{CYN}JOG: " + ("choose a joint 1-6" if self.jog_joint < 0 else f"joint {self.jog_joint + 1}: + / - (or arrows)") + f"   , . step   j / Esc leave{RST}")
        else:
            out.append("")
        out += [f"  {DIM}{x}{RST}" for x in self.log_lines]
        out += ["", f"{B}s{RST} start(enable+zero)  {B}z{RST} zero  {B}p{RST} park  {B}r{RST} ready  {B}j{RST} jog  {B}t{RST} end session  {B}d{RST} disable  {B}c{RST} clear e-stop",
                f"{B}o{RST} open grip  {B}l{RST} close grip  {B}g{RST} grip steps  {B}f{RST} grip force   {B}[ ]{RST} speed   {B}SPACE{RST} hold   {RED}{B}e EMERGENCY STOP{RST}   {B}q{RST} quit"]
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
        out.append("ENTER" if data[i] in (0x0D, 0x0A) else chr(data[i]))
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
