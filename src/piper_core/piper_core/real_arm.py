"""The real Piper without ROS: the same rules as the ROS stack, straight on the vendor SDK (pyAgxArm), for planners such as cuRobo.

    arm = RealArm.connect("can0")          # one firmware query; picks the SDK profile for the arm's firmware
    arm.start()                            # preflight, motors on, leave the slumped pose for zero, check zero
    arm.execute(trajectory)                # [(t_seconds, [q1..q6]), ...] from a planner, every point through the guard
    arm.stop()                             # zero, wait, park, wait, motors off

The rules are the ones `command_guard` applies (`piper_core.check_command`, `zero_pose`, `classify_state`): nothing is commanded outside
the limits, nothing from the slumped pose, nothing while the arm reports a fault; the only way out of the slumped pose is the one checked
move in `start()`. Joint names are the official ones (joint1 .. joint6), angles are radians.

NOT YET RUN ON THE REAL ARM. It is written against the SDK calls `tools/real_arm/arm_zero.py` used on 2026-10-14 and tested against a
fake SDK arm (test_real_arm.py). The first run needs the user at the arm: `execute()` streams one `move_j` per trajectory point, which
the firmware smooths (documented as streamable in docs/real_arm/REAL_ARM.md, never tried with 25 ms points); the speed percent derived
from the trajectory (assumes 100 % = the firmware's 3.0 rad/s) is unverified.
"""

from __future__ import annotations

import contextlib
import functools
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from piper_core.bus_lock import BusBusyError, BusLock
from piper_core.backend import (ARM_JOINTS, MAX_STEP, MAX_VELOCITY, ArmState, BackendError, ExecResult, Outcome, Trajectory, check_start,
                               check_trajectory)
from piper_core.guard import GuardConfig, check_command
from piper_core.poses import classify_state, zero_pose

JOINTS = tuple(f"joint{i}" for i in range(1, 7))
PARK_JOINT5 = 0.3840           # rad (22.0 deg): park = zero with joint 5 lowered (SRDF `park`, test_named_poses)
FIRMWARE_MAX_VELOCITY = MAX_VELOCITY   # rad/s, read from the arm (piper-01.readout.json); one definition, in piper_core.backend
DONE_TOLERANCE = 0.01          # rad (0.57 deg): "at the goal"
DONE_HOLD_S = 1.0
MODE_J = 1                     # mode_feedback value for CAN control + MOVE_J
DRIVER_FAULT_BITS = ("voltage_too_low", "motor_overheating", "driver_overcurrent", "driver_overheating",
                     "collision_status", "driver_error_status", "stall_status")


AUTO_LAG_BASE_S, AUTO_LAG_PER_RAD_S = 0.035, 0.073     # the arm trails a streamed setpoint by about 35 ms + 73 ms per rad/s (joint 1, 2026-11-10: 45 / 75 / 95 / 115 ms at 0.14 / 0.55 / 0.82 / 1.10 rad/s)
AUTO_LOOKAHEAD_MAX = 6
GRIPPER_STROKE = 0.07          # m, firmware-reported (max_range_config), docs/real_arm/arms/piper-01.md
GRIPPER_DEFAULT_EFFORT = 1.0   # N, the vendor default (agx_arm_ctrl `gripper_default_effort`)
GRIPPER_MAX_EFFORT = 3.0       # N, OUR cap for commands, not a vendor figure (none documented). Raise it deliberately: arm.gripper_max_effort = 5.0


@dataclass
class GripperResult:
    reason: str        # "reached" | "stalled" (stopped before the target: an object or the end stop) | "no_motion" | "timeout"
    width: float       # m, last measured
    force: float | None
    duration: float = 0.0                                   # s from the command to the end condition (includes the 0.3 s the width must hold for "reached")
    trace: list = field(default_factory=list)               # (t s from the command, width m, force N) about every 20 ms


@dataclass
class ArmStatus:
    """What a downstream user may know about the arm: whether it takes commands, and why not."""
    started: bool            # enabled, inside its bounds, no fault, feedback fresh: execute / move_to / command / hold are accepted
    enabled: bool            # all six motors on
    e_stop: bool
    faults: list
    at_zero: bool
    feedback_age: float      # seconds since the joint feedback last changed (0 = fresh)
    gripper: bool            # gripper feedback present


class GripperWatch:
    """Decides when a gripper move is over, from measured widths only. Closing on an object never reaches the target, so 'stopped moving' counts."""
    TOL, HOLD, WINDOW, STILL, MOVED, NO_MOTION_S = 0.001, 0.3, 0.5, 0.0003, 0.0015, 1.0

    def __init__(self, target: float, clock=time.monotonic, width0: float | None = None):
        self.target, self.now, self.w0, self.t0 = target, clock, width0, clock()
        self.hist, self.near_since = deque(), None

    def update(self, w: float) -> str | None:
        t = self.now()
        if self.w0 is None:
            self.w0 = w
        self.hist.append((t, w))
        while self.hist and t - self.hist[0][0] > 2 * self.WINDOW:
            self.hist.popleft()
        if abs(w - self.target) <= self.TOL:
            self.near_since = t if self.near_since is None else self.near_since
            if t - self.near_since >= self.HOLD:
                return "reached"
        else:
            self.near_since = None
        if abs(w - self.w0) >= self.MOVED and any(t - x >= self.WINDOW for x, _ in self.hist):
            recent = [v for x, v in self.hist if t - x <= self.WINDOW]
            if max(recent) - min(recent) < self.STILL:
                return "stalled"
        if t - self.t0 >= self.NO_MOTION_S and abs(w - self.w0) < self.MOVED:
            return "no_motion"
        return None


class RealArmError(BackendError):
    """A refused or aborted action. The arm is left holding (motors on) unless the message says otherwise."""


def _in_op(fn):
    """Run a motion method as an 'operation': abort() reaches it, and the abort flag is cleared when the last operation ends."""
    @functools.wraps(fn)
    def wrapper(self, *a, **k):
        with self._op():
            return fn(self, *a, **k)
    return wrapper


def _exclusive(busy):
    """One motion operation at a time (execute, start, stop, move_to), decided atomically: a second caller gets `busy(self)` and nothing is sent."""
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(self, *a, **k):
            with self._op_lock:
                if self._executing:
                    return busy(self)
                self._executing = True
            try:
                return fn(self, *a, **k)
            finally:
                self._executing = False
        return wrapper
    return deco


def _busy_outcome(self):
    return Outcome(False, "another motion is running (execute, start, stop or move_to): abort() it first", {"reason": "not_ready"})


def _busy_result(self):
    return ExecResult(False, "not_ready", "another motion is running (execute, start, stop or move_to): abort() it first")


class RealArm:
    def __init__(self, sdk_arm, limits: dict[str, tuple[float, float]] | None = None, margin: float = 0.01, speed_percent: int = 30,
                 clock=time.monotonic, sleep=time.sleep):
        """`sdk_arm` is a connected pyAgxArm arm. `limits` {joint1: (lo, hi), ...} default to the limits stored in the arm."""
        self.sdk, self.margin, self.speed_percent = sdk_arm, margin, speed_percent
        self._now, self._sleep = clock, sleep
        self.limits = limits or self._stored_limits()
        self.enabled = False
        self.gripper = None          # the SDK gripper driver; set by connect()
        self.gripper_stroke, self.gripper_effort, self.gripper_max_effort = GRIPPER_STROKE, GRIPPER_DEFAULT_EFFORT, GRIPPER_MAX_EFFORT
        self.last_gripper: GripperResult | None = None
        self._abort, self._executing = threading.Event(), False      # abort(): stop whatever is running (execute, start, stop, move_to) from another thread
        self._op_lock, self._active = threading.Lock(), 0           # operations in progress; abort() only does something while one runs
        self.max_feedback_age = 0.25     # s: joint feedback that has not changed for longer is a fault (arm silent or CAN lost)
        self._ts_last, self._ts_seen, self._lock = None, self._now(), None
        self.last_stream: dict | None = None                         # timing and fine-grained measured joints of the last execute(), see stream_metrics

    @classmethod
    def connect(cls, canbus: str = "can0", gripper: bool = True, **kwargs) -> "RealArm":
        """Open the arm with the SDK profile that matches its firmware (one firmware query, no motion). `gripper`: also read the gripper (sends nothing)."""
        from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config, resolve_firmware_profile
        state = f"/sys/class/net/{canbus}/operstate"
        lock = BusLock(canbus)
        if os.path.isdir("/sys/class/net"):
            if not os.path.exists(state):
                raise RealArmError(f"CAN bus '{canbus}' does not exist (USB-CAN adapter plugged in? lsusb shows 1d50:606f)")
            if open(state).read().strip() != "up":
                raise RealArmError(f"{canbus} is down: bring it up first (asks for sudo): scripts/can_up.sh {canbus}")
        try:
            lock.acquire()
        except BusBusyError as e:
            raise RealArmError(str(e)) from None
        try:
            probe = AgxArmFactory.create_arm(create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel=canbus))
            probe.connect()
            try:
                time.sleep(0.5)
                fw = probe.get_firmware(timeout=3.0)
            finally:
                probe.disconnect()
            if not fw:
                raise RealArmError("no firmware answer (arm silent, another program on the bus, master mode or teach playback?)")
            return cls._connected(canbus, fw, gripper, lock, kwargs)
        except BaseException:
            lock.release()
            raise

    @classmethod
    def _connected(cls, canbus, fw, gripper, lock, kwargs) -> "RealArm":
        from pyAgxArm import AgxArmFactory, ArmModel, create_agx_arm_config, resolve_firmware_profile
        profile = resolve_firmware_profile(ArmModel.PIPER, fw["software_version"])
        sdk = AgxArmFactory.create_arm(create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=profile, channel=canbus))
        grip = sdk.init_effector(sdk.OPTIONS.EFFECTOR.AGX_GRIPPER) if gripper else None     # before connect(); sends nothing
        sdk.connect()
        time.sleep(0.5)
        try:
            arm = cls(sdk, **kwargs)
        except BaseException:
            sdk.disconnect()
            raise
        arm.gripper = grip
        arm._lock = lock
        return arm

    @contextlib.contextmanager
    def _op(self):
        with self._op_lock:
            self._active += 1
        try:
            yield
        finally:
            with self._op_lock:
                self._active -= 1
                if self._active == 0:
                    self._abort.clear()

    def close(self):
        """Disconnect and free the bus. The motors are NOT touched: an enabled arm keeps holding its last target (stop() first for a clean end). If the process dies the arm also keeps holding;
        the operator stops it (piper-arm `e`, or tools/real_arm/arm_stop.py)."""
        try:
            self.sdk.disconnect()
        finally:                                          # the bus lock is freed even if the disconnect raises
            if self._lock is not None:
                self._lock.release()
                self._lock = None

    # ------------------------------------------------------------------ reads (send nothing)
    def _stored_limits(self) -> dict[str, tuple[float, float]]:
        out = {}
        for i, name in enumerate(JOINTS, 1):
            r = self.sdk.get_joint_angle_vel_limits(i, timeout=2.0)
            if r is None:
                raise RealArmError(f"stored limits of {name} unreadable")
            out[name] = (float(r.msg.min_angle_limit), float(r.msg.max_angle_limit))
        return out

    def positions(self) -> dict[str, float] | None:
        ja = self.sdk.get_joint_angles()
        return None if ja is None else dict(zip(JOINTS, (float(a) for a in ja.msg)))

    def feedback_age(self) -> float:
        """Seconds since the joint feedback last changed (0 when it just did or when the SDK gives no stamp). Works whatever clock the SDK stamps with: it watches the stamp advance."""
        ja = self.sdk.get_joint_angles()
        ts = getattr(ja, "timestamp", None)
        if ts is None:
            return 0.0
        now = self._now()
        if ts != self._ts_last:
            self._ts_last, self._ts_seen = ts, now
        return now - self._ts_seen

    def problems(self) -> list[str]:
        """Everything that must stop a motion: no feedback, status or error code, driver fault bits, teach or master mode."""
        out = []
        st = self.sdk.get_arm_status()
        if st is None or self.positions() is None:
            return ["no joint or status feedback"]
        m = st.msg
        age = self.feedback_age()
        if age > self.max_feedback_age:
            out.append(f"feedback is stale ({age:.2f} s without an update): arm silent or CAN lost")
        if int(m.ctrl_mode) not in (0, 1):
            out.append(f"control mode {int(m.ctrl_mode)} (not standby or CAN control)")
        if int(m.teach_status) != 0:
            out.append(f"teach state {int(m.teach_status)}")
        if int(m.arm_status) != 0 or int(m.err_code) != 0:
            out.append(f"arm_status={int(m.arm_status)} err_code={int(m.err_code)}")
        for i in range(1, 7):
            d = self.sdk.get_driver_states(i)
            if d is not None:
                out += [f"joint{i}:{k}" for k in DRIVER_FAULT_BITS if getattr(d.msg.foc_status, k, False)]
        return out

    def gripper_opening(self) -> float | None:
        """Measured gripper width in metres (CAN 0x2A8, 1e-6 m), None if unavailable. Reads only."""
        gs = self.gripper.get_gripper_status() if self.gripper is not None else None
        return None if gs is None else float(gs.msg.value)

    def gripper_force(self) -> float | None:
        """Measured gripper force in newtons, None if unavailable. Reads only."""
        gs = self.gripper.get_gripper_status() if self.gripper is not None else None
        return None if gs is None else float(gs.msg.force)

    def gripper_command(self, width: float, effort: float | None = None) -> Outcome:
        """Send one gripper target (non-blocking). Refused, nothing sent: no gripper, width outside [0, stroke], effort not in (0, max], a fault or e-stop reported, motors off
        (the gripper does not move unless the arm is started; it also keeps holding through an e-stop and a stop(): open it first if it should be released).
        To raise the force limit set `arm.gripper_max_effort` (our cap, 3 N); the default effort is `arm.gripper_effort` (1 N, the vendor default)."""
        effort = self.gripper_effort if effort is None else float(effort)
        if self.gripper is None:
            return Outcome(False, "no gripper driver (connect with gripper=True)")
        if not (math.isfinite(width) and -1e-9 <= width <= self.gripper_stroke + 1e-9):
            return Outcome(False, f"gripper width {width} m is outside [0, {self.gripper_stroke}]")
        if not (math.isfinite(effort) and 0.0 < effort <= self.gripper_max_effort):
            return Outcome(False, f"gripper effort {effort} N is outside (0, {self.gripper_max_effort}] (raise arm.gripper_max_effort deliberately)")
        bad = self.problems()
        if bad:
            return Outcome(False, "; ".join(bad))
        if not all(self.sdk.get_joint_enable_status(i) for i in range(1, 7)):       # measured 2026-11-10: the gripper does not move while the motors are off
            return Outcome(False, "the gripper moves only while the arm is started (motors on): call start()")
        self.gripper.move_gripper_m(value=min(max(float(width), 0.0), self.gripper_stroke), force=effort)
        return Outcome(True)

    def set_gripper(self, width: float, effort: float | None = None, timeout: float = 5.0) -> float:
        """Move the gripper and block until it reached the width, stopped short of it (object or end stop) or `timeout`. Returns the measured width in metres.
        `self.last_gripper` says how it ended ('reached' | 'stalled' | 'no_motion' | 'timeout'), the width and the force then; a stalled close is how a held object shows.
        Raises RealArmError when the command is refused (nothing was sent)."""
        out = self.gripper_command(width, effort)
        if not out:
            raise RealArmError(out.message)
        watch, t0, trace = GripperWatch(float(width), self._now, self.gripper_opening()), self._now(), []
        while True:
            self._sleep(0.02)
            w = self.gripper_opening()
            if w is not None:
                trace.append((self._now() - t0, w, self.gripper_force()))
            reason = watch.update(w) if w is not None else None
            if reason is None and self._now() - t0 > timeout:
                reason = "timeout"
            if reason:
                if reason != "reached" and w is not None:               # stop pushing: aim at where it is
                    self.gripper.move_gripper_m(value=min(max(w, 0.0), self.gripper_stroke), force=self.gripper_effort if effort is None else float(effort))
                self.last_gripper = GripperResult(reason, w if w is not None else math.nan, self.gripper_force(), self._now() - t0, trace)
                return self.last_gripper.width

    def status(self) -> ArmStatus:
        """Whether commands are accepted and, if not, why. Reads only. This is the one health call for downstream users."""
        st = self.sdk.get_arm_status()
        en = [bool(self.sdk.get_joint_enable_status(i)) for i in range(1, 7)]
        faults = self.problems()
        state = self._pose_state()
        return ArmStatus(started=self._not_started() is None and not faults, enabled=all(en), e_stop=st is not None and int(st.msg.arm_status) == 1, faults=faults,
                         at_zero=state == "zero", feedback_age=self.feedback_age(), gripper=self.gripper is not None and self.gripper_opening() is not None)

    def _pose_state(self) -> str:
        """'zero' | 'valid' | 'slumped' | 'invalid' (piper_core.classify_state on the measured joints)."""
        return classify_state(self.positions() or {}, self.limits, JOINTS, self.margin, DONE_TOLERANCE)

    def zero(self) -> dict[str, float]:
        return zero_pose(self.limits, JOINTS, self.margin)

    def park(self) -> dict[str, float]:
        return {**self.zero(), "joint5": PARK_JOINT5}

    # ------------------------------------------------------------------ the guard, as the ROS stack applies it
    def _guard(self, target: dict[str, float], max_step: float = math.inf) -> Outcome:
        measured = self.positions()
        if measured is None:
            return Outcome(False, "no joint feedback")
        v = check_command(JOINTS, [target[j] for j in JOINTS], self.limits, measured, GuardConfig(JOINTS, max_step, self.margin))
        return Outcome(v.ok, v.reason, {"positions": v.positions})

    # ------------------------------------------------------------------ control
    def _configure(self, speed_percent: int) -> Outcome:
        self.sdk.set_speed_percent(int(min(max(speed_percent, 1), 100)))
        self.sdk.set_motion_mode(self.sdk.OPTIONS.MOTION_MODE.J)
        t0 = self._now()
        while self._now() - t0 < 2.5:
            st = self.sdk.get_arm_status()
            if st is not None and int(st.msg.mode_feedback) == MODE_J and int(st.msg.ctrl_mode) == 1:
                return Outcome(True)
            self._sleep(0.05)
        return Outcome(False, "the arm did not report CAN control + MOVE_J")

    def enable(self) -> Outcome:
        t0 = self._now()
        while self._now() - t0 < 5.0 and not self.sdk.enable():
            self._sleep(0.05)
        on = [bool(self.sdk.get_joint_enable_status(i)) for i in range(1, 7)]
        self.enabled = any(on)
        return Outcome(all(on), "" if all(on) else f"could not enable all six joints (enabled: {on})")

    def disable(self) -> Outcome:
        for _ in range(10):
            self.sdk.disable()
            if not any(self.sdk.get_joint_enable_status(i) for i in range(1, 7)):
                break
            self._sleep(0.1)
        self.enabled = any(self.sdk.get_joint_enable_status(i) for i in range(1, 7))
        return Outcome(not self.enabled, "motors off" if not self.enabled else "motors still enabled")

    def _hold(self):
        """Hold the present pose with move_j (inside the limits): used after any abort."""
        q = self.positions()
        if q:
            self.sdk.move_j([min(max(q[j], self.limits[j][0]), self.limits[j][1]) for j in JOINTS])

    def _wait_at(self, goal: dict[str, float], timeout: float = 60.0) -> Outcome:
        t0, since = self._now(), None
        while self._now() - t0 < timeout:
            self._sleep(0.1)
            if self._abort.is_set():
                self._hold()
                return Outcome(False, "stopped by abort(), holding", {"reason": "stopped"})
            bad = self.problems()
            if bad:
                self._hold()
                return Outcome(False, "aborted, holding: " + "; ".join(bad), {"reason": "fault"})
            q = self.positions()
            err = max(abs(q[j] - goal[j]) for j in JOINTS)
            since = (self._now() if since is None else since) if err < DONE_TOLERANCE else None
            if since is not None and self._now() - since >= DONE_HOLD_S:
                return Outcome(True, "reached", {"error": err})
        self._hold()
        return Outcome(False, f"goal not reached within {timeout:.0f} s, holding", {"reason": "timeout"})

    def _not_started(self) -> str | None:
        """None when commands are accepted, else the reason in terms a downstream user can act on. A command to a disabled arm is silently ignored by the firmware: refuse it instead."""
        en = [bool(self.sdk.get_joint_enable_status(i)) for i in range(1, 7)]
        if not all(en):
            return f"not started: motors not all enabled {en}, call start()"
        if self._pose_state() not in ("zero", "valid"):
            return "not started: call start()"
        return None

    @_in_op
    def _move(self, target: dict[str, float], speed_percent: int | None = None) -> Outcome:
        """One checked point-to-point move (firmware interpolation) from a started arm (inside its bounds, motors on). The way in is start().
        A failure carries details["reason"]: not_ready | fault | guard:rejected | stopped | timeout."""
        why = self._not_started()
        if why:
            return Outcome(False, why, {"reason": "not_ready"})
        bad = self.problems()
        if bad:
            return Outcome(False, "; ".join(bad), {"reason": "fault"})
        g = self._guard(target)
        if not g:
            return Outcome(False, g.message, {"reason": "guard:rejected"})
        cfg = self._configure(speed_percent or self.speed_percent)
        if not cfg:
            return Outcome(False, cfg.message, {"reason": "fault"})
        self.sdk.move_j([g.details["positions"][k] for k in range(6)])
        return self._wait_at(dict(zip(JOINTS, g.details["positions"])))

    def _clear_estop(self) -> Outcome | None:
        """None when no e-stop is active; else reset it (motors stay off, the arm stays powered) and say whether that worked."""
        st = self.sdk.get_arm_status()
        if st is None or int(st.msg.arm_status) != 1:
            return None
        self.sdk.reset()
        self._sleep(1.5)
        st = self.sdk.get_arm_status()
        self.enabled = any(self.sdk.get_joint_enable_status(i) for i in range(1, 7))
        ok = st is not None and int(st.msg.arm_status) == 0
        return Outcome(ok, "e-stop cleared" if ok else "could not clear the e-stop (power-cycle the arm)")

    @_in_op
    def _leave_off_pose(self, speed_percent: int | None) -> Outcome:
        """Motors on, then the one move that is allowed from where a powered-off arm rests: straight to zero. Internal: this is what start() does for the caller."""
        cfg = self._configure(speed_percent or self.speed_percent)
        if not cfg:
            return cfg
        en = self.enable()
        if not en:
            return en
        self.sdk.move_j([self.zero()[j] for j in JOINTS])
        return self._wait_at(self.zero())

    @_exclusive(_busy_outcome)
    @_in_op
    def start(self, speed_percent: int | None = None) -> Outcome:
        """Make the arm ready: clear an e-stop if one is active, preflight, motors on, one checked move to zero, check zero. Success: started and at zero. Safe to call again (a started arm just goes to zero).
        The caller never sees or commands where a powered-off arm rests; this is the only way in."""
        note = ""
        cleared = self._clear_estop()
        if cleared is not None:
            if not cleared:
                return cleared
            note = " (cleared an e-stop first)"
        bad = self.problems()
        if bad:
            return Outcome(False, "not starting: " + "; ".join(bad))
        state = self._pose_state()
        if state == "invalid":
            return Outcome(False, "not starting: the arm is outside its bounds or has no feedback; use the operator console (piper-arm)")
        if state == "slumped" or self._not_started():
            r = self._leave_off_pose(speed_percent)
            if not r:
                return r
        else:
            r = self._move(self.zero(), speed_percent)
            if not r:
                return r
        ok = self._pose_state() == "zero"
        return Outcome(ok, ("ready: started, at zero" if ok else "not at zero after start") + note)

    @_exclusive(_busy_outcome)
    @_in_op
    def stop(self, speed_percent: int | None = None, settle: float = 1.0, ending: str = "estop") -> Outcome:
        """End the session from any state. A started arm: zero, check, wait, park, wait, then the ending: "estop" (default; verified on the arm 2026-11-10): electronic e-stop (damped lowering),
        wait until at rest, reset (clears the e-stop, motors off, the arm stays powered, start() works at once); "disable": a plain disable (a harder drop).
        Already off: nothing to do. An active e-stop: cleared. A fault: nothing is commanded and the motors stay on, the operator stops it (piper-arm `e`). Refused while execute() runs (abort() first)."""
        if ending not in ("estop", "disable"):
            return Outcome(False, f"ending must be 'estop' or 'disable', not {ending!r}")
        cleared = self._clear_estop()
        if cleared is not None:
            return Outcome(cleared.ok, "e-stop cleared; motors off" if cleared else cleared.message)
        if not any(self.sdk.get_joint_enable_status(i) for i in range(1, 7)):
            return Outcome(True, "already stopped (motors off)")
        bad = self.problems()
        if bad:
            return Outcome(False, "fault: " + "; ".join(bad) + "; motors left on, the operator e-stops (piper-arm e)")
        state = self._pose_state()
        if state == "invalid":
            return Outcome(False, "outside its bounds; motors left on, the operator e-stops (piper-arm e)")
        if state == "slumped":
            r = self._leave_off_pose(speed_percent)
            if not r:
                return Outcome(False, f"zero: {r.message}; motors left on")
        r = self._move(self.zero(), speed_percent)
        if not r:
            return Outcome(False, f"zero: {r.message}; motors left on")
        self._sleep(settle)
        if self._pose_state() != "zero":
            return Outcome(False, "the arm did not stay at zero; motors left on")
        r = self._move(self.park(), speed_percent)
        if not r:
            return Outcome(False, f"park: {r.message}; motors left on")
        self._sleep(settle)
        if ending == "disable":
            return self.disable()
        self.sdk.electronic_emergency_stop()
        t0, prev, t_prev = self._now(), self.positions(), self._now()
        while self._now() - t0 < 15.0:                                  # damped lowering: wait until no joint moved 0.2 deg in a second
            self._sleep(0.1)
            if self._now() - t_prev >= 1.0:
                cur = self.positions()
                if prev and cur and max(abs(cur[j] - prev[j]) for j in JOINTS) < math.radians(0.2):
                    break
                prev, t_prev = cur, self._now()
        self.sdk.reset()
        self._sleep(1.5)
        st = self.sdk.get_arm_status()
        self.enabled = any(self.sdk.get_joint_enable_status(i) for i in range(1, 7))
        ok = st is not None and int(st.msg.arm_status) == 0 and not self.enabled
        return Outcome(ok, "parked, e-stopped, reset: motors off, arm still powered" if ok else "e-stop sent but the arm did not report status normal with motors off")

    @staticmethod
    def auto_lookahead(speed: float, dt: float) -> int:
        """Setpoints to send ahead of the clock for a segment at `speed` rad/s: the arm's measured lag (35 ms + 73 ms per rad/s) in whole steps, 0..6."""
        return int(min(AUTO_LOOKAHEAD_MAX, max(0, round((AUTO_LAG_BASE_S + AUTO_LAG_PER_RAD_S * speed) / dt))))

    def abort(self) -> None:
        """Stop whatever is running (execute, start, stop, move_to), from another thread or a signal handler: it holds the present pose and returns ExecResult(False, 'stopped')
        or an Outcome that says so. Takes effect within about 0.1 s, also during the final settle. Does nothing when idle (the next operation is not affected)."""
        with self._op_lock:
            if self._active:
                self._abort.set()

    # ------------------------------------------------------------------ ArmBackend (piper_core.backend)
    joint_names = list(JOINTS)

    def state(self) -> ArmState:
        """The measured joints as an ArmState. dq is NaN: the SDK feedback used here has no velocity."""
        q = self.positions()
        if q is None:
            raise RealArmError("no joint feedback")
        now = self._now()
        stamp = now - self.feedback_age()                       # on the same (monotonic) clock as t: the SDK's own stamp may be on any epoch
        gw, gf = self.gripper_opening(), self.gripper_force()
        return ArmState(now, np.array([q[j] for j in JOINTS]), np.full(6, np.nan), stamp,
                        math.nan if gw is None else gw, math.nan if gf is None else gf)

    def hold(self) -> None:
        """Keep the present position (a hold command at the measured pose, unchanged). Does nothing unless the arm is started."""
        if self._not_started() is None:
            self._hold()

    def command(self, q, dq=None) -> None:
        """One setpoint through the guard (limits, margin, step from the measured pose, faults). BackendError if it is refused."""
        why = self._not_started()
        if why:
            raise RealArmError(why)
        bad = self.problems()
        if bad:
            raise RealArmError("; ".join(bad))
        g = self._guard(dict(zip(JOINTS, map(float, q))), max_step=MAX_STEP)
        if not g:
            raise RealArmError(g.message)
        self.sdk.move_j(g.details["positions"])

    @_exclusive(_busy_result)
    def move_to(self, q, duration: float | None = None) -> ExecResult:
        """One checked point-to-point move (firmware interpolation) from a valid arm. `q`: six values or {joint: value}."""
        target = dict(q) if isinstance(q, dict) else dict(zip(JOINTS, map(float, q)))
        pct = None
        if duration:
            now = self.positions()
            if now:
                pct = math.ceil(100.0 * max(abs(target[j] - now[j]) for j in JOINTS) / duration / FIRMWARE_MAX_VELOCITY)
        r = self._move(target, pct)
        qn = self.positions() or {}
        return ExecResult(r.ok, "done" if r else r.details.get("reason", "guard:rejected"), "" if r else r.message, np.array([self._now()]),
                          np.array([[target[j] for j in JOINTS]]), np.array([[qn.get(j, np.nan) for j in JOINTS]]))

    def execute(self, traj, *, margin_percent: float = 20.0, blocking: bool = True, mode: str = "fixed", lookahead: int | str = "auto", min_percent: int = 5) -> ExecResult:
        """Follow a trajectory with one move_j per trajectory point (each is a position target the firmware smooths toward; the points are sent on the trajectory's own clock).

        The whole trajectory is checked before anything moves (`check_trajectory`: limits, margin, step, velocity; `check_start`); nothing is clamped. Refused (nothing sent) when the motors are
        not enabled, the arm is slumped, a fault is reported. While it runs every point also passes the guard against the MEASURED pose (step of at most 0.15 rad, faults); a rejection, a fault,
        `abort()` or Ctrl-C stops and holds the pose (reasons 'guard:rejected', 'fault', 'stopped'; Ctrl-C re-raises after holding). Logs commanded and measured joints at each point.

        Streaming options (how to feed the firmware 25 ms points; which is best is a measurement, tools/real_arm/arm_stream_test.py):
          mode="fixed"      one speed percent for the whole trajectory: the fastest segment x (1 + margin_percent/100) of 3.0 rad/s (100 % = 3.0 rad/s, confirmed at 10 %).
          mode="per_point"  the speed percent is re-set before each move_j from the segment's own velocity (at least `min_percent`), so slow parts are not driven at the peak speed.
          lookahead (default "auto", 2026-11-10: rms 6x smaller than 2 steps at 22-44 %, same at low speed)
          lookahead=n       send the point n steps ahead of the clock (the firmware always has a target in front of the arm); the guard's 0.15 rad step limit applies to what is sent.
                            Measured on joint 1 at 25 ms: n=0 leaves a lag of 45 ms, n=2 removes it at low speed (rms 0.04 deg), but the lag grows with speed (25 / 45 / 65 ms left at 0.55 / 0.82 / 1.10 rad/s).
          lookahead="auto"  n follows the segment speed: the expected lag 35 ms + 73 ms per rad/s, rounded to whole steps, at most 6 (fitted on joint 1; check other joints), and backed off
                            until the point sent is within 90 % of the guard's 0.15 rad step from the measured pose (so at about 1.2 rad/s and above it compensates less instead of being rejected).
        After the run `self.last_stream` holds the send times, the percent used and a fine log of the measured joints (about every 5 ms) for `piper_core.stream_metrics.analyze`.
        """
        if not blocking:
            raise NotImplementedError("RealArm.execute is blocking only")
        auto = lookahead == "auto"
        if mode not in ("fixed", "per_point") or not (auto or (isinstance(lookahead, int) and not isinstance(lookahead, bool) and 0 <= lookahead <= 10)):
            return ExecResult(False, "bad_trajectory", "mode must be 'fixed' or 'per_point' and lookahead an integer 0..10 or 'auto'")
        try:
            traj = traj if isinstance(traj, Trajectory) else Trajectory.from_pairs(traj)
        except (ValueError, TypeError) as e:
            return ExecResult(False, "bad_trajectory", str(e))
        if list(traj.joint_names) != list(JOINTS):
            return ExecResult(False, "bad_trajectory", f"joint_names must be {list(JOINTS)}")
        with self._op_lock:                                      # atomic: two callers can never both pass this check
            if self._executing:
                return ExecResult(False, "not_ready", "another trajectory is running")
            self._executing = True
        try:
            with self._op():
                return self._execute_run(traj, margin_percent, mode, lookahead, auto, min_percent)
        finally:
            self._executing = False

    def _execute_run(self, traj, margin_percent, mode, lookahead, auto, min_percent) -> ExecResult:
        why = self._not_started()
        if why:
            return ExecResult(False, "not_ready", why)
        bad = self.problems()
        if bad:
            return ExecResult(False, "fault", "; ".join(bad))
        refused = check_trajectory(traj, self.limits, margin=self.margin, max_step=MAX_STEP, max_velocity=FIRMWARE_MAX_VELOCITY)
        if refused:
            return ExecResult(False, *refused)
        start = self.positions()
        refused = check_start(traj, [start[j] for j in JOINTS])
        if refused:
            return ExecResult(False, *refused)
        seg = np.abs(np.diff(traj.q, axis=0)).max(axis=1) / np.diff(traj.t)          # rad/s of each segment
        scale = (1 + margin_percent / 100.0) * 100.0 / FIRMWARE_MAX_VELOCITY

        def percent(v):
            return int(min(100, max(1, math.ceil(float(v) * scale))))
        def ahead(k):                                             # steps ahead of the clock for the point sent at index k
            if not auto:
                return lookahead
            steps = self.auto_lookahead(float(seg[k - 1:k + 1].max()), float(traj.t[k] - traj.t[k - 1]))
            pos = self.positions()
            while steps > 0 and pos and max(abs(float(traj.q[min(k + steps, n - 1)][i]) - pos[j]) for i, j in enumerate(JOINTS)) > 0.9 * MAX_STEP:
                steps -= 1                                         # the guard would reject a point this far from the measured pose
            return steps
        first = percent(seg.max()) if mode == "fixed" else max(min_percent, percent(seg[: 1 + (AUTO_LOOKAHEAD_MAX if auto else lookahead)].max()))
        cfg = self._configure(first)
        if not cfg:
            return ExecResult(False, "fault", cfg.message)
        log_t, log_c, log_m, events, fine_t, fine_q, used, used_ahead = [], [], [], [], [], [], [], []
        t_start = self._now()

        def sample():
            q = self.positions()
            if q:
                fine_t.append(self._now() - t_start)
                fine_q.append([q[j] for j in JOINTS])

        def result(ok, reason, detail=""):
            self.last_stream = {"mode": mode, "lookahead": lookahead, "lookahead_used": list(used_ahead), "t_sched": traj.t[1:len(log_t) + 1].tolist(), "t_sent": list(log_t), "percent": used,
                                "t": fine_t, "q": fine_q}
            return ExecResult(ok, reason, detail, np.array(log_t), np.array(log_c).reshape(-1, 6), np.array(log_m).reshape(-1, 6), events)
        try:
            n = len(traj.t)                                          # (also used by ahead())
            sample()
            for k in range(1, n):
                while True:                                               # wait for the clock, sampling the measured joints about every 5 ms
                    wait = t_start + traj.t[k] - self._now()
                    if wait <= 0 or self._abort.is_set():
                        break
                    self._sleep(min(0.005, wait))
                    sample()
                if self._abort.is_set():
                    self._hold()
                    events.append((float(traj.t[k]), "stopped", "abort()"))
                    return result(False, "stopped", "aborted, holding")
                bad = self.problems()
                if bad:
                    self._hold()
                    events.append((float(traj.t[k]), "fault", "; ".join(bad)))
                    return result(False, "fault", "aborted, holding: " + "; ".join(bad))
                steps = ahead(k)
                j = min(k + steps, n - 1)
                g = self._guard(dict(zip(JOINTS, map(float, traj.q[j]))), max_step=MAX_STEP)
                if not g:
                    self._hold()
                    events.append((float(traj.t[k]), "guard:rejected", g.message))
                    return result(False, "guard:rejected", f"point at t={traj.t[k]:.3f}: {g.message}; holding")
                used_ahead.append(steps)
                if mode == "per_point":
                    pct = max(min_percent, percent(seg[k - 1:j].max()))
                    if not used or pct != used[-1]:
                        self.sdk.set_speed_percent(pct)
                    used.append(pct)
                else:
                    used.append(first)
                self.sdk.move_j(g.details["positions"])
                log_t.append(self._now() - t_start)                      # stamped right after the send: the timing figures do not include the logging read below
                now = self.positions() or {}
                log_c.append(g.details["positions"])
                log_m.append([now.get(jn, np.nan) for jn in JOINTS])
            end = self._wait_at(dict(zip(JOINTS, map(float, traj.q[-1]))), timeout=10.0)
            sample()
            if not end:
                events.append((float(traj.t[-1]), "end", end.message))
            return result(end.ok, "done" if end.ok else end.details.get("reason", "timeout"), end.message)
        except KeyboardInterrupt:
            self._hold()
            raise
        except Exception as e:                                        # CAN or SDK error in the middle of a run: hold if the bus still takes it, report either way
            events.append((float(log_t[-1]) if log_t else 0.0, "fault", repr(e)))
            try:
                self._hold()
            except Exception:
                pass
            return result(False, "fault", f"CAN/SDK error: {e!r}")
