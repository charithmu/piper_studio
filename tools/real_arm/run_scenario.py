#!/usr/bin/env python3
"""Run the cuRobo test scenario (approach -> grasp -> close gripper -> lift -> open gripper -> back to zero) on one backend, from a scenario.json made by curobo_plan_scenario.py.

    source scripts/env.sh                                   # real arm and MuJoCo: no ROS needed
    python tools/real_arm/run_scenario.py SCENARIO.json --dry-run                  # no backend at all: checks every leg offline, prints speeds and durations
    python tools/real_arm/run_scenario.py SCENARIO.json --backend mujoco --yes     # the same plan in simulation first (no hardware)
    python tools/real_arm/run_scenario.py SCENARIO.json --backend real             # RealArm.start / execute / set_gripper: user present, an Enter before every step
    source scripts/session.sh 71; python tools/real_arm/run_scenario.py SCENARIO.json --backend ros   # through the ROS stack (launch backend:=real first, see docs/real_arm/CHECKLIST.md R3)

The plan is cuRobo's (fast by design: it peaks at 1.4 rad/s), so every leg is slowed to --max-speed (default 0.3 rad/s, about 10 % speed percent) with the same path; --max-speed 0 keeps the plan as it is.
Each leg is checked offline against the limits and the guard rules before anything is connected; on the arm, `RealArm.execute` / the ROS driver check it again and hold on any problem.
The report (per step: ok, message, planned and wall time, tracking error, gripper width and force) is written to ~/data/ml/isaac/piper_studio/scenario/reports/. The arm is left started at zero (end the session with
`piper-arm` `t` or `RealArm.stop()`). Real arm and ROS need the user at the arm, hand on the power switch; Ctrl-C holds where it is.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

import piper_assets as pa
from piper_core.backend import ARM_JOINTS, Trajectory, check_trajectory
from piper_core.stream_plan import peak_speed, required_percent, retime


def say(*a):
    print(*a, flush=True)


# ---------------------------------------------------------------------------------------------------- backends
class Real:
    name = "real arm (RealArm)"

    def __init__(self, args):
        from piper_core import RealArm
        self.arm = RealArm.connect(args.canbus)
        self.effort = args.effort

    def start(self):
        r = self.arm.start()
        return r.ok, r.message

    def run(self, traj):
        return self.arm.execute(traj)

    def gripper(self, width):
        w = self.arm.set_gripper(width, effort=self.effort)
        g = self.arm.last_gripper
        return w, f"{g.reason}, force {g.force}"

    def at_zero(self):
        return self.arm.status().at_zero

    def hold(self):
        self.arm.abort()
        self.arm.hold()

    def close(self):
        self.arm.close()


class Mujoco:
    name = "MuJoCo (PiperSim)"

    def __init__(self, args):
        from piper_mujoco import PiperSim
        self.sim = PiperSim(scene="table", gripper_width=0.07, camera=bool(args.camera))

    def start(self):
        r = self.sim.start()
        return r.ok, r.message

    def run(self, traj):
        return self.sim.execute(traj)

    def gripper(self, width):
        return self.sim.set_gripper(width), "simulated (no force)"

    def at_zero(self):
        return bool(np.max(np.abs(self.sim.state().q - np.array(pa.zero_pose()))) < 0.02)

    def hold(self):
        self.sim.hold()

    def close(self):
        self.sim.close()


class Ros:
    name = "ROS stack (piper_py + piper_real_driver)"

    def __init__(self, args):
        from piper_py import Piper
        self.arm = Piper()
        if not self.arm.wait_ready(60.0):
            raise SystemExit("the ROS stack is not up: start `ros2 launch piper_sim piper.launch.py backend:=real canbus:=can0 speed_percent:=20` in another terminal (same ROS_DOMAIN_ID)")
        self.effort = args.effort

    def start(self):
        r = self.arm.start()
        return bool(r), f"{r.code} {r.message}"

    def run(self, traj):
        return self.arm.execute_trajectory(traj)

    def gripper(self, width):
        r = self.arm.gripper(width, max_effort=self.effort)
        return (r.details or {}).get("width", float("nan")), f"{r.code} {r.message}"

    def at_zero(self):
        q = self.arm.joint_positions()
        return bool(np.max(np.abs(np.array(q) - np.array(pa.zero_pose()))) < 0.02)

    def hold(self):
        self.arm.deactivate()

    def close(self):
        self.arm.close()


BACKENDS = {"real": Real, "mujoco": Mujoco, "ros": Ros}


# ---------------------------------------------------------------------------------------------------- the scenario
def tracking(traj, r):
    n = len(r.q_meas)
    if not n:
        return {}
    ref = np.stack([np.interp(r.t, traj.t, traj.q[:, j]) for j in range(6)], axis=1)
    err = np.abs(r.q_meas - ref)
    return {"max_err_deg": round(math.degrees(float(err.max())), 2), "end_err_deg": round(math.degrees(float(np.abs(r.q_meas[-1] - traj.q[-1]).max())), 2)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", type=Path)
    ap.add_argument("--backend", choices=sorted(BACKENDS), default=None)
    ap.add_argument("--max-speed", type=float, default=0.3, help="rad/s cap for every leg (0: as planned)")
    ap.add_argument("--effort", type=float, default=1.0, help="gripper force in N (cap 3 N)")
    ap.add_argument("--canbus", "-b", default="can0")
    ap.add_argument("--arm", default="piper-01", help="which real arm: its docs/real_arm/arms/<arm>.yaml says what is mounted (camera); the scenario must be planned for that hardware")
    ap.add_argument("--arm-config", type=Path, default=None, help="another arm config file (tests)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--yes", action="store_true", help="no Enter before each step (simulation only)")
    ap.add_argument("--sim-stack", action="store_true", help="with --backend ros: the ROS stack is a simulation (launch backend:=mock), so --yes is allowed")
    args = ap.parse_args()

    doc = json.loads(args.scenario.read_text())
    if doc.get("schema") != "piper.scenario/1":
        raise SystemExit("not a piper.scenario/1 file (make one with curobo_plan_scenario.py)")
    args.camera = doc.get("camera")                      # None: planned for the robot only (older scenario files too); "zedxm": planned with the bracket and camera
    legs = {k: Trajectory.from_json(json.dumps(v)) for k, v in doc["legs"].items()}
    if args.max_speed > 0:
        legs = {k: retime(t, args.max_speed) for k, t in legs.items()}
    lim = pa.limits()
    say(f"scenario: TCP target {doc['target_xyz']} m, yaw {doc['yaw_deg']} deg, {doc['planner']}; arm: {args.camera or 'robot only'}; scene: {doc['scene']}")
    bad = False
    for k, t in legs.items():
        why = check_trajectory(t, lim.hard, margin=lim.margin, max_step=lim.max_step, max_velocity=lim.velocity)
        say(f"  leg {k:9s} {len(t.t):4d} points {t.t[-1]:6.2f} s  peak {peak_speed(t):.2f} rad/s -> {required_percent(t)} %   offline check: {'OK' if not why else why}")
        bad = bad or bool(why)
    if bad:
        raise SystemExit("a leg fails the offline check: nothing was connected")
    if args.backend in ("real", "ros") and not args.sim_stack:       # the plan must be for the hardware that is there, in both directions (a plan without the bracket is not safe with it, and the reverse)
        import arm_config
        mounted = arm_config.camera(args.arm, args.arm_config)
        if mounted != args.camera:
            raise SystemExit(f"refused: this scenario was planned for the arm {'WITH the ZED X Mini and its bracket' if args.camera else 'WITHOUT a camera (the robot only)'}, but {args.arm} has "
                             f"{'the ZED X Mini and its bracket' if mounted else 'no camera'} mounted (docs/real_arm/arms/{args.arm}.yaml). Plan again: curobo_plan_scenario.py --camera {mounted or 'none'}")
    if args.dry_run or args.backend is None:
        say("dry run: nothing connected." if args.dry_run else "no --backend given: nothing connected.")
        return
    if args.yes and not (args.backend == "mujoco" or (args.backend == "ros" and args.sim_stack)):
        raise SystemExit("--yes is for the simulation only: the arm needs an Enter before every step")

    def ask(text):
        if not args.yes:
            input(f"\n>>> {text}  [Enter = go, Ctrl-C = hold and exit] ")

    b = BACKENDS[args.backend](args)
    report, t_all = [], time.time()
    try:
        say(f"\nbackend: {b.name}")
        ask("start: clear any e-stop, motors on, move to zero")
        ok, msg = b.start()
        say("  start:", ok, msg)
        report.append({"step": "start", "ok": bool(ok), "message": msg})
        if not ok:
            return
        for i, st in enumerate(doc["steps"], 1):
            if st["op"] == "gripper":
                ask(f"step {i}: gripper {st['label']} to {st['width'] * 1000:.0f} mm at {args.effort} N (fingers clear)")
                t0 = time.time()
                w, note = b.gripper(st["width"])
                say(f"  gripper -> {w * 1000:.1f} mm  {note}  ({time.time() - t0:.1f} s)")
                report.append({"step": f"{i} gripper {st['label']}", "ok": True, "width_m": float(w), "note": note, "wall_s": round(time.time() - t0, 2)})
            else:
                traj = legs[st["leg"]]
                ask(f"step {i}: move {st['leg']} ({traj.t[-1]:.1f} s, peak {peak_speed(traj):.2f} rad/s)")
                t0 = time.time()
                r = b.run(traj)
                say(f"  {st['leg']}: ok={r.ok} {r.message}  ({time.time() - t0:.1f} s)", tracking(traj, r) or "")
                report.append({"step": f"{i} move {st['leg']}", "ok": bool(r.ok), "message": r.message, "plan_s": round(float(traj.t[-1]), 2),
                               "wall_s": round(time.time() - t0, 2), **tracking(traj, r)})
                if not r.ok:
                    say("  STOPPED: the arm holds where it is.")
                    b.hold()
                    return
        z = b.at_zero()
        say(f"\nback at zero: {z}")
        report.append({"step": "end", "ok": bool(z), "at_zero": bool(z)})
    except KeyboardInterrupt:
        say("\nCtrl-C: holding where it is.")
        b.hold()
    finally:
        rep_dir = Path.home() / "data/ml/isaac/piper_studio/scenario/reports"              # results are data: not next to a scenario file that may live in the repo
        rep_dir.mkdir(parents=True, exist_ok=True)
        out = rep_dir / f"report_{args.backend}_{time.strftime('%Y%m%d_%H%M%S')}.json"
        out.write_text(json.dumps({"backend": b.name, "max_speed": args.max_speed, "effort": args.effort, "steps": report, "wall_s": round(time.time() - t_all, 1)}, indent=1, default=float))
        say("report:", out)
        b.close()


if __name__ == "__main__":
    main()
