"""Command-line access to a running Piper (any backend).

    piper state
    piper controllers
    piper activate        # backend:=real: start commanding (checks driver feedback first)
    piper deactivate
    piper enable          # low level, backend:=real: motors on only (use start)
    piper disable         # low level, backend:=real: 5 s countdown, motors off where the arm is (use stop)
    piper zero            # the zero pose, from anywhere inside the limits: start and end pose of every session
    piper start           # begin a session: motors on, leave the powered-off pose, checked arrival at zero, controllers active
    piper stop            # end a session: zero, wait, park, wait, motors off (real arm)
    piper joints 0 1.0 -1.0 0 1.0 0 [--duration 3] [--plan]
    piper named ready
    piper pose 0.25 0 0.10 [--quat 0 1 0 0] [--linear]   # tool-down reach is limited to z < ~0.12 m
    piper gripper 0.05
    piper execute trajectory.json --result result.json   # a planner trajectory (piper.trajectory/1) as given; also: piper-ros-execute
    piper demo --backend NAME --out run.json    # scripted sequence over every control level, recorded
"""

import argparse
import json
import sys
import time
from pathlib import Path

from piper_py.robot import Piper


def main(argv=None):
    p = argparse.ArgumentParser(prog="piper", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--namespace", default="")
    p.add_argument("--prefix", default="piper_", help="prefix of the robot's joint and frame names (launch argument prefix:=)")
    p.add_argument("--speed", type=float, default=0.2, help="MoveIt velocity scaling (0-1]")
    sub = p.add_subparsers(dest="cmd", required=True)
    st_p = sub.add_parser("state", help="print measured joint state as JSON")
    st_p.add_argument("--full", action="store_true", help="also velocity and effort (real arm: motor torque estimate, N.m)")
    sub.add_parser("status", help="real arm: health reported by the vendor driver (arm, motors, gripper); read-only")
    sub.add_parser("controllers", help="list controllers and their states")
    sub.add_parser("activate", help="activate arm/gripper controllers after checking feedback")
    sub.add_parser("deactivate", help="deactivate all command controllers")
    sub.add_parser("start", help="begin a session from anywhere (powered-off arm included): motors, checked move to zero, controllers active")
    sub.add_parser("stop", help="end a session: go to zero, check, park (real arm), motors off. Use this instead of disable")
    sub.add_parser("enable", help="low level, real arm: enable the motors only (use start)")
    dis = sub.add_parser("disable", help="low level, real arm: count down, then disable the motors where the arm is (use stop; hold the arm!)")
    dis.add_argument("--countdown", type=int, default=5, help="seconds to get a hand on the arm (default 5)")
    sub.add_parser("zero", help="move to the 'zero' pose, from anywhere inside the limits: the start and end pose of every session")
    ex = sub.add_parser("execute", help="run a piper.trajectory/1 JSON file on arm_controller as given and write the piper.exec_result/1 JSON")
    ex.add_argument("trajectory", type=Path)
    ex.add_argument("--result", type=Path, default=None, help="write the ExecResult here (default: print a summary only)")
    j = sub.add_parser("joints", help="move to six joint values (rad)")
    j.add_argument("q", type=float, nargs=6)
    j.add_argument("--duration", type=float, default=3.0, help="direct trajectory duration")
    j.add_argument("--plan", action="store_true", help="plan with MoveIt (collision-aware) instead")
    n = sub.add_parser("named", help="move to an SRDF named pose")
    n.add_argument("name", nargs="?", help="omit to list names")
    ps = sub.add_parser("pose", help="move tcp_link to x y z (base_link)")
    ps.add_argument("xyz", type=float, nargs=3)
    ps.add_argument("--quat", type=float, nargs=4, default=[0.0, 1.0, 0.0, 0.0],
                    metavar=("X", "Y", "Z", "W"), help="default: tool pointing down")
    ps.add_argument("--frame", default=None)
    ps.add_argument("--linear", action="store_true", help="straight-line Cartesian motion (Pilz LIN)")
    g = sub.add_parser("gripper", help="set gripper opening width (m)")
    g.add_argument("width", type=float)
    g.add_argument("--effort", type=float, default=0.0)
    d = sub.add_parser("demo", help="run the scripted demo sequence and record it")
    d.add_argument("--backend", default="unknown", help="label stored in the record")
    d.add_argument("--out", type=Path, required=True)
    sr = sub.add_parser("stepresp", help="measure the joint position servo's step response (streaming controller)")
    sr.add_argument("--backend", default="unknown")
    sr.add_argument("--out", type=Path, required=True)
    a = p.parse_args(argv)
    if a.cmd == "stepresp":
        from piper_py import stepresp
        return stepresp.main(a.namespace, a.backend, a.out)
    if a.cmd == "demo":
        from piper_py import demo
        return demo.main(a.namespace, a.backend, a.out)

    with Piper(namespace=a.namespace, prefix=a.prefix) as arm:
        if a.cmd == "state":
            if a.full:
                js, vel, eff = arm.joint_state(), arm.joint_velocities(), arm.joint_efforts()
                print(json.dumps({n: {"position": js[n], "velocity": vel.get(n), "effort": eff.get(n)} for n in js}, indent=2))
            else:
                print(json.dumps(arm.joint_state(), indent=2))
            return 0
        if a.cmd == "status":
            st = arm.status()
            if not (st["arm"] or st["motors"] or st["gripper"]):
                print("no status from a vendor driver (simulators publish none; on the real arm check the driver is running)")
                return 1
            if st["arm"]:
                v = st["arm"]
                print(f"arm      {v['level']:5s} {v.get('ctrl_mode_name', '?')}, {v.get('arm_status_name', '?')}, {v.get('mode_feedback_name', '?')}"
                      f"  (motion_status {v['values'].get('motion_status')}, err {v['values'].get('err_status')}, age {v['age']:.1f} s)")
            for m in st["motors"]:
                v = m["values"]
                print(f"{m['name']:8s} {m['level']:5s} {m['message']:10s} {v.get('bus_voltage_v', '?'):>6} V  driver {v.get('driver_temp_deg', '?'):>5} C"
                      f"  motor {v.get('motor_temp_deg', '?'):>5} C  {v.get('bus_current', '?')} A")
            if st["gripper"]:
                print(f"gripper  {st['gripper']['level']:5s} {st['gripper']['message']}")
            return 1 if arm.faults() else 0
        if a.cmd == "controllers":
            for name, state in sorted(arm.controllers().items()):
                print(f"{name:28s} {state}")
            return 0
        if a.cmd in ("activate", "deactivate"):
            r = arm.activate() if a.cmd == "activate" else arm.deactivate()
        elif a.cmd == "enable":
            r = arm.set_motors(True)
        elif a.cmd == "disable":
            for left in range(a.countdown, 0, -1):
                print(f"HOLD THE ARM: motors off in {left} s (Ctrl-C cancels)", flush=True)
                time.sleep(1.0)
            r = arm.set_motors(False)
        elif a.cmd == "zero":
            r = arm.move_named("zero", a.speed)
        elif a.cmd == "start":
            r = arm.start(a.speed)
        elif a.cmd == "stop":
            r = arm.stop(a.speed)
        elif a.cmd == "execute":
            from piper_core.backend import Trajectory
            res = arm.execute_trajectory(Trajectory.from_json(a.trajectory.read_text()))
            if a.result:
                a.result.write_text(res.to_json())
            print(f"{'OK' if res else 'FAILED'} {res.reason} {res.detail}".strip(), f"({len(res.t)} samples)")
            return 0 if res else 1
        elif a.cmd == "joints":
            r = arm.move_to_joints(a.q, a.speed) if a.plan else arm.move_joints(a.q, a.duration)
        elif a.cmd == "named":
            if not a.name:
                print("\n".join(sorted(arm.named_poses())))
                return 0
            r = arm.move_named(a.name, a.speed)
        elif a.cmd == "pose":
            r = arm.move_pose(a.xyz, a.quat, a.frame, a.linear, a.speed)
        else:
            r = arm.gripper(a.width, a.effort)
        print(f"{'OK' if r else 'FAILED'} {r.code} {r.message} {r.details or ''}".strip())
        return 0 if r else 1


def execute_main(argv=None):
    """`piper-ros-execute trajectory.json [--result result.json]`: the same as `piper execute`."""
    return main(["execute", *(sys.argv[1:] if argv is None else argv)])


if __name__ == "__main__":
    sys.exit(main())
