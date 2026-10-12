"""Command-line access to a running Piper (any backend).

    piper state
    piper controllers
    piper activate        # backend:=real: start commanding (checks driver feedback first)
    piper deactivate
    piper joints 0 1.0 -1.0 0 1.0 0 [--duration 3] [--plan]
    piper named ready
    piper pose 0.25 0 0.10 [--quat 0 1 0 0] [--linear]   # tool-down reach is limited to z < ~0.12 m
    piper gripper 0.05
    piper demo --backend NAME --out run.json    # scripted sequence over every control level, recorded
"""

import argparse
import json
import sys
from pathlib import Path

from piper_py.robot import Piper


def main(argv=None):
    p = argparse.ArgumentParser(prog="piper", description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--namespace", default="")
    p.add_argument("--speed", type=float, default=0.2, help="MoveIt velocity scaling (0-1]")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("state", help="print measured joint state as JSON")
    sub.add_parser("controllers", help="list controllers and their states")
    sub.add_parser("activate", help="activate arm/gripper controllers after checking feedback")
    sub.add_parser("deactivate", help="deactivate all command controllers")
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

    with Piper(namespace=a.namespace) as arm:
        if a.cmd == "state":
            print(json.dumps(arm.joint_state(), indent=2))
            return 0
        if a.cmd == "controllers":
            for name, state in sorted(arm.controllers().items()):
                print(f"{name:28s} {state}")
            return 0
        if a.cmd in ("activate", "deactivate"):
            r = arm.activate() if a.cmd == "activate" else arm.deactivate()
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


if __name__ == "__main__":
    sys.exit(main())
