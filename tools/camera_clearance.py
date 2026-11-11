#!/usr/bin/env python3
"""Offline check before the wrist camera goes on the real arm: does the bracket or the ZED X Mini touch the arm or the fingers, and how close do they come?

    python tools/camera_clearance.py [--poses 4000] [--calibration zedxm_bracket_v3]

Takes the collision meshes of the description (`camera_calibration` selects the bracket version), the zero pose, the `ready` pose, the gripper at 0 / 35 / 70 mm
and random arm poses inside the URDF limits, and reports per pair of links (bracket or camera against the others): the poses in collision, and for the zero and
ready poses the smallest distance in mm. Contacts with link1, link2 and the base only in extreme poses are real (the planner avoids them through the SRDF);
contacts with link3..link6 or the fingers would be a design problem. Needs pinocchio and coal (in the workspace venv). Moves nothing; no arm needed.
"""
import argparse
import sys
from collections import Counter
from pathlib import Path

import numpy as np

import coal
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "piper_sim"))
from piper_sim import robot_description  # noqa: E402

P = "piper_"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", type=int, default=4000)
    ap.add_argument("--calibration", default="")
    a = ap.parse_args()
    maps = {"camera_calibration": a.calibration} if a.calibration else {}
    urdf = Path("/tmp") / "piper_camera_clearance.urdf"
    urdf.write_text(robot_description("mock", **maps))
    model = pin.buildModelFromUrdf(str(urdf))
    gm = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION, package_dirs=[str(Path(get_package_share_directory("agx_arm_description")).parent)])
    names = [g.name.removesuffix("_0").removeprefix(P) for g in gm.geometryObjects]
    cam = {i for i, n in enumerate(names) if n in ("camera_mount_link", "camera_link")}
    gm.addAllCollisionPairs()
    pairs = [(i, p) for i, p in enumerate(gm.collisionPairs) if p.first in cam or p.second in cam]
    data, gdata = pin.Data(model), pin.GeometryData(gm)

    def config(arm, width):
        q = pin.neutral(model).copy()
        for k, v in enumerate(arm, 1):
            q[model.joints[model.getJointId(f"{P}joint{k}")].idx_q] = v
        for jn, m in ((f"{P}gripper", 1.0), (f"{P}gripper_joint1", 0.5), (f"{P}gripper_joint2", -0.5)):
            if model.existJointName(jn):
                q[model.joints[model.getJointId(jn)].idx_q] = m * width
        return q

    def colliding(q):
        pin.computeCollisions(model, data, gm, gdata, q, False)
        return {(names[p.first], names[p.second]) for i, p in pairs if gdata.collisionResults[i].isCollision()}

    def distances(q):
        pin.computeDistances(model, data, gm, gdata, q)
        return {(names[p.first], names[p.second]): gdata.distanceResults[i].min_distance * 1000 for i, p in pairs}

    print("pose / gripper                         colliding pairs")
    for label, arm in (("zero", [0, 0.01, -0.01, 0, 0, 0]), ("ready", [0.0, 1.0, -1.0, 0.0, 1.0, 0.0])):
        for w in (0.0, 0.035, 0.07):
            c = colliding(config(arm, w))
            print(f"{label:6s} gripper {w * 1000:4.0f} mm              {sorted(c) if c else 'none'}")
        d = distances(config(arm, 0.07))
        near = sorted(d.items(), key=lambda kv: kv[1])[:4]
        print(f"       closest at {label} (gripper open):   " + ", ".join(f"{x}-{y} {mm:.0f} mm" for (x, y), mm in near))
    rng = np.random.default_rng(1)
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    cnt = Counter()
    for _ in range(a.poses):
        arm = [rng.uniform(lo[model.joints[model.getJointId(f"{P}joint{k}")].idx_q], hi[model.joints[model.getJointId(f"{P}joint{k}")].idx_q]) for k in range(1, 7)]
        for pair in colliding(config(arm, 0.07)):
            cnt[pair] += 1
    print(f"\n{a.poses} random poses inside the limits, gripper open, poses in collision per pair:", dict(cnt) if cnt else "none")
    bad = [p for p in cnt if any(n in p[0] + p[1] for n in ("link3", "link4", "link5", "link6", "gripper_link"))]
    print("RESULT:", "contact with link3..6 or a finger: " + str(bad) if bad else "ok (only link1, link2 and the base in extreme poses, if any)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
