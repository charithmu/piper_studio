#!/usr/bin/env python3
"""Do our model and the arm's kinematics agree? Compares the flange pose of (1) our URDF (the vendor's official one, which MoveIt and all simulators use) with (2) the SDK's modified-DH forward
kinematics (the DH numbers the arm's firmware uses), at many joint angles inside the stored limits. No arm needed; nothing is sent.

    source scripts/env.sh
    python tools/real_arm/fk_compare.py                    # 300 random poses: worst position difference (mm) and orientation difference (deg)
    python tools/real_arm/fk_compare.py --poses            # also print the zero pose and the named poses

The frames may differ by a constant (URDF link6 vs the SDK flange): it is estimated at the zero pose and removed; what is reported is what remains. A real difference here would mean a planner using the URDF
aims where the firmware does not go. The same comparison against the pose the arm reports itself (CAN 0x2A2-0x2A4) is `fk_compare.py --arm` (needs the arm started; user present; reads only).
"""
import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "external/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper/urdf/piper_description.urdf"
JOINTS = [f"joint{i}" for i in range(1, 7)]


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr], [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def rot_axis(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * k + (1 - math.cos(q)) * k @ k


def load_chain(path=URDF):
    chain = {}
    for j in ET.parse(path).getroot().findall("joint"):
        if j.get("name") in JOINTS:
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
            chain[j.get("name")] = (np.array(xyz), rot_rpy(*rpy), [float(v) for v in j.find("axis").get("xyz").split()],
                                    float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
    return chain


def urdf_fk(chain, q):
    T = np.eye(4)
    for name, qi in zip(JOINTS, q):
        xyz, R, axis, _, _ = chain[name]
        step = np.eye(4)
        step[:3, :3], step[:3, 3] = R @ rot_axis(axis, qi), xyz
        T = T @ step
    return T


def sdk_fk(q):
    from pyAgxArm.utiles.mdh_kinematics import fk_from_mdh, get_mdh
    x, y, z, r, p, yw = fk_from_mdh(list(get_mdh("piper")), list(q))
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = rot_rpy(r, p, yw), (x, y, z)
    return T


def angle_between(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return math.degrees(math.acos(min(1.0, max(-1.0, c))))


def compare(n=300, seed=1):
    chain = load_chain()
    q0 = [0.0] * 6
    A0, B0 = urdf_fk(chain, q0), sdk_fk(q0)
    offset = A0[:3, :3].T @ B0[:3, :3]                       # constant frame difference, removed
    rng = np.random.default_rng(seed)
    lo = np.array([chain[j][3] for j in JOINTS])
    hi = np.array([chain[j][4] for j in JOINTS])
    worst_mm, worst_deg, rows = 0.0, 0.0, []
    for _ in range(n):
        q = rng.uniform(lo + 0.05, hi - 0.05)
        A, B = urdf_fk(chain, q), sdk_fk(q)
        d = np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000
        a = angle_between(A[:3, :3] @ offset, B[:3, :3])
        worst_mm, worst_deg = max(worst_mm, d), max(worst_deg, a)
        rows.append((d, a, q))
    return worst_mm, worst_deg, rows, (A0, B0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", type=int, default=300)
    ap.add_argument("--poses", action="store_true")
    ap.add_argument("--arm", action="store_true", help="also read the flange pose the arm reports, now (reads only; the arm must be started)")
    ap.add_argument("--canbus", "-b", default="can0")
    args = ap.parse_args()
    mm, deg, rows, (A0, B0) = compare(args.n)
    print(f"URDF {URDF.relative_to(ROOT)} vs SDK modified-DH, {args.n} random poses inside the stored limits")
    print(f"flange at all-zero: URDF {np.round(A0[:3, 3] * 1000, 2)} mm, SDK {np.round(B0[:3, 3] * 1000, 2)} mm")
    print(f"worst position difference {mm:.3f} mm, worst orientation difference {deg:.3f} deg (constant frame offset removed)")
    if args.poses:
        for name, q in {"zero": [0, 0.01, -0.01, 0, 0, 0], "park": [0, 0.01, -0.01, 0, 0.384, 0], "ready": [0, 1.0, -1.0, 0, 1.0, 0]}.items():
            print(f"  {name:5s} URDF {np.round(urdf_fk(load_chain(), q)[:3, 3] * 1000, 1)} mm  SDK {np.round(sdk_fk(q)[:3, 3] * 1000, 1)} mm")
    if args.arm:
        from piper_core import RealArm
        arm = RealArm.connect(args.canbus)
        try:
            q = [arm.positions()[j] for j in JOINTS]
            fp = arm.sdk.get_flange_pose()
            rep = np.array(fp.msg[:3]) if fp is not None else None
            print("joints (deg):", [round(math.degrees(x), 2) for x in q])
            if rep is None:
                print("the arm reports no flange pose")
            else:
                for label, T in (("URDF", urdf_fk(load_chain(), q)), ("SDK DH", sdk_fk(q))):
                    print(f"  arm-reported flange {np.round(rep * 1000, 2)} mm vs {label} {np.round(T[:3, 3] * 1000, 2)} mm -> {np.linalg.norm(rep - T[:3, 3]) * 1000:.2f} mm")
        finally:
            arm.close()
    sys.exit(0 if mm < 1.0 and deg < 0.5 else 1)


if __name__ == "__main__":
    main()
