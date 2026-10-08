#!/usr/bin/env python3
"""Compare Piper robot descriptions (URDF, xacro, MJCF) against a reference.

Usage:
    model_audit.py --ref official=path/to/piper.urdf.xacro \
        isaac=path/to/piper_description.urdf menagerie=path/to/piper.xml \
        [--out report.md] [--samples 200]

The first source given with --ref is the reference. URDF/xacro sources are
parsed directly; MJCF sources are compiled with MuJoCo so defaults/classes are
resolved. Forward kinematics compares the link6 frame (the arm flange in all
known Piper models) over random joint configurations inside the reference
limits.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
FK_LINK = "link6"


@dataclass
class Joint:
    name: str
    type: str
    lower: float | None = None
    upper: float | None = None
    effort: float | None = None
    velocity: float | None = None
    mimic: str | None = None


@dataclass
class Model:
    name: str
    path: str
    kind: str
    joints: dict[str, Joint] = field(default_factory=dict)
    masses: dict[str, float] = field(default_factory=dict)
    fk: object = None  # callable(q: dict[str, float]) -> 4x4 pose of FK_LINK in base_link, or None


# ----------------------------------------------------------------------------- URDF

def _rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(r), math.sin(r), math.cos(p),
                                                 math.sin(p), math.cos(y), math.sin(y)))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _axis_angle(axis, angle):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _floats(text, default):
    return [float(v) for v in text.split()] if text else list(default)


def _expand(path: Path) -> str:
    if path.suffix == ".xacro" or path.name.endswith(".urdf.xacro"):
        import xacro  # from /opt/ros/jazzy
        return xacro.process_file(str(path)).toxml()
    return path.read_text()


def load_urdf(name: str, path: Path) -> Model:
    root = ET.fromstring(_expand(path))
    model = Model(name, str(path), "urdf")
    for link in root.findall("link"):
        m = link.find("inertial/mass")
        if m is not None:
            model.masses[link.get("name")] = float(m.get("value"))
    chain = {}  # child link -> (joint element)
    for j in root.findall("joint"):
        lim = j.find("limit")
        mim = j.find("mimic")
        jt = Joint(j.get("name"), j.get("type"))
        if lim is not None:
            jt.lower = float(lim.get("lower")) if lim.get("lower") else None
            jt.upper = float(lim.get("upper")) if lim.get("upper") else None
            jt.effort = float(lim.get("effort")) if lim.get("effort") else None
            jt.velocity = float(lim.get("velocity")) if lim.get("velocity") else None
        if mim is not None:
            jt.mimic = mim.get("joint")
        model.joints[jt.name] = jt
        chain[j.find("child").get("link")] = j

    def fk(q, target=FK_LINK):
        T = np.eye(4)
        link, joints = target, []
        while link in chain:
            j = chain[link]
            joints.append(j)
            link = j.find("parent").get("link")
        for j in reversed(joints):
            o = j.find("origin")
            xyz = _floats(o.get("xyz") if o is not None else None, (0, 0, 0))
            rpy = _floats(o.get("rpy") if o is not None else None, (0, 0, 0))
            A = np.eye(4)
            A[:3, :3] = _rpy_to_mat(*rpy)
            A[:3, 3] = xyz
            T = T @ A
            if j.get("type") in ("revolute", "continuous"):
                ax = j.find("axis")
                R = np.eye(4)
                R[:3, :3] = _axis_angle(_floats(ax.get("xyz") if ax is not None else None,
                                                (1, 0, 0)), q.get(j.get("name"), 0.0))
                T = T @ R
        return T

    model.fk = fk if FK_LINK in chain else None
    return model


# ----------------------------------------------------------------------------- MJCF

def load_mjcf(name: str, path: Path) -> Model:
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    model = Model(name, str(path), f"mjcf (mujoco {mujoco.__version__})")
    for b in range(1, m.nbody):
        model.masses[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)] = float(m.body_mass[b])
    jtype = {0: "free", 1: "ball", 2: "prismatic", 3: "revolute"}
    for j in range(m.njnt):
        jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        jt = Joint(jn, jtype[int(m.jnt_type[j])])
        if m.jnt_limited[j]:
            jt.lower, jt.upper = map(float, m.jnt_range[j])
        model.joints[jn] = jt
    for a in range(m.nu):  # actuator force limits as the closest analogue of URDF effort
        if m.actuator_trntype[a] == mujoco.mjtTrn.mjTRN_JOINT:
            jn = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a, 0])
            if jn in model.joints and m.actuator_forcelimited[a]:
                model.joints[jn].effort = float(m.actuator_forcerange[a, 1])
    body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, FK_LINK)
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    def fk(q):
        d.qpos[:] = m.qpos0
        for jn, v in q.items():
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                d.qpos[m.jnt_qposadr[jid]] = v
        mujoco.mj_kinematics(m, d)

        def pose(bid):
            T = np.eye(4)
            T[:3, :3] = d.xmat[bid].reshape(3, 3)
            T[:3, 3] = d.xpos[bid]
            return T
        # Express relative to base_link so a world offset of the base does not count as error.
        return np.linalg.inv(pose(base)) @ pose(body) if base >= 0 else pose(body)

    model.fk = fk if body >= 0 else None
    return model


def load(name: str, path: Path) -> Model:
    if path.suffix == ".xml":
        return load_mjcf(name, path)
    return load_urdf(name, path)


# ----------------------------------------------------------------------------- report

def _fmt(v, nd=4):
    return "—" if v is None else f"{v:.{nd}f}".rstrip("0").rstrip(".") if isinstance(v, float) else str(v)


def _mark(v, ref, tol=1e-3):
    if v is None or ref is None:
        return _fmt(v)
    return _fmt(v) if abs(v - ref) <= tol else f"**{_fmt(v)}**"


def report(models: list[Model], samples: int, seed: int) -> str:
    ref = models[0]
    out = ["# Piper description audit", "",
           f"Reference: `{ref.name}`. Values differing from the reference are **bold**. "
           f"FK compares `{FK_LINK}` relative to `base_link`.", "",
           "| Source | Kind | Path |", "|---|---|---|"]
    out += [f"| {m.name} | {m.kind} | `{m.path}` |" for m in models]

    out += ["", "## Arm joint limits (rad, rad/s, N·m)", "",
            "| Joint | " + " | ".join(m.name for m in models) + " |",
            "|---|" + "---|" * len(models)]
    for jn in ARM_JOINTS:
        r = ref.joints.get(jn)
        for field_, label in (("lower", "lower"), ("upper", "upper"),
                              ("velocity", "vel"), ("effort", "effort")):
            refv = getattr(r, field_) if r else None
            cells = [_mark(getattr(m.joints[jn], field_), refv) if jn in m.joints else "absent"
                     for m in models]
            out.append(f"| {jn} {label} | " + " | ".join(cells) + " |")

    out += ["", "## Gripper joints", "",
            "| Source | Joints (type, range, mimic) |", "|---|---|"]
    for m in models:
        gj = [j for n, j in m.joints.items() if n not in ARM_JOINTS and j.type != "fixed"]
        out.append(f"| {m.name} | " + "; ".join(
            f"`{j.name}` {j.type} [{_fmt(j.lower)}, {_fmt(j.upper)}]"
            + (f" mimic `{j.mimic}`" if j.mimic else "") for j in gj) + " |")

    out += ["", "## Masses (kg)", "",
            "| Link | " + " | ".join(m.name for m in models) + " |",
            "|---|" + "---|" * len(models)]
    links = list(dict.fromkeys(n for m in models for n in m.masses))
    for ln in links:
        refv = ref.masses.get(ln)
        out.append(f"| {ln} | " + " | ".join(
            _mark(m.masses[ln], refv, 1e-4) if ln in m.masses else "—" for m in models) + " |")
    out.append("| **total** | " + " | ".join(
        _mark(sum(m.masses.values()), sum(ref.masses.values()), 1e-3) for m in models) + " |")

    out += ["", f"## Forward kinematics of `{FK_LINK}` vs reference", "",
            f"{samples} random configurations inside the reference arm limits (seed {seed}), "
            "plus the zero pose.", "",
            "| Source | zero-pose pos err (mm) | max pos err (mm) | max rot err (deg) |",
            "|---|---|---|---|"]
    rng = np.random.default_rng(seed)
    lo = np.array([ref.joints[j].lower for j in ARM_JOINTS])
    hi = np.array([ref.joints[j].upper for j in ARM_JOINTS])
    qs = [np.zeros(6)] + [rng.uniform(lo, hi) for _ in range(samples)]
    for m in models[1:]:
        if m.fk is None or ref.fk is None:
            out.append(f"| {m.name} | n/a | n/a | n/a |")
            continue
        perr, rerr = [], []
        for q in qs:
            qd = dict(zip(ARM_JOINTS, q))
            A, B = ref.fk(qd), m.fk(qd)
            perr.append(1000 * np.linalg.norm(A[:3, 3] - B[:3, 3]))
            c = (np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2
            rerr.append(math.degrees(math.acos(max(-1.0, min(1.0, c)))))
        out.append(f"| {m.name} | {perr[0]:.3f} | {max(perr):.3f} | {max(rerr):.3f} |")
    return "\n".join(out) + "\n"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ref", required=True, help="name=path of the reference description")
    p.add_argument("sources", nargs="*", help="name=path of descriptions to compare")
    p.add_argument("--out", type=Path, help="write the Markdown report here")
    p.add_argument("--samples", type=int, default=200)
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args(argv)
    models = []
    for spec in [a.ref, *a.sources]:
        name, _, path = spec.partition("=")
        models.append(load(name, Path(path).expanduser()))
    text = report(models, a.samples, a.seed)
    if a.out:
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(text)
    sys.stdout.write(text)


if __name__ == "__main__":
    main()
