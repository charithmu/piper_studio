#!/usr/bin/env python3
"""Write reference forward kinematics (from the URDF) for isaac/check_model.py: random configs, TCP pose, total mass.

    python tools/make_fk_reference.py OUT.json [N]
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/piper_description/tools"))
import model_audit  # noqa: E402
from piper_description import physics_model, robot_description  # noqa: E402

out, n = Path(sys.argv[1]), int(sys.argv[2]) if len(sys.argv) > 2 else 25
urdf = physics_model(robot_description("isaac"))
tmp = out.with_suffix(".urdf")
tmp.write_text(urdf)
m = model_audit.load_urdf("ref", tmp)
lo = np.array([m.joints[j].lower for j in model_audit.ARM_JOINTS]) + 0.02
hi = np.array([m.joints[j].upper for j in model_audit.ARM_JOINTS]) - 0.02
rng = np.random.default_rng(3)
qs = [np.zeros(6) + [0, 0.05, -0.05, 0, 0, 0]] + [rng.uniform(lo, hi) for _ in range(n)]
refs = []
for q in qs:
    T = m.fk(dict(zip(model_audit.ARM_JOINTS, q)), "tcp_link")
    refs.append({"q": q.tolist(), "pos": T[:3, 3].tolist(), "rot": T[:3, :3].tolist()})
json.dump({"configs": refs, "total_mass": sum(m.masses.values()), "masses": m.masses}, out.open("w"))
print("wrote", out, "total mass", round(sum(m.masses.values()), 4))
