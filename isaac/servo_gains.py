"""Joint servo gains for Isaac Sim, read from gains.json: derived from the MuJoCo model that config/actuation.yaml makes (tools/release/build_gains.py), so the two simulators
carry the same servo, with the same force limits (the description's efforts; the gripper's 3 N ceiling).

(stiffness, damping, max force): N*m/rad, N*m*s/rad, N*m for revolute joints; N/m, N*s/m, N for the gripper. They stand in for the unidentified firmware position loop
(docs/description/DECISIONS.md). The wrist is heavier with the ZED X Mini on its bracket (the default), so the damping of joints 4-6 differs per variant.
"""

import json
from pathlib import Path

_DOC = json.loads((Path(__file__).resolve().parent / "gains.json").read_text())


def gains(camera: bool = True) -> dict:
    """{official joint name: (stiffness, damping, max force)} for the model with (default) or without the wrist camera."""
    return {k: tuple(v) for k, v in _DOC["camera" if camera else "no_camera"].items()}
