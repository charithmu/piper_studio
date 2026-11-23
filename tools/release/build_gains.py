#!/usr/bin/env python3
"""Write isaac/gains.json: the joint drives of the Isaac model, DERIVED from the MuJoCo model that config/actuation.yaml makes (needs the ROS environment).

    source scripts/env.sh && python tools/release/build_gains.py

Isaac's drives are (stiffness, damping, max force) per joint. Stiffness and max force are the yaml's `kp` and the description's force limits; the damping is the `kv` MuJoCo derives
from `dampratio` and the link inertia at the compile pose, read back from the compiled model, so the two simulators carry the same servo. The gripper's fingers get the gripper joint's
numbers. Committed (small, reviewable); a test compares it with a fresh derivation. Names are the official joint names (no prefix).
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import mujoco
from ament_index_python.packages import get_package_share_directory

import piper_sim
from piper_description import actuation

WS = Path(__file__).resolve().parents[2]


def derive(camera: str) -> dict:
    cfg = Path(get_package_share_directory("piper_sim")) / "config" / "mujoco"
    with tempfile.TemporaryDirectory() as tmp:
        scene = piper_sim.mujoco_model(piper_sim.robot_description("mujoco", prefix="", camera=camera), cfg / "inputs.xml", cfg / "scene.xml", cache_dir=tmp, prefix="")
        m = mujoco.MjModel.from_xml_path(str(scene))
    out = {}
    for i in range(m.nu):
        name = m.actuator(i).name
        out[name] = [round(float(m.actuator_gainprm[i, 0]), 3), round(float(-m.actuator_biasprm[i, 2]), 3), round(float(m.actuator_forcerange[i, 1]), 3)]
    for finger in ("gripper_joint1", "gripper_joint2"):
        out[finger] = list(out["gripper"])
    return out


def main():
    doc = {"schema": "piper.isaac_gains/1",
           "source": "derived by tools/release/build_gains.py from config/actuation.yaml + firmware_values.yaml through the MuJoCo model; (stiffness N.m/rad or N/m, damping, max force N.m or N)",
           "actuation_status": actuation()["arm"]["status"],
           "camera": derive("zedxm"), "no_camera": derive("none")}
    (WS / "isaac" / "gains.json").write_text(json.dumps(doc, indent=1) + "\n")
    print("wrote isaac/gains.json")


if __name__ == "__main__":
    main()
