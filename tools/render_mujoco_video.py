#!/usr/bin/env python3
"""Replay a recorded demo run (piper_py demo JSON) through the generated MuJoCo model and render it to mp4.

    MUJOCO_GL=egl python tools/render_mujoco_video.py run.json out.mp4

Shows what the MuJoCo backend did; poses come from the recorded /joint_states (fingers = +/-0.5 * gripper).
"""
import json
import sys
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from piper_description import mujoco_model, robot_description

run = json.loads(Path(sys.argv[1]).read_text())
share = Path(__import__("ament_index_python.packages", fromlist=["x"]).get_package_share_directory("piper_bringup"))
scene = mujoco_model(robot_description("mujoco"), share / "config/mujoco/inputs.xml", share / "config/mujoco/scene.xml")
m = mujoco.MjModel.from_xml_path(str(scene))
m.vis.global_.offwidth, m.vis.global_.offheight = 1280, 720
d = mujoco.MjData(m)
adr = {m.joint(i).name: m.jnt_qposadr[i] for i in range(m.njnt)}
t = np.array(run["t"]) - run["steps"][0]["t_start"]
q = np.array([[v if v is not None else 0.0 for v in row] for row in run["q"]])
names = run["joints"]
cam = mujoco.MjvCamera()
cam.lookat[:] = (0.12, 0.0, 0.2)
cam.distance, cam.azimuth, cam.elevation = 1.35, 130, -18
fps = 30
with mujoco.Renderer(m, 720, 1280) as renderer, imageio.get_writer(sys.argv[2], fps=fps, codec="libx264", quality=7) as w:
    for tt in np.arange(0, t[-1], 1 / fps):
        row = q[min(np.searchsorted(t, tt), len(t) - 1)]
        for n, v in zip(names, row):
            d.qpos[adr[n]] = v
        d.qpos[adr["gripper_joint1"]] = 0.5 * row[names.index("gripper")]
        d.qpos[adr["gripper_joint2"]] = -0.5 * row[names.index("gripper")]
        mujoco.mj_forward(m, d)
        renderer.update_scene(d, camera=cam)
        w.append_data(renderer.render())
print("wrote", sys.argv[2])
