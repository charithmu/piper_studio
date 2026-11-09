"""Before/after benchmark of the Isaac server: one identical trajectory, wall time and real-time factor (simulated s / wall s).

    source scripts/env.sh; python isaac/bench_step.py [--seconds 4] [--modes plain render record]

Runs in .venv (client side, through PiperIsaacSim, so it measures the whole stack). Modes: plain (no rendering), render (--render, no recording),
record (--render and an mp4 at 30 fps; the video is then checked: frame count and that consecutive frames differ, i.e. no stale frames).
"""

import argparse
import json
import math
import tempfile
import time
from pathlib import Path

import numpy as np
import piper_assets
from piper_isaac import PiperIsaacSim

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=4.0)
ap.add_argument("--modes", nargs="+", default=["plain", "render", "record"])
ap.add_argument("--ctrl-hz", type=float, default=250.0)
ap.add_argument("--physics-hz", type=float, default=250.0)
args = ap.parse_args()

results = {}
for mode in args.modes:
    sim = PiperIsaacSim(scene="table", ctrl_hz=args.ctrl_hz, physics_hz=args.physics_hz, render=mode != "plain")
    n = int(args.seconds * args.ctrl_hz)
    zero = np.array(piper_assets.zero_pose())
    t = np.arange(1, n + 1) / args.ctrl_hz
    ramp = np.minimum(t / 1.0, 1.0)[:, None]                          # smooth start: no step command
    rows = (zero + ramp * 0.3 * np.sin(2 * math.pi * 0.5 * t)[:, None] * np.array([1, 0.5, 0.5, 1, 0.5, 1])).tolist()
    video = Path(tempfile.mkdtemp()) / "bench.mp4"
    if mode == "record":
        sim.record_start(video, fps=30)
    sim.reset()
    t0 = time.perf_counter()
    out = sim._rpc(cmd="run", q=rows)
    wall = time.perf_counter() - t0
    state = sim._rpc(cmd="state")
    r = {"wall_s": round(wall, 3), "rtf": round(args.seconds / wall, 2), "sim_t": state["t"],
         "tracking_err_rad": round(float(np.max(np.abs(np.array(out["q_meas"][-50:]) - np.array(rows[-50:])))), 4),
         "q_final": [round(x, 4) for x in out["q_meas"][-1]]}
    if mode == "record":
        t1 = time.perf_counter()
        sim.record_stop()
        r["record_stop_s"] = round(time.perf_counter() - t1, 3)
        import imageio.v2 as imageio

        frames = [np.asarray(f, dtype=np.int16) for f in imageio.get_reader(str(video))]
        diffs = [float(np.mean(np.abs(a - b))) for a, b in zip(frames, frames[1:])]
        r.update(frames=len(frames), expected_frames=round(args.seconds * 30), stale_pairs=sum(d < 1e-3 for d in diffs), mean_frame_diff=round(float(np.mean(diffs)), 3))
    sim.close()
    results[mode] = r
    print(mode, json.dumps(r), flush=True)
