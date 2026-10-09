"""Metrics of the step-response records written by `piper stepresp` (see piper_py/stepresp.py)."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def analyze(path: Path) -> dict:
    r = json.loads(Path(path).read_text())
    t, q = np.array(r["t"]), np.array([[np.nan if v is None else v for v in row] for row in r["q"]])
    out = []
    for ev in r["events"]:
        j, d = ev["index"], ev["delta"]
        base_m = (t > ev["t_up"] - 0.2) & (t <= ev["t_up"])
        y0 = np.nanmean(q[base_m, j])
        m = (t >= ev["t_up"]) & (t <= ev["t_down"])
        tt, y = t[m] - ev["t_up"], (q[m, j] - y0) / d  # normalised: 0 -> 1
        final_m = tt > (tt[-1] - 0.3)
        final = float(np.mean(y[final_m]))
        def cross(level):
            idx = np.argmax(y >= level) if (y >= level).any() else None
            return None if idx is None or y[idx] < level else float(np.interp(level, y[idx - 1:idx + 1], tt[idx - 1:idx + 1])) if idx > 0 else float(tt[0])
        t05, t10, t90 = cross(0.05), cross(0.1), cross(0.9)
        outside = np.where(np.abs(y - final) > 0.02)[0]
        settle = float(tt[outside[-1]]) if len(outside) else 0.0
        out.append({"joint": ev["joint"], "delta": d, "dead_ms": None if t05 is None else t05 * 1000,
                    "rise_ms": None if (t10 is None or t90 is None) else (t90 - t10) * 1000,
                    "overshoot_pct": float(max(0.0, (np.max(y) - final)) * 100),
                    "settle_ms": settle * 1000, "steady_err_mrad": float((final - 1.0) * d * 1000),
                    "t": tt.tolist(), "y": y.tolist()})
    return {"backend": r["backend"], "steps": out}


if __name__ == "__main__":
    import sys
    for p in sys.argv[1:]:
        a = analyze(Path(p))
        print(a["backend"])
        for s in a["steps"]:
            print(f"  {s['joint']} {s['delta']:+.2f}: dead {s['dead_ms']:.0f} ms, rise10-90 {s['rise_ms']:.0f} ms, overshoot {s['overshoot_pct']:.1f}%, "
                  f"settle2% {s['settle_ms']:.0f} ms, steady err {s['steady_err_mrad']:+.1f} mrad")
