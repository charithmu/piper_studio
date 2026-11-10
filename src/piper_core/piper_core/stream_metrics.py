"""Numbers from a trajectory run: how well did the arm follow what it was sent? No hardware needed to compute them; numpy only.

    r = arm.execute(traj, mode="per_point")
    m = analyze(traj, arm.last_stream)             # dict, angles in degrees, times in milliseconds
    print(format_metrics(m))

What is measured (per joint that moves in the plan, plus the overall):
  rms_deg / max_deg      tracking error: measured joint minus the planned joint at the same time (the plan interpolated from the trajectory)
  lag_ms                 the delay of the arm behind the plan that minimises the rms error (searched 0..max_lag, 5 ms steps); residual_rms_deg is what is left at that lag
  final_deg              error at the end of the run
  overshoot_deg          how far the measured joint went past its final planned value in the direction of travel (0 for a joint that does not travel)
  peak_speed             the fastest measured speed (rad/s, over a 50 ms window) against the fastest planned one, and the nominal speed of the speed percent used
  timing                 how late each setpoint was sent against its scheduled time (mean, p95, max, ms) and the spacing of the sends (mean, std, ms)
`repeatability(finals)` is the spread of the final poses of several runs of the same trajectory.
The plan is the trajectory as given (linear between points): for a planner with dq the real curve is smoother, so lag is slightly over-stated at sharp corners.
"""

from __future__ import annotations

import math

import numpy as np

from piper_core.backend import Trajectory

DEG = 180.0 / math.pi
FIRMWARE_MAX_VELOCITY = 3.0       # rad/s at 100 %


def _plan(traj: Trajectory, t) -> np.ndarray:
    return np.stack([np.interp(t, traj.t, traj.q[:, k]) for k in range(traj.q.shape[1])], axis=1)


def _peak_speed(t: np.ndarray, q: np.ndarray, window: float = 0.05) -> np.ndarray:
    """Fastest speed per joint over `window` seconds (rad/s); robust against the 5 ms sampling noise."""
    if len(t) < 3:
        return np.zeros(q.shape[1])
    out = np.zeros(q.shape[1])
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + window)
        if j >= len(t):
            break
        out = np.maximum(out, np.abs(q[j] - q[i]) / (t[j] - t[i]))
    return out


def analyze(traj: Trajectory, stream: dict, max_lag: float = 0.6, step: float = 0.005, moving_deg: float = 0.1) -> dict:
    """`stream` is `RealArm.last_stream`. Joints whose plan moves less than `moving_deg` are left out of the per-joint table."""
    t, q = np.asarray(stream["t"], float), np.asarray(stream["q"], float)
    if len(t) < 5:
        raise ValueError("not enough measured samples to analyze")
    plan_range = np.ptp(traj.q, axis=0)
    moving = [k for k in range(traj.q.shape[1]) if plan_range[k] * DEG >= moving_deg]
    if not moving:
        raise ValueError("the plan does not move any joint")
    inside = (t >= 0.0) & (t <= traj.t[-1])
    err = q - _plan(traj, t)
    lags = np.arange(0.0, max_lag + 1e-9, step)
    rms_by_lag = []
    for lag in lags:
        tt = np.clip(t - lag, 0.0, traj.t[-1])
        rms_by_lag.append(float(np.sqrt(np.mean((q[inside][:, moving] - _plan(traj, tt[inside])[:, moving]) ** 2))))
    best = int(np.argmin(rms_by_lag))
    joints = {}
    speed_meas, speed_plan = _peak_speed(t, q), _peak_speed(np.asarray(traj.t, float), traj.q, window=0.05)
    pct = max(stream.get("percent") or [0])
    for k in moving:
        name = traj.joint_names[k]
        direction = np.sign(traj.q[-1, k] - traj.q[0, k])
        joints[name] = {
            "rms_deg": float(np.sqrt(np.mean(err[inside, k] ** 2))) * DEG,
            "max_deg": float(np.max(np.abs(err[inside, k]))) * DEG,
            "final_deg": float(q[-1, k] - traj.q[-1, k]) * DEG,
            "overshoot_deg": float(max(0.0, np.max((q[:, k] - traj.q[-1, k]) * direction))) * DEG if direction != 0 else 0.0,
            "peak_speed_rad_s": float(speed_meas[k]),
            "planned_peak_speed_rad_s": float(speed_plan[k]),
        }
    sent, sched = np.asarray(stream.get("t_sent", []), float), np.asarray(stream.get("t_sched", []), float)
    late = (sent - sched[: len(sent)]) * 1000.0 if len(sent) else np.zeros(0)
    gaps = np.diff(sent) * 1000.0 if len(sent) > 1 else np.zeros(0)
    return {
        "mode": stream.get("mode"), "lookahead": stream.get("lookahead"), "speed_percent_max": pct,
        "nominal_speed_rad_s": pct / 100.0 * FIRMWARE_MAX_VELOCITY,
        "lag_ms": float(lags[best] * 1000.0), "residual_rms_deg": rms_by_lag[best] * DEG, "rms_deg_at_zero_lag": rms_by_lag[0] * DEG,
        "joints": joints,
        "timing": {"late_mean_ms": float(late.mean()) if len(late) else math.nan, "late_p95_ms": float(np.percentile(late, 95)) if len(late) else math.nan,
                   "late_max_ms": float(late.max()) if len(late) else math.nan, "gap_mean_ms": float(gaps.mean()) if len(gaps) else math.nan,
                   "gap_std_ms": float(gaps.std()) if len(gaps) else math.nan, "sends": int(len(sent))},
    }


def repeatability(finals) -> dict:
    """Spread of the final poses (rows: runs, columns: six joints, rad) of repeated runs: per-joint std and max deviation from the mean, in degrees."""
    f = np.asarray(finals, float)
    if f.ndim != 2 or len(f) < 2:
        raise ValueError("need at least two runs")
    return {"std_deg": (f.std(axis=0, ddof=1) * DEG).tolist(), "max_dev_deg": (np.abs(f - f.mean(axis=0)).max(axis=0) * DEG).tolist(), "runs": int(len(f))}


def format_metrics(m: dict) -> str:
    t = m["timing"]
    lines = [f"mode {m['mode']} lookahead {m['lookahead']} · speed percent up to {m['speed_percent_max']} % (nominal {m['nominal_speed_rad_s']:.2f} rad/s)",
             f"lag {m['lag_ms']:.0f} ms · rms error {m['rms_deg_at_zero_lag']:.2f} deg, {m['residual_rms_deg']:.2f} deg once the lag is removed",
             f"sends {t['sends']} · late mean {t['late_mean_ms']:.1f} p95 {t['late_p95_ms']:.1f} max {t['late_max_ms']:.1f} ms · spacing {t['gap_mean_ms']:.1f} +- {t['gap_std_ms']:.1f} ms",
             "joint    rms    max  final  overshoot   peak speed (planned)  [deg, rad/s]"]
    for name, j in m["joints"].items():
        lines.append(f"{name:7s} {j['rms_deg']:5.2f} {j['max_deg']:6.2f} {j['final_deg']:6.2f} {j['overshoot_deg']:9.2f}   {j['peak_speed_rad_s']:6.3f} ({j['planned_peak_speed_rad_s']:.3f})")
    return "\n".join(lines)
