"""Known test trajectories for measuring how the arm follows 25 ms setpoints (tools/real_arm/arm_stream_test.py). One joint moves, the rest stay put; no ROS.

    traj = cosine(q0, joint=0, amplitude=0.17, period=4.0)       # smooth out and back, starts and ends at rest at q0 (peak speed = amplitude * pi / period)
    traj = ramp(q0, joint=0, amplitude=0.17, period=2.0)         # minimum-jerk out over `period`, then back (peak speed = 1.875 * amplitude / period)
    required_percent(traj)                                       # the speed percent execute() will ask for in "fixed" mode (100 % = 3.0 rad/s)
"""

from __future__ import annotations

import math

import numpy as np

from piper_core.backend import ARM_JOINTS, Trajectory

FIRMWARE_MAX_VELOCITY = 3.0


def _build(q0, joint, offsets, t) -> Trajectory:
    q = np.tile(np.asarray(q0, dtype=float), (len(t), 1))
    q[:, joint] += offsets
    return Trajectory(list(ARM_JOINTS), t, q)


def cosine(q0, joint: int, amplitude: float, period: float, cycles: int = 1, dt: float = 0.025) -> Trajectory:
    """q = q0 + amplitude/2 (1 - cos(2 pi t / period)) for `cycles` periods: smooth, zero speed at both ends. `amplitude` in rad (negative: the other way)."""
    n = int(round(cycles * period / dt))
    t = np.arange(n + 1) * (cycles * period / n)
    return _build(q0, joint, amplitude / 2.0 * (1.0 - np.cos(2.0 * math.pi * t / period)), t)


def ramp(q0, joint: int, amplitude: float, period: float, dt: float = 0.025) -> Trajectory:
    """Minimum-jerk move of `amplitude` over `period`, then the same back: total 2 x period."""
    n = int(round(period / dt))
    s = np.linspace(0.0, 1.0, n + 1)
    pos = 10 * s**3 - 15 * s**4 + 6 * s**5
    offsets = np.concatenate([amplitude * pos, amplitude * pos[::-1][1:]])
    t = np.arange(len(offsets)) * (period / n)
    return _build(q0, joint, offsets, t)


def peak_speed(traj: Trajectory) -> float:
    """Fastest planned joint speed between two points, rad/s."""
    return float((np.abs(np.diff(traj.q, axis=0)).max(axis=1) / np.diff(traj.t)).max())


def required_percent(traj: Trajectory, margin_percent: float = 20.0) -> int:
    """The speed percent `RealArm.execute(mode="fixed")` sets: the fastest segment plus the margin, against 3.0 rad/s at 100 %."""
    return int(min(100, max(1, math.ceil(peak_speed(traj) * (1 + margin_percent / 100.0) * 100.0 / FIRMWARE_MAX_VELOCITY))))
