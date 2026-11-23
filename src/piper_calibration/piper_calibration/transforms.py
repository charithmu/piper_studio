"""4x4 pose helpers shared by planning and the solver. numpy only."""
import numpy as np


def make_T(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.asarray(R, float).reshape(3, 3), np.asarray(t, float).reshape(3)
    return T


def inv_T(T):
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return out


def rotation_angle_deg(R):
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))))


def pose_difference(A, B):
    """(rotation deg, translation mm) between two 4x4 poses."""
    D = inv_T(A) @ B
    return rotation_angle_deg(D[:3, :3]), float(np.linalg.norm(D[:3, 3]) * 1000)
