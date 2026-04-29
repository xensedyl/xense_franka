"""Utility functions for torque computation and array handling."""

import numpy as np
from scipy.spatial.transform import Rotation as R


def as_array(value, size: int) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape == ():
        arr = np.full(size, float(arr), dtype=float)
    if arr.shape != (size,):
        raise ValueError(f"Expected shape ({size},), got {arr.shape}")
    return arr.copy()


def normalize_quat(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=float).copy()
    norm = np.linalg.norm(quat)
    if norm < 1e-12:
        raise ValueError("Quaternion norm is too small")
    return quat / norm


def orthonormalize_rotation(matrix: np.ndarray) -> np.ndarray:
    return R.from_matrix(np.asarray(matrix, dtype=float)).as_matrix()


def pose_copy(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (4, 4):
        raise ValueError(f"Expected pose shape (4, 4), got {pose.shape}")
    out = pose.copy()
    out[:3, :3] = orthonormalize_rotation(out[:3, :3])
    return out


def critical_damping(stiffness: np.ndarray) -> np.ndarray:
    return 2.0 * np.sqrt(np.maximum(stiffness, 0.0))


def saturate_torque_rate(
    tau_desired: np.ndarray, tau_reference: np.ndarray, max_delta_tau: np.ndarray
) -> np.ndarray:
    return tau_reference + np.clip(
        tau_desired - tau_reference, -max_delta_tau, max_delta_tau
    )


def compute_joint_limit_torque(
    q: np.ndarray,
    dq: np.ndarray,
    lower_limits: np.ndarray,
    upper_limits: np.ndarray,
    activation_distance: float,
    stiffness: float,
    damping: float,
    max_torque: float,
) -> np.ndarray:
    tau_limit = np.zeros(7, dtype=float)
    activation_distance = max(float(activation_distance), 1e-6)

    for i in range(7):
        lower_soft = lower_limits[i] + activation_distance
        upper_soft = upper_limits[i] - activation_distance

        if q[i] < lower_soft:
            penetration = (lower_soft - q[i]) / activation_distance
            tau = stiffness * (np.exp(penetration) - 1.0)
            if dq[i] < 0.0:
                tau += damping * (-dq[i])
            tau_limit[i] = min(tau, max_torque)
        elif q[i] > upper_soft:
            penetration = (q[i] - upper_soft) / activation_distance
            tau = -stiffness * (np.exp(penetration) - 1.0)
            if dq[i] > 0.0:
                tau -= damping * dq[i]
            tau_limit[i] = max(tau, -max_torque)

    return tau_limit


def pseudo_inverse(matrix: np.ndarray, tolerance: float = 1e-6) -> np.ndarray:
    u, s, vh = np.linalg.svd(matrix, full_matrices=False)
    s_inv = np.zeros_like(s)
    mask = s > tolerance
    s_inv[mask] = 1.0 / s[mask]
    return vh.T @ np.diag(s_inv) @ u.T
