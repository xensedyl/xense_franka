"""Franka FR3 robot constants."""

import numpy as np

FR3_JOINT_LIMITS_LOWER = np.array(
    [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159], dtype=float
)
FR3_JOINT_LIMITS_UPPER = np.array(
    [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159], dtype=float
)
FR3_TORQUE_LIMIT = np.array(
    [87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0], dtype=float
)
