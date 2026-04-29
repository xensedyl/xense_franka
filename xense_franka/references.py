"""Reference and gains dataclasses for impedance control."""

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


def _default_cart_stiffness() -> np.ndarray:
    return np.array([600.0, 600.0, 600.0, 50.0, 50.0, 50.0], dtype=float)


def _default_cart_damping() -> np.ndarray:
    stiffness = _default_cart_stiffness()
    return 2.0 * np.sqrt(stiffness)


@dataclass
class JointReference:
    q: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=float))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=float))
    tau_ff: np.ndarray = field(default_factory=lambda: np.zeros(7, dtype=float))

    def copy(self) -> "JointReference":
        return JointReference(self.q.copy(), self.dq.copy(), self.tau_ff.copy())


@dataclass
class CartesianReference:
    pose: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=float))
    twist: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=float))
    nullspace_target: Optional[np.ndarray] = None

    def copy(self) -> "CartesianReference":
        return CartesianReference(
            pose=self.pose.copy(),
            twist=self.twist.copy(),
            nullspace_target=(
                None if self.nullspace_target is None else self.nullspace_target.copy()
            ),
        )


@dataclass
class JointImpedanceGains:
    stiffness: np.ndarray = field(
        default_factory=lambda: np.ones(7, dtype=float) * 80.0
    )
    damping: np.ndarray = field(
        default_factory=lambda: 2.0 * np.sqrt(np.ones(7, dtype=float) * 80.0)
    )

    def copy(self) -> "JointImpedanceGains":
        return JointImpedanceGains(self.stiffness.copy(), self.damping.copy())


@dataclass
class CartesianImpedanceGains:
    stiffness: np.ndarray = field(default_factory=_default_cart_stiffness)
    damping: np.ndarray = field(default_factory=_default_cart_damping)
    nullspace_stiffness: float = 5.0

    def copy(self) -> "CartesianImpedanceGains":
        return CartesianImpedanceGains(
            self.stiffness.copy(),
            self.damping.copy(),
            float(self.nullspace_stiffness),
        )
