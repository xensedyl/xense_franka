"""Tracker context managers for structured impedance control sessions.

Each tracker switches the controller to the appropriate mode on entry and
optionally restores the previous mode and gains on exit.  Trackers support
both synchronous ``with`` and asynchronous ``async with`` usage.

Example::

    with JointImpedanceTracker(controller, stiffness=100) as t:
        t.set_target(q_desired)
        while t.ok():
            t.tick()
"""

from __future__ import annotations

import time
from typing import Optional, TYPE_CHECKING

import numpy as np

from xense_franka.references import (
    CartesianImpedanceGains,
    JointImpedanceGains,
)
from xense_franka.torque_utils import as_array, critical_damping, pose_copy

if TYPE_CHECKING:
    from xense_franka.controller import FrankaController


class JointImpedanceTracker:
    """Tracker for joint-space impedance control."""

    def __init__(
        self,
        controller: "FrankaController",
        stiffness=None,
        damping=None,
        damping_ratio: float = 1.0,
        restore_on_exit: bool = True,
    ):
        self._ctrl = controller
        self._restore = restore_on_exit
        self._prev_type: Optional[str] = None
        self._prev_gains: Optional[JointImpedanceGains] = None
        self._stopped = False

        if stiffness is not None:
            stiffness = as_array(stiffness, 7)
        if damping is not None:
            damping = as_array(damping, 7)
        elif stiffness is not None:
            damping = critical_damping(stiffness) * damping_ratio

        self._stiffness = stiffness
        self._damping = damping

    def _enter(self):
        self._prev_type = self._ctrl.effective_type
        self._prev_gains = self._ctrl._joint_gains_handle.get().copy()
        self._ctrl.switch("impedance")
        if self._stiffness is not None:
            self._ctrl.set_joint_gains(self._stiffness, self._damping)
        return self

    def _exit(self):
        if self._restore and self._prev_type is not None:
            # Restore gains first, then post atomic type change.
            if self._prev_gains is not None:
                self._ctrl._joint_gains_handle.set(self._prev_gains)
            self._ctrl._request_type_change(self._prev_type)
        self._stopped = True

    # Sync context manager
    def __enter__(self):
        return self._enter()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._exit()
        return False

    # Async context manager
    async def __aenter__(self):
        return self._enter()

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self._exit()
        return False

    def set_target(self, q, dq=None, tau_ff=None):
        self._ctrl._set_joint_reference(q, dq=dq, tau_ff=tau_ff)

    def set_gains(self, stiffness, damping=None, damping_ratio: float = 1.0):
        self._ctrl.set_joint_gains(stiffness, damping, damping_ratio=damping_ratio)

    def ok(self) -> bool:
        return self._ctrl.running and not self._stopped

    def tick(self, dt: float = 0.02):
        time.sleep(dt)

    def stop(self):
        self._stopped = True


class CartesianImpedanceTracker:
    """Tracker for Cartesian-space (OSC) impedance control."""

    def __init__(
        self,
        controller: "FrankaController",
        stiffness=None,
        damping=None,
        damping_ratio: float = 1.0,
        nullspace_stiffness: Optional[float] = None,
        restore_on_exit: bool = True,
    ):
        self._ctrl = controller
        self._restore = restore_on_exit
        self._prev_type: Optional[str] = None
        self._prev_gains: Optional[CartesianImpedanceGains] = None
        self._stopped = False

        if stiffness is not None:
            stiffness = as_array(stiffness, 6)
        if damping is not None:
            damping = as_array(damping, 6)
        elif stiffness is not None:
            damping = critical_damping(stiffness) * damping_ratio

        self._stiffness = stiffness
        self._damping = damping
        self._nullspace_stiffness = nullspace_stiffness

    def _enter(self):
        self._prev_type = self._ctrl.effective_type
        self._prev_gains = self._ctrl._cart_gains_handle.get().copy()
        self._ctrl.switch("osc")
        if self._stiffness is not None:
            self._ctrl.set_cartesian_gains(
                self._stiffness, self._damping,
                nullspace_stiffness=self._nullspace_stiffness,
            )
        return self

    def _exit(self):
        if self._restore and self._prev_type is not None:
            # Restore gains first, then post atomic type change.
            if self._prev_gains is not None:
                self._ctrl._cart_gains_handle.set(self._prev_gains)
            self._ctrl._request_type_change(self._prev_type)
        self._stopped = True

    # Sync context manager
    def __enter__(self):
        return self._enter()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._exit()
        return False

    # Async context manager
    async def __aenter__(self):
        return self._enter()

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self._exit()
        return False

    def set_target(self, pose, twist=None, nullspace_target=None):
        self._ctrl._set_cartesian_reference(pose, twist=twist, nullspace_target=nullspace_target)

    def set_gains(self, stiffness, damping=None, damping_ratio: float = 1.0, nullspace_stiffness=None):
        self._ctrl.set_cartesian_gains(
            stiffness, damping, damping_ratio=damping_ratio,
            nullspace_stiffness=nullspace_stiffness,
        )

    def ok(self) -> bool:
        return self._ctrl.running and not self._stopped

    def tick(self, dt: float = 0.02):
        time.sleep(dt)

    def stop(self):
        self._stopped = True


class ExponentialImpedanceTracker:
    """Tracker that exponentially smooths the reference toward a final target.

    Each call to ``tick()`` advances the reference by::

        ref += alpha * (goal - ref)

    where ``alpha = 1 - exp(-tick_dt / time_constant)``.
    """

    def __init__(
        self,
        controller: "FrankaController",
        mode: str = "impedance",
        time_constant: float = 0.5,
        stiffness=None,
        damping=None,
        damping_ratio: float = 1.0,
        nullspace_stiffness: Optional[float] = None,
        restore_on_exit: bool = True,
    ):
        if mode not in ("impedance", "osc"):
            raise ValueError(f"mode must be 'impedance' or 'osc', got '{mode}'")

        self._ctrl = controller
        self._mode = mode
        self._time_constant = max(float(time_constant), 1e-6)
        self._restore = restore_on_exit
        self._prev_type: Optional[str] = None
        self._prev_joint_gains: Optional[JointImpedanceGains] = None
        self._prev_cart_gains: Optional[CartesianImpedanceGains] = None
        self._stopped = False

        self._stiffness = stiffness
        self._damping = damping
        self._damping_ratio = damping_ratio
        self._nullspace_stiffness = nullspace_stiffness

        # Current smoothed reference (set on enter)
        self._current_q: Optional[np.ndarray] = None
        self._goal_q: Optional[np.ndarray] = None
        self._current_pose: Optional[np.ndarray] = None
        self._goal_pose: Optional[np.ndarray] = None

    def _enter(self):
        self._prev_type = self._ctrl.effective_type
        # Save gains for the mode we are about to use
        if self._mode == "impedance":
            self._prev_joint_gains = self._ctrl._joint_gains_handle.get().copy()
        else:
            self._prev_cart_gains = self._ctrl._cart_gains_handle.get().copy()
        self._ctrl.switch(self._mode)

        if self._mode == "impedance":
            if self._stiffness is not None:
                self._ctrl.set_joint_gains(
                    self._stiffness, self._damping, damping_ratio=self._damping_ratio
                )
            self._current_q = self._ctrl.get_current_joint_positions()
            self._goal_q = self._current_q.copy()
        else:
            if self._stiffness is not None:
                self._ctrl.set_cartesian_gains(
                    self._stiffness, self._damping, damping_ratio=self._damping_ratio,
                    nullspace_stiffness=self._nullspace_stiffness,
                )
            self._current_pose = self._ctrl.get_current_ee_pose()
            self._goal_pose = self._current_pose.copy()
        return self

    def _exit(self):
        if self._restore and self._prev_type is not None:
            # Restore gains first, then post atomic type change.
            if self._mode == "impedance" and self._prev_joint_gains is not None:
                self._ctrl._joint_gains_handle.set(self._prev_joint_gains)
            elif self._mode == "osc" and self._prev_cart_gains is not None:
                self._ctrl._cart_gains_handle.set(self._prev_cart_gains)
            self._ctrl._request_type_change(self._prev_type)
        self._stopped = True

    def __enter__(self):
        return self._enter()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._exit()
        return False

    async def __aenter__(self):
        return self._enter()

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self._exit()
        return False

    def set_target(self, target):
        """Set the final goal that the reference smooths toward."""
        if self._mode == "impedance":
            self._goal_q = as_array(target, 7)
        else:
            self._goal_pose = pose_copy(target)

    def ok(self) -> bool:
        return self._ctrl.running and not self._stopped

    def tick(self, dt: float = 0.02):
        """Advance the smoothed reference by one step and sleep *dt* seconds."""
        alpha = 1.0 - np.exp(-dt / self._time_constant)

        if self._mode == "impedance" and self._current_q is not None and self._goal_q is not None:
            self._current_q = self._current_q + alpha * (self._goal_q - self._current_q)
            self._ctrl._set_joint_reference(self._current_q)
        elif self._mode == "osc" and self._current_pose is not None and self._goal_pose is not None:
            # Interpolate position linearly
            pos = self._current_pose[:3, 3] + alpha * (
                self._goal_pose[:3, 3] - self._current_pose[:3, 3]
            )
            # Interpolate orientation via slerp
            from scipy.spatial.transform import Rotation, Slerp

            rots = Rotation.concatenate([
                Rotation.from_matrix(self._current_pose[:3, :3]),
                Rotation.from_matrix(self._goal_pose[:3, :3]),
            ])
            slerp = Slerp([0.0, 1.0], rots)
            interp_rot = slerp([alpha]).as_matrix()[0]

            self._current_pose = self._current_pose.copy()
            self._current_pose[:3, 3] = pos
            self._current_pose[:3, :3] = interp_rot
            self._ctrl._set_cartesian_reference(self._current_pose)

        time.sleep(dt)

    def stop(self):
        self._stopped = True
