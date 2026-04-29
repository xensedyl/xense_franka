"""
xense_franka: Synchronous command API for Franka torque control.

The public API is synchronous, while the 1kHz impedance loop runs in a
dedicated background thread.

Main Components:
    RobotInterface: Low-level robot interface (real or simulation)
    FrankaController: High-level synchronous controller with multiple modes
    SyncFrankaController: Convenience wrapper that manages robot + controller lifecycle

Quick Example:
    >>> from xense_franka import RobotInterface, FrankaController
    >>>
    >>> robot = RobotInterface("172.16.0.2")
    >>> controller = FrankaController(robot)
    >>> controller.start()
    >>> controller.move()  # Move to home
    >>> controller.stop()

For detailed documentation, see README.md and USAGE_GUIDE.md
"""

from xense_franka.constants import FR3_JOINT_LIMITS_LOWER, FR3_JOINT_LIMITS_UPPER, FR3_TORQUE_LIMIT
from xense_franka.controller import FrankaController
from xense_franka.handles import (
    CartesianGainsHandle,
    CartesianReferenceHandle,
    Handle,
    JointGainsHandle,
    JointReferenceHandle,
)
from xense_franka.references import (
    CartesianImpedanceGains,
    CartesianReference,
    JointImpedanceGains,
    JointReference,
)
from xense_franka.robot import RobotInterface
from xense_franka.sync_controller import SyncFrankaController
from xense_franka.trackers import (
    CartesianImpedanceTracker,
    ExponentialImpedanceTracker,
    JointImpedanceTracker,
)

__version__ = "0.4.0"
__all__ = [
    # Core
    "RobotInterface",
    "FrankaController",
    "SyncFrankaController",
    # Constants
    "FR3_JOINT_LIMITS_LOWER",
    "FR3_JOINT_LIMITS_UPPER",
    "FR3_TORQUE_LIMIT",
    # References / Gains
    "JointReference",
    "CartesianReference",
    "JointImpedanceGains",
    "CartesianImpedanceGains",
    # Handles
    "Handle",
    "JointReferenceHandle",
    "CartesianReferenceHandle",
    "JointGainsHandle",
    "CartesianGainsHandle",
    # Trackers
    "JointImpedanceTracker",
    "CartesianImpedanceTracker",
    "ExponentialImpedanceTracker",
]
