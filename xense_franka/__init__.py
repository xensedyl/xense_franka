"""
xense_franka: Async command API for Franka torque control.

The public API remains async-friendly, while the 1kHz impedance loop runs in a
dedicated background thread.

Main Components:
    RobotInterface: Low-level robot interface (real or simulation)
    FrankaController: High-level asyncio controller with multiple modes
    FrankaLockUnlock: Client for robot authentication and brake control

Quick Example:
    >>> import asyncio
    >>> from xense_franka import RobotInterface, FrankaController
    >>> 
    >>> async def main():
    ...     robot = RobotInterface("172.16.0.2")
    ...     controller = FrankaController(robot)
    ...     await controller.start()
    ...     await controller.move()  # Move to home
    ...     await controller.stop()
    >>> 
    >>> asyncio.run(main())

For detailed documentation, see README.md and USAGE_GUIDE.md
"""

from xense_franka.controller import FrankaController
from xense_franka.robot import RobotInterface
from xense_franka.sync_controller import SyncFrankaController

__version__ = "0.2.0"
__all__ = ["RobotInterface", "FrankaController", "SyncFrankaController"]
