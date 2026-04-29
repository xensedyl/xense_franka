"""Lock-free double-buffer handles for real-time control data exchange.

Uses Python's GIL to guarantee atomic index swaps. The 1kHz control loop
calls ``get()`` (reads the active buffer) while user code calls ``set()``
(writes the inactive buffer and then flips the index).
"""

from typing import Generic, TypeVar

from xense_franka.references import (
    CartesianImpedanceGains,
    CartesianReference,
    JointImpedanceGains,
    JointReference,
)

T = TypeVar("T")


class Handle(Generic[T]):
    """Double-buffered handle for lock-free producer/consumer exchange."""

    __slots__ = ("_buffers", "_active")

    def __init__(self, initial: T, *, copy_fn=None):
        if copy_fn is None:
            copy_fn = getattr(initial, "copy", None)
            if copy_fn is None:
                raise TypeError("initial value must have a .copy() method or provide copy_fn")
        self._buffers = [initial, copy_fn()]
        self._active: int = 0

    def get(self) -> T:
        """Read from the active buffer (called by the control loop)."""
        return self._buffers[self._active]

    def set(self, value: T) -> None:
        """Write *value* into the inactive buffer, then swap."""
        inactive = 1 - self._active
        self._buffers[inactive] = value
        # GIL guarantees this integer assignment is atomic
        self._active = inactive


class JointReferenceHandle(Handle["JointReference"]):
    def __init__(self, initial: "JointReference | None" = None):
        if initial is None:
            initial = JointReference()
        super().__init__(initial, copy_fn=initial.copy)


class CartesianReferenceHandle(Handle["CartesianReference"]):
    def __init__(self, initial: "CartesianReference | None" = None):
        if initial is None:
            initial = CartesianReference()
        super().__init__(initial, copy_fn=initial.copy)


class JointGainsHandle(Handle["JointImpedanceGains"]):
    def __init__(self, initial: "JointImpedanceGains | None" = None):
        if initial is None:
            initial = JointImpedanceGains()
        super().__init__(initial, copy_fn=initial.copy)


class CartesianGainsHandle(Handle["CartesianImpedanceGains"]):
    def __init__(self, initial: "CartesianImpedanceGains | None" = None):
        if initial is None:
            initial = CartesianImpedanceGains()
        super().__init__(initial, copy_fn=initial.copy)
