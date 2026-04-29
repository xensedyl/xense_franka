import threading
from pathlib import Path

import numpy as np

CUR_DIR = Path(__file__).parent.resolve()


class RobotInterface:
    """
    Thin Franka FCI wrapper with serialized read/write access.

    The control loop owns the fast readOnce/writeOnce cycle. External callers
    can still query the latest cached state without racing the FCI session.
    """

    def __init__(self, ip: str):
        self.real = True
        self.torque_controller = None
        self._robot_state = None
        self._latest_state = None
        self._latest_state_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._control_thread_id = None
        self._last_duration_sec = 1e-3

        import pylibfranka

        self.robot = pylibfranka.Robot(ip, pylibfranka.RealtimeConfig.kIgnore)
        self.model = self.robot.load_model()

        self.robot.set_collision_behavior(
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
            [100.0, 100.0, 100.0, 100.0, 100.0, 100.0],
        )

        self._sync_state()
        self._cache_latest_state(self._build_state_dict(self._robot_state, dt=self._last_duration_sec))

    def start(self):
        with self._io_lock:
            if self.torque_controller is None:
                self.torque_controller = self.robot.start_torque_control()
                self._control_thread_id = None

    def stop(self):
        with self._io_lock:
            try:
                self.robot.stop()
            finally:
                self.torque_controller = None
                self._control_thread_id = None

    def _sync_state(self):
        if self.torque_controller is None:
            self._robot_state = self.robot.read_once()
            self._last_duration_sec = 1e-3
        else:
            self._robot_state, duration = self.torque_controller.readOnce()
            if hasattr(duration, "to_sec"):
                self._last_duration_sec = max(float(duration.to_sec()), 1e-6)
            elif hasattr(duration, "toSec"):
                self._last_duration_sec = max(float(duration.toSec()), 1e-6)
            else:
                self._last_duration_sec = 1e-3

    def _build_state_dict(self, robot_state, dt: float):
        O_T_EE = np.array(robot_state.O_T_EE, dtype=float).reshape(4, 4).T
        jac = np.array(self.model.zero_jacobian(robot_state), dtype=float).reshape(6, 7, order="F")
        mm = np.array(self.model.mass(robot_state), dtype=float).reshape(7, 7, order="F")
        coriolis = np.array(self.model.coriolis(robot_state), dtype=float)

        return {
            "qpos": np.array(robot_state.q, dtype=float),
            "qvel": np.array(robot_state.dq, dtype=float),
            "O_T_EE": np.array(robot_state.O_T_EE, dtype=float),
            "ee": O_T_EE,
            "jac": jac,
            "mm": mm,
            "coriolis": coriolis,
            "last_torque": np.array(robot_state.tau_J_d, dtype=float),
            "ext_wrench": np.array(robot_state.O_F_ext_hat_K, dtype=float),
            "robot_state": robot_state,
            "dt": float(dt),
        }

    def _cache_latest_state(self, state: dict):
        with self._latest_state_lock:
            self._latest_state = {
                key: (value.copy() if isinstance(value, np.ndarray) else value)
                for key, value in state.items()
            }

    def read_control_state(self):
        current_thread_id = threading.get_ident()
        with self._io_lock:
            if self.torque_controller is None:
                raise RuntimeError("Torque control not started")
            if self._control_thread_id is None:
                self._control_thread_id = current_thread_id
            elif self._control_thread_id != current_thread_id:
                raise RuntimeError("read_control_state must be called from the owning control thread")

            self._sync_state()
            state = self._build_state_dict(self._robot_state, dt=self._last_duration_sec)
            self._cache_latest_state(state)
            return state

    @property
    def state(self):
        current_thread_id = threading.get_ident()
        with self._latest_state_lock:
            cached = self._latest_state
            owner_thread = self._control_thread_id

        if cached is not None and owner_thread is not None and owner_thread != current_thread_id:
            return {
                key: (value.copy() if isinstance(value, np.ndarray) else value)
                for key, value in cached.items()
            }

        with self._io_lock:
            self._sync_state()
            state = self._build_state_dict(self._robot_state, dt=self._last_duration_sec)
            self._cache_latest_state(state)
            return {
                key: (value.copy() if isinstance(value, np.ndarray) else value)
                for key, value in state.items()
            }

    def step(self, torque: np.ndarray):
        if self.torque_controller is None:
            raise RuntimeError("Torque control is not started")
        torque = np.asarray(torque, dtype=float)
        if torque.shape != (7,):
            raise ValueError(f"Expected torque shape (7,), got {torque.shape}")

        import pylibfranka

        torque_command = pylibfranka.Torques(torque.tolist())
        torque_command.motion_finished = False

        with self._io_lock:
            self.torque_controller.writeOnce(torque_command)

