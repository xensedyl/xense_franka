import asyncio
import threading
import time
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from ruckig import InputParameter, Result, Ruckig, Trajectory
from scipy.spatial.transform import Rotation as R

from xense_franka.robot import RobotInterface

CUR_DIR = Path(__file__).parent.resolve()


FR3_JOINT_LIMITS_LOWER = np.array(
    [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159], dtype=float
)
FR3_JOINT_LIMITS_UPPER = np.array(
    [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159], dtype=float
)
FR3_TORQUE_LIMIT = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0], dtype=float)


def _as_array(value, size: int) -> np.ndarray:
    arr = np.asarray(value, dtype=float)
    if arr.shape == ():
        arr = np.full(size, float(arr), dtype=float)
    if arr.shape != (size,):
        raise ValueError(f"Expected shape ({size},), got {arr.shape}")
    return arr.copy()


def _normalize_quat(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=float).copy()
    norm = np.linalg.norm(quat)
    if norm < 1e-12:
        raise ValueError("Quaternion norm is too small")
    return quat / norm


def _orthonormalize_rotation(matrix: np.ndarray) -> np.ndarray:
    return R.from_matrix(np.asarray(matrix, dtype=float)).as_matrix()


def _pose_copy(pose: np.ndarray) -> np.ndarray:
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (4, 4):
        raise ValueError(f"Expected pose shape (4, 4), got {pose.shape}")
    out = pose.copy()
    out[:3, :3] = _orthonormalize_rotation(out[:3, :3])
    return out


def _critical_damping(stiffness: np.ndarray) -> np.ndarray:
    return 2.0 * np.sqrt(np.maximum(stiffness, 0.0))


def _saturate_torque_rate(tau_desired: np.ndarray, tau_reference: np.ndarray, max_delta_tau: np.ndarray) -> np.ndarray:
    return tau_reference + np.clip(tau_desired - tau_reference, -max_delta_tau, max_delta_tau)


def _compute_joint_limit_torque(
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


def _pseudo_inverse(matrix: np.ndarray, tolerance: float = 1e-6) -> np.ndarray:
    u, s, vh = np.linalg.svd(matrix, full_matrices=False)
    s_inv = np.zeros_like(s)
    mask = s > tolerance
    s_inv[mask] = 1.0 / s[mask]
    return vh.T @ np.diag(s_inv) @ u.T


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
            nullspace_target=None if self.nullspace_target is None else self.nullspace_target.copy(),
        )


@dataclass
class JointImpedanceGains:
    stiffness: np.ndarray = field(default_factory=lambda: np.ones(7, dtype=float) * 80.0)
    damping: np.ndarray = field(default_factory=lambda: np.ones(7, dtype=float) * 4.0)

    def copy(self) -> "JointImpedanceGains":
        return JointImpedanceGains(self.stiffness.copy(), self.damping.copy())


@dataclass
class CartesianImpedanceGains:
    stiffness: np.ndarray = field(default_factory=lambda: np.ones(6, dtype=float) * 100.0)
    damping: np.ndarray = field(default_factory=lambda: np.ones(6, dtype=float) * 4.0)
    nullspace_stiffness: float = 0.0

    def copy(self) -> "CartesianImpedanceGains":
        return CartesianImpedanceGains(
            self.stiffness.copy(),
            self.damping.copy(),
            float(self.nullspace_stiffness),
        )


class FrankaController:
    """
    High-level Franka controller with an async command API and a dedicated
    background real-time thread for torque control.

    The async methods only publish new references or wait for motion timing.
    The 1kHz control loop runs in a dedicated Python thread and keeps one
    impedance controller alive, similar to franky's tracking motions.
    """

    def __init__(self, robot: RobotInterface):
        self.robot = robot

        self.state_lock = threading.Lock()
        self._control_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()
        self._loop_exception: Optional[BaseException] = None

        self.type = "impedance"
        self.running = False
        self.clip = True
        self.track = False
        self.verbose = False

        self.compensate_coriolis = True
        self.joint_limit_repulsion_active = True
        self.max_delta_tau = np.ones(7, dtype=float)
        self.torque_limit = FR3_TORQUE_LIMIT.copy()
        self.joint_limit_activation_distance = 0.1
        self.joint_limit_stiffness = 4.0
        self.joint_limit_damping = 1.0
        self.joint_limit_max_torque = 5.0
        self.lower_joint_limits = FR3_JOINT_LIMITS_LOWER.copy()
        self.upper_joint_limits = FR3_JOINT_LIMITS_UPPER.copy()
        self.cartesian_position_clip = np.array([0.10, 0.10, 0.10], dtype=float)
        self.cartesian_rotation_clip = np.array([0.25, 0.25, 0.25], dtype=float)
        self.gains_time_constant = 0.1
        self.torque = np.zeros(7, dtype=float)
        self.error_integral = np.zeros(7, dtype=float)
        self.integral_limit = 10.0

        self._publish_freq = 50.0
        self._publish_dt = 1.0 / self._publish_freq
        self._publish_next_deadline = {}

        self.state = None

        self._joint_reference = JointReference()
        self._cartesian_reference = CartesianReference()

        self.joint_gains = JointImpedanceGains()
        self._joint_gain_target = self.joint_gains.copy()
        self.cartesian_gains = CartesianImpedanceGains()
        self._cartesian_gain_target = self.cartesian_gains.copy()

        self.initialize()
        self._sync_public_aliases()

    def _sync_public_aliases(self):
        self.kp = self.joint_gains.stiffness.copy()
        self.kd = self.joint_gains.damping.copy()
        self.ee_kp = self.cartesian_gains.stiffness.copy()
        self.ee_kd = self.cartesian_gains.damping.copy()
        null_damping = _critical_damping(np.array([self.cartesian_gains.nullspace_stiffness], dtype=float))[0]
        self.null_kp = np.ones(7, dtype=float) * self.cartesian_gains.nullspace_stiffness
        self.null_kd = np.ones(7, dtype=float) * null_damping
        self.q_desired = self._joint_reference.q.copy()
        self.ee_desired = self._cartesian_reference.pose.copy()

    def initialize(self):
        initial_state = self.robot.state
        self.initial_ee = initial_state["ee"].copy()
        self.initial_qpos = initial_state["qpos"].copy()
        self.initial_qvel = initial_state["qvel"].copy()
        self.last_torque = initial_state["last_torque"].copy()
        self.state = initial_state

        self._joint_reference = JointReference(
            q=self.initial_qpos.copy(),
            dq=np.zeros(7, dtype=float),
            tau_ff=np.zeros(7, dtype=float),
        )
        self._cartesian_reference = CartesianReference(
            pose=self.initial_ee.copy(),
            twist=np.zeros(6, dtype=float),
            nullspace_target=self.initial_qpos.copy(),
        )

    async def test_connection(self):
        self.track = True
        await asyncio.sleep(5.0)
        self.track = False

    def set_freq(self, freq: float):
        if freq <= 0:
            raise ValueError("freq must be positive")
        self._publish_freq = float(freq)
        self._publish_dt = 1.0 / self._publish_freq
        self._publish_next_deadline.clear()

    async def _rate_limit_publish(self, key: str, dt: Optional[float] = None):
        now = time.perf_counter()
        dt = self._publish_dt if dt is None else float(dt)
        deadline = self._publish_next_deadline.get(key)
        if deadline is None:
            self._publish_next_deadline[key] = now + dt
            return
        if deadline > now:
            await asyncio.sleep(deadline - now)
            now = time.perf_counter()
        next_deadline = max(deadline + dt, now)
        self._publish_next_deadline[key] = next_deadline

    def _assert_loop_ok(self):
        if self._loop_exception is not None:
            raise RuntimeError("Control loop terminated unexpectedly") from self._loop_exception

    def _exp_smooth(self, current: np.ndarray, target: np.ndarray, dt: float) -> np.ndarray:
        if self.gains_time_constant <= 0.0:
            return target.copy()
        alpha = 1.0 - np.exp(-dt / self.gains_time_constant)
        return current + alpha * (target - current)

    def _compute_nullspace_torque(self, jacobian: np.ndarray, q: np.ndarray, dq: np.ndarray, target: np.ndarray) -> np.ndarray:
        stiffness = max(self.cartesian_gains.nullspace_stiffness, 0.0)
        if stiffness <= 0.0 or target is None:
            return np.zeros(7, dtype=float)
        damping = 2.0 * np.sqrt(stiffness)
        jacobian_transpose_pinv = _pseudo_inverse(jacobian.T)
        nullspace_projector = np.eye(7) - jacobian.T @ jacobian_transpose_pinv
        return nullspace_projector @ (stiffness * (target - q) - damping * dq)

    def _compute_joint_limit_torque(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        if not self.joint_limit_repulsion_active:
            return np.zeros(7, dtype=float)
        return _compute_joint_limit_torque(
            q=q,
            dq=dq,
            lower_limits=self.lower_joint_limits,
            upper_limits=self.upper_joint_limits,
            activation_distance=self.joint_limit_activation_distance,
            stiffness=self.joint_limit_stiffness,
            damping=self.joint_limit_damping,
            max_torque=self.joint_limit_max_torque,
        )

    def _refresh_gain_targets_from_aliases(self):
        self._joint_gain_target = JointImpedanceGains(
            stiffness=_as_array(self.kp, 7),
            damping=_as_array(self.kd, 7),
        )
        self._cartesian_gain_target = CartesianImpedanceGains(
            stiffness=_as_array(self.ee_kp, 6),
            damping=_as_array(self.ee_kd, 6),
            nullspace_stiffness=float(np.mean(_as_array(self.null_kp, 7))),
        )

    def set_joint_gains(self, stiffness, damping: Optional[np.ndarray] = None, damping_ratio: float = 1.0):
        stiffness = _as_array(stiffness, 7)
        if damping is None:
            damping = _critical_damping(stiffness) * float(damping_ratio)
        else:
            damping = _as_array(damping, 7)
        with self.state_lock:
            self._joint_gain_target = JointImpedanceGains(stiffness=stiffness, damping=damping)
            self.kp = stiffness.copy()
            self.kd = damping.copy()

    def set_cartesian_gains(
        self,
        stiffness,
        damping: Optional[np.ndarray] = None,
        damping_ratio: float = 1.0,
        nullspace_stiffness: Optional[float] = None,
    ):
        stiffness = _as_array(stiffness, 6)
        if damping is None:
            damping = _critical_damping(stiffness) * float(damping_ratio)
        else:
            damping = _as_array(damping, 6)
        if nullspace_stiffness is None:
            nullspace_stiffness = self._cartesian_gain_target.nullspace_stiffness
        with self.state_lock:
            self._cartesian_gain_target = CartesianImpedanceGains(
                stiffness=stiffness,
                damping=damping,
                nullspace_stiffness=float(nullspace_stiffness),
            )
            self.ee_kp = stiffness.copy()
            self.ee_kd = damping.copy()
            self.null_kp = np.ones(7, dtype=float) * float(nullspace_stiffness)
            self.null_kd = np.ones(7, dtype=float) * 2.0 * np.sqrt(max(float(nullspace_stiffness), 0.0))

    def _set_joint_reference(self, q, dq=None, tau_ff=None):
        q = _as_array(q, 7)
        if dq is None:
            dq = np.zeros(7, dtype=float)
        else:
            dq = _as_array(dq, 7)
        if tau_ff is None:
            tau_ff = np.zeros(7, dtype=float)
        else:
            tau_ff = _as_array(tau_ff, 7)
        with self.state_lock:
            self._joint_reference = JointReference(q=q, dq=dq, tau_ff=tau_ff)
            self.q_desired = q.copy()

    def _set_cartesian_reference(self, pose, twist=None, nullspace_target=None):
        pose = _pose_copy(pose)
        if twist is None:
            twist = np.zeros(6, dtype=float)
        else:
            twist = _as_array(twist, 6)
        if nullspace_target is not None:
            nullspace_target = _as_array(nullspace_target, 7)
        with self.state_lock:
            if nullspace_target is None and self._cartesian_reference.nullspace_target is not None:
                nullspace_target = self._cartesian_reference.nullspace_target.copy()
            self._cartesian_reference = CartesianReference(
                pose=pose,
                twist=twist,
                nullspace_target=nullspace_target,
            )
            self.ee_desired = pose.copy()

    async def set(self, attr: str, value):
        self._assert_loop_ok()
        await self._rate_limit_publish(attr)

        if attr == "q_desired":
            self._set_joint_reference(value)
            return
        if attr == "ee_desired":
            self._set_cartesian_reference(value)
            return
        if attr == "torque":
            value = _as_array(value, 7)
            with self.state_lock:
                self.torque = value.copy()
            return
        setattr(self, attr, value)

    async def set_joint_reference(self, q, dq=None, tau_ff=None):
        self._assert_loop_ok()
        await self._rate_limit_publish("joint_reference")
        self._set_joint_reference(q=q, dq=dq, tau_ff=tau_ff)

    async def set_cartesian_reference(self, pose, twist=None, nullspace_target=None):
        self._assert_loop_ok()
        await self._rate_limit_publish("cartesian_reference")
        self._set_cartesian_reference(pose=pose, twist=twist, nullspace_target=nullspace_target)

    def _loop(self):
        loop_times = []
        last_time = time.perf_counter()
        iteration = 0

        try:
            while not self._stop_event.is_set():
                state = self.robot.read_control_state()

                with self.state_lock:
                    self.state = state
                    self.last_torque = state["last_torque"].copy()
                    self._refresh_gain_targets_from_aliases()
                    dt = max(float(state.get("dt", 1e-3)), 1e-6)
                    self.joint_gains.stiffness = self._exp_smooth(
                        self.joint_gains.stiffness, self._joint_gain_target.stiffness, dt
                    )
                    self.joint_gains.damping = self._exp_smooth(
                        self.joint_gains.damping, self._joint_gain_target.damping, dt
                    )
                    self.cartesian_gains.stiffness = self._exp_smooth(
                        self.cartesian_gains.stiffness, self._cartesian_gain_target.stiffness, dt
                    )
                    self.cartesian_gains.damping = self._exp_smooth(
                        self.cartesian_gains.damping, self._cartesian_gain_target.damping, dt
                    )
                    if self.gains_time_constant <= 0.0:
                        self.cartesian_gains.nullspace_stiffness = self._cartesian_gain_target.nullspace_stiffness
                    else:
                        alpha = 1.0 - np.exp(-dt / self.gains_time_constant)
                        self.cartesian_gains.nullspace_stiffness += (
                            alpha * (self._cartesian_gain_target.nullspace_stiffness - self.cartesian_gains.nullspace_stiffness)
                        )
                    controller_type = self.type
                    joint_ref = self._joint_reference.copy()
                    cart_ref = self._cartesian_reference.copy()
                    direct_torque = self.torque.copy()

                tau_command = self._compute_command(
                    controller_type=controller_type,
                    state=state,
                    joint_ref=joint_ref,
                    cart_ref=cart_ref,
                    direct_torque=direct_torque,
                )
                self.robot.step(tau_command)

                if not self._ready_event.is_set():
                    self._ready_event.set()

                if self.track:
                    current_time = time.perf_counter()
                    loop_times.append(current_time - last_time)
                    last_time = current_time
                    iteration += 1
                    if iteration % 1000 == 0 and loop_times:
                        loop_times_array = np.asarray(loop_times)
                        mean_dt = np.mean(loop_times_array) * 1000.0
                        std_dt = np.std(loop_times_array) * 1000.0
                        min_dt = np.min(loop_times_array) * 1000.0
                        max_dt = np.max(loop_times_array) * 1000.0
                        actual_freq = 1.0 / np.mean(loop_times_array)
                        print(f"Control loop stats (last {len(loop_times)} iterations):")
                        print(f"  Frequency: {actual_freq:.1f} Hz (target: 1000 Hz)")
                        print(f"  Mean dt: {mean_dt:.3f} ms, Std: {std_dt:.3f} ms")
                        print(f"  Min dt: {min_dt:.3f} ms, Max dt: {max_dt:.3f} ms")
                        print(f"  Jitter (max-min): {max_dt - min_dt:.3f} ms")
                        loop_times.clear()
        except BaseException as exc:
            self._loop_exception = exc
        finally:
            self.running = False
            self._stop_event.set()
            self._ready_event.set()

    def _compute_command(
        self,
        controller_type: str,
        state: dict,
        joint_ref: JointReference,
        cart_ref: CartesianReference,
        direct_torque: np.ndarray,
    ) -> np.ndarray:
        if controller_type == "torque":
            tau_d = direct_torque
        elif controller_type == "pid":
            tau_d = self._pid_step(state, joint_ref)
        elif controller_type == "osc":
            tau_d = self._osc_step(state, cart_ref)
        else:
            tau_d = self._impedance_step(state, joint_ref)

        if self.clip:
            tau_d = _saturate_torque_rate(tau_d, state["last_torque"], self.max_delta_tau)
        tau_d = np.clip(tau_d, -self.torque_limit, self.torque_limit)
        self.torque = tau_d.copy()
        return tau_d

    async def start(self):
        self._assert_loop_ok()
        if self.running:
            return self._thread

        self._stop_event.clear()
        self._ready_event.clear()
        self._loop_exception = None

        self.robot.start()
        self.running = True
        self._thread = threading.Thread(target=self._loop, name="xense-franka-control", daemon=True)
        self._thread.start()

        start_deadline = time.time() + 2.0
        while time.time() < start_deadline:
            if self._ready_event.wait(timeout=0.05):
                self._assert_loop_ok()
                return self._thread
            await asyncio.sleep(0)

        self._assert_loop_ok()
        raise TimeoutError("Timed out waiting for control loop to start")

    async def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self.running = False
        self.robot.stop()
        await asyncio.sleep(0)

    def switch(self, controller_type: str):
        if controller_type not in {"impedance", "pid", "osc", "torque"}:
            raise ValueError(f"Unknown controller type: {controller_type}")

        with self.state_lock:
            self.type = controller_type
            self.initialize()
            self.error_integral = np.zeros(7, dtype=float)
            self._publish_next_deadline.clear()
            self._sync_public_aliases()

        if self.verbose:
            print("==================================")
            print(f"Switched to {controller_type} controller.")
            print("==================================")

    def _pid_step(self, state: dict, joint_ref: JointReference) -> np.ndarray:
        q = state["qpos"]
        dq = state["qvel"]
        coriolis = state["coriolis"] if self.compensate_coriolis else np.zeros(7, dtype=float)
        position_error = q - joint_ref.q
        dt = max(float(state.get("dt", 1e-3)), 1e-6)
        self.error_integral += (-position_error) * dt
        self.error_integral = np.clip(self.error_integral, -self.integral_limit, self.integral_limit)
        tau_task = (
            -self.joint_gains.stiffness * position_error
            + joint_ref.tau_ff
            - self.joint_gains.damping * (dq - joint_ref.dq)
        )
        tau_task += 0.1 * self.error_integral
        tau_d = tau_task + coriolis + self._compute_joint_limit_torque(q, dq)
        return tau_d

    def _impedance_step(self, state: dict, joint_ref: JointReference) -> np.ndarray:
        q = state["qpos"]
        dq = state["qvel"]
        coriolis = state["coriolis"] if self.compensate_coriolis else np.zeros(7, dtype=float)
        tau_task = (
            self.joint_gains.stiffness * (joint_ref.q - q)
            + self.joint_gains.damping * (joint_ref.dq - dq)
            + joint_ref.tau_ff
        )
        tau_d = tau_task + coriolis + self._compute_joint_limit_torque(q, dq)
        return tau_d

    def _osc_step(self, state: dict, cart_ref: CartesianReference) -> np.ndarray:
        jac = state["jac"]
        ee = state["ee"]
        q = state["qpos"]
        dq = state["qvel"]
        coriolis = state["coriolis"] if self.compensate_coriolis else np.zeros(7, dtype=float)

        position = ee[:3, 3]
        orientation = R.from_matrix(ee[:3, :3])
        target_position = cart_ref.pose[:3, 3]
        target_orientation = R.from_matrix(cart_ref.pose[:3, :3])

        error = np.zeros(6, dtype=float)
        error[:3] = np.clip(position - target_position, -self.cartesian_position_clip, self.cartesian_position_clip)

        orientation_quat = orientation.as_quat()
        target_quat = target_orientation.as_quat()
        if np.dot(target_quat, orientation_quat) < 0.0:
            orientation_quat = -orientation_quat

        orientation_corrected = R.from_quat(orientation_quat)
        error_quaternion = orientation_corrected.inv() * target_orientation
        error[3:] = error_quaternion.as_quat()[:3]
        error[3:] = -ee[:3, :3] @ error[3:]
        error[3:] = np.clip(error[3:], -self.cartesian_rotation_clip, self.cartesian_rotation_clip)

        measured_twist = jac @ dq
        wrench = -np.diag(self.cartesian_gains.stiffness) @ error
        wrench -= np.diag(self.cartesian_gains.damping) @ (measured_twist - cart_ref.twist)

        tau_task = jac.T @ wrench
        tau_nullspace = self._compute_nullspace_torque(jac, q, dq, cart_ref.nullspace_target)
        tau_limit = self._compute_joint_limit_torque(q, dq)
        return tau_task + tau_nullspace + tau_limit + coriolis

    async def move(
        self,
        qpos=None,
        vel=np.ones(7, dtype=float) * 0.1,
        acc=np.ones(7, dtype=float) * 0.5,
    ):
        self._assert_loop_ok()
        self.type = "impedance"

        if qpos is None:
            qpos = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, 0.7853], dtype=float)

        inp = InputParameter(7)
        current_state = self.state if self.state is not None else self.robot.state
        inp.current_position = current_state["qpos"]
        inp.current_velocity = current_state["qvel"]
        inp.current_acceleration = np.zeros(7, dtype=float)

        inp.target_position = _as_array(qpos, 7)
        inp.target_velocity = np.zeros(7, dtype=float)
        inp.target_acceleration = np.zeros(7, dtype=float)

        inp.max_velocity = _as_array(vel, 7)
        inp.max_acceleration = _as_array(acc, 7)
        inp.max_jerk = np.ones(7, dtype=float)

        otg = Ruckig(7)
        trajectory = Trajectory(7)
        result = otg.calculate(inp, trajectory)
        if result not in {Result.Working, Result.Finished}:
            raise RuntimeError(f"Ruckig trajectory generation failed: {result}")

        sample_hz = max(self._publish_freq, 50.0)
        sample_dt = 1.0 / sample_hz
        steps = max(int(np.ceil(trajectory.duration / sample_dt)), 1)

        for step in range(steps + 1):
            t = min(step * sample_dt, trajectory.duration)
            q_ref, dq_ref, _ = trajectory.at_time(t)
            await self._rate_limit_publish("move_joint_reference", dt=sample_dt)
            self._set_joint_reference(q=q_ref, dq=dq_ref)

        self._set_joint_reference(q=inp.target_position, dq=np.zeros(7, dtype=float))

    def get_current_ee_pose(self) -> np.ndarray:
        if self.state is None:
            return self.robot.state["ee"].copy()
        return self.state["ee"].copy()

    def get_current_joint_positions(self) -> np.ndarray:
        if self.state is None:
            return self.robot.state["qpos"].copy()
        return self.state["qpos"].copy()

    def get_current_joint_velocities(self) -> np.ndarray:
        if self.state is None:
            return self.robot.state["qvel"].copy()
        return self.state["qvel"].copy()
