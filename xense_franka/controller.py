import asyncio
import threading
import time
from typing import Optional

import numpy as np
from ruckig import InputParameter, Result, Ruckig, Trajectory
from scipy.spatial.transform import Rotation as R

from xense_franka.constants import (
    FR3_JOINT_LIMITS_LOWER,
    FR3_JOINT_LIMITS_UPPER,
    FR3_TORQUE_LIMIT,
)
from xense_franka.handles import (
    CartesianGainsHandle,
    CartesianReferenceHandle,
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
from xense_franka.torque_utils import (
    as_array,
    compute_joint_limit_torque,
    critical_damping,
    pose_copy,
    pseudo_inverse,
    saturate_torque_rate,
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

        # state_lock is kept as a public attribute for backward compatibility
        # (e.g. ``with controller.state_lock:``), but the 1kHz loop no longer
        # acquires it — all hot-path data goes through lock-free handles.
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

        # Lock-free handles for hot-path data
        self._joint_ref_handle = JointReferenceHandle()
        self._cart_ref_handle = CartesianReferenceHandle()
        self._joint_gains_handle = JointGainsHandle()
        self._cart_gains_handle = CartesianGainsHandle()

        # Smoothed gains used by the control thread only
        self._active_joint_gains = JointImpedanceGains()
        self._active_cart_gains = CartesianImpedanceGains()

        # Pending type transition consumed by the control loop.  Writing a
        # string here (GIL-atomic) tells the loop to change ``self.type`` and
        # reset the error integral on the *next* tick, **after** the caller has
        # already written fresh references to the handles.
        self._pending_transition: Optional[str] = None

        self.initialize()

    @property
    def effective_type(self) -> str:
        """The controller type that will be active on the next control tick.

        If a transition is pending (posted by ``switch()`` but not yet
        consumed by the control loop), this returns the *target* type.
        Otherwise it returns the current ``self.type``.
        """
        pending = self._pending_transition
        return pending if pending is not None else self.type

    # ------------------------------------------------------------------
    # Backward-compatible property accessors
    # ------------------------------------------------------------------

    @property
    def kp(self) -> np.ndarray:
        return self._joint_gains_handle.get().stiffness.copy()

    @kp.setter
    def kp(self, value):
        gains = self._joint_gains_handle.get().copy()
        gains.stiffness = as_array(value, 7)
        self._joint_gains_handle.set(gains)

    @property
    def kd(self) -> np.ndarray:
        return self._joint_gains_handle.get().damping.copy()

    @kd.setter
    def kd(self, value):
        gains = self._joint_gains_handle.get().copy()
        gains.damping = as_array(value, 7)
        self._joint_gains_handle.set(gains)

    @property
    def ee_kp(self) -> np.ndarray:
        return self._cart_gains_handle.get().stiffness.copy()

    @ee_kp.setter
    def ee_kp(self, value):
        gains = self._cart_gains_handle.get().copy()
        gains.stiffness = as_array(value, 6)
        self._cart_gains_handle.set(gains)

    @property
    def ee_kd(self) -> np.ndarray:
        return self._cart_gains_handle.get().damping.copy()

    @ee_kd.setter
    def ee_kd(self, value):
        gains = self._cart_gains_handle.get().copy()
        gains.damping = as_array(value, 6)
        self._cart_gains_handle.set(gains)

    @property
    def null_kp(self) -> np.ndarray:
        ns = self._cart_gains_handle.get().nullspace_stiffness
        return np.ones(7, dtype=float) * ns

    @null_kp.setter
    def null_kp(self, value):
        gains = self._cart_gains_handle.get().copy()
        gains.nullspace_stiffness = float(np.mean(as_array(value, 7)))
        self._cart_gains_handle.set(gains)

    @property
    def null_kd(self) -> np.ndarray:
        ns = self._cart_gains_handle.get().nullspace_stiffness
        return np.ones(7, dtype=float) * critical_damping(np.array([ns]))[0]

    @null_kd.setter
    def null_kd(self, _value):
        # Nullspace damping is always derived from nullspace stiffness.
        # Setter kept for backward compatibility but is a no-op.
        pass

    @property
    def q_desired(self) -> np.ndarray:
        return self._joint_ref_handle.get().q.copy()

    @q_desired.setter
    def q_desired(self, value):
        ref = self._joint_ref_handle.get().copy()
        ref.q = as_array(value, 7)
        self._joint_ref_handle.set(ref)

    @property
    def ee_desired(self) -> np.ndarray:
        return self._cart_ref_handle.get().pose.copy()

    @ee_desired.setter
    def ee_desired(self, value):
        ref = self._cart_ref_handle.get().copy()
        ref.pose = pose_copy(value)
        self._cart_ref_handle.set(ref)

    # Expose active (smoothed) gains as read-only for introspection
    @property
    def joint_gains(self) -> JointImpedanceGains:
        return self._active_joint_gains

    @property
    def cartesian_gains(self) -> CartesianImpedanceGains:
        return self._active_cart_gains

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def initialize(self):
        initial_state = self.robot.state
        self.initial_ee = initial_state["ee"].copy()
        self.initial_qpos = initial_state["qpos"].copy()
        self.initial_qvel = initial_state["qvel"].copy()
        self.last_torque = initial_state["last_torque"].copy()
        self.state = initial_state

        joint_ref = JointReference(
            q=self.initial_qpos.copy(),
            dq=np.zeros(7, dtype=float),
            tau_ff=np.zeros(7, dtype=float),
        )
        cart_ref = CartesianReference(
            pose=self.initial_ee.copy(),
            twist=np.zeros(6, dtype=float),
            nullspace_target=self.initial_qpos.copy(),
        )
        self._joint_ref_handle.set(joint_ref)
        self._cart_ref_handle.set(cart_ref)

    def _request_type_change(self, controller_type: str):
        """Post an atomic type transition for the control loop.

        Fresh references (based on the latest robot state) are written to the
        handles *before* the transition flag is set, so the control loop always
        sees a consistent ``(type, reference)`` pair.
        """
        state = self.state if self.state is not None else self.robot.state
        self._joint_ref_handle.set(JointReference(
            q=state["qpos"].copy(),
            dq=np.zeros(7, dtype=float),
            tau_ff=np.zeros(7, dtype=float),
        ))
        self._cart_ref_handle.set(CartesianReference(
            pose=state["ee"].copy(),
            twist=np.zeros(6, dtype=float),
            nullspace_target=state["qpos"].copy(),
        ))
        # GIL-atomic: the control loop will see the new type only after the
        # handles above have been updated.
        self._pending_transition = controller_type

    # ------------------------------------------------------------------
    # Public gain / reference setters
    # ------------------------------------------------------------------

    def set_joint_gains(self, stiffness, damping: Optional[np.ndarray] = None, damping_ratio: float = 1.0):
        stiffness = as_array(stiffness, 7)
        if damping is None:
            damping = critical_damping(stiffness) * float(damping_ratio)
        else:
            damping = as_array(damping, 7)
        self._joint_gains_handle.set(JointImpedanceGains(stiffness=stiffness, damping=damping))

    def set_cartesian_gains(
        self,
        stiffness,
        damping: Optional[np.ndarray] = None,
        damping_ratio: float = 1.0,
        nullspace_stiffness: Optional[float] = None,
    ):
        stiffness = as_array(stiffness, 6)
        if damping is None:
            damping = critical_damping(stiffness) * float(damping_ratio)
        else:
            damping = as_array(damping, 6)
        if nullspace_stiffness is None:
            nullspace_stiffness = self._cart_gains_handle.get().nullspace_stiffness
        self._cart_gains_handle.set(
            CartesianImpedanceGains(
                stiffness=stiffness,
                damping=damping,
                nullspace_stiffness=float(nullspace_stiffness),
            )
        )

    def _set_joint_reference(self, q, dq=None, tau_ff=None):
        q = as_array(q, 7)
        if dq is None:
            dq = np.zeros(7, dtype=float)
        else:
            dq = as_array(dq, 7)
        if tau_ff is None:
            tau_ff = np.zeros(7, dtype=float)
        else:
            tau_ff = as_array(tau_ff, 7)
        self._joint_ref_handle.set(JointReference(q=q, dq=dq, tau_ff=tau_ff))

    def _set_cartesian_reference(self, pose, twist=None, nullspace_target=None):
        pose = pose_copy(pose)
        if twist is None:
            twist = np.zeros(6, dtype=float)
        else:
            twist = as_array(twist, 6)
        if nullspace_target is not None:
            nullspace_target = as_array(nullspace_target, 7)
        else:
            existing = self._cart_ref_handle.get().nullspace_target
            if existing is not None:
                nullspace_target = existing.copy()
        self._cart_ref_handle.set(
            CartesianReference(pose=pose, twist=twist, nullspace_target=nullspace_target)
        )

    # ------------------------------------------------------------------
    # Async command API
    # ------------------------------------------------------------------

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
            value = as_array(value, 7)
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

    # ------------------------------------------------------------------
    # 1kHz control loop (lock-free)
    # ------------------------------------------------------------------

    def _exp_smooth(self, current: np.ndarray, target: np.ndarray, dt: float) -> np.ndarray:
        if self.gains_time_constant <= 0.0:
            return target.copy()
        alpha = 1.0 - np.exp(-dt / self.gains_time_constant)
        return current + alpha * (target - current)

    def _loop(self):
        loop_times = []
        last_time = time.perf_counter()
        iteration = 0

        try:
            while not self._stop_event.is_set():
                state = self.robot.read_control_state()

                # Update cached state (still behind state_lock for external readers)
                with self.state_lock:
                    self.state = state
                    self.last_torque = state["last_torque"].copy()

                # Consume pending type transition (atomic switch).
                # The caller has already written fresh references to the
                # handles, so by the time we read them below the pair
                # (type, reference) is consistent.
                pending = self._pending_transition
                if pending is not None:
                    self._pending_transition = None
                    self.type = pending
                    self.error_integral = np.zeros(7, dtype=float)

                dt = max(float(state.get("dt", 1e-3)), 1e-6)

                # Read targets from lock-free handles
                joint_gain_target = self._joint_gains_handle.get()
                cart_gain_target = self._cart_gains_handle.get()

                # Exponentially smooth gains (only control thread writes _active_*)
                self._active_joint_gains.stiffness = self._exp_smooth(
                    self._active_joint_gains.stiffness, joint_gain_target.stiffness, dt
                )
                self._active_joint_gains.damping = self._exp_smooth(
                    self._active_joint_gains.damping, joint_gain_target.damping, dt
                )
                self._active_cart_gains.stiffness = self._exp_smooth(
                    self._active_cart_gains.stiffness, cart_gain_target.stiffness, dt
                )
                self._active_cart_gains.damping = self._exp_smooth(
                    self._active_cart_gains.damping, cart_gain_target.damping, dt
                )
                if self.gains_time_constant <= 0.0:
                    self._active_cart_gains.nullspace_stiffness = cart_gain_target.nullspace_stiffness
                else:
                    alpha = 1.0 - np.exp(-dt / self.gains_time_constant)
                    self._active_cart_gains.nullspace_stiffness += alpha * (
                        cart_gain_target.nullspace_stiffness - self._active_cart_gains.nullspace_stiffness
                    )

                controller_type = self.type
                joint_ref = self._joint_ref_handle.get().copy()
                cart_ref = self._cart_ref_handle.get().copy()
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

    def _compute_nullspace_torque(self, jacobian: np.ndarray, q: np.ndarray, dq: np.ndarray, target: np.ndarray) -> np.ndarray:
        stiffness = max(self._active_cart_gains.nullspace_stiffness, 0.0)
        if stiffness <= 0.0 or target is None:
            return np.zeros(7, dtype=float)
        damping = 2.0 * np.sqrt(stiffness)
        jacobian_transpose_pinv = pseudo_inverse(jacobian.T)
        nullspace_projector = np.eye(7) - jacobian.T @ jacobian_transpose_pinv
        return nullspace_projector @ (stiffness * (target - q) - damping * dq)

    def _compute_joint_limit_torque(self, q: np.ndarray, dq: np.ndarray) -> np.ndarray:
        if not self.joint_limit_repulsion_active:
            return np.zeros(7, dtype=float)
        return compute_joint_limit_torque(
            q=q,
            dq=dq,
            lower_limits=self.lower_joint_limits,
            upper_limits=self.upper_joint_limits,
            activation_distance=self.joint_limit_activation_distance,
            stiffness=self.joint_limit_stiffness,
            damping=self.joint_limit_damping,
            max_torque=self.joint_limit_max_torque,
        )

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
            tau_d = saturate_torque_rate(tau_d, state["last_torque"], self.max_delta_tau)
        tau_d = np.clip(tau_d, -self.torque_limit, self.torque_limit)
        self.torque = tau_d.copy()
        return tau_d

    # ------------------------------------------------------------------
    # Controller steps
    # ------------------------------------------------------------------

    def _pid_step(self, state: dict, joint_ref: JointReference) -> np.ndarray:
        q = state["qpos"]
        dq = state["qvel"]
        coriolis = state["coriolis"] if self.compensate_coriolis else np.zeros(7, dtype=float)
        position_error = q - joint_ref.q
        dt = max(float(state.get("dt", 1e-3)), 1e-6)
        self.error_integral += (-position_error) * dt
        self.error_integral = np.clip(self.error_integral, -self.integral_limit, self.integral_limit)
        tau_task = (
            -self._active_joint_gains.stiffness * position_error
            + joint_ref.tau_ff
            - self._active_joint_gains.damping * (dq - joint_ref.dq)
        )
        tau_task += 0.1 * self.error_integral
        tau_d = tau_task + coriolis + self._compute_joint_limit_torque(q, dq)
        return tau_d

    def _impedance_step(self, state: dict, joint_ref: JointReference) -> np.ndarray:
        q = state["qpos"]
        dq = state["qvel"]
        coriolis = state["coriolis"] if self.compensate_coriolis else np.zeros(7, dtype=float)
        tau_task = (
            self._active_joint_gains.stiffness * (joint_ref.q - q)
            + self._active_joint_gains.damping * (joint_ref.dq - dq)
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
        wrench = -np.diag(self._active_cart_gains.stiffness) @ error
        wrench -= np.diag(self._active_cart_gains.damping) @ (measured_twist - cart_ref.twist)

        tau_task = jac.T @ wrench
        tau_nullspace = self._compute_nullspace_torque(jac, q, dq, cart_ref.nullspace_target)
        tau_limit = self._compute_joint_limit_torque(q, dq)
        return tau_task + tau_nullspace + tau_limit + coriolis

    # ------------------------------------------------------------------
    # Start / stop / switch
    # ------------------------------------------------------------------

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

        try:
            start_deadline = time.time() + 2.0
            while time.time() < start_deadline:
                if self._ready_event.wait(timeout=0.05):
                    self._assert_loop_ok()
                    return self._thread
                await asyncio.sleep(0)

            self._assert_loop_ok()
            raise TimeoutError("Timed out waiting for control loop to start")
        except BaseException:
            # Roll back: stop the control thread and release the FCI session.
            await self.stop()
            raise

    async def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                # Control thread is likely stuck in FCI I/O holding _io_lock.
                # Calling robot.stop() would deadlock on the same lock, so we
                # can only warn and let the daemon thread die with the process.
                import warnings
                warnings.warn(
                    "xense_franka: control thread did not exit within "
                    "timeout — robot.stop() skipped to avoid deadlock",
                    RuntimeWarning,
                    stacklevel=2,
                )
                self._thread = None
                self.running = False
                await asyncio.sleep(0)
                return
            self._thread = None
        self.running = False
        self.robot.stop()
        await asyncio.sleep(0)

    def switch(self, controller_type: str):
        """Switch controller mode.

        The type change takes effect on the **next** control-loop tick so
        that the new mode always starts with consistent references.
        Reading ``self.type`` immediately after this call may still return
        the previous value for up to one control period (~1 ms).
        """
        if controller_type not in {"impedance", "pid", "osc", "torque"}:
            raise ValueError(f"Unknown controller type: {controller_type}")

        # Update initial_ee / initial_qpos snapshots (user-visible)
        self.initialize()
        # Clear user-thread rate-limit state
        self._publish_next_deadline.clear()
        # Post atomic transition for the control loop
        self._request_type_change(controller_type)

        if self.verbose:
            print("==================================")
            print(f"Switched to {controller_type} controller.")
            print("==================================")

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    async def move(
        self,
        qpos=None,
        vel=np.ones(7, dtype=float) * 0.1,
        acc=np.ones(7, dtype=float) * 0.5,
    ):
        self._assert_loop_ok()
        self._request_type_change("impedance")

        if qpos is None:
            qpos = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, 0.7853], dtype=float)

        inp = InputParameter(7)
        current_state = self.state if self.state is not None else self.robot.state
        inp.current_position = current_state["qpos"]
        inp.current_velocity = current_state["qvel"]
        inp.current_acceleration = np.zeros(7, dtype=float)

        inp.target_position = as_array(qpos, 7)
        inp.target_velocity = np.zeros(7, dtype=float)
        inp.target_acceleration = np.zeros(7, dtype=float)

        inp.max_velocity = as_array(vel, 7)
        inp.max_acceleration = as_array(acc, 7)
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

    # ------------------------------------------------------------------
    # State queries
    # ------------------------------------------------------------------

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
