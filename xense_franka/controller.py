import multiprocessing as _mp
from multiprocessing.shared_memory import SharedMemory as _SharedMemory
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
from xense_franka.trackers import (
    CartesianImpedanceTracker,
    ExponentialImpedanceTracker,
    JointImpedanceTracker,
)


# ---------------------------------------------------------------------------
# Process-mode constants & helpers
# ---------------------------------------------------------------------------
_ST_N = 158  # state floats: qpos7+qvel7+tau7+wrench6+OT16+ee16+jac42+mm49+cor7+dt1
_RF_N = 92   # ref   floats: type1+jref21+cref35+dtorque7+jgains14+cgains13+tc1
_F64  = np.float64
_TMAP = {"impedance": 0., "pid": 1., "osc": 2., "torque": 3.}
_RMAP = {v: k for k, v in _TMAP.items()}

def _pk_state(s, buf, slot):
    b = buf[slot]
    b[0:7]=s["qpos"]; b[7:14]=s["qvel"]; b[14:21]=s["last_torque"]
    b[21:27]=s["ext_wrench"]; b[27:43]=s["O_T_EE"]; b[43:59]=s["ee"].ravel()
    b[59:101]=s["jac"].ravel(); b[101:150]=s["mm"].ravel()
    b[150:157]=s["coriolis"]; b[157]=s.get("dt",1e-3)

def _upk_state(buf, slot):
    b=buf[slot]
    return dict(qpos=b[0:7].copy(),qvel=b[7:14].copy(),last_torque=b[14:21].copy(),
                ext_wrench=b[21:27].copy(),O_T_EE=b[27:43].copy(),
                ee=b[43:59].copy().reshape(4,4),jac=b[59:101].copy().reshape(6,7),
                mm=b[101:150].copy().reshape(7,7),coriolis=b[150:157].copy(),dt=float(b[157]))


def _control_process_worker(
    fci_ip, st_name, rf_name, st_ctr, rf_ctr,
    stop_ev, ready_ev, track,
    comp_cor, jl_active, do_clip,
    max_dtau, tau_lim,
    jl_dist, jl_k, jl_d, jl_max,
    lo_jl, up_jl, c_pos_clip, c_rot_clip,
):
    """1 kHz control loop in a dedicated process (independent GIL)."""
    import os, sys, signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # Main process handles Ctrl+C
    try:
        os.sched_setaffinity(0, {4, 5})
    except OSError: pass
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(80))
    except (OSError, PermissionError): pass

    st_shm = rf_shm = None
    _diag, _DS, it = [], 5, 0
    try:
        from xense_franka.robot import RobotInterface
        from xense_franka.references import (
            JointReference, CartesianReference,
            JointImpedanceGains, CartesianImpedanceGains,
        )
        from xense_franka.torque_utils import (
            compute_joint_limit_torque, saturate_torque_rate, pseudo_inverse,
        )
        from scipy.spatial.transform import Rotation as R

        st_shm = _SharedMemory(name=st_name, create=False)
        rf_shm = _SharedMemory(name=rf_name, create=False)
        sb = np.ndarray((2,_ST_N), _F64, buffer=st_shm.buf)
        rb = np.ndarray((2,_RF_N), _F64, buffer=rf_shm.buf)

        robot = RobotInterface(ip=fci_ip)
        robot.start()

        ajg = JointImpedanceGains(); acg = CartesianImpedanceGains()
        ei = np.zeros(7); last_rc = 0; ctype = "impedance"; gtc = 0.1
        mdt = np.asarray(max_dtau); tlim = np.asarray(tau_lim)
        ljl = np.asarray(lo_jl); ujl = np.asarray(up_jl)
        cpc = np.asarray(c_pos_clip); crc = np.asarray(c_rot_clip)

        # Defaults until first ref arrives
        jref = None; cref = None; dtq = np.zeros(7)
        tjg = JointImpedanceGains(); tcg = CartesianImpedanceGains()

        ready_ev.set()

        while not stop_ev.is_set():
            t0 = time.perf_counter()
            state = robot.read_control_state()
            t1 = time.perf_counter()
            dt = max(float(state.get("dt",1e-3)),1e-6)

            ns = (st_ctr.value + 1) % 2
            _pk_state(state, sb, ns)
            st_ctr.value += 1

            rc = rf_ctr.value
            if rc > last_rc:
                last_rc = rc
                r = rb[(rc-1)%2].copy()
                nt = _RMAP.get(r[0], "impedance")
                if nt != ctype: ctype = nt; ei[:] = 0
                jref = JointReference(q=r[1:8],dq=r[8:15],tau_ff=r[15:22])
                cref = CartesianReference(pose=r[22:38].reshape(4,4),twist=r[38:44],
                                          nullspace_target=r[50:57])
                dtq = r[57:64].copy()
                tjg = JointImpedanceGains(stiffness=r[64:71],damping=r[71:78])
                tcg = CartesianImpedanceGains(stiffness=r[78:84],damping=r[84:90],
                                              nullspace_stiffness=float(r[90]))
                gtc = max(float(r[91]),1e-6)
            elif jref is None:
                jref = JointReference(q=state["qpos"].copy())
                cref = CartesianReference(pose=state["ee"].copy())

            a = 1.0 - np.exp(-dt/gtc) if gtc > 0 else 1.0
            ajg.stiffness += a*(tjg.stiffness-ajg.stiffness)
            ajg.damping   += a*(tjg.damping-ajg.damping)
            acg.stiffness += a*(tcg.stiffness-acg.stiffness)
            acg.damping   += a*(tcg.damping-acg.damping)
            acg.nullspace_stiffness += a*(tcg.nullspace_stiffness-acg.nullspace_stiffness)

            q=state["qpos"]; dq=state["qvel"]
            cor = state["coriolis"] if comp_cor else np.zeros(7)

            def _jlt():
                return compute_joint_limit_torque(q,dq,ljl,ujl,jl_dist,jl_k,jl_d,jl_max) if jl_active else np.zeros(7)

            if ctype == "torque":
                td = dtq.copy()
            elif ctype == "pid":
                pe = q - jref.q; ei += (-pe)*dt; ei = np.clip(ei,-10,10)
                td = -ajg.stiffness*pe + jref.tau_ff - ajg.damping*(dq-jref.dq) + 0.1*ei + cor + _jlt()
            elif ctype == "osc":
                jac=state["jac"]; ee=state["ee"]
                pos=ee[:3,3]; ori=R.from_matrix(ee[:3,:3])
                tp=cref.pose[:3,3]; to=R.from_matrix(cref.pose[:3,:3])
                err=np.zeros(6); err[:3]=np.clip(pos-tp,-cpc,cpc)
                oq=ori.as_quat(); tq=to.as_quat()
                if np.dot(tq,oq)<0: oq=-oq
                eq=(R.from_quat(oq).inv()*to).as_quat()[:3]
                err[3:]=np.clip(-ee[:3,:3]@eq,-crc,crc)
                mt=jac@dq
                w=-np.diag(acg.stiffness)@err-np.diag(acg.damping)@(mt-cref.twist)
                td=jac.T@w
                nk=max(acg.nullspace_stiffness,0.)
                if nk>0 and cref.nullspace_target is not None:
                    nd=2*np.sqrt(nk); jtpi=pseudo_inverse(jac.T)
                    td+=(np.eye(7)-jac.T@jtpi)@(nk*(cref.nullspace_target-q)-nd*dq)
                td+=_jlt()+cor
            else:
                td=ajg.stiffness*(jref.q-q)+ajg.damping*(jref.dq-dq)+jref.tau_ff+cor+_jlt()

            if do_clip: td=saturate_torque_rate(td,state["last_torque"],mdt)
            td=np.clip(td,-tlim,tlim)
            t2=time.perf_counter()
            robot.step(td)
            t3=time.perf_counter()

            _diag.append(dict(iter=it,type=ctype,tau_cmd=td.copy(),
                              tau_prev=state["last_torque"].copy(),
                              delta=(td-state["last_torque"]).copy(),
                              t_read_ms=(t1-t0)*1e3,t_compute_ms=(t2-t1)*1e3,
                              t_step_ms=(t3-t2)*1e3,t_total_ms=(t3-t0)*1e3))
            if len(_diag)>_DS: _diag.pop(0)
            it+=1

        robot.stop()
    except KeyboardInterrupt:
        pass  # Clean shutdown on Ctrl+C
    except BaseException:
        if _diag:
            print(f"\n===== CONTROL LOOP CRASH at iteration {it} =====")
            for d in _diag:
                print(f"  tick {d['iter']:6d} [{d['type']:>10s}] total={d['t_total_ms']:.2f}ms "
                      f"(read={d['t_read_ms']:.2f} compute={d['t_compute_ms']:.2f} step={d['t_step_ms']:.2f})")
                print(f"    tau_prev = {np.array2string(d['tau_prev'],precision=3,suppress_small=True)}")
                print(f"    tau_cmd  = {np.array2string(d['tau_cmd'],precision=3,suppress_small=True)}")
                print(f"    delta    = {np.array2string(d['delta'],precision=3,suppress_small=True)}")
            print("===== END DIAGNOSTIC =====\n")
        import traceback; traceback.print_exc()
        ready_ev.set()
    finally:
        try: robot.stop()
        except: pass
        for s in [st_shm, rf_shm]:
            if s:
                try: s.close()
                except: pass


class FrankaController:
    """
    High-level Franka controller with a synchronous command API and a dedicated
    background real-time thread for torque control.

    The public methods publish new references or wait for motion timing using
    plain ``time.sleep()``.  The 1kHz control loop runs in a dedicated Python
    thread and keeps one impedance controller alive, similar to franky's
    tracking motions.
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
        self.max_delta_tau = np.full(7, 0.5, dtype=float)
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

        self._publish_freq: Optional[float] = None
        self._publish_dt: Optional[float] = None
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

        # Process-mode attrs
        self._process_mode = False
        self._ctrl_process = None
        self._ctrl_stop_ev = None
        self._st_shm = None
        self._rf_shm = None
        self._st_view = None
        self._rf_view = None
        self._st_ctr = None
        self._rf_ctr = None
        self._rf_wc = 0

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
        self._pub_ref()

    @property
    def kd(self) -> np.ndarray:
        return self._joint_gains_handle.get().damping.copy()

    @kd.setter
    def kd(self, value):
        gains = self._joint_gains_handle.get().copy()
        gains.damping = as_array(value, 7)
        self._joint_gains_handle.set(gains)
        self._pub_ref()

    @property
    def ee_kp(self) -> np.ndarray:
        return self._cart_gains_handle.get().stiffness.copy()

    @ee_kp.setter
    def ee_kp(self, value):
        gains = self._cart_gains_handle.get().copy()
        gains.stiffness = as_array(value, 6)
        self._cart_gains_handle.set(gains)
        self._pub_ref()

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
        self._pub_ref()
        # GIL-atomic: the control loop will see the new type only after the
        # handles above have been updated.
        self._pending_transition = controller_type
        self._pub_ref()

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
        self._pub_ref()

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
    # Command API
    # ------------------------------------------------------------------

    def test_connection(self):
        self.track = True
        time.sleep(5.0)
        self.track = False

    def set_freq(self, freq: Optional[float] = None):
        """Set the publish rate limit for ``set`` / ``set_*_reference``.

        Pass a positive number (Hz) to enable rate limiting, or ``None`` / ``0``
        to disable it (commands are sent as fast as the caller can loop).
        """
        if freq is None or freq <= 0:
            self._publish_freq = None
            self._publish_dt = None
        else:
            self._publish_freq = float(freq)
            self._publish_dt = 1.0 / self._publish_freq
        self._publish_next_deadline.clear()

    def _rate_limit_publish(self, key: str, dt: Optional[float] = None):
        if dt is None:
            dt = self._publish_dt
        if dt is None:
            return
        now = time.perf_counter()
        dt = float(dt)
        deadline = self._publish_next_deadline.get(key)
        if deadline is None:
            self._publish_next_deadline[key] = now + dt
            return
        if deadline > now:
            time.sleep(deadline - now)
            now = time.perf_counter()
        next_deadline = max(deadline + dt, now)
        self._publish_next_deadline[key] = next_deadline

    def _assert_loop_ok(self):
        if self._process_mode:
            if self._ctrl_process is not None and not self._ctrl_process.is_alive():
                raise RuntimeError("Control process terminated unexpectedly")
            return
        if self._loop_exception is not None:
            raise RuntimeError("Control loop terminated unexpectedly") from self._loop_exception

    def set(self, attr: str, value):
        self._assert_loop_ok()
        self._rate_limit_publish(attr)

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

    def set_joint_reference(self, q, dq=None, tau_ff=None):
        self._assert_loop_ok()
        self._rate_limit_publish("joint_reference")
        self._set_joint_reference(q=q, dq=dq, tau_ff=tau_ff)
        self._pub_ref()


    def set_cartesian_reference(self, pose, twist=None, nullspace_target=None):
        self._assert_loop_ok()
        self._rate_limit_publish("cartesian_reference")
        self._set_cartesian_reference(pose=pose, twist=twist, nullspace_target=nullspace_target)

    # ------------------------------------------------------------------
    # 1kHz control loop (lock-free)
    # ------------------------------------------------------------------
        self._pub_ref()


    def _exp_smooth(self, current: np.ndarray, target: np.ndarray, dt: float) -> np.ndarray:
        if self.gains_time_constant <= 0.0:
            return target.copy()
        alpha = 1.0 - np.exp(-dt / self.gains_time_constant)
        return current + alpha * (target - current)

    def _pub_ref(self):
        if not self._process_mode or self._rf_view is None:
            return
        from xense_franka.references import CartesianReference
        sl = (self._rf_wc + 1) % 2
        b = self._rf_view[sl]
        b[0] = _TMAP.get(self.effective_type, 0.)
        jr = self._joint_ref_handle.get()
        b[1:8]=jr.q; b[8:15]=jr.dq; b[15:22]=jr.tau_ff
        cr = self._cart_ref_handle.get()
        b[22:38]=cr.pose.ravel(); b[38:44]=cr.twist; b[44:50]=0.
        b[50:57]=cr.nullspace_target if cr.nullspace_target is not None else 0.
        b[57:64]=self.torque
        jg=self._joint_gains_handle.get(); cg=self._cart_gains_handle.get()
        b[64:71]=jg.stiffness; b[71:78]=jg.damping
        b[78:84]=cg.stiffness; b[84:90]=cg.damping
        b[90]=cg.nullspace_stiffness; b[91]=self.gains_time_constant
        self._rf_wc += 1
        self._rf_ctr.value = self._rf_wc

    def _loop(self):
        loop_times = []
        last_time = time.perf_counter()
        iteration = 0
        # Diagnostic ring buffer: last 5 ticks before crash
        _diag_buf = []
        _DIAG_SIZE = 5

        try:
            while not self._stop_event.is_set():
                t_tick_start = time.perf_counter()
                state = self.robot.read_control_state()
                t_after_read = time.perf_counter()

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
                t_after_compute = time.perf_counter()
                self.robot.step(tau_command)
                t_after_step = time.perf_counter()

                # Record diagnostic info
                _diag_buf.append({
                    "iter": iteration,
                    "type": controller_type,
                    "tau_cmd": tau_command.copy(),
                    "tau_prev": state["last_torque"].copy(),
                    "delta": (tau_command - state["last_torque"]).copy(),
                    "t_read_ms": (t_after_read - t_tick_start) * 1000,
                    "t_compute_ms": (t_after_compute - t_after_read) * 1000,
                    "t_step_ms": (t_after_step - t_after_compute) * 1000,
                    "t_total_ms": (t_after_step - t_tick_start) * 1000,
                })
                if len(_diag_buf) > _DIAG_SIZE:
                    _diag_buf.pop(0)

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
                else:
                    iteration += 1
        except BaseException as exc:
            # Print diagnostic ring buffer on crash
            if _diag_buf:
                print(f"\n===== CONTROL LOOP CRASH at iteration {iteration} =====")
                for d in _diag_buf:
                    print(f"  tick {d['iter']:6d} [{d['type']:>10s}] "
                          f"total={d['t_total_ms']:.2f}ms "
                          f"(read={d['t_read_ms']:.2f} compute={d['t_compute_ms']:.2f} step={d['t_step_ms']:.2f})")
                    print(f"    tau_prev = {np.array2string(d['tau_prev'], precision=3, suppress_small=True)}")
                    print(f"    tau_cmd  = {np.array2string(d['tau_cmd'], precision=3, suppress_small=True)}")
                    print(f"    delta    = {np.array2string(d['delta'], precision=3, suppress_small=True)}")
                print("===== END DIAGNOSTIC =====\n")
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

    def start(self, use_process: bool = True):
        self._assert_loop_ok()
        if self.running:
            return self._thread or self._ctrl_process

        self._stop_event.clear()
        self._ready_event.clear()
        self._loop_exception = None

        if use_process:
            return self._start_process()

        self.robot.start()
        self.running = True
        self._thread = threading.Thread(target=self._loop, name="xense-franka-control", daemon=True)
        self._thread.start()
        try:
            if not self._ready_event.wait(timeout=2.0):
                self._assert_loop_ok()
                raise TimeoutError("Timed out waiting for control loop to start")
            self._assert_loop_ok()
            return self._thread
        except BaseException:
            self.stop()
            raise

    def _start_process(self):
        ctx = _mp.get_context("spawn")
        sb = 2*_ST_N*np.dtype(_F64).itemsize
        rb = 2*_RF_N*np.dtype(_F64).itemsize
        self._st_shm = _SharedMemory(create=True, size=sb)
        self._rf_shm = _SharedMemory(create=True, size=rb)
        self._st_view = np.ndarray((2,_ST_N), _F64, buffer=self._st_shm.buf)
        self._rf_view = np.ndarray((2,_RF_N), _F64, buffer=self._rf_shm.buf)
        self._st_ctr = ctx.Value("Q", 0, lock=False)
        self._rf_ctr = ctx.Value("Q", 0, lock=False)
        self._rf_wc = 0
        self._ctrl_stop_ev = ctx.Event()
        rdy = ctx.Event()
        self._ctrl_process = ctx.Process(
            target=_control_process_worker,
            args=(
                self.robot.fci_ip,
                self._st_shm.name, self._rf_shm.name,
                self._st_ctr, self._rf_ctr,
                self._ctrl_stop_ev, rdy, self.track,
                self.compensate_coriolis, self.joint_limit_repulsion_active,
                self.clip, self.max_delta_tau.tolist(), self.torque_limit.tolist(),
                self.joint_limit_activation_distance, self.joint_limit_stiffness,
                self.joint_limit_damping, self.joint_limit_max_torque,
                self.lower_joint_limits.tolist(), self.upper_joint_limits.tolist(),
                self.cartesian_position_clip.tolist(), self.cartesian_rotation_clip.tolist(),
            ),
            daemon=True, name="xense-franka-ctrl",
        )
        self._process_mode = True
        self.running = True
        self._ctrl_process.start()
        try:
            if not rdy.wait(timeout=5.0):
                raise TimeoutError("Control process failed to start within 5 s")
            return self._ctrl_process
        except BaseException:
            self.stop()
            raise

    def stop(self):
        if self._process_mode:
            return self._stop_process()
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                import warnings
                warnings.warn(
                    "xense_franka: control thread did not exit within "
                    "timeout — robot.stop() skipped to avoid deadlock",
                    RuntimeWarning, stacklevel=2)
                self._thread = None; self.running = False; return
            self._thread = None
        self.running = False
        self.robot.stop()

    def _stop_process(self):
        if self._ctrl_stop_ev: self._ctrl_stop_ev.set()
        if self._ctrl_process and self._ctrl_process.is_alive():
            self._ctrl_process.join(timeout=3.0)
            if self._ctrl_process.is_alive(): self._ctrl_process.terminate()
        self._ctrl_process = None; self._ctrl_stop_ev = None
        self._st_view = self._rf_view = None
        for s in [self._st_shm, self._rf_shm]:
            if s:
                try: s.close(); s.unlink()
                except: pass
        self._st_shm = self._rf_shm = None
        self._st_ctr = self._rf_ctr = None
        self.running = False; self._process_mode = False

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        return False

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

    def move(
        self,
        qpos=None,
        vel=np.ones(7, dtype=float) * 0.8,
        acc=np.ones(7, dtype=float) * 0.5,
    ):
        self._assert_loop_ok()
        self._request_type_change("impedance")

        if qpos is None:
            qpos = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, 0.7853], dtype=float)

        inp = InputParameter(7)
        current_state = self.get_state()
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

        sample_hz = max(self._publish_freq or 50.0, 50.0)
        sample_dt = 1.0 / sample_hz
        steps = max(int(np.ceil(trajectory.duration / sample_dt)), 1)

        for step in range(steps + 1):
            t = min(step * sample_dt, trajectory.duration)
            q_ref, dq_ref, _ = trajectory.at_time(t)
            self._rate_limit_publish("move_joint_reference", dt=sample_dt)
            self._set_joint_reference(q=q_ref, dq=dq_ref)

        self._set_joint_reference(q=inp.target_position, dq=np.zeros(7, dtype=float))

    # ------------------------------------------------------------------
    # State queries
    # ------------------------------------------------------------------

    def get_current_ee_pose(self) -> np.ndarray:
        return self.get_state()["ee"].copy()

    def get_current_joint_positions(self) -> np.ndarray:
        return self.get_state()["qpos"].copy()

    def get_current_joint_velocities(self) -> np.ndarray:
        return self.get_state()["qvel"].copy()

    # ------------------------------------------------------------------
    # Convenience aliases
    # ------------------------------------------------------------------

    def get_ee_pose(self) -> np.ndarray:
        return self.get_current_ee_pose()

    def get_joint_positions(self) -> np.ndarray:
        return self.get_current_joint_positions()

    def get_joint_velocities(self) -> np.ndarray:
        return self.get_current_joint_velocities()

    def get_state(self) -> dict:
        if self._process_mode and self._st_ctr is not None:
            # Wait for first state from control process (up to 2s)
            if self._st_ctr.value == 0:
                for _ in range(200):
                    if self._st_ctr.value > 0:
                        break
                    time.sleep(0.01)
            if self._st_ctr.value > 0:
                s = _upk_state(self._st_view, (self._st_ctr.value - 1) % 2)
                self.state = s
                self.last_torque = s.get("last_torque")
                return s
        return self.robot.state

    def get_external_wrench(self) -> np.ndarray:
        return self.get_state()['ext_wrench'].copy()

    def set_ee_pose(self, pose: np.ndarray):
        self.set_cartesian_reference(pose)

    def set_joint_positions(self, q: np.ndarray):
        self.set_joint_reference(q)

    def set_gains(self, kp, kd=None, mode: str = "osc"):
        if mode == "osc":
            self.set_cartesian_gains(kp, kd)
        else:
            self.set_joint_gains(kp, kd)

    def move_delta(self, dx: float = 0, dy: float = 0, dz: float = 0,
                   drx: float = 0, dry: float = 0, drz: float = 0):
        current_ee = self.get_current_ee_pose()
        current_ee[:3, 3] += np.array([dx, dy, dz])
        if drx != 0 or dry != 0 or drz != 0:
            rotation_delta = R.from_euler('xyz', [drx, dry, drz], degrees=True).as_matrix()
            current_ee[:3, :3] = rotation_delta @ current_ee[:3, :3]
        self.set_cartesian_reference(current_ee)

    # ------------------------------------------------------------------
    # Tracker factories
    # ------------------------------------------------------------------

    def joint_tracker(self, stiffness=None, damping=None, damping_ratio: float = 1.0,
                      restore_on_exit: bool = True) -> JointImpedanceTracker:
        return JointImpedanceTracker(
            self, stiffness=stiffness, damping=damping,
            damping_ratio=damping_ratio, restore_on_exit=restore_on_exit,
        )

    def cartesian_tracker(self, stiffness=None, damping=None, damping_ratio: float = 1.0,
                           nullspace_stiffness: Optional[float] = None,
                           restore_on_exit: bool = True) -> CartesianImpedanceTracker:
        return CartesianImpedanceTracker(
            self, stiffness=stiffness, damping=damping,
            damping_ratio=damping_ratio, nullspace_stiffness=nullspace_stiffness,
            restore_on_exit=restore_on_exit,
        )

    def exponential_tracker(self, mode: str = "impedance", time_constant: float = 0.5,
                             stiffness=None, damping=None, damping_ratio: float = 1.0,
                             nullspace_stiffness: Optional[float] = None,
                             restore_on_exit: bool = True) -> ExponentialImpedanceTracker:
        return ExponentialImpedanceTracker(
            self, mode=mode, time_constant=time_constant,
            stiffness=stiffness, damping=damping, damping_ratio=damping_ratio,
            nullspace_stiffness=nullspace_stiffness, restore_on_exit=restore_on_exit,
        )
