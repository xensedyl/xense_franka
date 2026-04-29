"""
同步接口封装。

控制循环在 FrankaController 的后台线程中运行；这里仅提供同步调用包装。
"""
import numpy as np
from typing import Optional, List
from scipy.spatial.transform import Rotation as R

from .robot import RobotInterface
from .controller import FrankaController
from .trackers import JointImpedanceTracker, CartesianImpedanceTracker, ExponentialImpedanceTracker


class SyncFrankaController:
    """
    同步的 Franka 控制器接口。

    示例用法:
    ```python
    from xense_franka import SyncFrankaController

    controller = SyncFrankaController("192.168.99.111")
    controller.start()
    controller.move([0, 0, 0, -1.57, 0, 1.57, 0.78])
    controller.switch("osc")

    for i in range(100):
        ee = controller.get_ee_pose()
        ee[:3, 3] += [0.001, 0, 0]
        controller.set_ee_pose(ee)

    controller.stop()
    ```
    """

    def __init__(self, robot_ip: str):
        """
        初始化同步控制器。

        Args:
            robot_ip: 机器人 IP 地址，如 "192.168.99.111"
        """
        self.robot_ip = robot_ip
        self._robot: Optional[RobotInterface] = None
        self._controller: Optional[FrankaController] = None
        self._started = False

    def start(self):
        """启动控制器和 1kHz 控制循环"""
        if self._started:
            return

        self._robot = RobotInterface(self.robot_ip)
        self._controller = FrankaController(self._robot)
        try:
            self._controller.start()
        except Exception:
            self._controller = None
            self._robot = None
            raise
        self._started = True

    def stop(self):
        """停止控制器"""
        if not self._started:
            return

        try:
            if self._controller:
                self._controller.stop()
        except Exception:
            pass
        finally:
            self._controller = None
            self._robot = None
            self._started = False

    def move(self, q_target: Optional[List[float]] = None):
        """
        移动到目标关节位置（使用 Ruckig 轨迹规划）。

        Args:
            q_target: 目标关节角度（7个值），None 则移动到默认位置
        """
        self._controller.move(q_target)

    def switch(self, mode: str):
        """
        切换控制模式。

        Args:
            mode: "impedance" (关节阻抗) 或 "osc" (笛卡尔阻抗)
        """
        self._controller.switch(mode)

    def set_freq(self, freq: int):
        """
        设置 set 命令的更新频率。

        Args:
            freq: 频率 (Hz)，如 50
        """
        self._controller.set_freq(freq)

    def set_gains(self, kp: np.ndarray, kd: np.ndarray, mode: str = "osc"):
        """
        设置控制增益。

        Args:
            kp: 刚度增益
            kd: 阻尼增益
            mode: "osc" 或 "impedance"
        """
        if mode == "osc":
            self._controller.set_cartesian_gains(kp, kd)
        else:
            self._controller.set_joint_gains(kp, kd)

    def get_ee_pose(self) -> np.ndarray:
        """
        获取当前末端执行器位姿。

        Returns:
            4x4 齐次变换矩阵
        """
        return self._controller.get_current_ee_pose()

    def get_state(self) -> dict:
        """
        获取机器人完整状态。

        Returns:
            包含关节位置、速度、力矩等的字典
        """
        return self._robot.state

    def get_joint_positions(self) -> np.ndarray:
        """获取当前关节位置"""
        return self._controller.get_current_joint_positions()

    def get_joint_velocities(self) -> np.ndarray:
        """获取当前关节速度"""
        return self._controller.get_current_joint_velocities()

    def get_external_wrench(self) -> np.ndarray:
        """获取外部力/力矩"""
        state = self._robot.state
        return np.array(state['ext_wrench'])

    def set_ee_pose(self, pose: np.ndarray):
        """
        设置期望的末端执行器位姿（OSC 模式）。

        Args:
            pose: 4x4 齐次变换矩阵
        """
        self._controller.set_cartesian_reference(pose)

    def set_joint_positions(self, q: np.ndarray):
        """
        设置期望的关节位置（阻抗模式）。

        Args:
            q: 7 个关节角度
        """
        self._controller.set_joint_reference(q)

    def move_delta(self, dx: float = 0, dy: float = 0, dz: float = 0,
                   drx: float = 0, dry: float = 0, drz: float = 0):
        """
        相对当前位置移动。

        Args:
            dx, dy, dz: 平移增量（米）
            drx, dry, drz: 旋转增量（度）
        """
        current_ee = self.get_ee_pose()

        # 应用平移
        current_ee[:3, 3] += np.array([dx, dy, dz])

        # 应用旋转
        if drx != 0 or dry != 0 or drz != 0:
            rotation_delta = R.from_euler('xyz', [drx, dry, drz], degrees=True).as_matrix()
            current_ee[:3, :3] = rotation_delta @ current_ee[:3, :3]

        self.set_ee_pose(current_ee)

    @property
    def initial_ee(self) -> np.ndarray:
        """获取初始末端位姿"""
        return self._controller.initial_ee.copy()

    @property
    def initial_qpos(self) -> np.ndarray:
        """获取初始关节位置"""
        return self._controller.initial_qpos.copy()

    def joint_tracker(self, stiffness=None, damping=None, damping_ratio: float = 1.0, restore_on_exit: bool = True) -> JointImpedanceTracker:
        """Create a JointImpedanceTracker bound to this controller."""
        return JointImpedanceTracker(
            self._controller, stiffness=stiffness, damping=damping,
            damping_ratio=damping_ratio, restore_on_exit=restore_on_exit,
        )

    def cartesian_tracker(self, stiffness=None, damping=None, damping_ratio: float = 1.0,
                          nullspace_stiffness: Optional[float] = None,
                          restore_on_exit: bool = True) -> CartesianImpedanceTracker:
        """Create a CartesianImpedanceTracker bound to this controller."""
        return CartesianImpedanceTracker(
            self._controller, stiffness=stiffness, damping=damping,
            damping_ratio=damping_ratio, nullspace_stiffness=nullspace_stiffness,
            restore_on_exit=restore_on_exit,
        )

    def exponential_tracker(self, mode: str = "impedance", time_constant: float = 0.5,
                            stiffness=None, damping=None, damping_ratio: float = 1.0,
                            nullspace_stiffness: Optional[float] = None,
                            restore_on_exit: bool = True) -> ExponentialImpedanceTracker:
        """Create an ExponentialImpedanceTracker bound to this controller."""
        return ExponentialImpedanceTracker(
            self._controller, mode=mode, time_constant=time_constant,
            stiffness=stiffness, damping=damping, damping_ratio=damping_ratio,
            nullspace_stiffness=nullspace_stiffness, restore_on_exit=restore_on_exit,
        )

    def __enter__(self):
        """支持 with 语句"""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持 with 语句"""
        self.stop()
        return False
