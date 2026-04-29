#!/usr/bin/env python3
"""
Tracker 模式示例

演示三种 Tracker 上下文管理器的用法：
1. JointImpedanceTracker — 关节空间阻抗控制
2. CartesianImpedanceTracker — 笛卡尔空间阻抗控制
3. ExponentialImpedanceTracker — 指数平滑运动
"""

import time
import numpy as np
from xense_franka import SyncFrankaController


ROBOT_IP = "192.168.99.111"
HOME_JOINTS = [0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, 0.7853]


def demo_joint_tracker(ctrl: SyncFrankaController):
    """关节空间阻抗跟踪"""
    print("\n=== Joint Impedance Tracker ===")
    ctrl.move(HOME_JOINTS)
    time.sleep(0.5)

    with ctrl.joint_tracker(stiffness=80) as t:
        q0 = ctrl.get_joint_positions()
        target = q0.copy()
        target[5] += 0.3  # 移动第 6 关节

        t.set_target(target)
        while t.ok():
            current = ctrl.get_joint_positions()
            if np.linalg.norm(current - target) < 0.01:
                break
            t.tick(0.02)

    print("Joint tracker done — gains and mode restored.")


def demo_cartesian_tracker(ctrl: SyncFrankaController):
    """笛卡尔空间阻抗跟踪"""
    print("\n=== Cartesian Impedance Tracker ===")
    ctrl.move(HOME_JOINTS)
    time.sleep(0.5)

    stiffness = np.array([600.0, 600.0, 600.0, 50.0, 50.0, 50.0])
    with ctrl.cartesian_tracker(stiffness=stiffness) as t:
        ee0 = ctrl.get_ee_pose()

        # 沿 X 方向移动 5cm
        steps = 50
        for i in range(steps):
            pose = ee0.copy()
            pose[0, 3] += 0.05 * (i + 1) / steps
            t.set_target(pose)
            t.tick(0.02)

        # 保持 1 秒
        time.sleep(1.0)

    print("Cartesian tracker done — gains and mode restored.")


def demo_exponential_tracker(ctrl: SyncFrankaController):
    """指数平滑运动"""
    print("\n=== Exponential Impedance Tracker ===")
    ctrl.move(HOME_JOINTS)
    time.sleep(0.5)

    with ctrl.exponential_tracker(mode="impedance", time_constant=0.3, stiffness=100) as t:
        q0 = ctrl.get_joint_positions()
        target = q0.copy()
        target[3] += 0.2  # 移动第 4 关节

        t.set_target(target)
        for _ in range(100):  # ~2 秒
            if not t.ok():
                break
            t.tick(0.02)

    print("Exponential tracker done.")


def main():
    ctrl = SyncFrankaController(ROBOT_IP)
    ctrl.start()

    try:
        demo_joint_tracker(ctrl)
        demo_cartesian_tracker(ctrl)
        demo_exponential_tracker(ctrl)
    finally:
        ctrl.stop()

    print("\nAll demos completed!")


if __name__ == "__main__":
    main()
