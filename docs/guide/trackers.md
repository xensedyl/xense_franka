# Trackers

Tracker 是更结构化的控制会话。它们在进入 `with` 时切换控制模式并设置增益，退出时默认恢复之前的模式和增益。

## JointImpedanceTracker

```python
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")

with FrankaController(robot) as ctrl:
    ctrl.move()

    with ctrl.joint_tracker(stiffness=80) as tracker:
        target = ctrl.get_joint_positions()
        target[5] += 0.3

        tracker.set_target(target)
        while tracker.ok():
            if abs(ctrl.get_joint_positions()[5] - target[5]) < 0.01:
                break
            tracker.tick(0.02)
```

等价类：

```python
from xense_franka import JointImpedanceTracker

with JointImpedanceTracker(ctrl, stiffness=80) as tracker:
    tracker.set_target(target)
```

## CartesianImpedanceTracker

```python
import numpy as np

with ctrl.cartesian_tracker(
    stiffness=[600, 600, 600, 50, 50, 50],
    nullspace_stiffness=5.0,
) as tracker:
    ee0 = ctrl.get_ee_pose()

    for i in range(50):
        pose = ee0.copy()
        pose[0, 3] += 0.05 * (i + 1) / 50
        tracker.set_target(pose)
        tracker.tick(0.02)
```

`set_target()` 接收 `(4, 4)` 齐次变换矩阵。

## ExponentialImpedanceTracker

`ExponentialImpedanceTracker` 会把当前 reference 指数平滑到目标 reference。

关节空间：

```python
with ctrl.exponential_tracker(
    mode="impedance",
    time_constant=0.3,
    stiffness=100,
) as tracker:
    target = ctrl.get_joint_positions()
    target[3] += 0.2
    tracker.set_target(target)

    for _ in range(100):
        tracker.tick(0.02)
```

笛卡尔空间：

```python
with ctrl.exponential_tracker(
    mode="osc",
    time_constant=0.5,
    stiffness=[300, 300, 300, 30, 30, 30],
) as tracker:
    target = ctrl.get_ee_pose()
    target[2, 3] += 0.05
    tracker.set_target(target)

    for _ in range(100):
        tracker.tick(0.02)
```

## 退出行为

默认情况下，Tracker 退出时会恢复进入前的控制模式和增益：

```python
with ctrl.joint_tracker(stiffness=80, restore_on_exit=True):
    ...
```

如果希望退出后保留当前模式：

```python
with ctrl.joint_tracker(stiffness=80, restore_on_exit=False):
    ...
```

## 方法摘要

| Tracker | 主要方法 | 说明 |
| --- | --- | --- |
| `JointImpedanceTracker` | `set_target(q, dq=None, tau_ff=None)` | 发布关节参考 |
| `CartesianImpedanceTracker` | `set_target(pose, twist=None, nullspace_target=None)` | 发布笛卡尔参考 |
| `ExponentialImpedanceTracker` | `set_target(target)` | 设置最终目标 |
| 全部 | `tick(dt=0.02)` | 睡眠并推进 reference |
| 全部 | `ok()` | 控制器仍在运行且 tracker 未停止 |
| 全部 | `stop()` | 停止 tracker 循环 |
