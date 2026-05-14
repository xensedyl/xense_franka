# 控制模式

`FrankaController` 支持四种模式：

| 模式 | 名称 | 主要用途 |
| --- | --- | --- |
| `impedance` | 关节阻抗 | 关节空间跟踪、平滑移动、系统辨识 |
| `osc` | 笛卡尔阻抗 | 末端位姿控制、遥操作、接触任务 |
| `pid` | 关节 PID | 带积分项的关节位置控制 |
| `torque` | 直接力矩 | 研究自定义控制律 |

## 关节阻抗

关节阻抗模式根据目标关节位置、速度和前馈力矩计算输出：

```text
tau = Kp * (q_desired - q) + Kd * (dq_desired - dq) + tau_ff + coriolis
```

示例：

```python
import numpy as np
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")

with FrankaController(robot) as ctrl:
    ctrl.move()
    ctrl.switch("impedance")
    ctrl.set_joint_gains(stiffness=80.0)
    ctrl.set_freq(50)

    q0 = ctrl.get_joint_positions()
    for i in range(200):
        q = q0.copy()
        q[5] += 0.2 * np.sin(i / 50.0 * np.pi)
        ctrl.set_joint_reference(q)
```

`set_joint_gains()` 的 `stiffness` 可以是标量或 `(7,)` 数组。未传入 `damping` 时会按临界阻尼计算：

```python
ctrl.set_joint_gains(stiffness=[80, 80, 80, 80, 50, 50, 30])
ctrl.set_joint_gains(stiffness=100.0, damping=10.0)
ctrl.set_joint_gains(stiffness=80.0, damping_ratio=0.7)
```

## 笛卡尔阻抗 OSC

OSC 模式使用末端位姿误差和速度误差计算任务空间 wrench，再映射到关节力矩：

```text
tau = J.T @ wrench + nullspace + coriolis
```

示例：

```python
import numpy as np
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")

with FrankaController(robot) as ctrl:
    ctrl.move()
    ctrl.switch("osc")
    ctrl.set_cartesian_gains(
        stiffness=[600, 600, 600, 50, 50, 50],
        nullspace_stiffness=5.0,
    )
    ctrl.set_freq(50)

    ee0 = ctrl.get_ee_pose()
    for i in range(200):
        target = ee0.copy()
        target[1, 3] += 0.05 * np.sin(i / 50.0 * np.pi)
        ctrl.set_cartesian_reference(
            pose=target,
            nullspace_target=ctrl.initial_qpos,
        )
```

常用刚度参考：

| 场景 | stiffness |
| --- | --- |
| 精确跟踪 | `[600, 600, 600, 50, 50, 50]` |
| 遥操作 | `[300, 300, 300, 30, 30, 30]` |
| 柔顺交互 | `[100, 100, 100, 10, 10, 10]` |

`stiffness` 前三项对应平移，后三项对应旋转。未传入阻尼时会按临界阻尼计算。

## PID

PID 模式在关节阻抗基础上增加积分项：

```python
ctrl.switch("pid")
ctrl.set_joint_gains(stiffness=160.0)
ctrl.set_joint_reference(target_q)
```

积分项由控制器内部维护，并在模式切换时清零。默认积分限幅为：

```python
ctrl.integral_limit = 10.0
```

## 直接力矩

直接力矩模式适合验证自定义控制律。使用前需要非常谨慎，确保输出力矩、力矩变化率和机器人状态都在安全范围内。

```python
import numpy as np

ctrl.switch("torque")
ctrl.set("torque", np.zeros(7))
```

也可以直接设置属性：

```python
ctrl.torque = np.array([0, 0, 0, 0, 0, 0, 0.5], dtype=float)
```

默认情况下，控制器仍会执行力矩变化率限制和力矩上限裁剪。

## move()

`move()` 使用 Ruckig 生成关节轨迹，并在关节阻抗模式下执行：

```python
ctrl.move()
ctrl.move([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, 0.7853])
```

可调整最大速度和加速度：

```python
ctrl.move(
    qpos=[0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, 0.7853],
    vel=[0.5] * 7,
    acc=[0.3] * 7,
)
```
