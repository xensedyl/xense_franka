# API 总览

公开 API 从 `xense_franka/__init__.py` 导出：

```python
from xense_franka import (
    RobotInterface,
    FrankaController,
    JointReference,
    CartesianReference,
    JointImpedanceGains,
    CartesianImpedanceGains,
    JointImpedanceTracker,
    CartesianImpedanceTracker,
    ExponentialImpedanceTracker,
)
```

## 核心类

| 类 | 模块 | 说明 |
| --- | --- | --- |
| `RobotInterface` | `xense_franka.robot` | pylibfranka FCI 封装，负责机器人状态读取和力矩写入 |
| `FrankaController` | `xense_franka.controller` | 高层同步控制器，负责模式切换、轨迹、reference 和控制循环 |
| `JointImpedanceTracker` | `xense_franka.trackers` | 关节阻抗控制会话 |
| `CartesianImpedanceTracker` | `xense_franka.trackers` | 笛卡尔阻抗控制会话 |
| `ExponentialImpedanceTracker` | `xense_franka.trackers` | 指数平滑 reference 控制会话 |

## 数据类

| 数据类 | 说明 |
| --- | --- |
| `JointReference` | 关节目标位置、速度和前馈力矩 |
| `CartesianReference` | 末端目标位姿、twist 和 nullspace 目标 |
| `JointImpedanceGains` | 关节刚度和阻尼 |
| `CartesianImpedanceGains` | 笛卡尔刚度、阻尼和 nullspace 刚度 |

## 常量

```python
from xense_franka import (
    FR3_JOINT_LIMITS_LOWER,
    FR3_JOINT_LIMITS_UPPER,
    FR3_TORQUE_LIMIT,
)
```

这些常量用于默认关节限位和力矩裁剪。
