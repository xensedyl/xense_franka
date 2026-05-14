# 数据结构

数据结构定义在 `xense_franka.references` 中，也会从 `xense_franka` 包入口导出。

```python
from xense_franka import (
    JointReference,
    CartesianReference,
    JointImpedanceGains,
    CartesianImpedanceGains,
)
```

## JointReference

```python
JointReference(
    q=np.zeros(7),
    dq=np.zeros(7),
    tau_ff=np.zeros(7),
)
```

字段：

| 字段 | 形状 | 说明 |
| --- | --- | --- |
| `q` | `(7,)` | 目标关节位置 |
| `dq` | `(7,)` | 目标关节速度 |
| `tau_ff` | `(7,)` | 前馈关节力矩 |

复制：

```python
ref2 = ref.copy()
```

## CartesianReference

```python
CartesianReference(
    pose=np.eye(4),
    twist=np.zeros(6),
    nullspace_target=None,
)
```

字段：

| 字段 | 形状 | 说明 |
| --- | --- | --- |
| `pose` | `(4, 4)` | 目标末端齐次变换矩阵 |
| `twist` | `(6,)` | 目标末端速度 |
| `nullspace_target` | `(7,)` 或 `None` | 零空间关节目标 |

复制：

```python
ref2 = ref.copy()
```

## JointImpedanceGains

```python
JointImpedanceGains(
    stiffness=np.ones(7) * 80.0,
    damping=2.0 * np.sqrt(np.ones(7) * 80.0),
)
```

字段：

| 字段 | 形状 | 说明 |
| --- | --- | --- |
| `stiffness` | `(7,)` | 关节刚度 |
| `damping` | `(7,)` | 关节阻尼 |

## CartesianImpedanceGains

```python
CartesianImpedanceGains(
    stiffness=np.array([600, 600, 600, 50, 50, 50]),
    damping=2.0 * np.sqrt(stiffness),
    nullspace_stiffness=5.0,
)
```

字段：

| 字段 | 形状/类型 | 说明 |
| --- | --- | --- |
| `stiffness` | `(6,)` | 平移和旋转刚度 |
| `damping` | `(6,)` | 平移和旋转阻尼 |
| `nullspace_stiffness` | `float` | 零空间刚度 |

## 工具函数

`xense_franka.torque_utils` 中包含控制器内部使用的工具函数：

| 函数 | 说明 |
| --- | --- |
| `as_array(value, size)` | 将标量或数组转换为固定长度数组 |
| `pose_copy(pose)` | 复制并正交化 `(4, 4)` pose |
| `critical_damping(stiffness)` | 计算 `2 * sqrt(stiffness)` |
| `saturate_torque_rate(...)` | 限制力矩变化率 |
| `compute_joint_limit_torque(...)` | 计算关节限位排斥力矩 |
| `pseudo_inverse(matrix)` | SVD 伪逆 |
