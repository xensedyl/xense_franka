# RobotInterface

`RobotInterface` 是 `pylibfranka` 的轻量封装，负责：

- 创建 `pylibfranka.Robot`
- 加载模型
- 读取机器人状态
- 启动/停止 torque control
- 向 FCI 写入关节力矩

导入：

```python
from xense_franka import RobotInterface
```

## 初始化

```python
robot = RobotInterface("192.168.99.111")
```

参数：

| 参数 | 类型 | 说明 |
| --- | --- | --- |
| `ip` | `str` | 机器人 FCI IP |

初始化时会连接机器人、加载模型并设置 collision behavior。

## start()

```python
robot.start()
```

启动 `pylibfranka` torque control session。通常不需要手动调用，`FrankaController.start()` 会调用它。

## stop()

```python
robot.stop()
```

停止机器人当前控制会话。通常由 `FrankaController.stop()` 调用。

## state

```python
state = robot.state
```

返回状态字典。常用字段：

| 键 | 形状 | 说明 |
| --- | --- | --- |
| `qpos` | `(7,)` | 当前关节位置 |
| `qvel` | `(7,)` | 当前关节速度 |
| `ee` | `(4, 4)` | 末端齐次变换矩阵 |
| `jac` | `(6, 7)` | 雅可比矩阵 |
| `mm` | `(7, 7)` | 质量矩阵 |
| `coriolis` | `(7,)` | 科氏项 |
| `last_torque` | `(7,)` | 上一帧目标力矩 |
| `ext_wrench` | `(6,)` | 外部力/力矩估计 |
| `dt` | `float` | 控制周期 |

## read_control_state()

```python
state = robot.read_control_state()
```

控制循环内部使用的方法。它要求 torque control 已启动，并且同一个控制线程/进程拥有读取权。

## step()

```python
robot.step(torque)
```

写入 `(7,)` 关节力矩命令。正常使用中应通过 `FrankaController` 计算和写入力矩，而不是在用户代码中直接调用。
