# FrankaController

`FrankaController` 是主要用户入口。它提供同步命令 API，并在后台运行 1 kHz 控制循环。

导入：

```python
from xense_franka import RobotInterface, FrankaController
```

创建：

```python
robot = RobotInterface("192.168.99.111")
ctrl = FrankaController(robot)
```

## 生命周期

### start()

```python
ctrl.start(use_process=True)
```

启动控制循环。默认 `use_process=True`，控制环运行在独立进程中。调试时可以使用线程模式：

```python
ctrl.start(use_process=False)
```

### stop()

```python
ctrl.stop()
```

停止控制循环并释放 robot control session。

### 上下文管理器

```python
with FrankaController(robot) as ctrl:
    ctrl.move()
```

进入时调用 `start()`，退出时调用 `stop()`。

## 模式切换

```python
ctrl.switch("impedance")
ctrl.switch("pid")
ctrl.switch("osc")
ctrl.switch("torque")
```

模式切换在下一个控制 tick 生效。`effective_type` 可以看到当前或待切换目标模式：

```python
print(ctrl.effective_type)
```

## 关节命令

```python
ctrl.set_joint_gains(stiffness=80.0)
ctrl.set_joint_reference(q)
ctrl.set_joint_reference(q, dq=dq, tau_ff=tau_ff)
```

兼容属性：

```python
ctrl.kp = [80] * 7
ctrl.kd = [10] * 7
ctrl.q_desired = q
```

便利别名：

```python
ctrl.set_joint_positions(q)
```

## 笛卡尔命令

```python
ctrl.set_cartesian_gains(
    stiffness=[600, 600, 600, 50, 50, 50],
    nullspace_stiffness=5.0,
)
ctrl.set_cartesian_reference(pose)
ctrl.set_cartesian_reference(
    pose=pose,
    twist=twist,
    nullspace_target=ctrl.initial_qpos,
)
```

兼容属性：

```python
ctrl.ee_kp = [600, 600, 600, 50, 50, 50]
ctrl.ee_kd = [49, 49, 49, 14, 14, 14]
ctrl.ee_desired = pose
```

便利别名：

```python
ctrl.set_ee_pose(pose)
ctrl.move_delta(dx=0.01, dy=0, dz=0)
```

## 通用 set()

```python
ctrl.set("q_desired", q)
ctrl.set("ee_desired", pose)
ctrl.set("torque", tau)
```

`set()` 会先执行发布限频，再根据属性名分发到对应 setter。

## 发布频率

```python
ctrl.set_freq(50)
ctrl.set_freq(None)
```

传入正数时启用发布限频。传入 `None` 或 `0` 时关闭限频。

## move()

```python
ctrl.move()
ctrl.move(qpos=[0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, 0.7853])
```

`move()` 使用 Ruckig 规划关节轨迹，并以关节阻抗控制执行。

参数：

| 参数 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `qpos` | `(7,)` 或 `None` | 默认 home 姿态 | 目标关节位置 |
| `vel` | 标量或 `(7,)` | `0.8` | 最大速度 |
| `acc` | 标量或 `(7,)` | `0.5` | 最大加速度 |

## 状态查询

```python
state = ctrl.get_state()
ee = ctrl.get_current_ee_pose()
q = ctrl.get_current_joint_positions()
dq = ctrl.get_current_joint_velocities()
wrench = ctrl.get_external_wrench()
```

短别名：

```python
ctrl.get_ee_pose()
ctrl.get_joint_positions()
ctrl.get_joint_velocities()
```

## Tracker 工厂

```python
ctrl.joint_tracker(stiffness=80)
ctrl.cartesian_tracker(stiffness=[600, 600, 600, 50, 50, 50])
ctrl.exponential_tracker(mode="impedance", time_constant=0.3)
```

详见 [Trackers](../guide/trackers.md)。

## 常用属性

| 属性 | 说明 |
| --- | --- |
| `running` | 控制循环是否运行 |
| `type` | 控制循环已生效的模式 |
| `effective_type` | 已生效或待生效的模式 |
| `initial_qpos` | 最近初始化时的关节位置 |
| `initial_ee` | 最近初始化时的末端位姿 |
| `gains_time_constant` | 增益指数平滑时间常数 |
| `clip` | 是否启用力矩变化率限制 |
| `max_delta_tau` | 每 tick 最大力矩变化 |
| `torque_limit` | 关节力矩限幅 |
| `joint_limit_repulsion_active` | 是否启用关节限位排斥 |
