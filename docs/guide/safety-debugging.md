# 安全与调试

真实机器人控制有风险。运行示例前，请确认机器人、末端工具、线缆、周围人员和工作空间都处于安全状态。

## 推荐启动流程

1. 在 Franka Desk 中确认机器人没有错误。
2. 解锁机器人并激活 FCI。
3. 先运行只读状态查询，确认 IP 和依赖正常。
4. 使用较低速度执行 `move()` 到已知安全姿态。
5. 从较低刚度开始测试阻抗控制。

只读状态查询：

```python
from xense_franka import RobotInterface

robot = RobotInterface("192.168.99.111")
state = robot.state
print(state["qpos"])
print(state["ee"])
```

## 控制安全参数

常用参数：

```python
ctrl.clip = True
ctrl.max_delta_tau = [0.5] * 7
ctrl.torque_limit
ctrl.compensate_coriolis = True
ctrl.joint_limit_repulsion_active = True
```

`max_delta_tau` 默认是 `0.5 Nm/tick`，用于降低触发 `controller_torque_discontinuity` reflex 的概率。

关节限位排斥参数：

```python
ctrl.joint_limit_activation_distance = 0.1
ctrl.joint_limit_stiffness = 4.0
ctrl.joint_limit_damping = 1.0
ctrl.joint_limit_max_torque = 5.0
```

## 笛卡尔误差裁剪

OSC 模式会裁剪每 tick 参与控制的位姿误差：

```python
ctrl.cartesian_position_clip = [0.10, 0.10, 0.10]
ctrl.cartesian_rotation_clip = [0.25, 0.25, 0.25]
```

这可以避免一次发送过大的目标位姿时产生过大的瞬时 wrench。

## 控制循环健康检查

`test_connection()` 会打开 timing 统计，持续约 5 秒：

```python
ctrl.start()
ctrl.test_connection()
ctrl.stop()
```

如果控制循环异常退出，控制器会打印最近 5 帧诊断信息，包括：

- 控制器类型
- read/compute/step 耗时
- 上一帧力矩
- 当前命令力矩
- 力矩变化量

## 常见问题

### Torque control not started

说明还没有成功调用 `ctrl.start()`，或者机器人控制会话已经停止。检查启动流程和 `finally: ctrl.stop()` 的位置。

### Control process terminated unexpectedly

默认 `start()` 使用独立进程。这个错误说明控制进程已经退出。通常需要查看终端里控制进程打印的 traceback 和最后 5 帧诊断信息。

### controller_torque_discontinuity

常见原因：

- 目标 reference 跳变过大
- 刚度设置过高
- 控制循环 timing 抖动
- 用户代码直接写入过大的力矩

处理建议：

- 降低刚度或阻尼
- 用 `set_freq(50)` 让 reference 发布节奏稳定
- 保持 `clip=True`
- 保持 `max_delta_tau` 在保守范围
- 避免在 OSC 模式中一次发送很远的末端目标

### OSC 姿态跳变

`pose` 必须是 `(4, 4)` 齐次变换矩阵，旋转部分会被正交化。生成姿态时建议使用 `scipy.spatial.transform.Rotation`。

```python
from scipy.spatial.transform import Rotation as R

pose = ctrl.get_ee_pose()
delta_R = R.from_euler("xyz", [0, 0, 5], degrees=True).as_matrix()
pose[:3, :3] = delta_R @ pose[:3, :3]
ctrl.set_cartesian_reference(pose)
```

## 停止控制器

建议始终用 `try/finally` 或上下文管理器：

```python
ctrl.start()
try:
    ctrl.move()
finally:
    ctrl.stop()
```

上下文管理器更简洁：

```python
with FrankaController(robot) as ctrl:
    ctrl.move()
```
