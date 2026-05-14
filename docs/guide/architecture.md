# 控制架构

`xense_franka` 的公开接口是同步的，但真实力矩控制在后台 1 kHz 循环中执行。用户线程只负责设置目标、增益和模式，控制循环负责读取机器人状态、计算力矩并写入 FCI。

## 线程/进程模型

默认启动方式：

```python
ctrl.start()
```

等价于：

```python
ctrl.start(use_process=True)
```

默认的进程模式会创建一个独立控制进程，并用 shared memory 交换状态和参考。这样可以减少 Python GIL 对 1 kHz 控制循环的影响。

如需调试或降低复杂度，可以使用线程模式：

```python
ctrl.start(use_process=False)
```

## 数据流

```text
User code                         Control loop
---------                         ------------
ctrl.switch("osc")        --->    next tick consumes mode change
ctrl.set_cartesian_gains() --->    gains target updated
ctrl.set_cartesian_reference()
                          --->    read state from pylibfranka
                                  smooth gains
                                  compute torque
                                  saturate torque rate
                                  write torque to FCI
```

核心数据通过 double-buffer handle 或 shared memory 传递，控制循环热路径避免使用 Python 锁。

## 状态访问

常用状态查询：

```python
state = ctrl.get_state()
q = ctrl.get_joint_positions()
dq = ctrl.get_joint_velocities()
ee = ctrl.get_ee_pose()
wrench = ctrl.get_external_wrench()
```

`state` 字典包含：

| 键 | 形状 | 说明 |
| --- | --- | --- |
| `qpos` | `(7,)` | 当前关节位置 |
| `qvel` | `(7,)` | 当前关节速度 |
| `ee` | `(4, 4)` | 末端齐次变换矩阵 |
| `jac` | `(6, 7)` | 零空间雅可比 |
| `mm` | `(7, 7)` | 质量矩阵 |
| `coriolis` | `(7,)` | 科氏项 |
| `last_torque` | `(7,)` | 上一帧期望关节力矩 |
| `ext_wrench` | `(6,)` | 外部力/力矩估计 |
| `dt` | `float` | 控制周期，单位秒 |

## 模式切换

```python
ctrl.switch("impedance")
ctrl.switch("osc")
ctrl.switch("pid")
ctrl.switch("torque")
```

模式切换会在下一个控制 tick 生效。切换时控制器会先把当前关节和末端位姿写入 reference handle，再发布目标模式，避免新模式刚开始时使用旧 reference。

## 增益平滑

增益不会在控制环中瞬间跳变，而是以指数形式平滑到目标值：

```python
ctrl.gains_time_constant = 0.1
```

默认时间常数为 `0.1` 秒。较小的值响应更快，较大的值更平滑。

## 力矩限制

默认安全相关参数：

```python
ctrl.clip = True
ctrl.max_delta_tau = [0.5] * 7
ctrl.compensate_coriolis = True
ctrl.joint_limit_repulsion_active = True
```

控制器会先限制每个 tick 的力矩变化率，再按 FR3 力矩上限裁剪输出。
