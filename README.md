# xense_franka

**xense_franka** provides a synchronous Python API for Franka Emika FR3 robots. It uses **`pylibfranka`** for the official low-level torque interface and runs the 1kHz impedance loop in a dedicated background thread, while user code publishes references from the main thread.

This library is inspired by aiofranka and reworked around a `franky`-style tracking controller model.

## v0.4.0 Changelog

### Breaking Changes

#### 1. Async → Sync API

所有公开方法从 `async def` 改为普通 `def`，不再依赖 `asyncio`。

```python
# Before (v0.3.0)
import asyncio
from xense_franka import RobotInterface, FrankaController

async def main():
    robot = RobotInterface("172.16.0.2")
    controller = FrankaController(robot)
    await controller.start()
    await controller.move()
    await controller.set("q_desired", target)
    await controller.stop()

asyncio.run(main())

# After (v0.4.0)
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("172.16.0.2")
controller = FrankaController(robot)
controller.start()
controller.move()
controller.set("q_desired", target)
controller.stop()
```

涉及的 8 个方法：`start`、`stop`、`move`、`set`、`set_joint_reference`、`set_cartesian_reference`、`test_connection`、`_rate_limit_publish`。

内部 `await asyncio.sleep()` 全部替换为 `time.sleep()`。`import asyncio` 从库源码中完全移除。

#### 2. 删除 SyncFrankaController

`SyncFrankaController` 存在的唯一目的是把 async 的 `FrankaController` 包装成同步调用。现在 `FrankaController` 本身就是同步的，`SyncFrankaController` 已删除。

原来 `SyncFrankaController` 的便利方法全部迁移到 `FrankaController`：

| 方法 | 说明 |
|------|------|
| `get_ee_pose()` | `get_current_ee_pose()` 的短别名 |
| `get_joint_positions()` | `get_current_joint_positions()` 的短别名 |
| `get_joint_velocities()` | `get_current_joint_velocities()` 的短别名 |
| `get_state()` | 返回 `robot.state` |
| `get_external_wrench()` | 获取外部力/力矩 |
| `set_ee_pose(pose)` | `set_cartesian_reference(pose)` 的短别名 |
| `set_joint_positions(q)` | `set_joint_reference(q)` 的短别名 |
| `set_gains(kp, kd, mode)` | 统一设 joint/cart gains |
| `move_delta(dx, dy, dz, ...)` | 相对当前 EE 位姿增量运动 |
| `joint_tracker(...)` | 创建 JointImpedanceTracker |
| `cartesian_tracker(...)` | 创建 CartesianImpedanceTracker |
| `exponential_tracker(...)` | 创建 ExponentialImpedanceTracker |

迁移方式：

```python
# Before (v0.3.0)
from xense_franka import SyncFrankaController
with SyncFrankaController("192.168.99.111") as ctrl:
    ctrl.move()

# After (v0.4.0)
from xense_franka import RobotInterface, FrankaController
robot = RobotInterface("192.168.99.111")
with FrankaController(robot) as ctrl:
    ctrl.move()
```

#### 3. 删除 Tracker 的 async 上下文管理器

三个 Tracker 类（`JointImpedanceTracker`、`CartesianImpedanceTracker`、`ExponentialImpedanceTracker`）的 `__aenter__` / `__aexit__` 已删除，只保留同步 `with` 语法。

#### 4. set_freq 默认不限频

`set_freq()` 默认值从 50Hz 改为 `None`（不限频）。用户不调 `set_freq()` 时，`set()`/`set_joint_reference()`/`set_cartesian_reference()` 立即返回，不 sleep。

```python
# 不限频（默认）：发多快跑多快
controller.set_joint_reference(target)

# 限频 50Hz：自动 sleep 保持节奏
controller.set_freq(50)
controller.set_joint_reference(target)

# 关闭限频
controller.set_freq(None)  # 或 set_freq(0)
```

注意：`move()` 内部轨迹采样始终 ≥ 50Hz，不受 `set_freq()` 影响。

### Bug Fixes

#### 5. max_delta_tau 降至 0.5 Nm/tick

`max_delta_tau` 从 `1.0` 降至 `0.5` Nm/tick。之前 1.0 Nm 正好踩在 libfranka 内部力矩变化率阈值上，在 timing 抖动时触发 `controller_torque_discontinuity` reflex。0.5 提供 2x 安全裕量。

#### 6. 控制循环崩溃诊断

控制循环崩溃时自动打印最后 5 帧的诊断信息：

```
===== CONTROL LOOP CRASH at iteration 2442 =====
  tick   2441 [       osc] total=0.94ms (read=0.71 compute=0.22 step=0.02)
    tau_prev = [ 0.048 -0.005  0.049  0.004  0.02  -0.001 -0.042]
    tau_cmd  = [ 1.048  0.066  1.049 -0.067  0.435 -0.05  -0.022]
    delta    = [ 1.     0.071  1.    -0.072  0.414 -0.049  0.019]
===== END DIAGNOSTIC =====
```

包含每帧的：控制器类型、read/compute/step 耗时、上一帧力矩、当前力矩、变化量。

### Internal Changes

- 版本号 → `0.4.0`（`__init__.py`、`setup.py`、`pyproject.toml`）
- `setup.py` / `pyproject.toml` description 去掉 "asyncio"
- 库源码中零 `asyncio` 引用（`grep -r "async def\|await \|import asyncio" xense_franka/` = 0 命中）
- `FrankaController` 添加 `__enter__` / `__exit__` 上下文管理器

### Examples 更新

- **Category A**（~10 个文件）：去掉 `import asyncio`、`async def main()` → `def main()`、`await` → 直接调用、`asyncio.run(main())` → `main()`
- **Category B**（2 个 WebSocket server）：保留 `asyncio.run()` 给 WebSocket 事件循环，controller 调用去掉 `await`
- **Category C**（`tracker_example.py` 等）：改用 `FrankaController` 替代 `SyncFrankaController`

---

## Installation

```bash
git clone https://github.com/xensedyl/xense_franka.git
cd xense_franka
pip install -e .
```

## Quick Start

```python
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")
controller = FrankaController(robot)
controller.start()
controller.move()   # Ruckig 轨迹规划到默认位姿
controller.stop()
```

也可以用 context manager：

```python
robot = RobotInterface("192.168.99.111")
with FrankaController(robot) as controller:
    controller.move()
```

## Architecture

```
User Thread                          Control Thread (1kHz)
─────────────                        ────────────────────
controller.start()  ──────────────►  _loop() starts
controller.switch("osc")             │
controller.set_cartesian_gains(...)   │  read state
controller.set_cartesian_reference()  │  smooth gains
  │                                   │  compute torque
  │  write to lock-free handle ──────►│  read from handle
  │                                   │  saturate_torque_rate
  │                                   │  robot.step(tau)
controller.stop()   ──────────────►  _loop() exits
```

核心设计：
- **Lock-free double-buffer handles**：用户线程写 inactive buffer → GIL-atomic flip → 控制线程读 active buffer
- **Exponential gain smoothing**：gains 在控制线程以 `gains_time_constant`（默认 0.1s）指数平滑，避免力矩跳变
- **Torque rate limiting**：`saturate_torque_rate` 限制每 tick 力矩变化 ≤ `max_delta_tau`（默认 0.5 Nm）

## Project Structure

```
xense_franka/
├── __init__.py          # 公开 API 导出，版本号
├── robot.py             # RobotInterface — pylibfranka FCI 封装
├── controller.py        # FrankaController — 1kHz 控制循环 + 同步命令 API
├── trackers.py          # Tracker 上下文管理器（Joint/Cartesian/Exponential）
├── handles.py           # Lock-free double-buffer handles
├── references.py        # 数据类：JointReference, CartesianReference, Gains
├── constants.py         # FR3 关节/力矩限位常量
├── torque_utils.py      # 工具函数（critical_damping, saturate_torque_rate 等）
└── client.py            # Franka Desk HTTP 客户端（解锁/锁定）
```

## Controllers

### 1. Joint Impedance Control

`τ = Kp * (q_d - q) + Kd * (dq_d - dq) + τ_ff + coriolis`

```python
controller.switch("impedance")
controller.set_joint_gains(stiffness=80.0)  # damping 自动计算（临界阻尼）

for i in range(100):
    controller.set_joint_reference(target_q)
```

### 2. Cartesian Impedance Control (OSC)

`τ = J^T @ (-K @ error - D @ twist) + nullspace + coriolis`

```python
controller.switch("osc")
controller.set_cartesian_gains(
    stiffness=[600, 600, 600, 50, 50, 50],
    nullspace_stiffness=5.0,
)

initial_ee = controller.get_ee_pose()
for i in range(200):
    target = initial_ee.copy()
    target[1, 3] += np.sin(i / 50.0 * np.pi) * 0.05
    controller.set_cartesian_reference(pose=target, nullspace_target=controller.initial_qpos)
```

Gains 参考值：

| 场景 | stiffness |
|------|-----------|
| 精确跟踪 | `[600, 600, 600, 50, 50, 50]` |
| 遥操作 | `[300, 300, 300, 30, 30, 30]` |
| 柔顺交互 | `[100, 100, 100, 10, 10, 10]` |

### 3. PID Control

```python
controller.switch("pid")
controller.set_joint_gains(stiffness=160.0)
```

### 4. Direct Torque Control

```python
controller.switch("torque")
controller.torque = np.array([0, 0, 0, 0, 0, 0, 0.5])
```

## Trackers

Tracker 是上下文管理器，自动切换控制模式并在退出时恢复：

```python
robot = RobotInterface("192.168.99.111")
with FrankaController(robot) as ctrl:
    ctrl.move()

    with ctrl.cartesian_tracker(stiffness=[600, 600, 600, 50, 50, 50]) as t:
        while t.ok():
            ee = ctrl.get_ee_pose()
            ee[:3, 3] += [0.001, 0, 0]
            t.set_target(ee)
            t.tick(dt=0.02)

    with ctrl.joint_tracker(stiffness=100) as t:
        t.set_target(ctrl.get_joint_positions())
        while t.ok():
            t.tick()
```

## Acknowledgments

- Built on [libfranka](https://frankarobotics.github.io/docs/) by Franka Emika
- Trajectory generation with [Ruckig](https://github.com/pantor/ruckig)
