# xense_franka

**xense_franka** provides a synchronous Python API for Franka Emika robots. It uses **`pylibfranka`** for the official low-level torque interface and runs the 1kHz impedance loop in a dedicated background thread, while user code publishes references from the main thread.

The library is designed for research applications requiring precise, real-time control with minimal latency and maximum flexibility.

This library is inspired by aiofranka and reworked around a `franky`-style tracking controller model.

## Installation

Make sure you can access Franka Desk GUI from your machine's browser by typing in the robot's IP (e.g. 172.16.0.2). Then, install:

```bash
git clone https://github.com/xensedyl/xense_franka.git
cd xense_franka
pip install -e .
```

## Quick Start

```bash
python test.py
```

Basic usage pattern:

```python
import time
import numpy as np
from xense_franka import RobotInterface, FrankaController

# Connect to robot (use IP address for real robot, None for simulation)
robot = RobotInterface("172.16.0.2")
controller = FrankaController(robot)

# Start the 1kHz control loop
controller.start()

# Move to home position using smooth trajectory
controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

# Switch to impedance control and send targets
controller.switch("impedance")
controller.set_joint_gains(80.0)
controller.set_freq(50)

for cnt in range(100):
    delta = np.sin(cnt / 50.0 * np.pi) * 0.1
    controller.set_joint_reference(delta + controller.initial_qpos)

# Stop control loop
controller.stop()
```

也可以用 context manager：

```python
robot = RobotInterface("172.16.0.2")
with FrankaController(robot) as controller:
    controller.move()
    # ...
```

## Core Concepts

### Synchronous API

所有公开方法都是同步的。1kHz 力矩控制循环运行在独立后台线程中，用户线程通过 `set_freq()` 控制发布节奏：

```python
controller.start()          # 启动 1kHz 控制循环
controller.move()           # 轨迹规划移动到目标
controller.set("q_desired", target)  # 发布新目标（自动限频）
controller.stop()           # 停止
```

### Rate Limiting

Use `set_freq()` to pace how often your publisher emits new references:

```python
controller.set_freq(50)  # Set 50Hz update rate

# set / set_joint_reference / set_cartesian_reference
# will automatically sleep to maintain 50Hz timing
for i in range(100):
    controller.set_joint_reference(compute_target())
```

### State Access

Robot state is continuously updated at 1kHz and accessible via `controller.state`:

```python
state = controller.state  # Thread-safe access
# Contains: qpos, qvel, ee, jac, mm, last_torque, ext_wrench, coriolis
print(f"Joint positions: {state['qpos']}")
print(f"End-effector pose: {state['ee']}")  # 4x4 homogeneous transform
```

## Controllers

### 1. Joint Impedance Control

Controls joint positions with a spring-damper law:  `τ = Kp * (q_d - q) + Kd * (dq_d - dq) + τ_ff + coriolis`

```python
controller.switch("impedance")
controller.set_joint_gains(stiffness=80.0)  # damping auto-calculated (critical damping)
controller.set_freq(50)

for i in range(100):
    controller.set_joint_reference(target_q)
```

**Use case**: Precise joint-space motions, compliant behavior

### 2. Cartesian Impedance Control (OSC)

Controls end-effector pose with operational space control:  `τ = J^T @ (-K @ error - D @ twist) + nullspace + coriolis`

```python
import time
import numpy as np
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")
controller = FrankaController(robot)
controller.start()

# 1. Move to a safe starting position
controller.move([0, 0, 0, -1.57079, 0, 1.57079, 0.7853])
time.sleep(1.0)

# 2. Switch to Cartesian impedance control
controller.switch("osc")

# 3. Set gains
#    stiffness: [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad]
#    damping defaults to critical damping: 2*sqrt(K)
controller.set_cartesian_gains(
    stiffness=[600, 600, 600, 50, 50, 50],
    nullspace_stiffness=5.0,
)
controller.set_freq(50)

# 4. Record initial pose
initial_ee = controller.get_current_ee_pose()

# 5. Send Cartesian targets
for i in range(200):
    target_ee = initial_ee.copy()
    target_ee[1, 3] += np.sin(i / 50.0 * np.pi) * 0.05  # Y direction ±5cm

    controller.set_cartesian_reference(
        pose=target_ee,
        nullspace_target=controller.initial_qpos,  # keep joints near initial config
    )

controller.stop()
```

**Use case**: Cartesian trajectories, end-effector tracking, teleoperation

#### Cartesian gains explained

| Parameter | Type | Description |
|-----------|------|-------------|
| `stiffness` | 6-dim | 前 3 维是平移刚度 (N/m)，后 3 维是旋转刚度 (Nm/rad) |
| `damping` | 6-dim or None | 默认自动计算临界阻尼 `2*sqrt(K)`，也可手动指定 |
| `damping_ratio` | float | 阻尼比，1.0 = 临界阻尼（默认），<1.0 欠阻尼，>1.0 过阻尼 |
| `nullspace_stiffness` | float | 零空间刚度，控制冗余关节偏好（默认 5.0） |

常用刚度参考值：
- **高刚度** (精确跟踪): `[600, 600, 600, 50, 50, 50]`
- **中等刚度** (遥操作): `[300, 300, 300, 30, 30, 30]`
- **低刚度** (柔顺交互): `[100, 100, 100, 10, 10, 10]`

### 3. PID Control

Joint-space PID with integral term:

```python
controller.switch("pid")
controller.set_joint_gains(stiffness=160.0)
```

### 4. Direct Torque Control

Bypass all impedance logic and send raw torques:

```python
controller.switch("torque")
controller.torque = np.array([0, 0, 0, 0, 0, 0, 0.5])  # Nm
```

## Trackers

Trackers are context managers that handle mode switching and gain restore automatically:

```python
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")
with FrankaController(robot) as ctrl:
    ctrl.move()

    # Cartesian impedance tracker
    with ctrl.cartesian_tracker(stiffness=[600, 600, 600, 50, 50, 50]) as t:
        while t.ok():
            ee = ctrl.get_ee_pose()
            ee[:3, 3] += [0.001, 0, 0]
            t.set_target(ee)
            t.tick(dt=0.02)  # 50Hz

    # Joint impedance tracker
    with ctrl.joint_tracker(stiffness=100) as t:
        t.set_target(ctrl.get_joint_positions())
        while t.ok():
            t.tick()
```

## Acknowledgments

- Built on [libfranka](https://frankarobotics.github.io/docs/) by Franka Emika
- Trajectory generation with [Ruckig](https://github.com/pantor/ruckig)
