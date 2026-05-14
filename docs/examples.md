# 示例

项目示例位于 `examples/` 目录。运行前请检查脚本中的机器人 IP，并确认 Franka Desk 已解锁且 FCI 已激活。

## 基础示例

| 文件 | 说明 |
| --- | --- |
| `cartesian_impedance_example.py` | OSC 笛卡尔阻抗控制，执行 XY 平面方形轨迹 |
| `tracker_example.py` | 演示三种 Tracker |
| `gamepad_teleop_example.py` | 手柄遥操作示例 |
| `02_spacemouse_teleop.py` | SpaceMouse 末端遥操作 |
| `04_replay.py` | 轨迹回放 |
| `06_sysid.py` / `06_sysid_all.py` | 系统辨识数据采集 |

## 运行方式

```bash
python examples/cartesian_impedance_example.py
```

如果当前环境不是项目根目录，请先进入仓库：

```bash
cd /home/xense/pylibfranka_controllers/xense_franka
python examples/tracker_example.py
```

## 笛卡尔阻抗示例

核心流程：

```python
robot = RobotInterface("192.168.99.111")
controller = FrankaController(robot)

controller.start()
try:
    controller.move()
    controller.switch("osc")
    controller.set_cartesian_gains([600, 600, 600, 50, 50, 50])
    controller.set_freq(50)

    initial_ee = controller.get_ee_pose()
    target_ee = initial_ee.copy()
    target_ee[0, 3] += 0.05
    controller.set_cartesian_reference(target_ee)
finally:
    controller.stop()
```

## Tracker 示例

```python
with FrankaController(robot) as ctrl:
    ctrl.move()

    with ctrl.cartesian_tracker(stiffness=[600, 600, 600, 50, 50, 50]) as t:
        ee = ctrl.get_ee_pose()
        ee[0, 3] += 0.05
        t.set_target(ee)

        for _ in range(50):
            t.tick(0.02)
```

## WebSocket 遥操作

仓库中包含 WebSocket server/client：

| 文件 | 说明 |
| --- | --- |
| `ws_teleop_server.py` | 笛卡尔遥操作服务端 |
| `ws_teleop_client.py` | 笛卡尔遥操作客户端 |
| `ws_joint_teleop_server.py` | 关节遥操作服务端 |
| `ws_joint_teleop_client.py` | 关节遥操作客户端 |

这些脚本可能依赖额外包，例如 `websockets`、手柄或 SpaceMouse 相关库。运行前请根据 import 报错补齐依赖。

## 注意

部分示例来自早期实验脚本，可能包含固定 IP、固定输出目录或特定硬件依赖。建议先阅读脚本开头的参数和 import，再连接真实机器人运行。
