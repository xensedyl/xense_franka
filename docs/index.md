# xense_franka

`xense_franka` 是面向 Franka Emika FR3 的同步 Python 控制 SDK。它基于 `pylibfranka` 的官方低层 FCI 力矩接口，在后台运行 1 kHz 控制循环，用户代码在主线程中同步发布关节或笛卡尔参考。

当前文档对应 `xense_franka` v0.4.0。

## 主要特性

- 同步 API：`start()`、`move()`、`set_joint_reference()`、`stop()` 等公开方法都是普通函数。
- 后台 1 kHz 控制循环：默认使用独立进程运行控制环，也可以用线程模式。
- 多种控制模式：关节阻抗、PID、笛卡尔 OSC、直接力矩。
- Tracker 上下文管理器：自动切换控制模式，并在退出时恢复之前的模式和增益。
- Ruckig 轨迹规划：`move()` 用平滑关节轨迹移动到目标位姿。
- 控制安全机制：力矩变化率限制、关节限位排斥、增益指数平滑、崩溃诊断环形缓存。

## 最小示例

```python
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")

with FrankaController(robot) as controller:
    controller.move()
```

## 适用场景

- FR3 真实机器人上的阻抗控制实验
- 末端位姿轨迹跟踪和遥操作
- 不同刚度/阻尼组合的系统辨识
- 需要在 Python 中同步编写控制逻辑的研究原型

## 项目结构

```text
xense_franka/
├── robot.py          # RobotInterface: pylibfranka FCI 封装
├── controller.py     # FrankaController: 控制循环和同步命令 API
├── trackers.py       # Tracker 上下文管理器
├── handles.py        # lock-free double-buffer handles
├── references.py     # reference/gains 数据结构
├── constants.py      # FR3 关节和力矩限位常量
├── torque_utils.py   # 力矩计算工具函数
└── client.py         # Franka Desk HTTP/HTTPS 客户端
```

## 下一步

- [快速开始](quickstart.md)：安装、机器人准备、运行第一个脚本。
- [控制模式](guide/controllers.md)：理解 impedance、osc、pid、torque 的使用方式。
- [API 总览](api/index.md)：查看核心类和数据结构。
- [示例](examples.md)：根据 `examples/` 目录选择可运行脚本。
