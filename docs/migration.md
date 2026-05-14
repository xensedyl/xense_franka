# 迁移指南

`xense_franka` v0.4.0 的主要变化是公开 API 从 async 改为同步。旧代码中 `await`、`async def main()` 和 `asyncio.run()` 需要移除。

## Async 改同步

旧写法：

```python
import asyncio
from xense_franka import RobotInterface, FrankaController

async def main():
    robot = RobotInterface("192.168.99.111")
    controller = FrankaController(robot)
    await controller.start()
    await controller.move()
    await controller.set("q_desired", target)
    await controller.stop()

asyncio.run(main())
```

新写法：

```python
from xense_franka import RobotInterface, FrankaController

def main():
    robot = RobotInterface("192.168.99.111")
    controller = FrankaController(robot)
    controller.start()
    try:
        controller.move()
        controller.set("q_desired", target)
    finally:
        controller.stop()

if __name__ == "__main__":
    main()
```

也可以使用上下文管理器：

```python
robot = RobotInterface("192.168.99.111")

with FrankaController(robot) as controller:
    controller.move()
```

## 删除 SyncFrankaController

`SyncFrankaController` 已删除。现在 `FrankaController` 本身就是同步接口。

旧写法：

```python
from xense_franka import SyncFrankaController

with SyncFrankaController("192.168.99.111") as ctrl:
    ctrl.move()
```

新写法：

```python
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")
with FrankaController(robot) as ctrl:
    ctrl.move()
```

## Tracker 改同步 with

旧的 async context manager 已删除：

```python
async with ctrl.cartesian_tracker(stiffness=300) as t:
    ...
```

改为：

```python
with ctrl.cartesian_tracker(stiffness=300) as t:
    ...
```

## set_freq 默认不限频

v0.4.0 中默认不限频：

```python
ctrl.set_joint_reference(target)
```

如果需要固定发布节奏：

```python
ctrl.set_freq(50)
ctrl.set_joint_reference(target)
```

关闭限频：

```python
ctrl.set_freq(None)
ctrl.set_freq(0)
```

## 迁移方法对照

| 旧用法 | 新用法 |
| --- | --- |
| `await controller.start()` | `controller.start()` |
| `await controller.stop()` | `controller.stop()` |
| `await controller.move()` | `controller.move()` |
| `await controller.set(...)` | `controller.set(...)` |
| `await controller.set_joint_reference(...)` | `controller.set_joint_reference(...)` |
| `await controller.set_cartesian_reference(...)` | `controller.set_cartesian_reference(...)` |
| `async with tracker` | `with tracker` |
| `SyncFrankaController` | `RobotInterface` + `FrankaController` |

## WebSocket 脚本

WebSocket server/client 仍然可能需要 `asyncio` 事件循环，但控制器调用本身不再 `await`：

```python
async def handle_message(...):
    pose = ...
    controller.set_cartesian_reference(pose)
```
