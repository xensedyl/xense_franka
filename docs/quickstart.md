# 快速开始

## 环境要求

- Python 3.8+
- 已安装并可导入 `pylibfranka`
- Franka FCI 网络已配置完成
- 运行机器可以访问 Franka Desk，例如在浏览器中打开机器人 IP
- 已在当前 Python 环境中安装 `mkdocs-material`，用于构建本文档

安装项目：

```bash
git clone https://github.com/xensedyl/xense_franka.git
cd xense_franka
pip install -e .
```

## 准备机器人

真实机器人运行前，请确认：

1. 机器人处于可操作区域内，周围无遮挡。
2. 急停、碰撞阈值、网络延迟和 FCI 配置符合实验要求。
3. Franka Desk 中已经解锁机器人并激活 FCI。
4. 脚本中的 IP 与机器人 FCI IP 一致。

如果需要用代码请求控制权、解锁并激活 FCI，可以使用 `FrankaLockUnlock`：

```python
from xense_franka.client import FrankaLockUnlock

client = FrankaLockUnlock(
    hostname="192.168.99.111",
    username="admin",
    password="your-password",
)
client.run(unlock=True, fci=True, persistent=True)
```

## 第一个脚本

```python
from xense_franka import RobotInterface, FrankaController

ROBOT_IP = "192.168.99.111"

robot = RobotInterface(ROBOT_IP)

with FrankaController(robot) as ctrl:
    ctrl.move()
    print(ctrl.get_joint_positions())
```

`FrankaController` 支持同步上下文管理器。进入 `with` 时会调用 `start()`，退出时会调用 `stop()`。

## 手动 start/stop

```python
from xense_franka import RobotInterface, FrankaController

robot = RobotInterface("192.168.99.111")
ctrl = FrankaController(robot)

ctrl.start()
try:
    ctrl.move([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, 0.7853])
finally:
    ctrl.stop()
```

`start()` 默认使用独立进程运行控制循环。如果需要线程模式：

```python
ctrl.start(use_process=False)
```

## 发布频率

默认情况下，`set()`、`set_joint_reference()` 和 `set_cartesian_reference()` 不限频，调用会尽快返回。

```python
ctrl.set_freq(50)    # 50 Hz 发布节奏
ctrl.set_freq(None)  # 关闭限频
ctrl.set_freq(0)     # 同样关闭限频
```

`move()` 内部轨迹采样至少为 50 Hz，不受默认不限频行为影响。

## 构建文档

在你已经安装 `Material for MkDocs` 的 `yolo` 环境中：

```bash
mkdocs serve
```

浏览器打开终端输出的本地地址即可预览。生成静态站点：

```bash
mkdocs build
```
