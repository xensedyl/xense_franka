# Franka Desk 客户端

`xense_franka.client` 提供 Franka Desk HTTP/HTTPS API 的简单封装，用于登录、请求 control token、解锁/上锁、激活 FCI 和 gripper homing。

导入：

```python
from xense_franka.client import FrankaLockUnlock
```

## 初始化

```python
client = FrankaLockUnlock(
    hostname="192.168.99.111",
    username="admin",
    password="your-password",
    protocol="https",
    relock=False,
)
```

参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `hostname` | 必填 | 机器人 IP 或 hostname |
| `username` | 必填 | Franka Desk 用户名 |
| `password` | 必填 | Franka Desk 密码 |
| `protocol` | `"https"` | `http` 或 `https` |
| `relock` | `False` | 进程退出时是否自动上锁 |

## run()

```python
client.run(
    unlock=True,
    force=False,
    wait=False,
    request=False,
    persistent=True,
    fci=True,
    home=False,
)
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `unlock` | `True` 解锁，`False` 上锁 |
| `force` | 强制开/关 brakes |
| `wait` | 等待当前控制权释放或用户确认 |
| `request` | 请求 physical access，需要按机器人按钮确认 |
| `persistent` | 保持 token 和登录状态，便于后续脚本控制 |
| `fci` | 激活 Franka Control Interface |
| `home` | 执行 gripper homing |

## 典型用法

解锁并激活 FCI：

```python
client = FrankaLockUnlock("192.168.99.111", "admin", "your-password")
client.run(unlock=True, fci=True, persistent=True)
```

请求 physical access：

```python
client.run(unlock=True, request=True, wait=True, persistent=True)
```

上锁：

```python
client.run(unlock=False)
```

## 注意事项

- 同一时间只有一个用户能持有 control token。
- 默认禁用 HTTPS 证书校验，以兼容 Franka 自签名证书。
- `fci=True` 要求 `unlock=True` 且 `persistent=True`。
- `home=True` 要求 `unlock=True`。
- 如果 `persistent=False`，`run()` 结束时会释放 token 并退出登录。
