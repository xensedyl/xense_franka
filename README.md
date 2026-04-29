# xense_franka

**xense_franka** provides an async-friendly Python API for Franka Emika robots. It uses **`pylibfranka`** for the official low-level torque interface and runs the 1kHz impedance loop in a dedicated background thread, while user code publishes references asynchronously.

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
import asyncio 
import numpy as np 
from xense_franka import RobotInterface, FrankaController

async def main():
    # Connect to robot (use IP address for real robot, None for simulation)
    robot = RobotInterface("172.16.0.2") 
    controller = FrankaController(robot)
    
    # Start the 1kHz control loop
    await controller.start()

    # Test connection quality
    await controller.test_connection()

    # Move to home position using smooth trajectory
    await controller.move([0, 0, 0.0, -1.57079, 0, 1.57079, -0.7853])

    # Switch to impedance control
    controller.switch("impedance")
    controller.set_joint_gains(np.ones(7) * 80.0, np.ones(7) * 4.0)
    controller.set_freq(50)  # 50Hz update rate for set() commands
    
    for cnt in range(100): 
        delta = np.sin(cnt / 50.0 * np.pi) * 0.1
        init = controller.initial_qpos
        await controller.set_joint_reference(delta + init)

    # Switch to operational space control (OSC)
    controller.switch("osc")
    controller.set_cartesian_gains(
        np.array([300, 300, 300, 1000, 1000, 1000]),
        2.0 * np.sqrt(np.array([300, 300, 300, 1000, 1000, 1000])),
        nullspace_stiffness=5.0,
    )
    controller.set_freq(50)

    for cnt in range(100): 
        delta = np.sin(cnt / 50.0 * np.pi) * 0.1
        init = controller.initial_ee 

        desired_ee = np.eye(4) 
        desired_ee[:3, :3] = init[:3, :3]
        desired_ee[:3, 3] = init[:3, 3] + np.array([0, delta, 0])

        await controller.set_cartesian_reference(desired_ee, nullspace_target=controller.initial_qpos)

    # Stop control loop
    await controller.stop()

if __name__ == "__main__":
    asyncio.run(main()) 
```

## Core Concepts

### Async Command API

The library exposes an `asyncio`-friendly API, but the torque loop itself runs in a dedicated background thread. This avoids coupling the 1kHz loop to event-loop scheduling:

```python
# Control loop runs in a background control thread at 1kHz
await controller.start()

# Your code can await other operations without blocking the control loop
await asyncio.sleep(1.0)
await controller.set_joint_reference(target)
```

### Rate Limiting

Use `set_freq()` to pace how often your async publisher emits new references:

```python
controller.set_freq(50)  # Set 50Hz update rate

# This will automatically sleep to maintain 50Hz timing
for i in range(100):
    await controller.set_joint_reference(compute_target())
```


### State Access

Robot state is continuously updated at 1kHz and accessible via `controller.state`:

```python
state = controller.state  # Thread-safe access
# Contains: qpos, qvel, ee, jac, mm, last_torque
print(f"Joint positions: {state['qpos']}")
print(f"End-effector pose: {state['ee']}")  # 4x4 homogeneous transform
```

## Controllers

### 1. Impedance Control (Joint Space)

Controls joint positions with a spring-damper law plus optional desired velocity and feedforward torque:

```python
controller.switch("impedance")
controller.set_joint_gains(np.ones(7) * 80.0, np.ones(7) * 4.0)

await controller.set_joint_reference(target_joint_angles)
```

**Use case**: Precise joint-space motions, compliant behavior


### 2. Operational Space Control (Task Space)

Controls end-effector pose with Cartesian impedance, desired twist damping, and optional nullspace posture:

```python
controller.switch("osc")
controller.set_cartesian_gains(
    np.array([300, 300, 300, 1000, 1000, 1000]),
    nullspace_stiffness=5.0,
)

desired_ee = np.eye(4)  # 4x4 homogeneous transform
desired_ee[:3, 3] = [0.4, 0.0, 0.5]  # Position
await controller.set_cartesian_reference(desired_ee, nullspace_target=controller.initial_qpos)
```

**Use case**: Cartesian trajectories, end-effector tracking


## Acknowledgments

- Built on [libfranka](https://frankarobotics.github.io/docs/) by Franka Emika
- Trajectory generation with [Ruckig](https://github.com/pantor/ruckig)
