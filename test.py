import time
import numpy as np
from xense_franka import RobotInterface, FrankaController

# robot = RobotInterface("192.168.99.111")
# controller = FrankaController(robot)
# controller.start()
# controller._assert_loop_ok()
# print("[OK] start")

# controller.move([0, 0, 0, -1.57079, 0, 1.57079, 0.7853])
# controller._assert_loop_ok()
# print("[OK] move")

# time.sleep(1.0)
# controller._assert_loop_ok()
# print("[OK] sleep after move")

# controller.switch("osc")
# controller._assert_loop_ok()
# print("[OK] switch osc")

# controller.set_cartesian_gains(
#     stiffness=[600, 600, 600, 50, 50, 50],
#     nullspace_stiffness=5.0,
# )
# controller.set_freq(50)

# time.sleep(0.5)
# controller._assert_loop_ok()
# print("[OK] gains settled")

# initial_ee = controller.get_current_ee_pose()
# print(f"[OK] initial_ee position: {initial_ee[:3, 3]}")

# for i in range(2000):
#     target_ee = initial_ee.copy()
#     target_ee[1, 3] += np.sin(i / 50.0 * np.pi) * 0.05

#     controller.set_cartesian_reference(
#         pose=target_ee,
#         nullspace_target=controller.initial_qpos,
#     )
#     if i % 50 == 0:
#         print(f"[OK] step {i}")

# controller.stop()
# print("[OK] done")




robot2 = RobotInterface("192.168.99.111")
with FrankaController(robot2) as ctrl:
    ctrl.move()
    print("aaa")

    # Cartesian impedance tracker
    with ctrl.cartesian_tracker(stiffness=[600, 600, 600, 50, 50, 50],damping=[10, 10, 10, 5, 5, 5]) as t:
        while t.ok():
            ee = ctrl.get_ee_pose()
            ee[:3, 3] += [0.001, 0, 0]
            t.set_target(ee)
            t.tick(dt=0.02)  # 50Hz

    # # Joint impedance tracker
    # with ctrl.joint_tracker(stiffness=100,damping=10) as t:
    #     t.set_target(ctrl.get_joint_positions())
    #     while t.ok():
    #         t.tick()