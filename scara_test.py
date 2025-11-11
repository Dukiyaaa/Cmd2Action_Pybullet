import pybullet as p
import pybullet_data
import time
import math

# 定义宏
action_time = 250
# 连接PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# 加载地面和平面
planeId = p.loadURDF("plane.urdf")

# 设置摄像机视角
# p.resetDebugVisualizerCamera(
#     cameraDistance=1.2,  # 摄像机距离目标点的距离
#     cameraYaw=90,  # 摄像机绕目标点的水平旋转角度（度）
#     cameraPitch=-30,  # 摄像机绕目标点的俯仰角（度）
#     cameraTargetPosition=[0, 0, 0.2]  # 摄像机观察的目标点坐标
# )

# 加载scara机械臂
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("my_scara.urdf", startPos, startOrientation, useFixedBase=True)

# 加载红色小方块
cube_pos = [-0.8, 0, 0.05]
cube_orientation = p.getQuaternionFromEuler([0, 0, 0])
cube_id = p.loadURDF("cube_small.urdf", cube_pos, cube_orientation)
p.changeVisualShape(cube_id, -1, rgbaColor=[1, 0, 0, 1])

# 保持一段时间观察
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1. / 240.)

p.disconnect()