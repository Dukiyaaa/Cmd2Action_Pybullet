import pybullet as p
import pybullet_data
import time
import math


def get_ik_target(cube_pos, z_offset=0.2):
    # 目标点在方块正上方
    target_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + z_offset]
    # 判断主要方向
    x, y = cube_pos[0], cube_pos[1]
    if abs(x) >= abs(y):  # 主要在x轴方向
        if x >= 0:
            orn = p.getQuaternionFromEuler([math.pi, 0, 0])  # X正
        else:
            orn = p.getQuaternionFromEuler([math.pi, 0, math.pi])  # X负
    else:  # 主要在y轴方向
        if y >= 0:
            orn = p.getQuaternionFromEuler([math.pi, 0, math.pi / 2])  # Y正
        else:
            orn = p.getQuaternionFromEuler([math.pi, 0, -math.pi / 2])  # Y负
    return target_pos, orn


# 定义宏
action_time = 250
# 连接PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# 加载地面和平面
planeId = p.loadURDF("plane.urdf")

# 设置摄像机视角
p.resetDebugVisualizerCamera(
    cameraDistance=1.2,  # 摄像机距离目标点的距离
    cameraYaw=90,  # 摄像机绕目标点的水平旋转角度（度）
    cameraPitch=-30,  # 摄像机绕目标点的俯仰角（度）
    cameraTargetPosition=[0, 0, 0.2]  # 摄像机观察的目标点坐标
)

# 加载Panda机械臂
startPos = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("franka_panda/panda.urdf", startPos, startOrientation, useFixedBase=True)

# 加载红色小方块
cube_pos = [0, 0.4, 0.05]
cube_orientation = p.getQuaternionFromEuler([0, 0, 0])
cube_id = p.loadURDF("cube_small.urdf", cube_pos, cube_orientation)
p.changeVisualShape(cube_id, -1, rgbaColor=[1, 0, 0, 1])

# 夹爪张开
for _ in range(100):
    # 0.4是最大张开角度
    p.setJointMotorControl2(boxId, 9, p.POSITION_CONTROL, targetPosition=0.04, force=100)
    p.setJointMotorControl2(boxId, 10, p.POSITION_CONTROL, targetPosition=0.04, force=100)
    p.stepSimulation()  # 加了这一步，仿真才会进行
    time.sleep(1. / 240.)

# 1. 机械臂移动到方块正上方
# target_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.2]  # 方块正上方
# # target_orn = p.getQuaternionFromEuler([math.pi, 0, 0])
# # target_orn = p.getQuaternionFromEuler([0, 0, 0])
# target_orn = p.getQuaternionFromEuler([math.pi, 0, math.pi/2])
# joint_poses = p.calculateInverseKinematics(boxId, 11, target_pos, target_orn)

target_pos, target_orn = get_ik_target(cube_pos)
joint_poses = p.calculateInverseKinematics(boxId, 11, target_pos, target_orn)
for _ in range(action_time):
    for j in range(7):
        p.setJointMotorControl2(boxId, j, p.POSITION_CONTROL, targetPosition=joint_poses[j], force=1000)

    # 这里再次将夹爪打开，是为了防止在物理仿真中的扰动干扰了夹爪之前的状态
    p.setJointMotorControl2(boxId, 9, p.POSITION_CONTROL, targetPosition=0.04, force=100)
    p.setJointMotorControl2(boxId, 10, p.POSITION_CONTROL, targetPosition=0.04, force=100)
    p.stepSimulation()
    time.sleep(1. / 240.)

# 2. 机械臂下降到方块两侧
# target_pos = [cube_pos[0], cube_pos[1], cube_pos[2] - 0.02]
# joint_poses = p.calculateInverseKinematics(boxId, 11, target_pos, target_orn)

target_pos, target_orn = get_ik_target(cube_pos, z_offset=-0.02)
joint_poses = p.calculateInverseKinematics(boxId, 11, target_pos, target_orn)
for _ in range(action_time):
    for j in range(7):
        p.setJointMotorControl2(boxId, j, p.POSITION_CONTROL, targetPosition=joint_poses[j], force=1000)
    p.setJointMotorControl2(boxId, 9, p.POSITION_CONTROL, targetPosition=0.04, force=100)
    p.setJointMotorControl2(boxId, 10, p.POSITION_CONTROL, targetPosition=0.04, force=100)
    p.stepSimulation()
    time.sleep(1. / 240.)

# 3. 闭合夹爪夹住方块（force调小，targetPosition不设为0）
for _ in range(action_time):
    for j in range(7):
        p.setJointMotorControl2(boxId, j, p.POSITION_CONTROL, targetPosition=joint_poses[j], force=1000)
    p.setJointMotorControl2(boxId, 9, p.POSITION_CONTROL, targetPosition=0.012, force=30)
    p.setJointMotorControl2(boxId, 10, p.POSITION_CONTROL, targetPosition=0.012, force=30)
    p.stepSimulation()
    time.sleep(1. / 240.)

# 闭合后多等待一段时间
for _ in range(100):
    p.stepSimulation()
    time.sleep(1. / 240.)

# 4. 抬升机械臂带起方块（分多步慢慢升高）
for step in range(action_time):
    # lift_z = 0.8  # 从当前高度平滑升到高处
    # target_pos = [cube_pos[0], cube_pos[1], lift_z]
    # joint_poses = p.calculateInverseKinematics(boxId, 11, target_pos, target_orn)
    target_pos, target_orn = get_ik_target(cube_pos, z_offset=0.5)
    joint_poses = p.calculateInverseKinematics(boxId, 11, target_pos, target_orn)
    for j in range(7):
        p.setJointMotorControl2(boxId, j, p.POSITION_CONTROL, targetPosition=joint_poses[j], force=1000)
    p.setJointMotorControl2(boxId, 9, p.POSITION_CONTROL, targetPosition=0.012, force=30)
    p.setJointMotorControl2(boxId, 10, p.POSITION_CONTROL, targetPosition=0.012, force=30)
    p.stepSimulation()
    time.sleep(1. / 240.)

# 保持一段时间观察
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1. / 240.)

# 获取末端执行器的状态
link_state = p.getLinkState(boxId, 11)
ee_pos = link_state[0]
ee_orn = link_state[1]  # 四元数

# 转换为欧拉角
ee_euler = p.getEulerFromQuaternion(ee_orn)
print("末端执行器位置：", ee_pos)
print("末端执行器朝向（四元数）：", ee_orn)
print("末端执行器朝向（欧拉角）：", ee_euler)

p.disconnect()