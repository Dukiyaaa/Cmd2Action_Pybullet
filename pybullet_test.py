import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np

# 导入必要的模块
import time
import numpy as np
import math

useNullSpace = 1
ikSolver = 0
pandaEndEffectorIndex = 11
pandaNumDofs = 7

ll = [-7] * pandaNumDofs
ul = [7] * pandaNumDofs
jr = [7] * pandaNumDofs
jointPositions = [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02]
rp = jointPositions
from PIL import Image
import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import matplotlib.pyplot as plt


class PandaSim(object):
    def __init__(self, bullet_client, offset):
        self.bullet_client = bullet_client
        self.bullet_client.setPhysicsEngineParameter(solverResidualThreshold=0)
        self.offset = np.array(offset)

        flags = self.bullet_client.URDF_ENABLE_CACHED_GRAPHICS_SHAPES
        self.legos = []

        # 加载托盘和乐高积木
        self.bullet_client.loadURDF("tray/traybox.urdf", [0 + offset[0], 0 + offset[1], -0.6 + offset[2]],
                                    [-0.5, -0.5, -0.5, 0.5], flags=flags)
        self.legos.append(
            self.bullet_client.loadURDF("lego/lego.urdf", np.array([0.1, 0.3, -0.5]) + self.offset, flags=flags))
        self.bullet_client.changeVisualShape(self.legos[0], -1, rgbaColor=[1, 0, 0, 1])
        self.legos.append(
            self.bullet_client.loadURDF("lego/lego.urdf", np.array([-0.1, 0.3, -0.5]) + self.offset, flags=flags))
        self.legos.append(
            self.bullet_client.loadURDF("lego/lego.urdf", np.array([0.1, 0.3, -0.7]) + self.offset, flags=flags))

        # 加载小球
        self.sphereId = self.bullet_client.loadURDF("sphere_small.urdf", np.array([0, 0.3, -0.6]) + self.offset,
                                                    flags=flags)
        self.bullet_client.loadURDF("sphere_small.urdf", np.array([0, 0.3, -0.5]) + self.offset, flags=flags)
        self.bullet_client.loadURDF("sphere_small.urdf", np.array([0, 0.3, -0.7]) + self.offset, flags=flags)

        orn = [-0.707107, 0.0, 0.0, 0.707107]
        eul = self.bullet_client.getEulerFromQuaternion([-0.5, -0.5, -0.5, 0.5])

        # 加载Panda机器人
        self.panda = self.bullet_client.loadURDF("franka_panda/panda.urdf", np.array([0, 0, 0]) + self.offset, orn,
                                                 useFixedBase=True, flags=flags)

        index = 0
        self.state = 0
        self.control_dt = 1. / 240.
        self.finger_target = 0
        self.gripper_height = 0.2

        c = self.bullet_client.createConstraint(self.panda,
                                                9,
                                                self.panda,
                                                10,
                                                jointType=self.bullet_client.JOINT_GEAR,
                                                jointAxis=[1, 0, 0],
                                                parentFramePosition=[0, 0, 0],
                                                childFramePosition=[0, 0, 0])
        self.bullet_client.changeConstraint(c, gearRatio=-1, erp=0.1, maxForce=50)

        for j in range(self.bullet_client.getNumJoints(self.panda)):
            self.bullet_client.changeDynamics(self.panda, j, linearDamping=0, angularDamping=0)
            info = self.bullet_client.getJointInfo(self.panda, j)
            jointName = info[1]
            jointType = info[2]
            if jointType == self.bullet_client.JOINT_PRISMATIC:
                self.bullet_client.resetJointState(self.panda, j, jointPositions[index])
                index += 1
            if jointType == self.bullet_client.JOINT_REVOLUTE:
                self.bullet_client.resetJointState(self.panda, j, jointPositions[index])
                index += 1
        self.t = 0.0

        # 添加虚拟摄像头初始位置
        self.camera_distance = 0.05  # 摄像机距离末端的位置
        self.camera_yaw = 90  # 摄像机的偏航角
        self.camera_pitch = 0  # 摄像机的俯仰角
        self.camera_target_position = [0, 0, 0]  # 摄像机目标位置初始化

        # 定义虚拟摄像头参数
        self.width = 640
        self.height = 480
        self.fov = 80  # 视场角
        self.aspect = self.width / self.height
        self.near = 0.02
        self.far = 1.0

    def reset(self):
        # 重置所有物品的位置
        self.bullet_client.resetBasePositionAndOrientation(self.legos[0], np.array([0.1, 0.3, -0.5]) + self.offset,
                                                           [0, 0, 0, 1])
        self.bullet_client.resetBasePositionAndOrientation(self.legos[1], np.array([-0.1, 0.3, -0.5]) + self.offset,
                                                           [0, 0, 0, 1])
        self.bullet_client.resetBasePositionAndOrientation(self.legos[2], np.array([0.1, 0.3, -0.7]) + self.offset,
                                                           [0, 0, 0, 1])
        self.bullet_client.resetBasePositionAndOrientation(self.sphereId, np.array([0, 0.3, -0.6]) + self.offset,
                                                           [0, 0, 0, 1])

    def update_camera(self):
        # 获取末端执行器的位置和方向
        end_effector_state = self.bullet_client.getLinkState(self.panda, pandaEndEffectorIndex)
        end_effector_pos = end_effector_state[0]
        camera_target_position = list(end_effector_pos)
        camera_target_position[1] = camera_target_position[1] - 0.05
        camera_target_position[2] = camera_target_position[2]

        # 显示全局坐标系
        self.bullet_client.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], 2, 0)  # X轴，红色
        self.bullet_client.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], 2, 0)  # Y轴，绿色
        self.bullet_client.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], 2, 0)  # Z轴，蓝色

        # 显示虚拟相机位置
        camera_position = [
            camera_target_position[0] - self.camera_distance * np.cos(np.radians(self.camera_yaw)),
            camera_target_position[1] - self.camera_distance * np.sin(np.radians(self.camera_yaw)),
            camera_target_position[2] - self.camera_distance * np.sin(np.radians(self.camera_pitch))
        ]
        self.bullet_client.addUserDebugLine(camera_position, camera_target_position, [1, 1, 0], 2, 0)  # 相机视线，黄色
        self.bullet_client.addUserDebugText("Camera", camera_position, [1, 1, 0], textSize=1.5)

        # 获取虚拟摄像头视角图像
        view_matrix = self.bullet_client.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=camera_target_position,
            distance=self.camera_distance,
            yaw=self.camera_yaw,
            pitch=self.camera_pitch,
            roll=0,  # 如有需要可以调整
            upAxisIndex=2  # 确保Z轴朝上
        )

        proj_matrix = self.bullet_client.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=self.aspect,
            nearVal=self.near,
            farVal=self.far
        )

        # 捕获图像 (RGB, 深度和分割图)
        _, _, rgb_img, depth_img, seg_img = self.bullet_client.getCameraImage(
            self.width, self.height, viewMatrix=view_matrix, projectionMatrix=proj_matrix
        )

        # 将RGB图像数据转换为NumPy数组并确保格式正确
        rgb_array = np.array(rgb_img).astype(np.uint8)
        rgb_array = rgb_array[:, :, :3]  # 确保图像形状为 (宽度, 高度, 3通道)

        # 将NumPy数组转换为PIL图像并保存
        img = Image.fromarray(rgb_array)
        img.save("camera_view_corrected.png")  # 保存图像

    def update_state(self):
        keys = self.bullet_client.getKeyboardEvents()
        if len(keys) > 0:
            for k, v in keys.items():
                if v & self.bullet_client.KEY_WAS_TRIGGERED:
                    if k == ord('1'):
                        self.state = 1
                    if k == ord('2'):
                        self.state = 2
                    if k == ord('3'):
                        self.state = 3
                    if k == ord('4'):
                        self.state = 4
                    if k == ord('5'):
                        self.state = 5
                    if k == ord('6'):
                        self.state = 6
                if v & self.bullet_client.KEY_WAS_RELEASED:
                    self.state = 0

    def step(self):
        # 更新摄像头
        self.update_camera()

        # 根据机器人的当前状态，设置夹爪的目标位置
        print(self.state)
        if self.state == 6:
            self.finger_target = 0.01  # 设置夹爪目标位置
        if self.state == 5:
            self.finger_target = 0.04  # 设置夹爪目标位置

        # 向 bullet_client 提交一个名为“step”的性能计时器，用于性能分析
        self.bullet_client.submitProfileTiming("step")

        # 调用 update_state 方法更新机器人的状态
        self.update_state()

        alpha = 0.9  # 夹爪高度的平滑因子

        # 根据当前状态，平滑更新夹爪的高度
        if self.state in [1, 2, 3, 4, 7]:
            self.gripper_height = alpha * self.gripper_height + (1. - alpha) * 0.03  # 平滑更新夹爪高度
            if self.state in [2, 3, 7]:
                self.gripper_height = alpha * self.gripper_height + (1. - alpha) * 0.2  # 平滑更新夹爪高度

            # 获取当前时间并更新时间
            t = self.t
            self.t += self.control_dt

            # 根据当前时间和偏移量计算目标位置
            pos = [
                self.offset[0] + 0.2 * math.sin(1.5 * t),
                self.offset[1] + self.gripper_height,
                self.offset[2] + -0.6 + 0.1 * math.cos(1.5 * t)
            ]

            # 处理特定状态下的目标位置更新
            if self.state in [3, 4]:
                pos, o = self.bullet_client.getBasePositionAndOrientation(self.legos[0])  # 获取乐高块的位置和姿态
                pos = [pos[0], self.gripper_height, pos[2]]  # 更新目标位置
                self.prev_pos = pos  # 保存当前位置
            if self.state == 7:
                pos = self.prev_pos  # 获取之前的位置
                diffX = pos[0] - self.offset[0]  # 计算x方向的差异
                diffZ = pos[2] - (self.offset[2] - 0.6)  # 计算z方向的差异
                self.prev_pos = [self.prev_pos[0] - diffX * 0.1, self.prev_pos[1],
                                 self.prev_pos[2] - diffZ * 0.1]  # 更新之前的位置

            # 使用欧拉角计算目标姿态
            orn = self.bullet_client.getQuaternionFromEuler([math.pi / 2., 0., 0.])
            # 提交“IK”（逆运动学）的性能计时
            self.bullet_client.submitProfileTiming("IK")

            # 计算逆运动学以获得关节位置
            jointPoses = self.bullet_client.calculateInverseKinematics(
                self.panda, pandaEndEffectorIndex, pos, orn, ll, ul, jr, rp, maxNumIterations=20
            )
            # 提交性能计时
            self.bullet_client.submitProfileTiming()

            # 设置关节电机控制，使机器人移动到计算出的关节位置
            for i in range(pandaNumDofs):
                self.bullet_client.setJointMotorControl2(
                    self.panda, i, self.bullet_client.POSITION_CONTROL, jointPoses[i], force=5 * 240.
                )

        # 设置夹爪电机的控制，使夹爪移动到目标位置
        for i in [9, 10]:
            self.bullet_client.setJointMotorControl2(
                self.panda, i, self.bullet_client.POSITION_CONTROL, self.finger_target, force=10
            )

        # 提交性能计时
        self.bullet_client.submitProfileTiming()


# 定义PandaSimAuto类，继承自PandaSim类
class PandaSimAuto(PandaSim):
    def __init__(self, bullet_client, offset):
        PandaSim.__init__(self, bullet_client, offset)
        self.state_t = 0
        self.cur_state = 0
        self.states = [0, 3, 5, 4, 6, 3, 7]
        self.state_durations = [1, 0.1, 0.1, 0.2, 0.1, 0.1, 1]

    def update_state(self):
        self.state_t += self.control_dt
        if self.state_t > self.state_durations[self.cur_state]:
            self.cur_state += 1
            if self.cur_state >= len(self.states):
                self.cur_state = 0
            self.state_t = 0
            self.state = self.states[self.cur_state]
            # 如果状态是7（抓取完成），重置所有物品并重新放置
            if self.state == 7:
                time.sleep(1)  # 等待1秒
                self.reset()  # 重置物品位置


# 如果需要创建视频，需要确保ffmpeg可用
createVideo = False
fps = 100.0
timeStep = 1.0 / fps

# 根据createVideo的值选择连接参数
if createVideo:
    p.connect(p.GUI, options="--minGraphicsUpdateTimeMs=0 --mp4=\"pybullet_grasp.mp4\" --mp4fps=" + str(fps))
else:
    p.connect(p.GUI)

# 配置可视化调试器
p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP, 1)  # 将Y轴设置为向上
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)  # 禁用GUI
p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)  # 设置每毫秒最大命令数
p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=38, cameraPitch=-22,
                             cameraTargetPosition=[0.35, -0.13, 0])  # 重置摄像机视角
p.setAdditionalSearchPath(pd.getDataPath())  # 设置额外的搜索路径

# 设置时间步长和重力
p.setTimeStep(timeStep)
p.setGravity(0, -9.8, 0)
# 初始化Panda机器人
panda = PandaSimAuto(p, [0, 0, 0])
panda.control_dt = timeStep

# 开始状态日志记录
logId = panda.bullet_client.startStateLogging(panda.bullet_client.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
panda.bullet_client.submitProfileTiming("start")

# 执行仿真步骤，共执行100次抓取
for iteration in range(100):
    # 重置仿真和状态
    panda.reset()
    panda.state_t = 0
    panda.cur_state = 0
    panda.state = panda.states[panda.cur_state]

    while panda.cur_state < len(panda.states):
        panda.bullet_client.submitProfileTiming("full_step")
        panda.step()  # 执行Panda机器人的一步操作
        p.stepSimulation()  # 执行物理仿真一步
        time.sleep(timeStep)  # 如果不创建视频，则按照时间步长暂停
        panda.bullet_client.submitProfileTiming()  # 提交性能计时数据

