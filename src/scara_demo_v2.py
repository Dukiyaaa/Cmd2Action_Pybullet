import time
import numpy as np
import pybullet as p
import pybullet_data
from my_kinematics import forward_kinematics, inverse_kinematics


def setup_scene(rotate_base: bool = True, cube_pos=[1.2, -0.8, 0.05]):
    """创建仿真场景并返回 (robot_id, cube_id)。"""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")
    if rotate_base:
        quat = p.getQuaternionFromEuler([0, 0, np.pi / 2])  # 绕 Z +90°
        robot = p.loadURDF("../urdf/my_scara.urdf", [0, 0, 0], quat, useFixedBase=True)
    else:
        robot = p.loadURDF("../urdf/my_scara.urdf", [0, 0, 0], useFixedBase=True)
    cube_id = p.loadURDF("cube_small.urdf", cube_pos)
    p.changeVisualShape(cube_id, -1, rgbaColor=[1, 0, 0, 1])
    p.changeDynamics(cube_id, -1, mass=0.002)
    return robot, cube_id


def set_joint_states(robot_id, theta1, theta2, d3):
    """重置关节状态到指定值。"""
    p.resetJointState(robot_id, 1, theta1)  # rotation1
    p.resetJointState(robot_id, 2, theta2)  # rotation2
    p.resetJointState(robot_id, 3, d3)  # gripper_joint (prismatic)


def interpolate_and_set(robot_id, start, target, steps=250, sleep=1 / 240.):
    """简单线性插值关节运动，避免瞬移。start/target: (t1,t2,d3)."""
    s1, s2, s3 = start
    e1, e2, e3 = target
    for i in range(1, steps + 1):
        a = i / steps
        t1 = s1 + (e1 - s1) * a
        t2 = s2 + (e2 - s2) * a
        d3 = s3 + (e3 - s3) * a
        set_joint_states(robot_id, t1, t2, d3)
        p.stepSimulation()
        time.sleep(sleep)


def control_gripper(robot_id, width: float, steps: int = 60, sleep: float = 1 / 240.):
    """控制夹爪开合。

    width: 期望 X 方向与 Y 方向打开的总宽度(简化为同一个数)，将映射为四个手指的对称位移。
    限制：每个手指最大行程 0.02 (由 URDF limit)。
    设计：finger1(+Y) finger3(-Y) finger4(+X) finger2(-X)。
    由于 finger2_joint/finger3_joint 的 limit lower 为负，需要注意方向。
    """
    # 关节索引（依据当前 URDF 加载顺序推断）
    FINGER1 = 4  # 0->fix,1->rot1,2->rot2,3->gripper,4->finger1 ...
    FINGER2 = 5
    FINGER3 = 6
    FINGER4 = 7
    half = max(0.0, width / 2.0)
    # 单指最大位移
    max_open = 0.02
    half = min(half, max_open)
    # 目标位姿
    target_f1 = half  # 0 .. 0.02
    target_f3 = -half  # -0.02 .. 0
    target_f4 = half  # 0 .. 0.02
    target_f2 = -half  # -0.02 .. 0

    target_f1 = target_f1 * -1
    target_f3 = target_f3 * -1
    target_f4 = target_f4 * -1
    target_f2 = target_f2 * -1
    # 当前位姿
    cur_f1 = p.getJointState(robot_id, FINGER1)[0]
    cur_f3 = p.getJointState(robot_id, FINGER3)[0]
    cur_f4 = p.getJointState(robot_id, FINGER4)[0]
    cur_f2 = p.getJointState(robot_id, FINGER2)[0]

    print("cur_f1:{}, cur_f2:{}, cur_f3:{}, cue_f4:{}".format(cur_f1, cur_f2, cur_f3, cur_f4))

    # for i in range(1, steps+1):
    #     a = i/steps
    #     f1 = cur_f1 + (target_f1 - cur_f1)*a
    #     f3 = cur_f3 + (target_f3 - cur_f3)*a
    #     f4 = cur_f4 + (target_f4 - cur_f4)*a
    #     f2 = cur_f2 + (target_f2 - cur_f2)*a
    #     p.resetJointState(robot_id, FINGER1, f1)
    #     p.resetJointState(robot_id, FINGER3, f3)
    #     p.resetJointState(robot_id, FINGER4, f4)
    #     p.resetJointState(robot_id, FINGER2, f2)
    #     p.stepSimulation()
    #     time.sleep(sleep)

    # p.resetJointState(robot_id, FINGER1, -0.02)
    # p.resetJointState(robot_id, FINGER3, 0.02)
    # p.resetJointState(robot_id, FINGER4, -0.02)
    # p.resetJointState(robot_id, FINGER2, 0.02)

    p.resetJointState(robot_id, FINGER1, 0.02)
    p.resetJointState(robot_id, FINGER3, -0.02)
    p.resetJointState(robot_id, FINGER4, 0.02)
    p.resetJointState(robot_id, FINGER2, -0.02)
    cur_f1 = p.getJointState(robot_id, FINGER1)[0]
    cur_f3 = p.getJointState(robot_id, FINGER3)[0]
    cur_f4 = p.getJointState(robot_id, FINGER4)[0]
    cur_f2 = p.getJointState(robot_id, FINGER2)[0]

    print("cur_f1:{}, cur_f2:{}, cur_f3:{}, cue_f4:{}".format(cur_f1, cur_f2, cur_f3, cur_f4))


def open_gripper(robot_id, steps: int = 60):
    """最大打开夹爪。"""
    # control_gripper(robot_id, width=0.04, steps=steps)  # 2*0.02
    FINGER1 = 4  # 0->fix,1->rot1,2->rot2,3->gripper,4->finger1 ...
    FINGER2 = 5
    FINGER3 = 6
    FINGER4 = 7
    p.resetJointState(robot_id, FINGER1, -0.02)
    p.resetJointState(robot_id, FINGER3, 0.02)
    p.resetJointState(robot_id, FINGER4, -0.02)
    p.resetJointState(robot_id, FINGER2, 0.02)
    cur_f1 = p.getJointState(robot_id, FINGER1)[0]
    cur_f3 = p.getJointState(robot_id, FINGER3)[0]
    cur_f4 = p.getJointState(robot_id, FINGER4)[0]
    cur_f2 = p.getJointState(robot_id, FINGER2)[0]

    print("cur_f1:{}, cur_f2:{}, cur_f3:{}, cue_f4:{}".format(cur_f1, cur_f2, cur_f3, cur_f4))


def close_gripper(robot_id, steps: int = 60, distance: float = 0.15):
    """闭合夹爪，保留极小间隙 residual_gap，防止数值震荡或穿透。"""
    # target_width = max(residual_gap, 0.0)
    # control_gripper(robot_id, width=target_width, steps=steps)
    FINGER1 = 4  # 0->fix,1->rot1,2->rot2,3->gripper,4->finger1 ...
    FINGER2 = 5
    FINGER3 = 6
    FINGER4 = 7
    p.resetJointState(robot_id, FINGER1, distance)
    p.resetJointState(robot_id, FINGER3, -distance)
    p.resetJointState(robot_id, FINGER4, distance)
    p.resetJointState(robot_id, FINGER2, -distance)
    cur_f1 = p.getJointState(robot_id, FINGER1)[0]
    cur_f3 = p.getJointState(robot_id, FINGER3)[0]
    cur_f4 = p.getJointState(robot_id, FINGER4)[0]
    cur_f2 = p.getJointState(robot_id, FINGER2)[0]

    print("cur_f1:{}, cur_f2:{}, cur_f3:{}, cue_f4:{}".format(cur_f1, cur_f2, cur_f3, cur_f4))


def configure_gripper_physics(robot_id):
    """提高手指摩擦，减少滑落。"""
    for finger_link in [4, 5, 6, 7]:
        p.changeDynamics(robot_id, finger_link, lateralFriction=100.0, spinningFriction=100, rollingFriction=100)


def detect_finger_contacts(robot_id, cube_id):
    """返回与方块接触的手指 link 集合及全部接触记录。"""
    contacts = p.getContactPoints(bodyA=robot_id, bodyB=cube_id)
    finger_links = {4, 5, 6, 7}
    active = set()
    for c in contacts:
        if c[3] in finger_links:
            active.add(c[3])
        if c[4] in finger_links:
            active.add(c[4])
    return active, contacts


def ensure_grasp(robot_id, cube_id, required_contacts: int = 2):
    """若接触手指不足则使用固定约束辅助抓取。返回 (grasp_ok, constraint_id)。"""
    active, contacts = detect_finger_contacts(robot_id, cube_id)
    if len(active) >= required_contacts:
        print(f"[grasp] 接触手指数={len(active)} 满足要求，使用摩擦保持。")
        return True, None
    print(f"[grasp] 接触数={len(active)} < {required_contacts}，创建约束辅助抓取。")
    cid = p.createConstraint(robot_id, 3, cube_id, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
    return False, cid


def pick(robot_id, cube_id, elbow: str = "down"):
    # 获取方块的坐标
    cx, cy, cz = p.getBasePositionAndOrientation(cube_id)[0]
    print("cx:{}, cy:{}, cz:{}".format(cx, cy, cz))

    # scara的逆运动学核心就在于theta1, theta2，d3的计算，这里先获得初始值，用于后面的插值运动
    t1_origin = p.getJointState(robot_id, 1)[0]
    t2_origin = p.getJointState(robot_id, 2)[0]
    d3_origin = p.getJointState(robot_id, 3)[0]
    print("t1_cur:{}, t2_cur:{}, d3_cur:{}".format(t1_origin, t2_origin, d3_origin))

    # 利用逆运动学算出要想移动到方块上方需要的关节值
    theta1_c, theta2_c, d3_c, reachable = inverse_kinematics(cx, cy, cz + 0.2, elbow='down')
    if not reachable:
        print("目标点不可达，结束。")
        return

    # 打开夹爪并配置摩擦
    open_gripper(robot_id, steps=30)
    # configure_gripper_physics(robot_id)

    # 1. 移动到方块上方，保持原高度避免碰撞
    interpolate_and_set(robot_id, start=(t1_origin, t2_origin, d3_origin), target=(theta1_c, theta2_c, d3_origin),
                        steps=240)
    t1_cur = p.getJointState(robot_id, 1)[0]
    t2_cur = p.getJointState(robot_id, 2)[0]
    d3_cur = p.getJointState(robot_id, 3)[0]
    print("t1_cur:{}, t2_cur:{}, d3_cur:{}".format(t1_cur, t2_cur, d3_cur))

    # 2. 下降到抓取高度 (使用 IK 给的 d3_c)
    interpolate_and_set(robot_id, start=(t1_cur, t2_cur, d3_cur), target=(theta1_c, theta2_c, d3_c), steps=260)

    # 3. 闭合夹爪（留少量间隙，避免过度穿透）
    close_gripper(robot_id, steps=60, distance=0.025)
    # cid = p.createConstraint(robot_id, 4, cube_id, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0])
    # cid = p.createConstraint(robot_id, 5, cube_id, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0])
    # cid = p.createConstraint(robot_id, 6, cube_id, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0])
    # cid = p.createConstraint(robot_id, 7, cube_id, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], [0,0,0])
    cid = p.createConstraint(robot_id, 3, cube_id, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, -0.2], [0, 0, 0.005])
    # 4. 稳定若干步建立接触
    for _ in range(30):
        p.stepSimulation();
        time.sleep(1 / 240.)

    # 5. 检测抓取，必要时用约束辅助
    # grasp_ok, constraint_id = ensure_grasp(robot_id, cube_id)
    # if constraint_id:
    #     print(f"[pick] 使用约束 id={constraint_id} 协助提起。")

    # 6. 抬起
    interpolate_and_set(robot_id, start=(theta1_c, theta2_c, d3_c), target=(theta1_c, theta2_c, d3_cur), steps=300)
    close_gripper(robot_id, steps=60, distance=0.025)
    for _ in range(100):
        p.stepSimulation();
        time.sleep(1 / 240.)
    # 7. 回到原来的位置展示携带
    interpolate_and_set(robot_id, start=(theta1_c, theta2_c, d3_cur), target=(t1_origin, t2_origin, d3_origin),
                        steps=500)

    close_gripper(robot_id, steps=60, distance=0.025)
    for _ in range(150):
        p.stepSimulation();
        time.sleep(1 / 240.)
    # 先降低一点再放开
    interpolate_and_set(robot_id, start=(t1_origin, t2_origin, d3_origin),
                        target=(t1_origin, t2_origin, d3_origin - 0.2), steps=300)
    open_gripper(robot_id, steps=60)
    p.removeConstraint(cid)
    interpolate_and_set(robot_id, start=(t1_origin, t2_origin, d3_origin - 0.2),
                        target=(t1_origin, t2_origin, d3_origin), steps=300)
    # print(f"[pick] 抓取完成，grasp_ok={grasp_ok}")


def main():
    cube_pos_test = {
        1: [1.2, -0.8, 0.05],
        2: [0.8, -1.0, 0.05],
        3: [1.2, 1.2, 0.05],
    }

    robot_id, cube_id = setup_scene(True, cube_pos_test[3])  # 通过旋转基座实现与理论FK对齐

    cur_f1 = p.getJointState(robot_id, 4)[0]
    cur_f3 = p.getJointState(robot_id, 5)[0]
    cur_f4 = p.getJointState(robot_id, 6)[0]
    cur_f2 = p.getJointState(robot_id, 7)[0]

    print("cur_f1:{}, cur_f2:{}, cur_f3:{}, cue_f4:{}".format(cur_f1, cur_f2, cur_f3, cur_f4))
    pick(robot_id, cube_id, elbow='down')

    try:
        for _ in range(20000):
            if not p.isConnected():
                break
            p.stepSimulation()
            time.sleep(1 / 240.)
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
