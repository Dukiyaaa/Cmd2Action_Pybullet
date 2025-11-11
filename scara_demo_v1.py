
import time
import numpy as np
import pybullet as p
import pybullet_data
from my_kinematics import forward_kinematics, inverse_kinematics

cube_pos = [1.2, -0.8, 0.05]
def setup_scene(rotate_base: bool = True):
    """创建仿真场景并返回 (robot_id, cube_id)。"""
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")
    if rotate_base:
        quat = p.getQuaternionFromEuler([0, 0, np.pi/2])  # 绕 Z +90°
        robot = p.loadURDF("my_scara.urdf", [0, 0, 0], quat, useFixedBase=True)
    else:
        robot = p.loadURDF("my_scara.urdf", [0, 0, 0], useFixedBase=True)
    cube_id = p.loadURDF("cube_small.urdf", cube_pos)
    p.changeVisualShape(cube_id, -1, rgbaColor=[1, 0, 0, 1])
    return robot, cube_id

def set_joint_states(robot_id, theta1, theta2, d3):
    """重置关节状态到指定值。"""
    p.resetJointState(robot_id, 1, theta1)  # rotation1
    p.resetJointState(robot_id, 2, theta2)  # rotation2
    p.resetJointState(robot_id, 3, d3)      # gripper_joint (prismatic)

def get_end_effector_position(robot_id):
    """读取末端 link (索引3) 的世界位置。"""
    state = p.getLinkState(robot_id, 3, computeForwardKinematics=True)
    return state[0]  # position tuple (x,y,z)

def verify_positions(robot_id, test_sets):
    print("== SCARA End-Effector Position Verification (Base Rotated at Load) ==")
    for (th1, th2, d3) in test_sets:
        th1c, th2c, d3c = th1, th2, d3
        set_joint_states(robot_id, th1c, th2c, d3c)
        p.stepSimulation()
        # 理论矩阵 (未内置对齐旋转)
        theo_T = forward_kinematics(th1c, th2c, d3c)
        bullet_pos = get_end_effector_position(robot_id)
        theo_pos = (float(theo_T[0,3]), float(theo_T[1,3]), float(theo_T[2,3]))
        diff = np.array(bullet_pos) - np.array(theo_pos)
        err = float(np.linalg.norm(diff))
        print(f"Input (θ1={th1:.3f}, θ2={th2:.3f}, d3={d3:.3f})")
        print("  Theo FK (X,Y,Z):", theo_pos)
        print("  PyBullet  (X,Y,Z):", bullet_pos)
        print("  Error vector      :", tuple(diff.tolist()), "| norm=", err)
        print("  T (homogeneous):\n", theo_T)
        print("  ------------------------------------")

def verify_ik(robot_id, target_sets, elbow: str = "down"):
    """利用逆运动学: 给定目标末端坐标 -> 计算关节值 -> 应用到仿真 -> 再读取实际末端比较误差。"""
    print("\n== SCARA Inverse Kinematics Verification ==")
    for (x, y, z) in target_sets:
        th1, th2, d3, reachable = inverse_kinematics(x, y, z, elbow=elbow)
        status = "OK" if reachable else "OUT_OF_RANGE"
        # 若不可达仍演示夹紧后的姿态效果
        set_joint_states(robot_id, th1, th2, d3)
        p.stepSimulation()
        ef_pos = get_end_effector_position(robot_id)
        # 理论再正向验证
        T = forward_kinematics(th1, th2, d3)
        fk_pos = (float(T[0,3]), float(T[1,3]), float(T[2,3]))
        target = (x, y, z)
        err_vec = np.array(ef_pos) - np.array(target)
        err_norm = float(np.linalg.norm(err_vec))
        print(f"Target=({x:.3f},{y:.3f},{z:.3f}) | IK -> (θ1={th1:.3f}, θ2={th2:.3f}, d3={d3:.3f}) | reachable={status}")
        print("  FK pos       :", fk_pos)
        print("  PyBullet pos :", ef_pos)
        print("  Error vector :", tuple(err_vec.tolist()), "| norm=", err_norm)
        print("  ------------------------------------")

def interpolate_and_set(robot_id, start, target, steps=250, sleep=1/240.):
    """简单线性插值关节运动，避免瞬移。start/target: (t1,t2,d3)."""
    s1,s2,s3 = start
    e1,e2,e3 = target
    for i in range(1, steps+1):
        a = i/steps
        t1 = s1 + (e1 - s1)*a
        t2 = s2 + (e2 - s2)*a
        d3 = s3 + (e3 - s3)*a
        set_joint_states(robot_id, t1, t2, d3)
        p.stepSimulation()
        time.sleep(sleep)

def pick(robot_id, cube_id, elbow='down', approach_offset=0.2, lift_offset=0.25):
    """执行抓取: 走到方块上方 -> 下落抓取 -> 抬起。

    步骤:
      1. 获取方块世界坐标 (cx,cy,cz)
      2. 规划 approach 点: (cx, cy, cz + approach_offset)
      3. IK 求解并插值移动到 approach
      4. 下降到 grasp_height = cz (或略低)
      5. 创建固定约束 (末端 link 与方块)
      6. 抬起到 (cx, cy, cz + lift_offset)
    """
    cx, cy, cz = p.getBasePositionAndOrientation(cube_id)[0]
    print("cx:{}, cy:{}, cz:{}".format(cx,cy,cz))
    # 末端上方目标
    approach_z = cz + approach_offset
    grasp_z = cz  # 如需压入可减去少量
    lift_z = cz + lift_offset

    # 当前关节位置（读 jointState）
    t1_cur = p.getJointState(robot_id, 1)[0]
    t2_cur = p.getJointState(robot_id, 2)[0]
    d3_cur = p.getJointState(robot_id, 3)[0]

    # Approach IK
    t1_a, t2_a, d3_a, reach_a = inverse_kinematics(cx, cy, approach_z, elbow=elbow)
    if not reach_a:
        print("[pick] Approach 点不可达，放弃抓取。", (cx, cy, approach_z))
        return None
    interpolate_and_set(robot_id, (t1_cur,t2_cur,d3_cur), (t1_a,t2_a,d3_a))
    # set_joint_states(robot_id, t1_a, t2_a, d3_a)

    # Grasp IK
    # t1_g, t2_g, d3_g, reach_g = inverse_kinematics(cx, cy, grasp_z, elbow=elbow)
    # if not reach_g:
    #     print("[pick] 抓取高度不可达，放弃抓取。", (cx, cy, grasp_z))
    #     return None
    # # interpolate_and_set(robot_id, (t1_a,t2_a,d3_a), (t1_g,t2_g,d3_g))
    # set_joint_states(robot_id, t1_g, t2_g, d3_g)
    # 创建约束 (末端 link index = 3)
    constraint_id = p.createConstraint(robot_id, 3, cube_id, -1, p.JOINT_FIXED,
                                       [0,0,0], [0,0,0], [0,0,0])
    print(f"[pick] 已创建约束 id={constraint_id}")

    # Lift IK
    t1_l, t2_l, d3_l, reach_l = inverse_kinematics(cx, cy, lift_z, elbow=elbow)
    if not reach_l:
        print("[pick] 抬起高度不可达，仅停留抓取位。")
        return constraint_id
    # interpolate_and_set(robot_id, (t1_a,t2_a,d3_a), (t1_l,t2_l,d3_l))
    set_joint_states(robot_id, t1_l, t2_l, d3_l)
    print("[pick] 抓取并抬起完成。")
    return 1

def main():
    robot_id, cube_id = setup_scene(rotate_base=True)  # 通过旋转基座实现与理论FK对齐
    # 测试关节集合：可加入更多覆盖边界

    # 逆运动学目标点集合（可调）
    ik_targets = [
        (cube_pos[0], cube_pos[1], cube_pos[2] + 0.2)
    ]
    # verify_ik(robot_id, ik_targets, elbow="down")

    # 执行抓取演示
    constraint_id = pick(robot_id, cube_id, elbow='down', approach_offset=0.2, lift_offset=0.2)
    # 可选：移动到新位置(简单示例：绕 y 轴方向做一个小圆弧)略。

    print("保持最后一个姿态进行观察，关闭窗口以结束...")
    try:
        for _ in range(20000):
            if not p.isConnected():
                break
            p.stepSimulation()
            time.sleep(1/240.)
    finally:
        if p.isConnected():
            p.disconnect()

if __name__ == "__main__":
    main()
