#!/usr/bin/env python3
from piper_sdk import *
import rospy
import time
import sys
import numpy as np
import math
from piper_arm import PiperArm
from utils.utils_piper import read_joints,read_gripper_status
from utils.utils_piper import enable_fun
from utils.utils_ros import publish_tf, publish_sphere_marker, publish_trajectory
from utils.utils_math import quaternion_to_rotation_matrix
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, PoseStamped
import threading 

PI = math.pi
factor = 1000 * 180 / PI
receive_object_center = False
object_center = []
simulation = True
busy = False
return_client = None
adjust_goal_pub = None  # 发布调整后的goal
piper_arm = None
ik_retry_requested = False  # 标记是否请求了重定位


def check_gripper_grasped(piper, min_gap=0.1, max_gap=0.8):
    """
    检查夹爪是否成功夹住物体
    Args:
        piper: 机械臂接口
        min_gap: 最小夹爪开口(归一化值),小于此值说明完全闭合,没夹到东西
        max_gap: 最大夹爪开口(归一化值),大于此值说明没有闭合
    Returns:
        bool: True表示成功夹住物体
    """
    from utils.utils_piper import read_gripper_status
    gripper_value = read_gripper_status(piper)
    rospy.loginfo("夹爪当前开口度: %.3f", gripper_value)
    
    if gripper_value < min_gap:
        rospy.logwarn("夹爪完全闭合,可能未夹到物体")
        return False
    elif gripper_value > max_gap:
        rospy.logwarn("夹爪未充分闭合")
        return False
    else:
        rospy.loginfo("夹爪闭合正常,成功夹住物体")
        return True

def check_position_reached(current_joints, target_joints, tolerance=0.05):
    """
    检查机械臂是否到达目标位置
    Args:
        current_joints: 当前关节角度列表
        target_joints: 目标关节角度列表
        tolerance: 容差(弧度)
    Returns:
        bool: True表示到达目标位置
    """
    for i in range(min(len(current_joints), len(target_joints))):
        diff = abs(current_joints[i] - target_joints[i])
        if diff > tolerance:
            rospy.logwarn("关节%d未到位: 当前=%.3f, 目标=%.3f, 偏差=%.3f", 
                         i, current_joints[i], target_joints[i], diff)
            return False
    rospy.loginfo("机械臂已到达目标位置")
    return True



def try_ik_at_position(base_ob_center, piper_arm, x_offset=0.0, y_offset=0.0):
    """
    尝试在指定偏移位置进行逆解
    Args:
        base_ob_center: 基座坐标系下的目标点
        piper_arm: 机械臂对象
        x_offset: x方向偏移(正值为远离,负值为靠近)
        y_offset: y方向偏移
    Returns:
        joints_plan: 关节角度列表,失败返回None
    """
    targetT = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
    targetT[0, 3] = base_ob_center[0] - 0.02 + x_offset
    targetT[1, 3] = base_ob_center[1] - 0.005 + y_offset
    targetT[2, 3] = base_ob_center[2]
    
    joints_plan = piper_arm.inverse_kinematics(targetT)
    if joints_plan:
        rospy.loginfo("逆解成功! x_offset=%.3f, y_offset=%.3f", x_offset, y_offset)
        return joints_plan
    return None


def find_feasible_position(base_ob_center, piper_arm, current_yaw):
    """
    查找可行的抓取位置
    Returns:
        (success, joints_plan, x_offset, y_offset): 
            success: 是否找到可行解
            joints_plan: 关节角度
            x_offset, y_offset: 相对当前位置的偏移量(车辆坐标系)
    """
    rospy.loginfo("=" * 60)
    rospy.loginfo("开始搜索可行抓取位置...")
    
    # 1. 尝试当前位置
    joints_plan = try_ik_at_position(base_ob_center, piper_arm, 0.0, 0.0)
    if joints_plan:
        rospy.loginfo("当前位置可行,无需调整")
        return True, joints_plan, 0.0, 0.0
    
    rospy.logwarn("当前位置逆解失败,尝试前后调整...")
    
    # 2. 尝试前后调整 (在机械臂基座坐标系的x方向)
    # 注意: 这里的偏移是机械臂坐标系下的,需要转换到车辆移动的偏移
    test_offsets = [
        -0.05,  # 靠近一点
        0.05,   # 远离一点
        -0.10,  # 靠近更多
        0.10,   # 远离更多
        -0.15,
        0.15,
    ]
    
    for x_offset in test_offsets:
        joints_plan = try_ik_at_position(base_ob_center, piper_arm, x_offset, 0.0)
        if joints_plan:
            # 将机械臂坐标系的x偏移转换为车辆坐标系的前后偏移
            # 假设车辆前进方向与机械臂x轴基本一致
            # 实际需要根据车辆与机械臂的相对姿态调整
            car_x_offset = x_offset * np.cos(current_yaw)
            car_y_offset = x_offset * np.sin(current_yaw)
            
            rospy.loginfo("找到可行位置! 车辆需要调整: x=%.3f, y=%.3f", 
                         car_x_offset, car_y_offset)
            return True, joints_plan, car_x_offset, car_y_offset
    
    rospy.logerr("未找到可行抓取位置")
    return False, None, 0.0, 0.0


def request_car_repositioning(x_offset, y_offset, current_pose):
    """
    请求车辆重新定位
    Args:
        x_offset: 车辆坐标系下的x偏移(前后)
        y_offset: 车辆坐标系下的y偏移(左右)
        current_pose: 当前车辆位姿 (x, y, yaw)
    """
    if not adjust_goal_pub:
        rospy.logerr("adjust_goal_pub 未初始化")
        return False
    
    # 计算新的目标位置(在map坐标系下)
    current_x, current_y, current_yaw = current_pose
    
    # 将车辆坐标系的偏移转换到map坐标系
    map_x_offset = x_offset * np.cos(current_yaw) - y_offset * np.sin(current_yaw)
    map_y_offset = x_offset * np.sin(current_yaw) + y_offset * np.cos(current_yaw)
    
    new_goal_x = current_x + map_x_offset
    new_goal_y = current_y + map_y_offset
    
    # 发布新的goal
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"
    goal_msg.pose.position.x = new_goal_x
    goal_msg.pose.position.y = new_goal_y
    goal_msg.pose.position.z = 0.0
    
    # 保持朝向不变
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = np.sin(current_yaw / 2.0)
    goal_msg.pose.orientation.w = np.cos(current_yaw / 2.0)
    
    adjust_goal_pub.publish(goal_msg)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("请求车辆重新定位:")
    rospy.loginfo("  当前位置: (%.3f, %.3f, %.1f°)", current_x, current_y, current_yaw*180/PI)
    rospy.loginfo("  偏移量: (%.3f, %.3f)", map_x_offset, map_y_offset)
    rospy.loginfo("  新目标: (%.3f, %.3f)", new_goal_x, new_goal_y)
    rospy.loginfo("=" * 60)
    
    global ik_retry_requested
    ik_retry_requested = True
    
    return True


def control_arm(joints, speed=2):
    """控制机械臂运动"""
    position = joints
    joint_0 = int(position[0] * factor)
    joint_1 = int(position[1] * factor)
    joint_2 = int(position[2] * factor)
    joint_3 = int(position[3] * factor)
    joint_4 = int(position[4] * factor)
    joint_5 = int(position[5] * factor)

    if (joint_4 < -70000):
        joint_4 = -70000

    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    if len(joints) > 6:
        joint_6 = round(position[6] * 100 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

    print(piper.GetArmStatus())
    print(position)


def object_point_callback(msg):
    """目标点回调函数"""
    if(np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z)):
        return
    global receive_object_center, object_center
    receive_object_center = True
    object_center = [msg.point.x, msg.point.y, msg.point.z]
    rospy.loginfo("收到目标点: x=%.4f y=%.4f z=%.4f", msg.point.x, msg.point.y, msg.point.z)


def do_place():
    """放置物体"""
    rospy.loginfo("开始放置物体...")
    approach_joints = [0, 1.45, -0.64, 0, 0, 0, 0]
    control_arm(approach_joints, 20)
    time.sleep(2)

    release_joints = [0, 1.45, -0.64, 0, 0, 0, 1.0]
    control_arm(release_joints, 20)
    time.sleep(1)
    
    rospy.loginfo("放置完成")
    return True


def move_and_grasp(object_center, joints, piper_arm, current_yaw=0.0):
    """
    执行抓取动作
    Returns:
        (success, need_reposition): 
            success: 是否成功
            need_reposition: 是否需要车辆重新定位
    """
    rospy.loginfo("=" * 60)
    rospy.loginfo("开始抓取规划...")
    
    # 1. 计算目标点在基座坐标系下的位置
    base_T_link6 = piper_arm.forward_kinematics(joints)
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera
    base_ob_center = base_T_link6 @ link6_T_cam @ np.array([object_center[0], object_center[1], object_center[2], 1])

    rospy.loginfo("相机坐标系下目标点: [%.3f, %.3f, %.3f]", 
                  object_center[0], object_center[1], object_center[2])
    rospy.loginfo("基座坐标系下目标点: [%.3f, %.3f, %.3f]", 
                  base_ob_center[0], base_ob_center[1], base_ob_center[2])

    # 可视化目标点
    pub_marker = rospy.Publisher('/target_point_under_based', Marker, queue_size=10)
    publish_sphere_marker(pub_marker, base_ob_center, frame_id="arm_base", 
                         color=(0.0, 1.0, 0.0, 1.0), radius=0.02)

    # 2. 尝试逆解
    joints_plan = try_ik_at_position(base_ob_center, piper_arm, 0.0, 0.0)
    
    if not joints_plan:
        rospy.logwarn("当前位置逆解失败,返回需要重新定位")
        return False, True  # 失败,需要重新定位
    
    joints_array = np.array(joints_plan)
    rospy.loginfo("逆解成功! 目标关节角度[度]: %s", joints_array / PI * 180)

    # 3. 执行抓取
    rospy.loginfo("阶段1: 移动到抓取位置(夹爪张开)...")
    grasp_joints = list(joints_plan)
    grasp_joints.append(1.0)
    control_arm(grasp_joints, 20)
    time.sleep(5)

    # 验证位置
    current_joints = read_joints(piper)
    if not check_position_reached(current_joints, joints_plan, tolerance=0.08):
        rospy.logerr("机械臂未到达抓取位置")
        return False, False

    # 4. 闭合夹爪
    rospy.loginfo("阶段2: 闭合夹爪...")
    grasp_joints[6] = 0.00
    control_arm(grasp_joints, 20)
    time.sleep(2)

    # 验证夹爪
    if not check_gripper_grasped(piper, min_gap=0.1, max_gap=0.8):
        rospy.logerr("夹爪未成功夹住物体")
        return False, False

    # 5. 抬起到保持姿态
    rospy.loginfo("阶段3: 抬起物体...")
    hold_joints = [0, 1.0, -1.0, 0, 0, 0, 0.00]
    control_arm(hold_joints, 20)
    time.sleep(2)

    rospy.loginfo("=" * 60)
    rospy.loginfo("抓取成功!")
    return True, False


def do_pick():
    """
    执行抓取任务
    Returns:
        (success, need_retry): 
            success: 是否成功
            need_retry: 是否需要车辆重新定位后重试
    """
    if len(object_center) != 3:
        rospy.logwarn("未收到有效目标点")
        return False, False

    rospy.loginfo("=" * 80)
    rospy.loginfo("开始抓取任务")
    rospy.loginfo("=" * 80)

    # 获取当前关节角度
    current_joints = read_joints(piper)
    
    # 尝试抓取
    success, need_reposition = move_and_grasp(object_center, current_joints, piper_arm)
    
    if success:
        rospy.loginfo("抓取成功!")
        global receive_object_center
        receive_object_center = False
        return True, False
    
    if need_reposition:
        rospy.logwarn("逆解失败,尝试寻找可行位置...")
        
        # 计算目标点位置(用于IK测试)
        base_T_link6 = piper_arm.forward_kinematics(current_joints)
        link6_T_cam = np.eye(4)
        link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
        link6_T_cam[:3, 3] = piper_arm.link6_t_camera
        base_ob_center = base_T_link6 @ link6_T_cam @ np.array([object_center[0], object_center[1], object_center[2], 1])
        
        # 假设当前yaw为0(需要从车辆获取实际值)
        # TODO: 这里需要从某处获取车辆当前yaw
        current_yaw = 0.0
        
        # 搜索可行位置
        found, _, x_offset, y_offset = find_feasible_position(base_ob_center, piper_arm, current_yaw)
        
        if found and (abs(x_offset) > 0.01 or abs(y_offset) > 0.01):
            # 需要车辆移动
            rospy.loginfo("找到可行位置,请求车辆重新定位...")
            # TODO: 需要获取车辆当前位姿
            # request_car_repositioning(x_offset, y_offset, (current_x, current_y, current_yaw))
            return False, True  # 失败,但需要重试
        elif found:
            rospy.loginfo("找到可行位置但无需移动,重试抓取...")
            return False, False
        else:
            rospy.logerr("未找到可行位置,抓取失败")
            return False, False
    else:
        rospy.logerr("抓取执行失败")
        return False, False


def notify_car_return(reason):
    """通知车辆返回"""
    if not return_client:
        rospy.logwarn("return_task service client not ready")
        return
    try:
        resp = return_client()
        rospy.loginfo("通知车辆返回 (%s), 响应: %s", reason, resp.message)
    except rospy.ServiceException as e:
        rospy.logerr("调用 return_task 失败: %s", e)


def handle_start_grasp(_req):
    """处理抓取请求"""
    global busy, ik_retry_requested
    rospy.loginfo("收到 start_grasp_task 请求")
    
    if busy:
        return TriggerResponse(success=False, message="Arm busy")
    
    busy = True
    ik_retry_requested = False
    
    try:
        rospy.loginfo("开始抓取流程...")
        
        success, need_retry = do_pick()
        
        if success:
            # 抓取成功,通知车辆返回
            def async_notify():
                time.sleep(0.5)
                notify_car_return("grasp finished, return for drop")
            threading.Thread(target=async_notify, daemon=True).start()
            
            return TriggerResponse(success=True, message="Grasp success")
        
        elif need_retry:
            # 需要车辆重新定位
            rospy.loginfo("抓取失败,已请求车辆重新定位")
            return TriggerResponse(success=False, message="IK failed, repositioning requested")
        
        else:
            # 抓取失败
            rospy.logerr("抓取失败")
            return TriggerResponse(success=False, message="Grasp failed")
        
    except Exception as e:
        rospy.logerr("抓取异常: %s", e)
        import traceback
        traceback.print_exc()
        return TriggerResponse(success=False, message=str(e))
    finally:
        busy = False


def handle_place(_req):
    """处理放置请求"""
    global busy
    rospy.loginfo("收到 throw_bottle_task 请求")
    
    if busy:
        return TriggerResponse(success=False, message="Arm busy")
    
    busy = True
    try:
        rospy.loginfo("Start place sequence")
        if not do_place():
            return TriggerResponse(success=False, message="Place failed")
        # 如需放完后让车返回起点，可在此调用 notify_car_return
        return TriggerResponse(success=True, message="Place done")
    except Exception as e:
        rospy.logerr("Place failed: %s", e)
        return TriggerResponse(success=False, message=str(e))
    finally:
        busy = False


if __name__ == "__main__":
    piper = C_PiperInterface_V2("can1")
    piper.ConnectPort()
    piper.EnableArm(7)
    enable_fun(piper=piper)
    piper.GripperCtrl(0, 1000, 0x01, 0)

    # 设置初始位置
    joints = [0, 1.0, -1.0, 0, 0, 0, 1.00]
    control_arm(joints, 100)
    time.sleep(2)


    # 初始化节点
    rospy.init_node('vison_grasp_node', anonymous=True)
    
    # 连接返回服务
    try:
        rospy.wait_for_service("/return_task", timeout=5.0)
        return_client = rospy.ServiceProxy("/return_task", Trigger)
        rospy.loginfo("Connected to /return_task service.")
    except rospy.ROSException:
        rospy.logwarn("/return_task service not available yet, will retry when needed")
        return_client = None
    
    # 初始化发布者(用于请求车辆调整位置)
    adjust_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    # 初始化机械臂
    piper_arm = PiperArm()
    
    # 订阅目标点
    sub = rospy.Subscriber('/object_point',
                           PointStamped,
                           object_point_callback,
                           queue_size=10,
                           tcp_nodelay=True)
    
    # 注册服务
    rospy.Service("/start_grasp_task", Trigger, handle_start_grasp)
    rospy.Service("/throw_bottle_task", Trigger, handle_place)
    rospy.loginfo("Arm task services ready (start_grasp_task / throw_bottle_task).")
    rospy.spin()
