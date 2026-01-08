#!/usr/bin/env python3
from piper_sdk import *
import rospy
import time
import sys
import numpy as np
import math
from piper_arm import PiperArm
from utils.utils_piper import read_joints
from utils.utils_piper import enable_fun
from utils.utils_ros import publish_tf, publish_sphere_marker, publish_trajectory
from utils.utils_math import quaternion_to_rotation_matrix
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import threading 

PI = math.pi
factor = 1000 * 180 / PI
receive_object_center = False
object_center = []
simulation = True
busy = False
return_client = None
piper_arm = None


def control_arm(joints, speed=2):

    # joints [rad]

    position = joints

    joint_0 = int(position[0] * factor)
    joint_1 = int(position[1] * factor)
    joint_2 = int(position[2] * factor)
    joint_3 = int(position[3] * factor)
    joint_4 = int(position[4] * factor)
    joint_5 = int(position[5] * factor)

    if (joint_4 < -70000) :
        joint_4 = -70000

    # piper.MotionCtrl_1()
    piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
    piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)

    if len(joints) > 6:
        joint_6 = round(position[6] * 100 * 1000)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

    print(piper.GetArmStatus())
    print(position)


def object_point_callback(msg):
    # print("Receive visual detection result", msg.point.x, msg.point.y, msg.point.z)
    if(np.isnan(msg.point.x) or np.isnan(msg.point.y) or np.isnan(msg.point.z)):
        return
    global receive_object_center, object_center
    receive_object_center = True
    object_center = [msg.point.x, msg.point.y, msg.point.z]
    rospy.loginfo("收到目标点: x=%.4f y=%.4f z=%.4f", msg.point.x, msg.point.y, msg.point.z)


def do_place():
    # 先回到起始姿态，稍作停留再松开夹爪
    approach_joints = [0, 1.45, -0.64, 0, 0, 0, 0]
    control_arm(approach_joints, 20)
    time.sleep(1)

    release_joints = [0, 1.45, -0.64, 0, 0, 0, 1]
    control_arm(release_joints, 20)
    return True


def do_pick():
    attempts = 50
    success = False

    for i in range(attempts):
        if len(object_center) != 3:
            rospy.logwarn("未收到目标点，第 %d/%d 次抓取跳过", i + 1, attempts)
            time.sleep(0.2)
            continue

        msg = piper.GetArmJointMsgs()
        theta1 = msg.joint_state.joint_1 * 1e-3 * PI / 180.0
        theta2 = msg.joint_state.joint_2 * 1e-3 * PI / 180.0
        theta3 = msg.joint_state.joint_3 * 1e-3 * PI / 180.0
        theta4 = msg.joint_state.joint_4 * 1e-3 * PI / 180.0
        theta5 = msg.joint_state.joint_5 * 1e-3 * PI / 180.0
        theta6 = msg.joint_state.joint_6 * 1e-3 * PI / 180.0

        joints = [theta1, theta2, theta3, theta4, theta5, theta6]

        rospy.loginfo("第 %d/%d 次尝试抓取", i + 1, attempts)
        if move_and_grasp(object_center, joints, piper_arm):
            success = True
            break

        rospy.logwarn("第 %d/%d 次抓取失败，重试", i + 1, attempts)
        time.sleep(0.2)

    # 重置标志，方便后续接收新的目标点
    global receive_object_center
    receive_object_center = False
    return success


def move_and_grasp(object_center, joints, piper_arm):
    # Planning phase
    print("prepare to grasp point under camera frame", object_center[0], object_center[1], object_center[2])
    base_T_link6 = piper_arm.forward_kinematics(joints)
    link6_T_cam = np.eye(4)
    link6_T_cam[:3, :3] = quaternion_to_rotation_matrix(piper_arm.link6_q_camera)
    link6_T_cam[:3, 3] = piper_arm.link6_t_camera
    base_ob_center = base_T_link6 @ link6_T_cam @ np.array([object_center[0], object_center[1], object_center[2], 1])

    print("point under base frame", base_ob_center)
    pub_marker = rospy.Publisher('/target_point_under_based', Marker, queue_size=10)
    publish_sphere_marker(pub_marker, base_ob_center, frame_id="arm_base", color=(0.0, 1.0, 0.0, 1.0), radius=0.02)

    targetT = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
    targetT[0, 3] = base_ob_center[0]-0.02
    targetT[1, 3] = base_ob_center[1]-0.005
    targetT[2, 3] = base_ob_center[2]

    joints_plan = piper_arm.inverse_kinematics(targetT)
    joints_array = np.array(joints_plan)
    print("base ob center", base_ob_center)
    if not joints_plan :
        print("ik fail")
        return False
    print("Planed ik[degree]:", joints_array / PI * 180)

    # Grasp phase
    grasp_joints = list(joints_plan)
    grasp_joints.append(1.0)
    control_arm(grasp_joints, 20)
    time.sleep(5)
    grasp_joints[6] = 0.00
    control_arm(grasp_joints, 20)
    time.sleep(2)

    # Post-grasp hold phase
    hold_joints = [0, 1.0, -1.0, 0, 0, 0, 0.00]
    control_arm(hold_joints, 20)

    return True


def notify_car_return(reason):
    if not return_client:
        rospy.logwarn("return_task service client not ready, skip notify.")
        return
    try:
        resp = return_client()
        rospy.loginfo("Sent return_task (%s), car replied: %s", reason, resp.message)
    except rospy.ServiceException as e:
        rospy.logerr("return_task call failed: %s", e)


def handle_start_grasp(_req):
    global busy
    rospy.loginfo("收到 start_grasp_task 请求")
    if busy:
        return TriggerResponse(success=False, message="Arm busy")
    busy = True
    try:
        rospy.loginfo("Start grasp sequence")
        if not do_pick():
            return TriggerResponse(success=False, message="Grasp failed or object not ready")
        
        def async_notify():
            time.sleep(0.5)  # 等待当前服务响应发送完成
            notify_car_return("grasp finished, return for drop")
        threading.Thread(target=async_notify, daemon=True).start()
        
        return TriggerResponse(success=True, message="Grasp done")
    except Exception as e:
        rospy.logerr("Grasp failed: %s", e)
        return TriggerResponse(success=False, message=str(e))
    finally:
        busy = False


def handle_place(_req):
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
    try:
        rospy.wait_for_service("/return_task", timeout=5.0)
        return_client = rospy.ServiceProxy("/return_task", Trigger)
        rospy.loginfo("Connected to /return_task service.")
    except rospy.ROSException:
        rospy.logwarn("/return_task service not available yet, will retry when needed")
        return_client = None

    piper_arm = PiperArm()
    sub = rospy.Subscriber('/object_point',
                           PointStamped,
                           object_point_callback,
                           queue_size=10,
                           tcp_nodelay=True)
    rospy.Service("/start_grasp_task", Trigger, handle_start_grasp)
    rospy.Service("/throw_bottle_task", Trigger, handle_place)
    rospy.loginfo("Arm task services ready (start_grasp_task / throw_bottle_task).")
    rospy.spin()
