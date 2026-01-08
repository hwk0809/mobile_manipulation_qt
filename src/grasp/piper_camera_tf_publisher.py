#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from piper_arm import PiperArm


def main():
    rospy.init_node("piper_camera_tf_publisher", anonymous=True)
    arm = PiperArm()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "link6"
    transform.child_frame_id = "camera"
    transform.transform.translation.x = arm.link6_t_camera[0]
    transform.transform.translation.y = arm.link6_t_camera[1]
    transform.transform.translation.z = arm.link6_t_camera[2]
    transform.transform.rotation.x = arm.link6_q_camera[1]
    transform.transform.rotation.y = arm.link6_q_camera[2]
    transform.transform.rotation.z = arm.link6_q_camera[3]
    transform.transform.rotation.w = arm.link6_q_camera[0]

    broadcaster.sendTransform(transform)
    rospy.loginfo("Published static TF: link6 -> camera")
    rospy.spin()


if __name__ == "__main__":
    main()
