#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range


def arm_jointstate_callback(msg):
    pub = rospy.Publisher('', JointTrajectoryControllerState, queue_size=10)
    pub.publish(msg)


def depth_raw_callback(msg):
    pub = rospy.Publisher('/depth', Image, queue_size=10)
    pub.publish(msg)


def force_sensor_callback(msg):
    pub = rospy.Publisher('/force', WrenchStamped, queue_size=10)
    pub.publish(msg)


def odometry_callback(msg):
    pub = rospy.Publisher('/odom_proc', Odometry, queue_size=10)
    pub.publish(msg)


def rgb_raw_callback(msg):
    pub = rospy.Publisher('/camera', Image, queue_size=10)
    pub.publish(msg)


def scan_lidar_callback(msg):
    pub = rospy.Publisher('/lidar', LaserScan, queue_size=10)
    pub.publish(msg)


def sonar_callback(msg):
    pub = rospy.Publisher('/sonar', Range, queue_size=10)
    pub.publish(msg)


def listener():
    rospy.init_node('topic_adapter', anonymous=True)

    rospy.Subscriber('/arm_right_controller/state', JointTrajectoryControllerState, arm_jointstate_callback)
    rospy.Subscriber('/xtion/depth/image_raw', Image, depth_raw_callback)
    rospy.Subscriber('/wrist_right_ft', WrenchStamped, force_sensor_callback)
    rospy.Subscriber('/mobile_base_controller/odom', Odometry, odometry_callback)
    rospy.Subscriber('/xtion/rgb/image_raw', Image, rgb_raw_callback)
    rospy.Subscriber('/scan', LaserScan, scan_lidar_callback)
    rospy.Subscriber('/sonar_base', Range, sonar_callback)

    rospy.spin()


if __name__ == '__main__':
    listener()
