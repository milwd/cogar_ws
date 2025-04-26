#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
import sys


def arm_jointstate_callback(msg):
    pub1.publish(msg)


def depth_raw_callback(msg): 
    pub2.publish(msg)


def force_sensor_callback(msg):
    pub3.publish(msg)


def odometry_callback(msg):
    pub4.publish(msg)


def rgb_raw_callback(msg):
    pub5.publish(msg)


def scan_lidar_callback(msg):
    pub6.publish(msg)


def sonar_callback(msg):
    pub7.publish(msg)


if __name__ == '__main__':
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node('topic_adapter', anonymous=True)

    pub1 = rospy.Publisher('', JointTrajectoryControllerState, queue_size=10)
    pub2 = rospy.Publisher(f'/{robot_number}/depth', Image, queue_size=10)
    pub3 = rospy.Publisher(f'/{robot_number}/force', WrenchStamped, queue_size=10)
    pub4 = rospy.Publisher(f'/{robot_number}/odom_proc', Odometry, queue_size=10)
    pub5 = rospy.Publisher(f'/{robot_number}/camera', Image, queue_size=10)
    pub6 = rospy.Publisher(f'/{robot_number}/lidar', LaserScan, queue_size=10)
    pub7 = rospy.Publisher(f'/{robot_number}/sonar', Range, queue_size=10)

    rospy.Subscriber(f'/{robot_number}/arm_right_controller/state', JointTrajectoryControllerState, arm_jointstate_callback)
    rospy.Subscriber(f'/{robot_number}/xtion/depth/image_raw', Image, depth_raw_callback)
    rospy.Subscriber(f'/{robot_number}/wrist_right_ft', WrenchStamped, force_sensor_callback)
    rospy.Subscriber(f'/{robot_number}/mobile_base_controller/odom', Odometry, odometry_callback)
    rospy.Subscriber(f'/{robot_number}/xtion/rgb/image_raw', Image, rgb_raw_callback)
    rospy.Subscriber(f'/{robot_number}/scan', LaserScan, scan_lidar_callback)
    rospy.Subscriber(f'/{robot_number}/sonar_base', Range, sonar_callback)

    rospy.spin()
