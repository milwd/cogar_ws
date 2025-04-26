#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import random
import sys


def gripper_encoder():
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_encoder_gripper')
    pub = rospy.Publisher(f'/{robot_number}/encoder_gripper', Int32, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(Int32(random.uniform(0, 5)))
        rate.sleep()


if __name__ == '__main__':
    gripper_encoder()
