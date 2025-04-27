#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
import random
import sys


def force_sensor():
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_force')
    # pub = rospy.Publisher('/force', Float32, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
    #     pub.publish(Float32(random.uniform(0, 5)))
        rate.sleep()


if __name__ == '__main__':
    force_sensor()
