#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
import random


def arm_encoder():
    rospy.init_node('encoder_arm')
    pub = rospy.Publisher('/encoder_arm', Int32, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(Int32(random.uniform(0, 5)))
        rate.sleep()


if __name__ == '__main__':
    arm_encoder()
