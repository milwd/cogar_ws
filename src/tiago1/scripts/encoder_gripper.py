#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


def gripper_encoder():
    rospy.init_node('encoder_gripper')
    pub = rospy.Publisher('/encoder_gripper', Int32, queue_size=10)
    rate = rospy.Rate(10)
    count = 0
    while not rospy.is_shutdown():
        pub.publish(count)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    gripper_encoder()
