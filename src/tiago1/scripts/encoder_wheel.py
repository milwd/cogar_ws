#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


def wheel_encoder():
    rospy.init_node('encoder_wheel')
    pub = rospy.Publisher('/encoder_wheel', Int32, queue_size=10)
    rate = rospy.Rate(10)
    count = 0
    while not rospy.is_shutdown():
        pub.publish(count)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    wheel_encoder()
