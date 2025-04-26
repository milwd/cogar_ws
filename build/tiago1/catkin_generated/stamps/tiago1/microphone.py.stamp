#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32
import sys


def publish():
    a = np.random.randint(6)
    mic_publisher.publish(a)
    
if __name__ == "__main__":
    try:
        robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f"{robot_number}_microphone_node")
        mic_publisher = rospy.Publisher(f'/{robot_number}/mic_channel', Int32, queue_size=10)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            publish()
            rate.sleep()   
    except rospy.ROSInterruptException:
        pass