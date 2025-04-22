#! /usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Int32

mic_publisher = rospy.Publisher('/mic_channel', Int32, queue_size=10)
def publish():
    
    a = np.random.randint(6)
    mic_publisher.publish(a)
    
if __name__ == "__main__":
    try:
        rospy.init_node("microphone_node")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            publish()
            rate.sleep()   
    except rospy.ROSInterruptException:
        pass