#! /usr/bin/env python
"""
microphone_node.py

ROS node that simulates microphone input by publishing random channel values.
Publishes Int32 messages on '/mic_channel' at 1 Hz for testing audio-driven behaviors.
"""

import rospy
import numpy as np
from std_msgs.msg import Int32
import sys


<<<<<<< HEAD
def publish():
    """
    Generate and publish a random microphone channel value.

    Process
    -------
    1. Draw a random integer in [0, 5] using NumPy.
    2. Publish the value as an Int32 message on '/mic_channel'.
    """
    a = np.random.randint(6)
    mic_publisher.publish(a)

if __name__ == "__main__":
    """
    Main entrypoint: initialize the 'microphone_node' and loop at 1 Hz.
    """
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
