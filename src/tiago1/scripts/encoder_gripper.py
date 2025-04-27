#!/usr/bin/env python
"""
encoder_gripper.py

ROS node that simulates the gripper encoder by publishing random encoder readings.
Publishes Int32 messages on '/encoder_gripper' at 10 Hz for testing downstream controllers.
"""

import rospy
from std_msgs.msg import Int32
import random
import sys

def gripper_encoder():
    """
    Initialize the ROS node and publish simulated gripper encoder values.

    Process
    -------
    1. Initialize ROS node 'encoder_gripper'.
    2. Create a Publisher on '/encoder_gripper' for Int32 messages.
    3. Loop at 10 Hz until shutdown:
       a. Generate a random float between 0 and 5.
       b. Wrap the value in an Int32 message and publish.
       c. Sleep to maintain loop rate.
    """
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_encoder_gripper')
    pub = rospy.Publisher(f'/{robot_number}/encoder_gripper', Int32, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(Int32(random.uniform(0, 5)))
        rate.sleep()

if __name__ == '__main__':
    """
    Main entrypoint: start the gripper encoder publisher.
    """
    gripper_encoder()
