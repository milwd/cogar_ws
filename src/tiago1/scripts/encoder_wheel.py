# encoder_wheel.py

#!/usr/bin/env python
"""
encoder_wheel.py

ROS node that simulates wheel encoder readings. Publishes random encoder counts
on '/encoder_wheel' at 10 Hz for testing movement controllers.
"""

import rospy
from std_msgs.msg import Int32
import random
import sys

def wheel_encoder():
    """
    Initialize the ROS node and publish simulated wheel encoder values.

    Process
    -------
    1. Initialize ROS node 'encoder_wheel'.
    2. Advertise '/encoder_wheel' for Int32 messages.
    3. Loop at 10 Hz:
       a. Generate a random float between 0 and 5.
       b. Wrap the value in an Int32 message and publish.
       c. Sleep to maintain loop rate.
    """
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_encoder_wheel')
    pub = rospy.Publisher(f'/{robot_number}/encoder_wheel', Int32, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(Int32(random.uniform(0, 5)))
        rate.sleep()

if __name__ == '__main__':
    """
    Main entrypoint: start the wheel encoder publisher.
    """
    wheel_encoder()
