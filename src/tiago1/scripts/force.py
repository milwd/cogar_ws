# force.py

#!/usr/bin/env python
"""
force.py

ROS node that simulates a force sensor by publishing random force readings.
Publishes Float32 messages on '/force' at 10 Hz for testing manipulation tasks.
"""

import rospy
from std_msgs.msg import Float32
import random
import sys

def force_sensor():
    """
    Initialize the ROS node and publish simulated force sensor values.

    Process
    -------
    1. Initialize ROS node 'force'.
    2. Advertise '/force' for Float32 messages.
    3. Loop at 10 Hz:
       a. Generate a random force value between 0 and 5 Newtons.
       b. Wrap the value in a Float32 message and publish.
       c. Sleep to maintain loop rate.
    """
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_force')
    # pub = rospy.Publisher('/force', Float32, queue_size=10)
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     pub.publish(Float32(random.uniform(0, 5)))
    #     rate.sleep()


if __name__ == '__main__':
    """
    Main entrypoint: start the force sensor publisher.
    """
    force_sensor()
