#!/usr/bin/env python
"""
sonar.py
=========

Synthetic **ultrasound range-finder** for quick sensor-fusion prototyping
-------------------------------------------------------------------------

This node mimics a 1-D sonar module by broadcasting a constant
:pyclass:`sensor_msgs.msg.Range` message.  Navigation, fusion and obstacle
avoidance stacks can therefore be exercised on any machine—even one with no
hardware attached or ROS bag available.

Published topic
---------------
``/sonar`` ( ``sensor_msgs/Range`` ) 10 Hz, single beam

Message layout
~~~~~~~~~~~~~~
=======================  =======================================================
Field                    Value
-----------------------  -------------------------------------------------------
``header.frame_id``      ``"sonar"``
``radiation_type``       ``Range.ULTRASOUND``  (numeric constant 0)
``field_of_view``        0.1 rad ≈ 5.7 °
``min_range``            0.2 m
``max_range``            4.0 m
``range``                **3.0 m** (fixed reading)
=======================  =======================================================

Tweak any constant to emulate a different device or scenario.
"""

import rospy
from sensor_msgs.msg import Range
import sys


def publish():
    """
    Spawn the node and stream **Range** messages until ROS shuts down.

    Workflow
    --------
    #. **Advertise** the ``/sonar`` topic (queue size 10).  
    #. **Initialise** the node with the name ``sonar_node``; ``anonymous=True``
       allows multiple dummy sonars to coexist.  
    #. Enter a **10 Hz loop** that:

       a. Instantiates :pyclass:`sensor_msgs.msg.Range`.  
       b. Fills :pyattr:`~sensor_msgs.msg.Range.header` with *now()* and
          ``"sonar"`` as the frame.  
       c. Sets :pyattr:`~sensor_msgs.msg.Range.radiation_type` to
          ``Range.ULTRASOUND``.  
       d. Assigns field-of-view and valid range limits.  
       e. Stores the constant distance *3 m* in :pyattr:`~sensor_msgs.msg.Range.range`.  
       f. Publishes the message.  
       g. Sleeps the remaining time to keep the period at 100 ms.
    """
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_sonar_node', anonymous=True)
    pub = rospy.Publisher(f'/{robot_number}/sonar', Range, queue_size=10)
    # rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    #     sonar = Range()
    #     sonar.header.stamp = rospy.Time.now()
    #     sonar.header.frame_id = "sonar"
    #     sonar.radiation_type = Range.ULTRASOUND
    #     sonar.field_of_view = 0.1
    #     sonar.min_range = 0.2
    #     sonar.max_range = 4.0
    #     sonar.range = 3.0
    #     pub.publish(sonar)
    #     rate.sleep()


if __name__ == "__main__":
    """
    Entrypoint — launch the publisher and exit cleanly on ROS shutdown.

    Only ROS interruptions are swallowed; unexpected exceptions surface with a
    full traceback, aiding debugging.
    """
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
