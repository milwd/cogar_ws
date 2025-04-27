#! /usr/bin/env python
"""
lidar.py
========

Synthetic **2-D LiDAR** publisher for bench-testing navigation stacks
--------------------------------------------------------------------

This node emulates a planar laser scanner so that path-planning, SLAM or
obstacle-avoidance components can be developed on a laptop without any real
sensor or recorded bags.


Published topic
---------------
``/lidar``  ( ``sensor_msgs/LaserScan`` )   10 Hz, 180 beams

Message layout
~~~~~~~~~~~~~~
=======================  ======================================================
Field                    Value
-----------------------  ------------------------------------------------------
``header.frame_id``      ``"laser"``
``angle_min``            –π / 2 rad   (–90 °)
``angle_max``            +π / 2 rad   (+90 °)
``angle_increment``      π / 180 rad  (1 ° per beam)
``time_increment``       0 s (all beams considered simultaneous)
``range_min``            0.1 m  (typical safety threshold)
``range_max``            10.0 m
``ranges``               181 floats, each **5.0 m**
=======================  ======================================================

Change any of the numeric constants to mimic a different sensor model.
"""

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import sys


def publish():
    """
    Continuously construct and transmit **LaserScan** messages.

    Workflow
    --------
    #. **Initialise ROS** under the name ``lidar_node`` (anonymous to allow
       multiple instances).  
    #. **Advertise** the ``/lidar`` topic.  
    #. Enter a **10 Hz loop** that:

       a. Allocates a fresh ``LaserScan`` instance.  
       b. Fills the :pyattr:`~sensor_msgs.msg.LaserScan.header` with the
          current time stamp and the fixed frame ``"laser"``.  
       c. Sets angular limits and increment so that the scan spans 180 degrees
          with 1-degree resolution.  
       d. Populates :pyattr:`~sensor_msgs.msg.LaserScan.ranges` with a constant
          distance (*5 m* in this example).  
       e. Publishes the message.  
       f. Sleeps the remaining time to honour the 10 Hz rate.

    Notes
    -----
    * ``time_increment`` is left at 0 because the entire fake scan is assumed
      to be captured instantaneously.
    """

    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f"{robot_number}_lidar_node", anonymous=True)
    pub = rospy.Publisher(f"/{robot_number}/lidar", LaserScan, queue_size=10)
    # rate = rospy.Rate(10)  
    # while not rospy.is_shutdown():
    #     scan = LaserScan()
    #     scan.header = Header()
    #     scan.header.stamp = rospy.Time.now()
    #     scan.header.frame_id = "laser"
    #     scan.angle_min = -pi / 2
    #     scan.angle_max = pi / 2
    #     scan.angle_increment = pi / 180
    #     scan.time_increment = 0.0
    #     scan.range_min = 0.1
    #     scan.range_max = 10.0
    #     scan.ranges = [5.0] * int((scan.angle_max - scan.angle_min) / scan.angle_increment)
    #     pub.publish(scan)
    #     rate.sleep()


if __name__ == "__main__":
    """
    Entrypoint — start the publisher and exit cleanly on ROS shutdown.

    Only the ROS interruption is swallowed so that genuine bugs still raise a
    traceback, which simplifies debugging.
    """
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
