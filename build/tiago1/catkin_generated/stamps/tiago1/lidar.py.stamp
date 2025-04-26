#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from math import pi
from std_msgs.msg import Header


def publish():
    pub = rospy.Publisher("/lidar", LaserScan, queue_size=10)
    rospy.init_node("lidar_node", anonymous=True)
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
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
# lidar.py
# This script publishes LIDAR data to the /scan topic in ROS.   
# It initializes a ROS node, creates a publisher, and continuously publishes
# LIDAR data at a rate of 10 Hz. The LIDAR data includes header information,
# angle range, angle increment, time increment, minimum and maximum range,
# and the range values. The script can be run as a standalone program.
# It uses the rospy library for ROS communication and the sensor_msgs.msg
# module for the LaserScan message type. The script handles ROS interruptions gracefully.
# The LIDAR data is published in a loop until the node is shut down.
# The script is intended to be run in a ROS environment and requires the
# appropriate ROS setup and dependencies to be installed.