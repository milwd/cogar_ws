#! /usr/bin/env python
"""
slam.py

ROS node stub for Simultaneous Localization and Mapping (SLAM). Subscribes to
fused LaserScan data, publishes an empty OccupancyGrid map and Odometry message.
Replace stub logic with actual SLAM algorithms to update map and robot pose.
"""

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import sys


class SLAM:
    """
    SLAM node that generates and publishes map and odometry from sensor data.

    Attributes
    ----------
    map_pub : rospy.Publisher
        Publisher for the OccupancyGrid map on '/map'.
    odom_pub : rospy.Publisher
        Publisher for Odometry messages on '/odom'.
    """

    def __init__(self):
        """
        Initialize the SLAM node:

        - Initialize ROS node 'slam_node'.
        - Subscribe to '/fused_scan' topic for LaserScan messages.
        - Advertise '/map' and '/odom' topics.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_slam_node')
        rospy.Subscriber(f'/{self.robot_number}/fused_scan', LaserScan, self.callback)
        rospy.Subscriber(f'/{self.robot_number}/odom_proc', Odometry, self.callback_odom)
        self.map_pub = rospy.Publisher(f'/{self.robot_number}/map', OccupancyGrid, queue_size=1)
        self.odom_pub = rospy.Publisher(f'/{self.robot_number}/odom_slam', Odometry, queue_size=1)
        self.vel = Odometry()
    def callback_odom(self, data):
        self.vel = data
        # self.odom_pub.publish(data)


    def callback(self, data):
        """
        Callback for incoming fused LaserScan data.

        Parameters
        ----------
        data : sensor_msgs.msg.LaserScan
            Incoming fused scan to be used for SLAM.

        Process
        -------
        1. Create an empty OccupancyGrid with fixed resolution, size, and origin.
        2. Initialize map_msg.data as zeros (unknown/free space).
        3. Create an empty Odometry message with identity pose and zero twist.
        4. Publish both map_msg and odom_msg.
        5. Notes in code indicate where to replace with real SLAM updates.
        """
        # Build empty map
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = 0.05
        map_msg.info.width = 10
        map_msg.info.height = 10
        map_msg.info.origin.position.x = -2.5
        map_msg.info.origin.position.y = -2.5
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.x = 0
        map_msg.info.origin.orientation.y = 0
        map_msg.info.origin.orientation.z = 0
        map_msg.info.origin.orientation.w = 1
        map_msg.data = [0] * (map_msg.info.width * map_msg.info.height)

        # Build empty odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = 0
        odom_msg.pose.pose.position.y = 0
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation.x = 0
        odom_msg.pose.pose.orientation.y = 0
        odom_msg.pose.pose.orientation.z = 0
        odom_msg.pose.pose.orientation.w = 1
        odom_msg.twist.twist.linear.x = 0
        odom_msg.twist.twist.linear.y = 0
        odom_msg.twist.twist.linear.z = 0
        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = 0

        # TODO: replace stub with actual SLAM updates:
        # map_msg.data = self.update_map(data)
        # odom_msg.pose.pose.position.x = self.update_position(data)
        # odom_msg.pose.pose.orientation.z = self.update_orientation(data)
        self.odom_pub.publish(self.vel)
        self.map_pub.publish(map_msg)

if __name__ == '__main__':
    """
    Main entrypoint: instantiate SLAM node and publish map+odom at 1 Hz.
    """
    slam = SLAM()
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            # Force callback with dummy data if needed, or rely on incoming scans
            # slam.callback(dummy_data)  # remove if real scans drive updates
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
