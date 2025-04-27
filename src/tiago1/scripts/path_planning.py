#!/usr/bin/env python
"""
path_planning.py

ROS node for simple path planning. Subscribes to a map and odometry, generates a
dummy single‐waypoint path, and publishes it on '/planned_path' at 1 Hz.
"""

import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import sys

class PathPlanner:
    """
    Plans and publishes a navigation path based on the latest map and odometry.

    Attributes
    ----------
    map : nav_msgs.msg.OccupancyGrid or None
        Most recently received occupancy grid map.
    odom : nav_msgs.msg.Odometry or None
        Most recently received robot odometry.
    path_pub : rospy.Publisher
        Publishes generated Path messages on '/planned_path'.
    """

    def __init__(self):
        """
        Initialize the PathPlanner node.

        - Initialize ROS node 'path_planning_node'.
        - Subscribe to '/map' for OccupancyGrid messages.
        - Subscribe to '/odom' for Odometry messages.
        - Advertise '/planned_path' for Path messages.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_path_planning_node')

        self.map = None
        self.odom = None

        rospy.Subscriber(f'/{self.robot_number}/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber(f'/{self.robot_number}/odom_slam', Odometry, self.odom_callback)

        self.path_pub = rospy.Publisher(f'/{self.robot_number}/planned_path', Path, queue_size=1)

    def map_callback(self, msg):
        """
        Callback for receiving the map.

        Parameters
        ----------
        msg : nav_msgs.msg.OccupancyGrid
            The occupancy grid of the environment.
        """
        self.map = msg

    def odom_callback(self, msg):
        """
        Callback for receiving robot odometry.

        Parameters
        ----------
        msg : nav_msgs.msg.Odometry
            The current robot pose and twist.
        """
        self.odom = msg

    def plan_path(self):
        """
        Generate and publish a dummy path if map and odometry are available.

        Process
        -------
        1. Check that both self.map and self.odom are not None.
        2. Construct a Path message with a single PoseStamped waypoint at (1.0, 1.0).
        3. Log the action and publish the Path on '/planned_path'.
        """
        if self.map and self.odom:
            path = Path()
            path.header.stamp = rospy.Time.now()
            path.header.frame_id = self.map.header.frame_id

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = self.map.header.frame_id
            pose.pose.position.x = 1.0  # dummy goal X
            pose.pose.position.y = 1.0  # dummy goal Y

            path.poses.append(pose)

            rospy.loginfo("[PathPlanner] Publishing path to '/planned_path'.")
            self.path_pub.publish(path)

    def loop(self):
        """
        Main loop: call plan_path() at 1 Hz until shutdown.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.plan_path()
            rate.sleep()

if __name__ == '__main__':
    """
    Main entrypoint: instantiate PathPlanner and enter its loop.
    """
    try:
        planner = PathPlanner()
        planner.loop()
    except rospy.ROSInterruptException:
        pass
