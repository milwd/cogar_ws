#!/usr/bin/env python
"""
path_planning.py
================

Skeleton **Path-planning** node — generates a *single dummy waypoint* so that
navigation stacks downstream (local planners, controllers, visualisers) have
something to chew on while the *real* global planner is under development.


Topic interface
---------------

**Subscribes**

* ``/map``  (``nav_msgs/OccupancyGrid``) – static or SLAM‐generated map.
* ``/odom`` (``nav_msgs/Odometry``) – robot pose in the map frame.

**Publishes**

* ``/planned_path`` (``nav_msgs/Path``) – single-waypoint global plan, updated at 1 Hz.


Assumptions
--------------------------
* The map frame is identical to the odometry frame (no TF transform applied).
* Goal is hard-coded at **(1.0 m, 1.0 m)** in the map frame.
* Orientation is left at its default (all zeros → identity quaternion).
* Only **one** waypoint is produced; real planners would push a sequence.


"""

import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import sys


class PathPlanner:
    """
    Lightweight wrapper around two subscribers and one publisher.

    Variables
    ----------
    map : nav_msgs.msg.OccupancyGrid or None
        Latest occupancy grid; stored *verbatim* until needed.
    odom : nav_msgs.msg.Odometry or None
        Latest robot pose; used to demonstrate dependency on localisation.
    path_pub : rospy.Publisher
        Advertises ``/planned_path``; queue size 1 is fine at 1 Hz.
    """

    # --------------------------------------------------------------------- #
    #                               SETUP                                   #
    # --------------------------------------------------------------------- #
    def __init__(self):
        """
        Register the node and hook up I/O.

        Steps
        -----
        1. Initialise ROS with the node name *path_planning_node*.
        2. Subscribe to the global map (``/map``) and robot odometry (``/odom``).
        3. Create a latched publisher for ``/planned_path`` so that late-joining
           nodes immediately receive the most recent Path.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_path_planning_node')

        self.map = None
        self.odom = None

        rospy.Subscriber(f'/{self.robot_number}/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber(f'/{self.robot_number}/odom_slam', Odometry, self.odom_callback)

        self.path_pub = rospy.Publisher(f'/{self.robot_number}/planned_path', Path, queue_size=1)

    # --------------------------------------------------------------------- #
    #                             CALLBACKS                                 #
    # --------------------------------------------------------------------- #
    def map_callback(self, msg):
        """Store the latest occupancy grid for later use."""
        self.map = msg

    def odom_callback(self, msg):
        """Store the latest odometry message."""
        self.odom = msg

    # --------------------------------------------------------------------- #
    #                       PATH-GENERATION LOGIC                           #
    # --------------------------------------------------------------------- #
    def plan_path(self):
        """
        Publish a *one-step* global plan when prerequisites are met.

        Preconditions
        -------------
        • ``self.map``           is not *None*  
        • ``self.odom``          is not *None*

        Behaviour
        ---------
        * Construct :class:`nav_msgs.msg.Path` stamped in the **map frame**.
        * Append a single :class:`geometry_msgs.msg.PoseStamped` at *(1 m, 1 m)*.
        * Publish and log at INFO level (runs every second).
        """
        if not (self.map and self.odom):
            return  # Wait until both pieces of data arrive

        # ---------- build Path message ----------------------------------- #
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.map.header.frame_id  # keep frames aligned

        # Dummy goal pose -------------------------------------------------- #
        pose = PoseStamped()
        pose.header.stamp = path.header.stamp
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = 1.0
        pose.pose.position.y = 1.0
        # Orientation left at default (identity)

        path.poses.append(pose)

        # Publish once per call (1 Hz from main loop)
        self.path_pub.publish(path)
        rospy.loginfo_throttle(30, "[PathPlanner] Published dummy path.")

    # --------------------------------------------------------------------- #
    #                               MAIN LOOP                               #
    # --------------------------------------------------------------------- #
    def loop(self):
        """Invoke :py:meth:`plan_path` at **1 Hz** until ROS shutdown."""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.plan_path()
            rate.sleep()


# ------------------------------------------------------------------------- #
#                               ENTRY POINT                                 #
# ------------------------------------------------------------------------- #
if __name__ == "__main__":
    try:
        PathPlanner().loop()
    except rospy.ROSInterruptException:
        pass
