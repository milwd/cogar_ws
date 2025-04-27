#!/usr/bin/env python
"""
path_planning.py
================

Overview
--------
`path_planning.py` is a **skeleton global planner** that emits a single
dummy waypoint at 1 Hz.  It lets downstream local planners, controllers and
visualisers run before the real global planner is available.

Design goals
------------
* **Bring-up** – ensure `/planned_path` exists so navigation stacks can start.  
* **Contract documentation** – downstream nodes see the required Path message
  and coordinate frames.  
* **Simplicity** – one waypoint only, easy to replace with a full planner.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic
     - Message type / Notes
   * - **Required**
     - ``/{robot}/map``
     - ``nav_msgs/OccupancyGrid`` – static or SLAM map
   * - **Required**
     - ``/{robot}/odom_slam``
     - ``nav_msgs/Odometry`` – robot pose in map frame
   * - **Provided**
     - ``/{robot}/planned_path``
     - ``nav_msgs/Path`` – single-PoseStamped at (1.0, 1.0), 1 Hz

Contract
--------
**Pre-conditions**  

• Messages published on `/map` and `/odom_slam` share the same frame.  
• Both topics must have published at least once before planning.

**Post-conditions**  

• Every second, exactly one `nav_msgs/Path` is published.  
• Path header uses the map’s `frame_id` and current time stamp.  
• A single `PoseStamped` goal at coordinates (1.0, 1.0) is appended.

**Invariants**  

• Planner does not block; if data is missing it waits silently.  
• Orientation of the goal pose remains the default identity quaternion.

Implementation notes
--------------------
* Use `rospy.Rate(1)` to enforce 1 Hz.  
* `queue_size=1` on the publisher to always keep the latest plan.  
* Replace `plan_path()` with a full search algorithm when ready.
"""

import sys
import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped


class PathPlanner:
    """
    Skeleton path planner that publishes a one-step plan each second.

    Variables
    ----------
    map_msg : nav_msgs.msg.OccupancyGrid | None
        Latest map message.
    odom_msg : nav_msgs.msg.Odometry | None
        Latest odometry message.
    path_pub : rospy.Publisher
        Publisher for `/planned_path`.
    """

    def __init__(self):
        """
        1. Read `robot_id` from CLI.
        2. Initialise ROS node `<robot>_path_planning_node`.
        3. Subscribe to `/map` and `/odom_slam`.
        4. Advertise `/planned_path` with queue_size=1.
        """
        self.robot_number = sys.argv[1]  # e.g. "R1"
        rospy.init_node(f'{self.robot_number}_path_planning_node')

        self.map_msg = None
        self.odom_msg = None

        rospy.Subscriber(
            f'/{self.robot_number}/map',
            OccupancyGrid,
            self.map_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            f'/{self.robot_number}/odom_slam',
            Odometry,
            self.odom_callback,
            queue_size=1,
        )

        self.path_pub = rospy.Publisher(
            f'/{self.robot_number}/planned_path',
            Path,
            queue_size=1,
        )
        rospy.loginfo("[PathPlanner] Node initialised – waiting for map & odom.")

    def map_callback(self, msg: OccupancyGrid) -> None:
        """
        Store the latest occupancy grid.

        Parameters
        ----------
        msg : nav_msgs.msg.OccupancyGrid
            Static or SLAM-produced map.
        """
        self.map_msg = msg

    def odom_callback(self, msg: Odometry) -> None:
        """
        Store the latest odometry.

        Parameters
        ----------
        msg : nav_msgs.msg.Odometry
            Robot pose in the map frame.
        """
        self.odom_msg = msg

    def plan_path(self) -> None:
        """
        Publish a one-step Path when both map and odometry are available.

        Preconditions
        -------------
        • `self.map_msg` and `self.odom_msg` are not None.

        Workflow
        --------
        1. Create a `nav_msgs/Path`, stamp it with `rospy.Time.now()` and set
           `frame_id` to `self.map_msg.header.frame_id`.  
        2. Create a `geometry_msgs/PoseStamped` at (1.0, 1.0) in the map frame.  
        3. Append the pose to `path.poses`.  
        4. Publish on `/planned_path` and log at INFO level.
        """
        if not (self.map_msg and self.odom_msg):
            return

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.map_msg.header.frame_id

        goal = PoseStamped()
        goal.header.stamp = path.header.stamp
        goal.header.frame_id = path.header.frame_id
        goal.pose.position.x = 1.0
        goal.pose.position.y = 1.0
        # Orientation defaults to identity quaternion

        path.poses.append(goal)
        self.path_pub.publish(path)
        rospy.loginfo_throttle(30, "[PathPlanner] Published dummy path to (1.0, 1.0).")

    def loop(self) -> None:
        """
        Run `plan_path()` at 1 Hz until ROS shuts down.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.plan_path()
            rate.sleep()


if __name__ == "__main__":
    try:
        PathPlanner().loop()
    except rospy.ROSInterruptException:
        pass
