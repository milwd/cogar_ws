#!/usr/bin/env python
"""
slam.py
=======

Overview
--------
`slam.py` is a **minimal SLAM façade** that publishes a blank occupancy map
and identity odometry so that navigation stacks (e.g. `move_base`, `rviz`) can
start before a real SLAM backend is available.  It defines exactly which
message types and frames future SLAM solutions must honour.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 28 55

   * - Direction / Type
     - Topic
     - Semantics
   * - **Required** (sub)
     - ``/{robot}/fused_scan``
     - ``sensor_msgs/LaserScan`` – pre-filtered scan from sensor_fusion
   * - **Required** (sub)
     - ``/{robot}/odom_proc``
     - ``nav_msgs/Odometry`` – upstream pose estimate (ignored by stub)
   * - **Provided** (pub, latched)
     - ``/{robot}/map``
     - ``nav_msgs/OccupancyGrid`` – blank grid, 0.05 m cells, all free/unknown
   * - **Provided** (pub, latched)
     - ``/{robot}/odom_slam``
     - ``nav_msgs/Odometry`` – identity pose, zero twist, frame_id `odom`

Coordinate frames
-----------------
``map``  ←— identity —→ ``odom``  ←— real TF —→ ``base_link``  
No TF broadcaster here; downstream must supply static or real transforms.

Contract
--------
**Pre-conditions**  

• Nodes publish `/fused_scan` at ≥ 1 Hz.  
• `/odom_proc` provides any Odometry messages (not used by stub).

**Post-conditions**  

• On the first incoming scan or odom message each publisher latches and makes
  the blank map and identity odom available.  
• No further updates unless subscribers re-connect.

Implementation notes
--------------------
* Map: 10 × 10 cells, 0.05 m resolution, origin (–2.5, –2.5) so (0,0) is map centre.  
* Odometry: pose = (0,0,0), orientation = unit quaternion, twist = zero.  
* Replace `update_map` and `update_pose` stubs to integrate real SLAM algorithms.

"""

import sys
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry


class SLAM:
    """
    Stub container wiring subscribers and latched publishers.

    Variables
    ----------
    map_pub  : rospy.Publisher
        Latched publisher for the occupancy grid.
    odom_pub : rospy.Publisher
        Latched publisher for identity odometry.
    vel      : nav_msgs.msg.Odometry
        Last Odometry received from `/odom_proc` (not used by stub).
    """

    def __init__(self):
        """
        Initialise the SLAM stub node:

        1. Read `robot_number` from CLI.  
        2. Initialise ROS node `<robot>_slam_node`.  
        3. Subscribe to `/fused_scan` and `/odom_proc`.  
        4. Advertise `/map` and `/odom_slam` with `latch=True`, `queue_size=1`.
        """
        self.robot_number = sys.argv[1]  # rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_slam_node')

        # Subscribers
        rospy.Subscriber(
            f'/{self.robot_number}/fused_scan',
            LaserScan,
            self.callback,
            queue_size=1,
        )
        rospy.Subscriber(
            f'/{self.robot_number}/odom_proc',
            Odometry,
            self.callback_odom,
            queue_size=1,
        )

        # Latched publishers so first messages persist for new subscribers
        self.map_pub = rospy.Publisher(
            f'/{self.robot_number}/map',
            OccupancyGrid,
            queue_size=1,
            latch=True,
        )
        self.odom_pub = rospy.Publisher(
            f'/{self.robot_number}/odom_slam',
            Odometry,
            queue_size=1,
            latch=True,
        )

        # Placeholder for upstream odometry (unused in stub)
        self.vel = Odometry()

        rospy.loginfo("[SLAM] Stub node initialised – awaiting data.")

    def callback_odom(self, data: Odometry) -> None:
        """
        Handle incoming odometry from `/odom_proc`.

        Parameters
        ----------
        data : nav_msgs.msg.Odometry
            Upstream odometry (ignored; stored for possible future use).
        """
        self.vel = data
        # Publish identity odom immediately (latched)
        self.odom_pub.publish(self.vel)
        rospy.loginfo_once("[SLAM] Published identity odometry.")

    def callback(self, scan_msg: LaserScan) -> None:
        """
        Handle incoming fused scan from `/fused_scan`.

        Parameters
        ----------
        scan_msg : sensor_msgs.msg.LaserScan
            Pre-filtered scan (ignored by stub).
        """
        # Build blank map
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"

        map_msg.info.resolution = 0.05          # 5 cm per cell
        map_msg.info.width      = 10            # 10 cells → 0.5 m
        map_msg.info.height     = 10
        map_msg.info.origin.position.x = -2.5   # centre (0,0)
        map_msg.info.origin.position.y = -2.5
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = [0] * (map_msg.info.width * map_msg.info.height)

        # Publish blank map (latched)
        self.map_pub.publish(map_msg)
        rospy.loginfo_once("[SLAM] Published blank occupancy grid.")


if __name__ == "__main__":
    """
    Keep the node alive until shutdown. Real scans or odom messages trigger
    the latched publications above.
    """
    try:
        slam = SLAM()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
