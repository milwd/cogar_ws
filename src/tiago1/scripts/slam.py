#!/usr/bin/env python
"""
slam.py
=======

Minimal **SLAM façade** — publishes a blank map + identity odometry so the
navigation stack can launch before the real SLAM backend is ready.

Purpose
-------
* **Bring-up**: rviz, move_base and other consumers expect `/map` and `/odom`
  to exist; this stub unblocks integration work in early project stages.
* **Interface contract**: documents *exactly* which message types and frames
  the future SLAM solution must honour.

Topic interface
---------------
**Subscribes**

* ``/fused_scan`` (``sensor_msgs/LaserScan``) – pre-filtered 2-D scan produced
  by the sensor-fusion node.

**Publishes**

* ``/map``  (``nav_msgs/OccupancyGrid``) – 10 × 10 cell grid, 5 cm resolution,
  centred at 0,0; currently all *unknown/free* (value 0).  
* ``/odom`` (``nav_msgs/Odometry``) – identity pose in ``odom`` frame, zero
  twist; child frame ``base_link``.

Coordinate frames
-----------------
``map``  ←— identity TF —→ ``odom``  ←— [[ real TF broadcaster ]] —→ ``base_link``

The stub does **not** broadcast TF transforms; downstream nodes must rely on
static transforms or their own broadcasters until a full SLAM stack replaces
this script.

Replace-me sections
-------------------
* ``update_map(data)`` – insert occupancy grid update from scan + pose.
* ``update_pose(data)`` – insert state-estimation output (EKF, graph-SLAM…).

"""

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import sys



class SLAM:
    """
    Straight-forward container for publishers + subscriber.

    Variables
    ----------
    map_pub
        Latched publisher for the occupancy grid so late subscribers
        receive the most recent map instantly.
    odom_pub
        Latched publisher for odometry for the same reason.
    """

    # ------------------------------------------------------------------ #
    #                               SET-UP                               #
    # ------------------------------------------------------------------ #
    def __init__(self):
        """
        Initialise the ROS node and wire its single input + two outputs.

        Steps
        -----
        1. Call :pyfunc:`rospy.init_node` with a descriptive name.
        2. Subscribe to the pre-filtered **fused** laser scan.  Queue size 1
           keeps latency low while scans stream in (20–30 Hz typical).
        3. Advertise both outputs with *latch=True* so tools like rviz see
           the very first map/odom without waiting a full second.
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

        rospy.Subscriber("/fused_scan", LaserScan, self.callback,  queue_size=1)

        self.map_pub = rospy.Publisher(
            "/map",  OccupancyGrid, queue_size=1, latch=True)
        self.odom_pub = rospy.Publisher(
            "/odom", Odometry,        queue_size=1, latch=True)

        rospy.loginfo("[SLAM] Stub node running — publishing blank map + odom.")

    # ------------------------------------------------------------------ #
    #                             CALLBACK                               #
    # ------------------------------------------------------------------ #
    def callback(self, scan_msg):
        """
        Handle each incoming fused scan.

        Parameters
        ----------
        scan_msg : sensor_msgs.msg.LaserScan
            Currently ignored; present to show where real SLAM would ingest
            range data.
            

        Pipeline
        --------
        1. **Blank map** – 10 × 10 cells, 0.05 m resolution, centred at
           (-2.5, -2.5) so (0,0) lies roughly in the middle.  All cell values
           initialised to *0* (free/unknown).  
        2. **Identity odometry** – pose = (0,0,0) + unit quaternion,
           twist = (0,0,0).  
        3. Publish both messages.  In a real implementation steps 1 & 2 would
           be replaced by *update_map* and *update_pose* respectively.
        """
        # ---------- 1. Build blank occupancy grid ----------------------- #
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"

        map_msg.info.resolution = 0.05          # 5 cm per grid cell
        map_msg.info.width  = 10                # 10 cells → 0.5 m
        map_msg.info.height = 10
        map_msg.info.origin.position.x = -2.5   # centre map around (0,0)
        map_msg.info.origin.position.y = -2.5
        map_msg.info.origin.orientation.w = 1

        map_msg.data = [0] * (map_msg.info.width * map_msg.info.height)

        # ---------- 2. Build identity odometry -------------------------- #
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


# ---------------------------------------------------------------------- #
#                               MAIN LOOP                                #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    slam = SLAM()
    rate = rospy.Rate(1)  # keep script alive; real scans drive updates
    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
