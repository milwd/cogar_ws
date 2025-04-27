#! /usr/bin/env python
"""
sensor_fusion.py
================

ROS **fusion node** that combines planar LiDAR and single-beam sonar
--------------------------------------------------------------------

The node listens to

* ``/lidar``   ( :sensor_msgs:`LaserScan` )  
* ``/sonar``  ( :sensor_msgs:`Range` — ultrasound )

and emits a **synthetic, fused scan** on

* ``/fused_scan``  ( :sensor_msgs:`LaserScan` )

The fusion strategy is deliberately simple:

#. Convert the sonar’s *scalar* range into a **pseudo-scan** whose length
   matches the LiDAR beam count.  
#. Insert the sonar reading at the centre index, leave all other values «0».  
#. Add the two arrays element-wise.  

This yields a quick placeholder that exercises downstream consumers without
complex probabilistic filters.

Topic contracts
---------------

+-------------+-----------------------+------+-----------------------------------------------+
| Topic       | Type                  | Dir. | Notes                                         |
+=============+=======================+======+===============================================+
| ``/lidar``  | LaserScan             | sub  | 180 ° LiDAR produced by *lidar.py*            |
+-------------+-----------------------+------+-----------------------------------------------+
| ``/sonar``  | Range                 | sub  | 1-D ultrasound produced by *sonar.py*         |
+-------------+-----------------------+------+-----------------------------------------------+
| ``/fused_scan`` | LaserScan         | pub  | LiDAR beams + centred sonar reading           |
+-------------+-----------------------+------+-----------------------------------------------+

"""

import rospy
from sensor_msgs.msg import LaserScan, Range
import sys


class SensorFusionNode:
    """
    Merge LiDAR and sonar information into a single :class:`LaserScan`.

    Variables
    ----------
    
    lidar_sub : rospy.Subscriber
        Receives scans from ``/lidar``.
    sonar_sub : rospy.Subscriber
        Receives ranges from ``/sonar``.
    pub : rospy.Publisher
        Emits the fused scan on ``/fused_scan``.
    lidar_data : sensor_msgs.msg.LaserScan | None
        Last LiDAR scan received.
    sonar_data : sensor_msgs.msg.Range | None
        Last sonar reading received.
    adapted_sonar : list[float]
        Sonar value replicated into a list matching the LiDAR beam count.
    """

    # ------------------------------------------------------------------ #
    # Initialisation                                                     #
    # ------------------------------------------------------------------ #

    def __init__(self):
        """
        Register the node, wire subscriptions and publisher and
        initialise state containers.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_sensor_fusion_node')
        self.lidar_sub = rospy.Subscriber(f'/{self.robot_number}/lidar', LaserScan, self.lidar_callback)
        self.sonar_sub = rospy.Subscriber(f'/{self.robot_number}/sonar', Range, self.sonar_callback)
        self.pub = rospy.Publisher(f'/{self.robot_number}/fused_scan', LaserScan, queue_size=10)
        self.lidar_data = None
        self.sonar_data = None

    # ------------------------------------------------------------------ #
    # Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def lidar_callback(self, data: LaserScan) -> None:
        """
        Store the most recent LiDAR scan.

        Parameters
        ----------
        data : sensor_msgs.msg.LaserScan
            Scan received from ``/lidar``.
        """
        self.lidar_data = data

    def sonar_callback(self, data: Range) -> None:
        """
        Store the sonar reading **and** trigger a fusion attempt.

        Parameters
        ----------
        data : sensor_msgs.msg.Range
            Range measurement received from ``/sonar``.
        """
        self.sonar_data = data
        self.publish_fused()            # try to fuse as soon as both inputs exist

    # ------------------------------------------------------------------ #
    # Fusion logic                                                       #
    # ------------------------------------------------------------------ #
    def merge_data(self) -> list[float]:
        """
        Combine the LiDAR and adapted sonar arrays element-wise.

        Returns
        -------
        list[float]
            ``len == len(self.lidar_data.ranges)`` – sum of both sensors.
        """
        return [
            self.lidar_data.ranges[i] + self.adapted_sonar[i]
            for i in range(len(self.lidar_data.ranges))
        ]

    def publish_fused(self) -> None:
        """
        Produce and publish a fused :class:`LaserScan` if both inputs are ready.

        Workflow
        --------
        #. Verify that **both** ``self.lidar_data`` *and* ``self.sonar_data`` are
           populated.  
        #. Use :class:`Adapter` to expand the single sonar value into a list
           whose length equals the LiDAR beam count.  
        #. Call :meth:`merge_data` to add the two arrays.  
        #. Populate a fresh :class:`LaserScan`, copy the LiDAR header
           (time + frame), insert geometry fields, attach the fused range list
           and publish.
        """
        if not (self.lidar_data and self.sonar_data):
            return

        beams = len(self.lidar_data.ranges)
        self.adapted_sonar = Adapter()(self.sonar_data.range, beams)

        fused = LaserScan()
        fused.header = self.lidar_data.header          # reuse timestamp/frame
        fused.angle_min = self.lidar_data.angle_min
        fused.angle_max = self.lidar_data.angle_max
        fused.angle_increment = self.lidar_data.angle_increment
        fused.time_increment = self.lidar_data.time_increment
        fused.range_min = self.lidar_data.range_min
        fused.range_max = self.lidar_data.range_max
        fused.ranges = self.merge_data()

        self.pub.publish(fused)


class Adapter:
    """
    Expand a *scalar* sonar range into a list matching the LiDAR beam count.

    The sonar value is inserted at the **centre index**; all other positions
    are filled with zero so that simple addition effectively “embeds” the sonar
    reading into the LiDAR fan.
    """
    def __call__(self, sonar_data, length=0):
        """
        Parameters
        ----------
        sonar_range : float
            The distance reported by the ultrasound sensor.
        length : int
            Desired output length (number of LiDAR beams).

        Returns
        -------
        list[float]
            Zero-initialised list with ``sonar_range`` at index ``length // 2``.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        new_data = [0] * length
        new_data[length // 2] = sonar_data
        return new_data


# ---------------------------------------------------------------------- #
# Main                                                                   #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    """
    Instantiate :class:`SensorFusionNode` and publish fused scans at 10 Hz.

    The explicit loop guarantees output even if new sonar readings arrive
    slower than the LiDAR frequency.
    """
    node = SensorFusionNode()
    rate = rospy.Rate(10)               # 10 Hz

    while not rospy.is_shutdown():
        node.publish_fused()
        rate.sleep()
