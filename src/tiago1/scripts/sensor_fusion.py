#!/usr/bin/env python
"""
sensor_fusion.py
================

Overview
--------
`sensor_fusion.py` is a **fusion node** that synthesizes data from a planar
LiDAR scanner and a single-beam sonar into a unified `sensor_msgs/LaserScan`.
This lightweight placeholder lets sensor-fusion and obstacle-avoidance pipelines
be exercised without complex probabilistic filters or real hardware.

Design goals
------------
* **Simplicity** – embed the sonar reading into the LiDAR fan by element-wise addition.  
* **Determinism** – produces the same fused scan for reproducible bench tests.  
* **Decoupling** – downstream nodes subscribe only to `/fused_scan`, oblivious to
  the individual sensors.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic
     - Message type / Notes
   * - **Required** (sub)
     - ``/{robot}/lidar``
     - ``sensor_msgs/LaserScan`` – 180° planar scan
   * - **Required** (sub)
     - ``/{robot}/sonar``
     - ``sensor_msgs/Range`` – single-beam ultrasound
   * - **Provided** (pub)
     - ``/{robot}/fused_scan``
     - ``sensor_msgs/LaserScan`` – LiDAR + centred sonar reading fused

Contract
--------
**Pre-conditions**  

• LiDAR and sonar topics publish at compatible rates (~10 Hz).  
• LiDAR `ranges` length matches the pseudo-scan length generated from sonar.

**Post-conditions**  

• Each call to `publish_fused()` emits exactly one fused `LaserScan`.  
• Fused `ranges` = element-wise sum of LiDAR ranges and the adapted sonar array.

**Invariants**  

• Sonar reading is placed at the centre index; all other beam values are unchanged.

Implementation notes
--------------------
* Fusion is attempted on every sonar callback and in a fixed-rate loop.  
* `Adapter` expands a scalar sonar reading into a zero-filled list with the value
  at the centre.  
* Holds no extra state beyond the last messages, ideal for hot-reload and tests.
"""

import rospy
from sensor_msgs.msg import LaserScan, Range
import sys


class SensorFusionNode:
    """
    Merge LiDAR and sonar information into a single LaserScan.

    Variables
    ----------
    lidar_data : sensor_msgs.msg.LaserScan | None
        Last LiDAR scan received.
    sonar_data : sensor_msgs.msg.Range | None
        Last sonar reading received.
    adapted_sonar : list[float]
        Sonar value expanded to match LiDAR beam count.
    """

    def __init__(self):
        """
        Register the node, wire subscriptions and publisher, and initialise state.
        """
        self.robot_number = sys.argv[1]  # rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_sensor_fusion_node')

        self.lidar_data = None
        self.sonar_data = None

        self.lidar_sub = rospy.Subscriber(
            f'/{self.robot_number}/lidar',
            LaserScan,
            self.lidar_callback,
        )
        self.sonar_sub = rospy.Subscriber(
            f'/{self.robot_number}/sonar',
            Range,
            self.sonar_callback,
        )
        self.pub = rospy.Publisher(
            f'/{self.robot_number}/fused_scan',
            LaserScan,
            queue_size=10,
        )

    def lidar_callback(self, data: LaserScan) -> None:
        """
        Store the most recent LiDAR scan.

        Parameters
        ----------
        data : sensor_msgs.msg.LaserScan
            Scan received from `/lidar`.
        """
        self.lidar_data = data

    def sonar_callback(self, data: Range) -> None:
        """
        Store the sonar reading and trigger a fusion attempt.

        Parameters
        ----------
        data : sensor_msgs.msg.Range
            Range measurement received from `/sonar`.
        """
        self.sonar_data = data
        self.publish_fused()

    def merge_data(self) -> list[float]:
        """
        Combine the LiDAR and adapted sonar arrays element-wise.

        Returns
        -------
        list[float]
            Sum of both sensors; length == len(self.lidar_data.ranges).
        """
        return [
            self.lidar_data.ranges[i] + self.adapted_sonar[i]
            for i in range(len(self.lidar_data.ranges))
        ]

    def publish_fused(self) -> None:
        """
        Produce and publish a fused LaserScan if both inputs are ready.

        Workflow
        --------
        1. Verify both `self.lidar_data` and `self.sonar_data` are present.  
        2. Use `Adapter` to expand the scalar sonar reading into a list
           matching the LiDAR beam count.  
        3. Call `merge_data()` to sum the two arrays.  
        4. Populate a new `LaserScan`, copy LiDAR header and geometry fields,
           attach fused `ranges`, and publish.
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
    Expand a scalar sonar range into a list matching the LiDAR beam count.

    The sonar value is inserted at the centre index; all other positions
    are zero so that simple addition embeds the sonar reading into the LiDAR fan.
    """

    def __call__(self, sonar_range: float, length: int = 0) -> list[float]:
        """
        Parameters
        ----------
        sonar_range : float
            Distance reported by the ultrasound sensor.
        length : int
            Desired output length (number of LiDAR beams).

        Returns
        -------
        list[float]
            Zero-filled list with `sonar_range` at index `length // 2`.
        """
        data = [0.0] * length
        data[length // 2] = sonar_range
        return data


if __name__ == '__main__':
    """
    Instantiate `SensorFusionNode` and publish fused scans at 10 Hz.

    The explicit loop guarantees output even if sonar readings arrive slower
    than LiDAR.
    """
    node = SensorFusionNode()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        node.publish_fused()
        rate.sleep()
