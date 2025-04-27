#!/usr/bin/env python
"""
lidar.py
========

Overview
--------
`lidar.py` is a **synthetic 2-D LiDAR** publisher that emits planar laser scan
messages at 10 Hz.  It lets navigation, SLAM, and obstacle-avoidance stacks run
without real hardware or recorded bags.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic
     - Message type / Notes
   * - **Provided**
     - ``/{robot}/lidar``
     - ``sensor_msgs/LaserScan`` – 10 Hz, 180 beams spanning –90° to +90°.

Contract
--------
**Pre-conditions**  

• Node launched with positional `robot_id` argument.  
• Downstream subscribers expect `LaserScan` at ~10 Hz.

**Post-conditions**  

• Publishes exactly one `LaserScan` per loop iteration.  
• `ranges` list length = 181, each entry fixed at `_FIXED_RANGE` (5.0 m).  
• Header `frame_id` = `"laser"`, `stamp` = `rospy.Time.now()`.

**Invariants**  

• `angle_max – angle_min` = π radians, `angle_increment` = π/180.  
• `range_min` = 0.1 m, `range_max` = 10.0 m.

Quality-of-Service KPIs
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 25 20 45

   * - Metric
     - Target
     - Rationale
   * - Message rate
     - **10 Hz ± 0.5 Hz**
     - Keeps real-time filters stable.
   * - Latency
     - **< 50 ms**
     - Avoids stale obstacle data.
   * - CPU load
     - **< 1 %**
     - Safe on embedded CPUs.

Implementation notes
--------------------
* Uses `rospy.Rate(_RATE)` for loop timing.  
* All scan parameters defined as module-level constants for easy tweaking.  
* No state beyond the loop – perfect for hot-reload or unit tests.
"""

import sys
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# Configuration constants
_FRAME_ID        = "laser"
_ANGLE_MIN       = -math.pi / 2
_ANGLE_MAX       =  math.pi / 2
_ANGLE_INCREMENT =  math.pi / 180
_TIME_INCREMENT  =  0.0
_RANGE_MIN       =  0.1
_RANGE_MAX       = 10.0
_FIXED_RANGE     =  5.0
_RATE            = 10    # Hz

def publish():
    """
    Advertise `/lidar` and stream constant LaserScan messages until shutdown.
    """
    robot_id = sys.argv[1]  # e.g. "R1"
    rospy.init_node(f"{robot_id}_lidar_node", anonymous=True)
    pub = rospy.Publisher(f"/{robot_id}/lidar", LaserScan, queue_size=10)
    rate = rospy.Rate(_RATE)

    scan = LaserScan()
    scan.header = Header()
    scan.angle_min       = _ANGLE_MIN
    scan.angle_max       = _ANGLE_MAX
    scan.angle_increment = _ANGLE_INCREMENT
    scan.time_increment  = _TIME_INCREMENT
    scan.range_min       = _RANGE_MIN
    scan.range_max       = _RANGE_MAX
    scan.ranges          = [ _FIXED_RANGE ] * (
        int(( _ANGLE_MAX - _ANGLE_MIN ) / _ANGLE_INCREMENT) + 1
    )

    while not rospy.is_shutdown():
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = _FRAME_ID
        pub.publish(scan)
        rate.sleep()

if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            raise IndexError
        publish()
    except IndexError:
        rospy.logerr("Usage: lidar.py <robot_id>")
    except rospy.ROSInterruptException:
        pass
