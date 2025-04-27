#!/usr/bin/env python
"""
sonar.py
=========

Overview
--------
`sonar.py` is a **synthetic ultrasound range-finder** component that emits
constant `sensor_msgs/Range` messages at 10 Hz.  It enables sensor-fusion,
navigation and obstacle-avoidance pipelines to be exercised even without
real hardware or ROS bags.

Design goals
------------
* **Decoupling** – navigation and fusion code subscribe to a single `/sonar` topic,
  oblivious to real or dummy sensors.  
* **Determinism** – always publishes the same range value for reproducible tests.  
* **Lightweight** – no hardware dependencies or external config files required.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic
     - Message type / Notes
   * - **Provided**
     - ``/{robot}/sonar``
     - ``sensor_msgs/Range`` –  single-beam, 10 Hz

Contract
--------
**Pre-conditions**  

• Node launched with a valid `robot_id` CLI argument.  
• Downstream expects `Range.ULTRASOUND` readings at ∼10 Hz.

**Post-conditions**  

• Publishes exactly one `Range` per loop iteration.  
• `range` field is fixed at 3.0 m.  
• Header uses frame_id `"sonar"` and current time stamp.

**Invariants**  

• Loop frequency = 10 Hz ± 5%.  
• `field_of_view` = 0.1 rad, `min_range` = 0.2 m, `max_range` = 4.0 m.

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
* Uses `rospy.Rate(10)` for timing.  
* Message fields are hard-coded for simplicity; adjust constants below to
  emulate other sensors.

"""

import sys
import rospy
from sensor_msgs.msg import Range

# Constants – tweak to emulate different devices
_FRAME_ID      = "sonar"
_FOV           = 0.1   # radians
_MIN_RANGE     = 0.2   # meters
_MAX_RANGE     = 4.0   # meters
_FIXED_RANGE   = 3.0   # meters
_PUBLISH_RATE  = 10    # Hz

def publish():
    """
    Advertise `/sonar` and stream constant Range messages until shutdown.

    Workflow
    --------
    1. Initialise ROS node `<robot>_sonar_node`, anonymous=True for multiples.  
    2. Advertise `/robot/sonar` (queue_size=10).  
    3. In a 10 Hz loop:

       a. Create `sensor_msgs/Range`.  

       b. Set `header.stamp = now()`, `header.frame_id = _FRAME_ID`.  

       c. Set `radiation_type = Range.ULTRASOUND`.  

       d. Assign `_FOV`, `_MIN_RANGE`, `_MAX_RANGE`, `_FIXED_RANGE`. 
        
       e. Publish and sleep to maintain rate.
    """
    robot_id = sys.argv[1]
    rospy.init_node(f"{robot_id}_sonar_node", anonymous=True)
    pub = rospy.Publisher(f"/{robot_id}/sonar", Range, queue_size=10)
    rate = rospy.Rate(_PUBLISH_RATE)

    while not rospy.is_shutdown():
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = _FRAME_ID
        msg.radiation_type  = Range.ULTRASOUND
        msg.field_of_view   = _FOV
        msg.min_range       = _MIN_RANGE
        msg.max_range       = _MAX_RANGE
        msg.range           = _FIXED_RANGE

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
