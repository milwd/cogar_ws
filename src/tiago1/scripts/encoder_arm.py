#!/usr/bin/env python
"""
encoder_arm.py
==============

Overview
--------
`encoder_arm.py` is a **synthetic joint-position sensor** that publishes
pseudo-random encoder angles for the TIAGo arm at a fixed rate.  It enables
controllers, loggers, and dashboards to run when no real hardware is connected.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic
     - Semantics
   * - **Provided**
     - ``/{robot}/encoder_arm``
     - ``std_msgs/Int32`` – simulated joint angle in degrees [0–359]

Contract
--------
**Pre-conditions**  

• Node launched with a valid `robot_id` CLI argument.  
• Downstream subscribers expect arm encoder ticks at ~10 Hz.

**Post-conditions**  

• Publishes exactly one `Int32` per loop iteration.  
• `data` field is an integer uniformly drawn from [0, _MAX_ANGLE].  
• No other side-effects or retained state.

**Invariants**  

• Loop frequency = `_RATE` Hz ± ROS scheduler jitter.  
• Angle range fixed by `_MAX_ANGLE`.

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
     - Keeps controllers in sync.
   * - Latency
     - **< 100 ms**
     - Prevents stale joint readings.
   * - CPU load
     - **< 1 %**
     - Safe on onboard CPU.

Implementation notes
--------------------
* Uses `random.randint(0, _MAX_ANGLE)` for angle generation.  
* All logic resides in `arm_encoder()`; the main loop only maintains timing.  
* Seed the RNG (`random.seed(...)`) before calling for reproducible traces.
"""

import sys
import random
import rospy
from std_msgs.msg import Int32

_RATE = 10         # Hz
_MAX_ANGLE = 359   # maximum joint angle in degrees

def arm_encoder() -> None:
    """
    Initialise ROS publisher and broadcast random encoder angles until shutdown.

    Workflow
    --------
    1. Read `robot_id` from CLI and initialise node `<robot>_encoder_arm>`.  
    2. Advertise `/robot/encoder_arm` (queue_size=10).  
    3. Loop at `_RATE` Hz:

       a. Generate `angle = randint(0, _MAX_ANGLE)`.  

       b. Publish `Int32(angle)`.  

       c. Sleep to maintain rate.
    """
    if len(sys.argv) < 2:
        rospy.logerr("Usage: encoder_arm.py <robot_id>")
        return

    robot_id = sys.argv[1]
    rospy.init_node(f"{robot_id}_encoder_arm")
    pub = rospy.Publisher(f"/{robot_id}/encoder_arm", Int32, queue_size=10)

    rate = rospy.Rate(_RATE)
    while not rospy.is_shutdown():
        angle = random.randint(0, _MAX_ANGLE)
        pub.publish(Int32(data=angle))
        rate.sleep()

if __name__ == "__main__":
    try:
        arm_encoder()
    except rospy.ROSInterruptException:
        pass
