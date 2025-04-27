#!/usr/bin/env python
"""
encoder_gripper.py
==================

Overview
--------
`encoder_gripper.py` is a **dummy gripper-position sensor** that publishes
pseudo-random gripper opening widths at a fixed rate.  It enables PID loops,
dashboards and loggers to run when the physical gripper is offline or during CI.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic
     - Semantics
   * - **Provided**
     - ``/{robot}/encoder_gripper``
     - ``std_msgs/Int32`` – simulated gripper opening width in mm (0–100)

Contract
--------
**Pre-conditions**  

• Node launched with a valid `robot_id` CLI argument.  
• Subscribers expect gripper widths at ~10 Hz.

**Post-conditions**  

• Publishes exactly one `Int32` per loop iteration.  
• `data` field is an integer uniformly drawn from [0, _MAX_WIDTH].  
• No internal state retained beyond the RNG.

**Invariants**  

• Loop frequency = `_RATE` ± ROS scheduler jitter.  
• Width range fixed by `_MAX_WIDTH`.

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
     - Keeps gripper controllers in sync.
   * - Latency
     - **< 100 ms**
     - Prevents stale position readings.
   * - CPU load
     - **< 1 %**
     - Safe on embedded CPU.

Implementation notes
--------------------
* Uses `random.randint(0, _MAX_WIDTH)` for uniform width generation.  
* All logic is in `gripper_encoder()`; the main loop maintains timing.  
* Seed the RNG (`random.seed(...)`) before calling for reproducible tests.
"""

import sys
import random
import rospy
from std_msgs.msg import Int32

_RATE = 10         # publication rate in Hz
_MAX_WIDTH = 100   # maximum gripper opening in mm

def gripper_encoder() -> None:
    """
    Initialise ROS publisher and broadcast random gripper widths until shutdown.

    Workflow
    --------
    1. Read `robot_id` from CLI and initialise node `<robot>_encoder_gripper>`.  
    2. Advertise `/robot/encoder_gripper` (queue_size=10).  
    3. Loop at `_RATE` Hz:

       a. Generate `width = randint(0, _MAX_WIDTH)`.  

       b. Publish `Int32(width)`.  

       c. Sleep to maintain rate.
    """
    if len(sys.argv) < 2:
        rospy.logerr("Usage: encoder_gripper.py <robot_id>")
        return

    robot_id = sys.argv[1]
    rospy.init_node(f"{robot_id}_encoder_gripper")
    pub = rospy.Publisher(f"/{robot_id}/encoder_gripper", Int32, queue_size=10)

    rate = rospy.Rate(_RATE)
    while not rospy.is_shutdown():
        width = random.randint(0, _MAX_WIDTH)
        pub.publish(Int32(data=width))
        rate.sleep()

if __name__ == "__main__":
    try:
        gripper_encoder()
    except rospy.ROSInterruptException:
        pass
