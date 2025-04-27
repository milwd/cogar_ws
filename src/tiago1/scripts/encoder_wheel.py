#!/usr/bin/env python
"""
encoder_wheel.py
================

Overview
--------
`encoder_wheel.py` is a **synthetic wheel-odometry source** that emits
pseudo-random tick counts on each wheel revolution at a fixed rate.  
It enables movement stacks—dead-reckoning, PID controllers and loggers—to
run in simulation or CI without real encoder hardware.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic
     - Semantics
   * - **Provided**
     - ``/{robot}/encoder_wheel``
     - ``std_msgs/Int32`` – tick count per revolution (0–999)

Contract
--------
**Pre-conditions**  

• Node launched with valid `robot_id` CLI argument.  
• Downstream expects wheel ticks at ~10 Hz.

**Post-conditions**  

• Publishes exactly one `Int32` message each loop iteration.  
• `data` field is an integer uniformly drawn from [0, 999].  
• No other side-effects or retained state.

**Invariants**  

• Loop frequency = `_RATE` ± ROS scheduler jitter.  
• Tick range fixed by `_MAX_TICK`.

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
     - Keeps odometry loops stable.
   * - Latency
     - **< 100 ms**
     - Avoids stale encoder data.
   * - CPU load
     - **< 1 %**
     - Safe on embedded CPU.

Implementation notes
--------------------
* Uses `random.randint(0, _MAX_TICK)` for uniform tick generation.  
* All logic resides in `wheel_encoder()`; the main loop only handles timing.  
* Seed the RNG (`random.seed(...)`) before `wheel_encoder()` to reproduce traces.
"""

import sys
import random
import rospy
from std_msgs.msg import Int32

_RATE = 10        # Hz
_MAX_TICK = 999   # maximum tick count per revolution

def wheel_encoder() -> None:
    """
    Initialise ROS publisher and broadcast random tick counts until shutdown.

    Workflow
    --------
    1. Read `robot_id` from CLI and initialise node `<robot>_encoder_wheel>`.  
    2. Advertise `/robot/encoder_wheel` (queue_size=10).  
    3. Loop at `_RATE` Hz:
       a. Generate `ticks = randint(0, _MAX_TICK)`.  

       b. Publish `Int32(ticks)`.  

       c. Sleep to maintain rate.
    """
    robot_id = sys.argv[1]
    rospy.init_node(f"{robot_id}_encoder_wheel")
    pub = rospy.Publisher(f"/{robot_id}/encoder_wheel", Int32, queue_size=10)

    rate = rospy.Rate(_RATE)
    while not rospy.is_shutdown():
        ticks = random.randint(0, _MAX_TICK)
        pub.publish(Int32(data=ticks))
        rate.sleep()

if __name__ == "__main__":
    try:
        if len(sys.argv) < 2:
            raise IndexError
        wheel_encoder()
    except IndexError:
        rospy.logerr("Usage: encoder_wheel.py <robot_id>")
    except rospy.ROSInterruptException:
        pass
