#!/usr/bin/env python
"""
force.py
=========

Overview
--------
`force.py` is a **synthetic end-effector force sensor** that publishes
pseudo-random force readings so that grasp controllers, compliant planners,
and safety monitors can be developed and tested without real hardware.

Design goals
------------
* **Parallel development** – pipelines receive realistic-looking force data on CI  
* **Determinism** – uniform random forces, seedable for repeatable tests  
* **Lightweight** – no external dependencies or real sensor drivers

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic
     - Semantics
   * - **Provided**
     - ``/{robot}/force``
     - ``std_msgs/Float32`` – simulated contact force in Newtons

Contract
--------
**Pre-conditions**  

• Node launched with a valid `robot_id` CLI argument.  
• Downstream nodes subscribe to `/force` at ~10 Hz.

**Post-conditions**  

• Publishes one `Float32` message per loop iteration.  
• `data` field is a float uniformly drawn from [0, _MAX_FORCE].  

**Invariants**  

• Loop frequency = `_RATE` ± ROS scheduler jitter.  
• Force range fixed by `_MAX_FORCE`.

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
     - Ensures smooth controller input.
   * - Latency
     - **< 100 ms**
     - Prevents stale force readings.
   * - CPU load
     - **< 1 %**
     - Safe on embedded CPUs.

Implementation notes
--------------------
* Uses `random.uniform(0, _MAX_FORCE)` for force generation.  
* All logic in `force_sensor()`; the loop maintains timing.  
* Seed the RNG (`random.seed(...)`) before calling for reproducibility.
"""

import sys
import random
import rospy
from std_msgs.msg import Float32

_RATE = 10        # publication rate in Hz
_MAX_FORCE = 50.0 # maximum force in Newtons

def force_sensor() -> None:
    """
    Initialise ROS publisher and broadcast random force values until shutdown.

    Workflow
    --------
    1. Read `robot_id` from CLI and initialise node `<robot>_force>`.  
    2. Advertise `/robot/force` (Float32, queue_size=10).  
    3. Loop at `_RATE` Hz:

       a. Generate `f = uniform(0, _MAX_FORCE)`.  

       b. Publish `Float32(f)`.  
       
       c. Sleep to maintain rate.
    """
    if len(sys.argv) < 2:
        rospy.logerr("Usage: force.py <robot_id>")
        return

    robot_id = sys.argv[1]
    rospy.init_node(f"{robot_id}_force")
    pub = rospy.Publisher(f"/{robot_id}/force", Float32, queue_size=10)

    rate = rospy.Rate(_RATE)
    while not rospy.is_shutdown():
        force_val = random.uniform(0, _MAX_FORCE)
        pub.publish(Float32(data=force_val))
        rate.sleep()

if __name__ == "__main__":
    try:
        force_sensor()
    except rospy.ROSInterruptException:
        pass
