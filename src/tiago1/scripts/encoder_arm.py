#!/usr/bin/env python
"""
encoder_arm.py
==============

Synthetic **joint-position sensor** for development without hardware
-------------------------------------------------------------------

Many nodes—controllers, loggers, dashboards—need a continuous stream of arm
encoder data.  When the real TIAGo arm is not connected, this script fakes the
signal by emitting *pseudo-random* readings at a fixed rate so that the rest of
the stack can still run and be tested.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Direction / type
     - Name
     - Semantics
   * - **publish** ``std_msgs/Int32``
     - ``/encoder_arm``
     - Simulated joint angle in **degrees** (0 – 359)

Behaviour
---------
* **Frequency** 10 Hz (configurable via the :pydata:`rate` variable).  
* **Value generator** A plain call to :pymeth:`random.uniform(0, 359)` rounded
  to an integer gives the appearance of a freely moving joint without any
  kinematic constraints.  Replace this with a *random walk* or a deterministic
  table if you need smoother motion or repeatability.

The node quits automatically when ROS shuts down (Ctrl-C or ``rosnode kill``).
"""

import random
import rospy
from std_msgs.msg import Int32
import random
import sys


def arm_encoder() -> None:
    """
    Spin a 10 Hz loop that publishes fake encoder positions.

    Steps
    -----
    #. Initialise the ROS node under the name **encoder_arm**.  
    #. Advertise ``/encoder_arm`` as an :pyclass:`Int32` publisher.  
    #. Loop until shutdown: generate a random angle, publish, then sleep to
       maintain the target frequency.
    """
<<<<<<< HEAD
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_encoder_arm')
    pub = rospy.Publisher(f'/{robot_number}/encoder_arm', Int32, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        angle_deg = int(random.uniform(0, 359))             # 0 – 358°
        pub.publish(Int32(angle_deg))
        rate.sleep()


# ---------------------------------------------------------------------- #
#                                bootstrap                               #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    arm_encoder()
