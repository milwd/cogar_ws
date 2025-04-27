#!/usr/bin/env python
"""
encoder_wheel.py
================

Synthetic **wheel-odometry source** for base-controller development
-------------------------------------------------------------------

In simulation or on hardware-less CI runners, the movement stack still needs a
continuous stream of wheel encoder ticks so that dead-reckoning, PID loops and
logging tools stay active.  
`encoder_wheel.py` fulfils that role by broadcasting *pseudo-random* tick
counts at a fixed rate.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Direction / type
     - Name
     - Semantics
   * - **publish** ``std_msgs/Int32``
     - ``/encoder_wheel``
     - Raw tick count **per wheel revolution** (0 – 999)

Node behaviour
--------------
* **Frequency** 10 Hz (controlled by :pydata:`rate`).  
* **Tick generator** Uniform *U*(0, 999) – every message is independent.  
  For smoother traces swap the one-liner for a *random-walk* or sine table.

The node exits cleanly when ROS is shut down.
"""

import random
import rospy
from std_msgs.msg import Int32
import random
import sys


def wheel_encoder() -> None:
    """
    Publish fake wheel-encoder ticks until ROS shutdown.

    Steps
    -----
    #. Initialise the node as **encoder_wheel**.  
    #. Advertise ``/encoder_wheel``.  
    #. Loop at 10 Hz → generate a tick count, publish, sleep.
    """
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_encoder_wheel')
    pub = rospy.Publisher(f'/{robot_number}/encoder_wheel', Int32, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ticks = int(random.uniform(0, 999))             # 0 – 998 ticks
        pub.publish(Int32(ticks))
        rate.sleep()


# ---------------------------------------------------------------------- #
#                               bootstrap                                #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    wheel_encoder()
