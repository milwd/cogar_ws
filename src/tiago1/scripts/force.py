#!/usr/bin/env python
"""
force.py
========

Synthetic **end-effector force sensor** for manipulation pipelines
-----------------------------------------------------------------

Grasp controllers, compliant planners and safety monitors often rely on a
continuous force signal.  When real hardware is absent (desktop development,
CI/CD, remote teaching) this node publishes *pseudo-random* force values so
that the rest of the stack can still run and be exercised.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Direction / type
     - Name
     - Semantics
   * - **publish** ``std_msgs/Float32``
     - ``/force``
     - Simulated contact force **in Newtons** (0 – 50 N)

Characteristics
---------------
* **Rate** 10 Hz (set by :pydata:`rate`).  
* **Distribution** Uniform *U*(0 N, 50 N).  
  Swap for a Gaussian or a replay from a CSV file if you need repeatable
  sequences or more realistic dynamics.

The node shuts down automatically on Ctrl-C or ``rosnode kill``.
"""

import random
import rospy
from std_msgs.msg import Float32
import random
import sys


def force_sensor() -> None:
    """
    Emit fake force values at 10 Hz until ROS shutdown.

    Steps
    -----
    #. Initialise the node as **force**.  
    #. Advertise ``/force``.  
    #. In the main loop generate a random float, publish it, sleep.
    """
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_force')
    # pub = rospy.Publisher('/force', Float32, queue_size=10)
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     pub.publish(Float32(random.uniform(0, 5)))
    #     rate.sleep()



# ---------------------------------------------------------------------- #
#                               bootstrap                                #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    force_sensor()
