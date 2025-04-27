#!/usr/bin/env python
"""
encoder_gripper.py
==================

Dummy **gripper-position sensor** for controller and UI testing
---------------------------------------------------------------

When the physical TIAGo gripper is offline—or when you run the stack in CI—this
node supplies a stand-in encoder signal.  A stream of pseudo-random integers is
enough for downstream PID loops, dashboards or loggers to stay alive and be
exercised.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Direction / type
     - Name
     - Semantics
   * - **publish** ``std_msgs/Int32``
     - ``/encoder_gripper``
     - Simulated opening width **in millimetres** (0 – 100 mm)

Characteristics
---------------
* **Rate** 10 Hz (see :pydata:`rate`).  
* **Distribution** Uniform *U*(0, 100) → every tick is independent.  
  Swap for a random walk if you prefer smoother traces.

The node ends automatically on ROS shutdown.
"""

import random
import rospy
from std_msgs.msg import Int32
import random
import sys


def gripper_encoder() -> None:
    """
    Advertise ``/encoder_gripper`` and publish fake readings forever.

    Steps
    -----
    #. Initialise the node as **encoder_gripper**.  
    #. Create an :pyclass:`Int32` publisher.  
    #. In a 10 Hz loop generate a number in \ [0, 100], publish, sleep.
    """
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_encoder_gripper')
    pub = rospy.Publisher(f'/{robot_number}/encoder_gripper', Int32, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        width_mm = int(random.uniform(0, 100))        # 0 – 100 mm
        pub.publish(Int32(width_mm))
        rate.sleep()


# --------------------------------------------------------------------------- #
#                                 bootstrap                                   #
# --------------------------------------------------------------------------- #
if __name__ == '__main__':
    gripper_encoder()
