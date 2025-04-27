#! /usr/bin/env python
"""
microphone.py
=============

Synthetic **audio channel selector** for fleet experiments
----------------------------------------------------------

Real automatic-speech-recognition (ASR) chains output an *integer label* that
identifies the “best” sentence detected on a microphone array.  
While ASR is unavailable (CI, headless simulation, bandwidth-limited VPN)
**microphone.py** fills the gap by publishing a uniformly random label at a
fixed rate.

ROS interfaces
~~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Direction / type
     - Topic
     - Semantics
   * - **publish** ``std_msgs/Int32``
     - ``/<robot_id>/mic_channel``
     - Encoded utterance ID (0 – 5)

Node arguments & parameters
---------------------------
``robot_id`` *(positional CLI argument)*  
    String prefix that namespaces the topic and the node.  
    Example: ``python microphone.py R1`` publishes on ``/R1/mic_channel``.

Quality-of-Service KPIs
-----------------------
.. list-table::
   :header-rows: 1
   :widths: 25 20 55

   * - Metric
     - Target
     - Rationale
   * - Message rate
     - **1 Hz ± 0.1 Hz**
     - Keeps the downstream voice recogniser in sync
   * - End-to-end latency
     - **< 200 ms**
     - Prevents stale voice commands
   * - Packet loss
     - **< 0.1 %**
     - Ensures behaviour planners always receive input
   * - CPU load (1 instance)
     - **< 1 %**
     - Safe to run on the TIAGo’s on-board computer


Implementation notes
--------------------
* The distribution is *U*(0, 5); change ``_RANGE`` if you add more sentences.
* All computation happens in :pyfunc:`publish`; the main loop merely calls it
  and sleeps to maintain the KPI rate.

"""

import sys
import numpy as np
import rospy
from std_msgs.msg import Int32

_RANGE = 6        # labels 0 … 5
_RATE  = 1        # Hz


def publish() -> None:
    """
    Draw a random label and publish it on *mic_publisher*.
    """
    value = Int32(np.random.randint(_RANGE))
    mic_publisher.publish(value)


# ---------------------------------------------------------------------- #
#                                bootstrap                               #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    try:
        robot_number = sys.argv[1]               # e.g. "R1"
        rospy.init_node(f"{robot_number}_microphone_node")

        mic_publisher = rospy.Publisher(
            f'/{robot_number}/mic_channel',
            Int32,
            queue_size=10,
        )

        rate = rospy.Rate(_RATE)                 # 1 Hz loop
        while not rospy.is_shutdown():
            publish()
            rate.sleep()

    except IndexError:
        rospy.logerr("Usage: microphone.py <robot_id>")
    except rospy.ROSInterruptException:
        pass
