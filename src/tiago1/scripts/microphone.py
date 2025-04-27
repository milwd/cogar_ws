#! /usr/bin/env python
"""
microphone.py
=============

Overview
--------
`microphone.py` is a **synthetic audio-channel selector** that emulates the
front-end of an ASR pipeline by publishing random “utterance IDs” at a fixed
rate.  It’s ideal for CI, headless simulation, or bandwidth-constrained
environments where real ASR is unavailable.

Why use this stub?
------------------
• **Determinism** – reproducible, uniformly random utterance IDs.  
• **Parallel development** – downstream voice-recognition and order-verification
  nodes can be tested without real microphones.  
• **Resource light** – no heavy ASR models or audio I/O dependencies.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 45

   * - Direction / Type
     - Topic
     - Semantics
   * - **Provided**
     - ``/{robot}/mic_channel``
     - Encoded Int32 utterance ID ∈ [0, _RANGE)

Contract
--------
**Pre-conditions**  

• Node launched with a valid `robot_id` CLI argument.  
• `_RANGE` and `_RATE` constants set before import.

**Post-conditions**  

• Publishes exactly one `std_msgs/Int32` per loop iteration.  
• Value is uniformly random in [0, _RANGE).  
• No other side-effects or state.

**Invariants**  

• Loop frequency = `_RATE` ± 5% (dependant on ROS scheduler).  
• Peak CPU ≤ 1% per instance.

Quality-of-Service KPIs
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 25 20 45

   * - Metric
     - Target
     - Rationale
   * - Message rate
     - **1 Hz ± 0.1 Hz**
     - Keeps downstream recogniser in sync.
   * - Latency
     - **< 200 ms**
     - Avoids stale commands.
   * - Packet loss
     - **< 0.1 %**
     - Ensures planners always receive input.
   * - CPU load
     - **< 1 %**
     - Safe on TIAGo’s on-board computer.

Implementation notes
--------------------
* Draws labels via `np.random.randint(_RANGE)`.  
* All logic lives in `publish()`; the main loop only maintains timing.  
* To extend: adjust `_RANGE`, `_RATE`, or seed NumPy for reproducible tests.
"""

import sys
import numpy as np
import rospy
from std_msgs.msg import Int32

_RANGE = 6        # number of utterance IDs (0 … _RANGE-1)
_RATE  = 1        # publication rate in Hz

def publish() -> None:
    """
    Select a random utterance ID and publish it.

    Steps
    -----
    1. Generate a uniform random integer in [0, _RANGE).  
    2. Wrap it in std_msgs/Int32.  
    3. Publish on the namespaced mic_channel topic.
    """
    value = Int32(np.random.randint(_RANGE))
    mic_publisher.publish(value)
    rospy.logdebug(f"[microphone] Published utterance ID: {value.data}")

# ---------------------------------------------------------------------- #
#                                ENTRY POINT                             #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    """
    Bootstrap the synthetic microphone node:

    1. Read `robot_id` from CLI.
    2. Initialise ROS node.
    3. Create a Publisher for `/robot_id/mic_channel`.
    4. Enter a fixed-rate loop to call `publish()`.
    """
    try:
        robot_id = sys.argv[1]  # e.g. "R1"
        rospy.init_node(f"{robot_id}_microphone_node", log_level=rospy.DEBUG)

        mic_publisher = rospy.Publisher(
            f"/{robot_id}/mic_channel",
            Int32,
            queue_size=10,
        )

        rate = rospy.Rate(_RATE)
        while not rospy.is_shutdown():
            publish()
            rate.sleep()

    except IndexError:
        rospy.logerr("Usage: microphone.py <robot_id>")
    except rospy.ROSInterruptException:
        pass
