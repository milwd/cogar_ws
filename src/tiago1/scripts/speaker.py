#!/usr/bin/env python
"""
speaker.py
==========

Overview
--------
`speaker.py` is the **last-mile speech relay** that takes high-level sentences
from the cognitive layer and immediately forwards them to the TTS backend.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 28 55

   * - Direction
     - Topic
     - Semantics
   * - **Required**
     - ``/{robot}/speaker_channel``
     - High-level sentences from cognition
   * - **Provided**
     - ``/{robot}/speaker_output``
     - Identical text for TTS backends (latched)

Contract
--------
**Pre-conditions**  

• Upstream publishes valid UTF-8 strings on the speaker channel.

**Post-conditions**  

• Each incoming message is republished exactly once on the output topic.  
• Late subscribers to the latched output still get the last sentence.

Implementation notes
--------------------
* Publisher uses `latch=True` and `queue_size=10` to absorb bursts.  
* Callback logs every relayed sentence at INFO level.  
* No additional state is stored.

"""

import rospy
from std_msgs.msg import String
import sys


class Speaker:
    """
    Thin ROS node that immediately forwards sentences to the output topic.
    """

    def __init__(self):
        """
        1. Read `robot_number` from CLI.  
        2. Initialise ROS node `<robot>_speaker_node`.  
        3. Advertise latched `/speaker_output`.  
        4. Subscribe to `/speaker_channel`.
        """
        self.robot_number = sys.argv[1]  # rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_speaker_node')

        # Latched publisher so late subscribers still receive the last sentence
        self.pub_speak = rospy.Publisher(
            f'/{self.robot_number}/speaker_output',
            String,
            queue_size=10,
            latch=True,
        )

        # Forward high-level sentences instantly
        rospy.Subscriber(
            f'/{self.robot_number}/speaker_channel',
            String,
            self.speaker_callback,
            queue_size=10,
        )

        rospy.loginfo("[Speaker] Node ready – awaiting sentences.")

    def speaker_callback(self, msg: String) -> None:
        """
        Forward *msg* to the `/speaker_output` topic immediately.

        Parameters
        ----------
        msg : std_msgs.msg.String
            High-level text from the cognitive layer.
        """
        self.pub_speak.publish(msg)
        rospy.loginfo(f"[Speaker] Relayed: {msg.data!r}")


if __name__ == "__main__":
    try:
        Speaker()
        rospy.spin()  # no manual loop required
    except rospy.ROSInterruptException:
        pass
