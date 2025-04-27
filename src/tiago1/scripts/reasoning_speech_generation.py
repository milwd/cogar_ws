#!/usr/bin/env python
"""
speech_generator.py
===================

Overview
--------
`speech_generator.py` is a **text-to-speech relay**: it takes plain UTF-8
sentences from the cognitive layer and republishes them on the loud-speaker
channel at a steady 1 Hz.  The node is intentionally agnostic about the actual
TTS backend—you can plug in Festival, `sound_play`, a cloud API, or a
proprietary amplifier as long as it subscribes to ``/{robot}/speaker_channel``.

Design goals
------------
* **Decoupling** – reasoning publishes text once; TTS drivers only care about
  a single downstream topic.  
* **State buffering** – latest sentence is cached so late subscribers (e.g.,
  a driver that just restarted) still get the current utterance.  
* **Rate control** – re-emits at 1 Hz, protecting slow audio pipelines from
  bursty upstream chatter.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 25 55

   * - Direction
     - Topic
     - Message type
     - Notes
   * - **Required**
     - ``/{robot}/task_speech_command``
     - ``std_msgs/String``
     - Sentences from cognition, e.g. *“Table 3, your dragon roll is ready!”*
   * - **Provided**
     - ``/{robot}/speaker_channel``
     - ``std_msgs/String``
     - Same text, forwarded once every second

Contract
--------
**Pre-conditions**  

• Upstream publishes valid UTF-8 strings.

**Post-conditions**  

• Latest received sentence is re-sent every second until a new one arrives.  
• If no sentence has ever been received the node publishes **nothing** (avoids
  empty utterances).

Implementation summary
----------------------
1. Subscriber callback caches incoming message → ``self.msg``.  
2. Main loop publishes ``self.msg`` once/second if present.  
3. Holds **no additional state**, so the node can be hot-reloaded or replaced
   in unit tests without side-effects.

"""

import rospy
from std_msgs.msg import String
import sys


class SpeechGenerator:
    """
    Thin, timer-driven **republisher** that buffers the latest sentence.

    Attributes
    ----------
    msg : std_msgs.msg.String | None
        Cached sentence; ``None`` until the first message arrives.
    pub_speak : rospy.Publisher
        Outgoing channel for TTS drivers.
    """

    # ------------------------------------------------------------------ #
    #                      NODE INITIALISATION                           #
    # ------------------------------------------------------------------ #
    def __init__(self):
        """
        1. Initialise ROS under ``{robot}_speech_gen_node``.  
        2. Subscribe to upstream speech commands.  
        3. Advertise speaker channel.  
        """
        self.robot_number = sys.argv[1]            # namespace for multi-robot
        rospy.init_node(f'{self.robot_number}_speech_gen_node')

        self.msg: String | None = None

        rospy.Subscriber(
            f'/{self.robot_number}/task_speech_command',
            String,
            self._cb_speaker,
        )
        self.pub_speak = rospy.Publisher(
            f'/{self.robot_number}/speaker_channel',
            String,
            queue_size=10,
        )

        rospy.loginfo("[SpeechGenerator] Node ready.")

    # ------------------------------------------------------------------ #
    #                           CALLBACK                                 #
    # ------------------------------------------------------------------ #
    def _cb_speaker(self, msg: String):
        """Cache newest sentence for periodic replay."""
        self.msg = msg
        rospy.loginfo(f"[SpeechGenerator] Buffered: {msg.data!r}")

    # ------------------------------------------------------------------ #
    #                      PERIODIC REPUBLISHER                          #
    # ------------------------------------------------------------------ #
    def publish_msg(self) -> None:
        """Forward cached sentence if available."""
        if self.msg is not None:
            self.pub_speak.publish(self.msg)


# ---------------------------------------------------------------------- #
#                               MAIN LOOP                                #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    """
    One-line while-spin at 1 Hz until ROS is shut down.
    """
    try:
        node = SpeechGenerator()
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            node.publish_msg()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
