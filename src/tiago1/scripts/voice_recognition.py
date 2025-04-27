#!/usr/bin/env python
"""
voice_recognition.py
=====================

Overview
--------
`voice_recognition.py` is a **text stub for ASR**: it converts integer “utterance
IDs” into canned sentences so downstream components (order-verifier, task-manager,
etc.) can be developed and tested without a real speech recogniser.

Design goals
------------
* **Decoupling** – cognitive layers publish simple integers; this node turns them
  into full strings.  
* **Legacy support** – listens to both namespaced and root topics, republishes
  on both.  
* **Stateless replay** – the last valid sentence is latched so late subscribers
  still receive it.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 40

   * - Direction / Type
     - Topic
     - Semantics
   * - **Required** (sub)
     - ``/{robot}/mic_channel``
     - Encoded mic “utterance” IDs (std_msgs/Int32)
   * - **Required** (sub)
     - ``/mic_channel``
     - Legacy mic channel (std_msgs/Int32)
   * - **Provided** (pub)
     - ``/{robot}/voice_recogn``
     - Recognised text (std_msgs/String)
   * - **Provided** (pub)
     - ``/voice_recogn``
     - Legacy speech topic, latched (std_msgs/String)

Contract
--------
**Pre-conditions**  

• A `voice_strings: dict[int, str]` mapping is passed to the constructor.  

**Post-conditions**  

• On each valid key, publishes exactly one sentence on both topics.  
• Unknown keys are ignored (no publish, last sentence remains latched).  

**Invariants**  

• The node resets its internal key cache after a successful publish.  
• Publisher queues absorb short bursts (`queue_size=10`, latched for legacy).

Implementation summary
----------------------
1. **Callbacks** cache incoming `Int32` messages to `self.data`.  
2. **Main loop** checks `self.data` at 10 Hz:  
   a. If `self.data` matches a key in `voice_strings`, publishes the mapped
      sentence on both topics and logs it.  
   b. Clears `self.data` to avoid replaying the same phrase.  
3. Holds no other state—perfect for hot-reload or unit tests.

"""

import rospy
from std_msgs.msg import Int32, String
import sys


class VoiceRecognition:
    """
    Minimal dictionary-based ASR emulator.
    """

    def __init__(self, voice_strings: dict[int, str]):
        """
        Parameters
        ----------
        voice_strings : dict[int, str]
            Maps Int32 `.data` values to sentences.
        """
        self.robot_number = sys.argv[1]  # rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_voice_recognition_node')

        self.data: Int32 | None = None
        self.stringDS = voice_strings

        # Subscribe to namespaced and legacy mic channels
        rospy.Subscriber(f'/{self.robot_number}/mic_channel', Int32, self._mic_callback, queue_size=10)
        rospy.Subscriber('/mic_channel',                     Int32, self._mic_callback, queue_size=10)

        # Publishers: namespaced and legacy (latched)
        self.pub_ns   = rospy.Publisher(f'/{self.robot_number}/voice_recogn', String, queue_size=10)
        self.pub_legacy = rospy.Publisher('/voice_recogn',       String, queue_size=10, latch=True)

        rospy.loginfo("[VoiceRecognition] Node ready – awaiting keys.")

    def _mic_callback(self, msg: Int32) -> None:
        """Cache the most recent microphone key."""
        self.data = msg

    def process_data(self) -> None:
        """
        If cached `self.data` matches a key, publish the mapped sentence once.
        """
        if self.data is None:
            return

        phrase = self.stringDS.get(self.data.data)
        if phrase:
            out = String(data=phrase)
            self.pub_ns.publish(out)
            self.pub_legacy.publish(out)
            rospy.loginfo(f"[VoiceRecognition] Recognised: {phrase!r}")
            self.data = None  # reset to avoid duplicates


if __name__ == "__main__":
    try:
        # Define utterance IDs → sentences
        keys    = [0, 1, 2, 3, 4, 5]
        strings = [
            "Can I have some riso, shit",
            "Can I have some pasta",
            "Can I have some pasta, pizza, sushi",
            "Can I have steak",
            "Can I have hamburger",
            "Woowwww"
        ]
        voice_strings = {keys[i]: strings[i] for i in range(len(keys))}

        vr = VoiceRecognition(voice_strings)
        rate = rospy.Rate(10)  # 10 Hz processing

        while not rospy.is_shutdown():
            vr.process_data()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
