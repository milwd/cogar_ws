#!/usr/bin/env python
"""
voice_recognition.py
=====================

Audio-to-text **stub** – turns a numeric microphone channel into canned sentences
----------------------------------------------------------------------------------

Real ASR engines are heavyweight and sometimes unavailable (e.g. on CI).  
This node fakes the pipeline by translating an *integer* published on
``/mic_channel`` into a pre-programmed phrase and emitting it on
``/voice_recogn``.  Down-stream components can then be developed and tested
without an actual speech recogniser.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Direction / type
     - Name
     - Semantics
   * - **subscribe** ``std_msgs/Int32``
     - ``/mic_channel``
     - Encoded microphone “utterance” (key into the lookup table)
   * - **publish** ``std_msgs/String``
     - ``/voice_recogn``
     - Plain UTF-8 sentence selected from the table

Lookup strategy
---------------
A *dict[int, str]* supplied at construction time maps each key value to the
sentence that should be “ recognised ”.  Unknown keys are ignored; the last
valid utterance remains active until a new recognised value arrives.

"""

import rospy
from std_msgs.msg import Int32, String
# from tiago1.msg import Voice_rec
import sys


class VoiceRecognition:
    """
    Minimal, dictionary-based ASR emulator.

    Instance Variables
    ------------------
    data
        Latest :pyclass:`std_msgs.msg.Int32` received on ``/mic_channel``.
    stringDS
        User-provided mapping *int → str*.
    voice_recognized_pub
        :pyclass:`rospy.Publisher` that emits recognised sentences.
    """
    def __init__(self, voice_strings):
        """
        Initialize the VoiceRecognition node.

        Parameters
        ----------
        voice_strings : dict
            Predefined mapping from Int32 data values to phrases (str).
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_voice_recognition_node')
        self.data = None
        self.stringDS = voice_strings
        rospy.Subscriber(f'/{self.robot_number}/mic_channel', Int32, self.mic_callback)
        self.voice_recognized_pub =rospy.Publisher(f'/{self.robot_number}/voice_recogn', String, queue_size = 10)
        # self.voice_rec_message = Voice_rec()

        rospy.Subscriber('/mic_channel',
                         Int32,
                         self._mic_callback,
                         queue_size=10)

        self.voice_recognized_pub = rospy.Publisher('/voice_recogn',
                                                    String,
                                                    queue_size=10,
                                                    latch=True)

        rospy.loginfo("[VoiceRecognition] Node ready – waiting for keys.")

    # ------------------------------------------------------------------ #
    #                             callback                                #
    # ------------------------------------------------------------------ #
    def _mic_callback(self, msg: Int32) -> None:
        """Cache the most recent key code coming from the “microphone.”"""
        self.data = msg

    # ------------------------------------------------------------------ #
    #                           main logic                                #
    # ------------------------------------------------------------------ #
    def process_data(self) -> None:
        """
        If :pyattr:`data` contains a known key, publish the matching sentence.
        """
        if self.data is None:
            return

        phrase = self.stringDS.get(self.data.data)
        if phrase:
            self.voice_recognized_pub.publish(String(data=phrase))
            rospy.loginfo(f"[VoiceRecognition] Recognised: {phrase!r}")
            # Reset to avoid replaying the same sentence endlessly
            self.data = None


# ---------------------------------------------------------------------- #
#                                bootstrap                               #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    try:
        keys = [0, 1, 2, 3, 4, 5]
        strings =[
                    "Can I have some riso,shit",
                    "Can I have some pasta",
                    "Can I have some pasta, pizza, sushi",
                    "Can I have steak",
                    "Can I have hamburger",
                    "Woowwww"
                  ]
        voice_strings = {keys[i]: strings[i] for i in range(len(keys))}
        voiceRecog = VoiceRecognition(voice_strings)
        rate = rospy.Rate(10)  # Adjust rate as needed
        while not rospy.is_shutdown():
            vr.process_data()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
