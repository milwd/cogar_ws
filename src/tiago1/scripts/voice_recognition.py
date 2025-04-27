#!/usr/bin/env python
"""
voice_recognition.py

ROS node that simulates voice recognition by mapping microphone channel integers
to predefined phrases. Subscribes to '/mic_channel' (Int32) and publishes recognized
voice strings on '/voice_recogn'.
"""

import rospy
from std_msgs.msg import Int32, String
# from tiago1.msg import Voice_rec
import sys

class VoiceRecognition:
    """
    Node for mapping microphone channel inputs to voice recognition outputs.

    Attributes
    ----------
    data : std_msgs.msg.Int32 or None
        Last received microphone channel message.
    stringDS : dict of int to str
        Mapping from microphone channel values to phrases.
    voice_recognized_pub : rospy.Publisher
        Publisher for recognized voice strings on '/voice_recogn'.
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

    def mic_callback(self, msg):
        """
        Callback for incoming microphone channel data.

        Parameters
        ----------
        msg : std_msgs.msg.Int32
            The published microphone channel value.
        """
        self.data = msg

    def process_data(self):
        """
        Process the latest microphone data and publish the corresponding voice string.

        If self.data is set and maps to a known phrase in stringDS, publish it.
        """
        if self.data is not None and self.data.data in self.stringDS:
            recognized = String(data=self.stringDS[self.data.data])
            self.voice_recognized_pub.publish(recognized)

if __name__ == "__main__":
    """
    Main entrypoint: instantiate VoiceRecognition with predefined mappings and
    repeatedly call process_data() until shutdown.
    """
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
            voiceRecog.process_data()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
