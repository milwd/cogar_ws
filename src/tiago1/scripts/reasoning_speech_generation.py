#! /usr/bin/env python
"""
speech_generator.py

ROS node for publishing speech commands to a speaker channel. Subscribes to
'/task_speech_command' for incoming text commands and republishes them on
'/speaker_channel' at 1 Hz.
"""

import rospy
from std_msgs.msg import String
import sys


class SpeechGenerator:
    """
    Subscribes to speech command topics and republishes messages to the robot’s speaker.

    Attributes
    ----------
    msg : std_msgs.msg.String or None
        Most recent speech command message received.
    pub_speak : rospy.Publisher
        Publisher for outgoing speech on '/speaker_channel'.
    """

    def __init__(self):
        """
        Initialize the SpeechGenerator node:

        - Initialize ROS node 'speech_gen_node'.
        - Subscribe to '/task_speech_command' for speech commands.
        - Advertise '/speaker_channel' for speaking output.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_speech_gen_node')
        self.msg = None
        rospy.Subscriber(f'/{self.robot_number}/task_speech_command', String, self.speaker_callback)
        self.pub_speak = rospy.Publisher(f'/{self.robot_number}/speaker_channel', String, queue_size=10)
        
    def speaker_callback(self, msg):
        """
        Callback invoked when a new speech command arrives.

        Parameters
        ----------
        msg : std_msgs.msg.String
            Text to be spoken by the robot.
        """
        self.msg = msg
    
    def publish_msg(self):
        """
        Publish the most recent speech command to the speaker channel.

        Process
        -------
        - If a message has been received, republish it on '/speaker_channel'.
        """
        if self.msg is not None:
            self.pub_speak.publish(self.msg)

if __name__ == "__main__":
    """
    Main entrypoint: instantiate SpeechGenerator and publish messages at 1 Hz.
    """
    try:
        speecher = SpeechGenerator()
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            speecher.publish_msg()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
