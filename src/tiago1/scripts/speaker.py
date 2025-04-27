#! /usr/bin/env python
"""
speaker.py

ROS node for relaying speech output. Subscribes to '/speaker_channel' for incoming text
commands and republishes them on '/speaker_output' for the robot’s speaker system.
"""

import rospy
from std_msgs.msg import String
import sys


class Speaker:
    """
    Subscribes to an intermediate speech channel and republishes messages
    to the final speaker output.

    Attributes
    ----------
    msg : std_msgs.msg.String or None
        The most recent speech message received.
    pub_speak : rospy.Publisher
        Publisher for outgoing speech on '/speaker_output'.
    """

    def __init__(self):
        """
        Initialize the Speaker node:

        - Initialize ROS node 'speaker_node'.
        - Subscribe to '/speaker_channel' for intermediate speech commands.
        - Advertise '/speaker_output' for the final spoken output.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_speaker_node')
        self.msg = None
        rospy.Subscriber(f'/{self.robot_number}/speaker_channel',String, self.speaker_callback)
        self.pub_speak = rospy.Publisher(f'/{self.robot_number}/speaker_output',String,queue_size=10)
        
    def speaker_callback(self, msg):
        """
        Callback invoked when a new speech command arrives.

        Parameters
        ----------
        msg : std_msgs.msg.String
            The incoming text to be spoken.
        """
        self.msg = msg
    
    def publish_msg(self):
        """
        Publish the most recent speech message to the speaker output.

        Process
        -------
        - If a message has been received (self.msg is not None), republish it
          on '/speaker_output'.
        """
        if self.msg is not None:
            self.pub_speak.publish(self.msg)


if __name__ == "__main__":
    """
    Main entrypoint: instantiate Speaker and continuously publish messages at loop rate.
    """
    try:
        speaker = Speaker()
        while not rospy.is_shutdown():
            speaker.publish_msg()
    except rospy.ROSInterruptException:
        pass
