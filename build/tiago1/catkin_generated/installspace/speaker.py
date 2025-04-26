#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import sys


class Speaker:
    def __init__(self):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_speaker_node')
        self.msg = None
        rospy.Subscriber(f'/{self.robot_number}/speaker_channel',String, self.speaker_callback)
        self.pub_speak = rospy.Publisher(f'/{self.robot_number}/speaker_output',String,queue_size=10)
        
    def speaker_callback(self,msg):
        self.msg = msg
    
    def publish_msg(self):
        self.pub_speak.publish(self.msg)

        
if __name__ == "__main__":
    try:
        speaker = Speaker()
        while not rospy.is_shutdown():
            speaker.publish_msg()
    except rospy.ROSInterruptException:
        pass
