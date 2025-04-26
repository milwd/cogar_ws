#! /usr/bin/env python
import rospy
from std_msgs.msg import String
import sys


class SpeechGenerator:
    def __init__(self):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_speech_gen_node')
        self.msg = None
        rospy.Subscriber(f'/{self.robot_number}/task_speech_command', String, self.speaker_callback)
        self.pub_speak = rospy.Publisher(f'/{self.robot_number}/speaker_channel', String, queue_size=10)
        
    def speaker_callback(self,msg):
        self.msg = msg
    
    def publish_msg(self):
        self.pub_speak.publish(self.msg)

        
if __name__ == "__main__":
    try:
        speecher = SpeechGenerator()
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            speecher.publish_msg()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
