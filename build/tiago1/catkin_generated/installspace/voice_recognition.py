#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32, String
# from tiago1.msg import Voice_rec
import sys


class VoiceRecognition:
    def __init__(self, voice_strings):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_voice_recognition_node')
        self.data = None
        self.stringDS = voice_strings
        rospy.Subscriber(f'/{self.robot_number}/mic_channel', Int32, self.mic_callback)
        self.voice_recognized_pub =rospy.Publisher(f'/{self.robot_number}/voice_recogn', String, queue_size = 10)
        # self.voice_rec_message = Voice_rec()
    def mic_callback(self, msg):
        self.data = msg  # Access the value of the number inside the callback
    def process_data(self):    
        if self.data is not None and self.data.data in self.stringDS.keys():
            self.voice_recognized_pub.publish(self.stringDS[self.data.data])
            
            
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
        while not rospy.is_shutdown():
            voiceRecog.process_data()
            
    except rospy.ROSInterruptException:
        pass
            

