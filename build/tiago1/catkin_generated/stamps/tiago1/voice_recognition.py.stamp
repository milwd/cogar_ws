#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, String
# from tiago1.msg import Voice_rec


class VoiceRecognition:
    def __init__(self, voice_strings):
        rospy.init_node("voice_recognition_node")
        self.data = None
        self.stringDS = voice_strings
        rospy.Subscriber('/mic_channel', Int32, self.mic_callback)
        self.voice_recognized_pub =rospy.Publisher('/voice_recogn', String, queue_size = 10)
        # self.voice_rec_message = Voice_rec()
    def mic_callback(self, msg):
        self.data = msg  # Access the value of the number inside the callback
    def process_data(self):    
        if self.data is not None and self.data.data in self.stringDS.keys():
            self.voice_recognized_pub.publish(self.stringDS[self.data.data])
            
            
if __name__ == "__main__":
    try:
        keys = [0, 1, 2, 3, 4, 5]
        strings =["Hi there","Can I have some pasta","Can I have some pasta, pizza, sushi","Can I have","weee","Woowwww"]
        voice_strings = {keys[i]: strings[i] for i in range(len(keys))}
        voiceRecog = VoiceRecognition(voice_strings)
        while not rospy.is_shutdown():
            voiceRecog.process_data()
            
    except rospy.ROSInterruptException:
        pass
            

