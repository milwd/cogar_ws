#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from tiago1.msg import Voice_rec
from tiago1.srv import robotstatedecision, robotstatedecisionRequest


class TaskManager:
    def __init__(self):
        rospy.init_node("task_manager_node")
        rospy.wait_for_service('/robot_state_decision')  
        self.server_client = rospy.ServiceProxy('/robot_state_decision', robotstatedecision)  
        rospy.Subscriber("/feedback_acion",String,self.feed_callback_state)
        self.dialogue_pub = rospy.Publisher("/speaker_channel", String, queue_size=0)
        self.state = None

    def feed_callback_state(self,msg):
        self.state=msg
        
    def change_state(self):
        self.send_request()        
        
    def send_request(self):
        if self.state is not None:
            try:
                request = robotstatedecisionRequest() 
                request.state_input = str(self.state.data)
                response = self.server_client(request)  
                rospy.loginfo(f"Response from server: {response.success}")
                
                if response.state_output == "Busy":
                    if response.order == []:
                        mss = String(f"Order obtained before, getting to it!")
                        self.dialogue_pub.publish(mss)
                    else:
                        mss = String(f"I will get to {response.id_client}, whose order is {response.order} ")
                        self.dialogue_pub.publish(mss)
                elif response.state_output == "Wait":
                    mss = String(f"Wait, I've got better things to do")
                    self.dialogue_pub.publish(mss)
                
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    try:
        task_manager = TaskManager()
        rate = rospy.Rate(0.3)
        while not rospy.is_shutdown():
            task_manager.change_state()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass