#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from tiago1.srv import robotstatedecision

class TaskManager:
    def __init__(self):
        rospy.init_node("task_manager_node")
        rospy.wait_for_service('/robot_state_decision')  
        self.server_client = rospy.ServiceProxy('/robot_state_decision', robotstatedecision)  
        rospy.Subscriber("/feedback_acion",String,self.feed_callback_state)
        self.state = None
    def feed_callback_state(self,msg):
        self.state=msg.data
        
    def change_state(self):
        self.send_request()
        
        
        
    def send_request(self):
        if self.state is not None:
            try:
                request = robotstatedecision() 
                request.state_input =self.state
                response = self.server_client(request)  
                rospy.loginfo(f"Response from server: {response.message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    try:
        task_manager = TaskManager()
        while not rospy.is_shutdown():
            task_manager.change_state()
    except rospy.ROSInterruptException:
        pass