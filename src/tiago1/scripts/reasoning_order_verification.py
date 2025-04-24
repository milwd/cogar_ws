#! /usr/bin/env python
import rospy
from std_msgs.msg import String, Int32
from tiago1.msg import Voice_rec

from tiago1.srv import send_order,send_orderRequest


class ReasoningOrderVerification:
    def __init__(self, server_robots,food_list):
        rospy.init_node('reasoning_order_verification')
        self.msg=None
        self.server = server_robots
        self.food_list = food_list
        rospy.Subscriber('/voice_recogn',String,self.string_callback)
        rospy.wait_for_service('/robot_state_decision')  
        self.server_client = rospy.ServiceProxy('/robot_state_decision', send_order)  
        self.error_notification = rospy.Publisher('/error_from_interaction', Int32, queue_size=10)
        self.verific_taskmanager = rospy.Publisher('/verif_T_manager', Voice_rec, queue_size = 10)
        self.counter_id =0
        self.error_code =0 # 0-> no error, 1-> no dish in order, 2-> order doesnt have "can I have"
        
               
    def string_callback(self,msg):
        self.msg=msg
    def parse_order(self):
        if self.msg is not None and self.msg.data != '':
            if "Can I have" in self.msg.data:
                found_items = [food for food in self.food_list if food in self.msg.data]
                if found_items:
                    order = Voice_rec()
                    for item in found_items:
                        order.id_client = self.counter_id
                        order.list_of_orders.append(item)
                        self.send_request(order)
                    self.verific_taskmanager.publish(order)
                    self.counter_id += 1
                    return
                else:
                    self.error_code = 1
                    self.error_notification.publish(self.error_code)
                    self.error_code = 0
                    return
            else:
                self.error_code = 2
                self.error_notification.publish(self.error_code)
                self.error_code = 0
                return
            
    def send_request(self, order):
        try:
            request = send_orderRequest()  
            request.order = order
            response = self.server_client(request)
            rospy.loginfo(f"response from server: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
if __name__ == "__main__":
    try:    
        
        dish_list = ["ragu", "sugo", "pasta", "kebab", "sushi"]
        server_robots =1
        reasonerOrderVerification = ReasoningOrderVerification(server_robots,dish_list)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            reasonerOrderVerification.parse_order()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass     
