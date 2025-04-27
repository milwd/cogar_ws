#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32
from tiago1.msg import Voice_rec
import threading
from tiago1.srv import send_order,send_orderRequest
from tiago1.srv import GetNextId


import sys


class ReasoningOrderVerification:
    def __init__(self, server_robots,food_list):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_reasoning_order_verification_node')
        self.msg=None
        self.server = server_robots
        self.file_lock = threading.Lock()

        self.food_list = food_list
        rospy.Subscriber(f'/{self.robot_number}/voice_recogn',String,self.string_callback)
        rospy.wait_for_service(f'/{self.robot_number}/robot_state_decision_add')  
        self.server_client = rospy.ServiceProxy(f'/{self.robot_number}/robot_state_decision_add', send_order)  
        self.get_next_id = rospy.ServiceProxy('get_next_id', GetNextId)

        self.error_notification = rospy.Publisher(f'/{self.robot_number}/error_from_interaction', Int32, queue_size=10)
        self.verific_taskmanager = rospy.Publisher(f'/{self.robot_number}/verif_T_manager', Voice_rec, queue_size = 10)
        self.counter_id = 0
        self.error_code = 0 # 0-> no error, 1-> no dish in order, 2-> order doesnt have "can I have"

               
    def string_callback(self,msg):
        self.msg=msg
        

            
    def parse_order(self):
        # rospy.loginfo(f"Received voice message: {self.msg}")
        if self.msg is not None and self.msg != '':
            # rospy.loginfo(f"Received voice message: {self.msg.data}")
            if "Can I have" in self.msg.data:
                found_items = [food for food in self.food_list if food in self.msg.data]
                # rospy.loginfo(f"Found items: {found_items}")
                if found_items:
                    order = Voice_rec()
                    response =self.get_next_id()


                    order.id_client =  response.id()
                    
                    for item in found_items:
                        order.list_of_orders.append(item)
                    self.send_request(order)
                    self.verific_taskmanager.publish(order)
                    # rospy.loginfo(f"Published verified order: {order}")
                    
                    # rospy.set_param("id_client_counter",int(rospy.get_param("id_client_counter")) +1)
                else:
                    self.error_code = 1
                    self.error_notification.publish(self.error_code)
                    # rospy.logwarn("No valid dish found in the order.")
                    self.error_code = 0
            else:
                self.error_code = 2
                self.error_notification.publish(self.error_code)
                # rospy.logwarn("The sentence doesn't contain 'Can I have'")
                self.error_code = 0

            
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
        dish_list = ["ragu", "sugo", "pasta", "kebab", "sushi","riso", "frnach", "ramen", "hamburger","steak"]
        server_robots =1
        reasonerOrderVerification = ReasoningOrderVerification(server_robots,dish_list)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            reasonerOrderVerification.parse_order()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr(f"node failed")
        pass     
