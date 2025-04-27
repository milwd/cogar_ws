#! /usr/bin/env python
"""
reasoning_order_verification.py

ROS node that verifies voice-recognized orders against a predefined menu.
Subscribes to '/voice_recogn' for voice commands, parses phrases like
"Can I have <dish>", matches against valid dishes, publishes verified orders
to '/verif_T_manager', notifies errors on '/error_from_interaction', and
forwards orders via the '/robot_state_decision_add' service.
"""

import rospy
from std_msgs.msg import String, Int32
from tiago1.msg import Voice_rec
<<<<<<< HEAD

from tiago1.srv import send_order,send_orderRequest
import sys


class ReasoningOrderVerification:
    """
    Verifies voice orders and interacts with the orchestration service.

    Variables
    ----------
    server : int
        Identifier for the orchestration service (placeholder).
    food_list : list of str
        Valid dish names to recognize in voice commands.
    msg : std_msgs.msg.String or None
        Last received voice recognition message.
    server_client : rospy.ServiceProxy
        Client proxy for the '/robot_state_decision_add' send_order service.
    error_notification : rospy.Publisher
        Publishes error codes (Int32) on '/error_from_interaction'.
    verific_taskmanager : rospy.Publisher
        Publishes verified orders (Voice_rec) on '/verif_T_manager'.
    counter_id : int
        Unique identifier counter for each order.
    error_code : int
        Last error code: 0=no error, 1=no dish found, 2=missing request phrase.
    """
    def __init__(self, server_robots,food_list):
        """
        Initialize the ReasoningOrderVerification node.

        Parameters
        ----------
        server_robots : int
            Placeholder for robot server identifier.
        food_list : list of str
            List of valid dishes to match in voice commands.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_reasoning_order_verification_node')
        self.msg=None
        self.server = server_robots
        self.food_list = food_list
        rospy.Subscriber(f'/{self.robot_number}/voice_recogn',String,self.string_callback)
        rospy.wait_for_service(f'/{self.robot_number}/robot_state_decision_add')  
        self.server_client = rospy.ServiceProxy(f'/{self.robot_number}/robot_state_decision_add', send_order)  
        self.error_notification = rospy.Publisher(f'/{self.robot_number}/error_from_interaction', Int32, queue_size=10)
        self.verific_taskmanager = rospy.Publisher(f'/{self.robot_number}/verif_T_manager', Voice_rec, queue_size = 10)
        self.counter_id = 0
        self.error_code = 0 # 0-> no error, 1-> no dish in order, 2-> order doesnt have "can I have"

               
    def string_callback(self,msg):
        """
        Callback invoked when a voice recognition message arrives.

        Parameters
        ----------
        msg : std_msgs.msg.String
            The raw voice recognition output.
        """
        self.msg=msg
        

            
    def parse_order(self):
        """
        Parse and verify the latest voice command, then publish or report errors.

        Process
        -------
        1. Log the received message.
        2. Check that self.msg contains "Can I have".
        3. Extract valid dishes from self.food_list present in the message.
        4. If dishes found:
           a. Create a Voice_rec order with id_client and list_of_orders.
           b. Publish on '/verif_T_manager' and increment counter_id.
        5. If no dishes found or phrase missing:
           a. Set error_code (1=no dish, 2=missing phrase), publish on '/error_from_interaction'.
           b. Reset error_code to 0.
        """
        # rospy.loginfo(f"Received voice message: {self.msg}")
        if self.msg is not None and self.msg != '':
            # rospy.loginfo(f"Received voice message: {self.msg.data}")
            if "Can I have" in self.msg.data:
                found_items = [food for food in self.food_list if food in self.msg.data]
                # rospy.loginfo(f"Found items: {found_items}")
                if found_items:
                    order = Voice_rec()
                    order.id_client = self.counter_id
                    for item in found_items:
                        order.list_of_orders.append(item)
                    self.send_request(order)
                    self.verific_taskmanager.publish(order)
                    # rospy.loginfo(f"Published verified order: {order}")
                    self.counter_id += 1
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
        """
        Send a verified order to the orchestration service.

        Parameters
        ----------
        order : tiago1.msg.Voice_rec
            Order message containing id_client and list_of_orders fields.
        """
        try:
            request = send_orderRequest()
            request.order = order
            response = self.server_client(request)
            rospy.loginfo(f"Response from service: {response.message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    """
    Main entrypoint: instantiate the verification node and run parse_order() at 1 Hz.
    """
    try:    
        dish_list = ["ragu", "sugo", "pasta", "kebab", "sushi","riso", "frnach", "shit", "hamburger","steak"]
        server_robots =1
        reasonerOrderVerification = ReasoningOrderVerification(server_robots,dish_list)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            verifier.parse_order()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr("ReasoningOrderVerification node interrupted.")
        pass
