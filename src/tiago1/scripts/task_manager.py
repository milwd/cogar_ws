#! /usr/bin/env python
"""
task_manager.py

ROS node that manages robot tasks by querying the robot state decision service.
Subscribes to action feedback and verified orders, tracks the current robot state,
and invokes the '/robot_state_decision' service to retrieve next tasks.
"""

import rospy
from std_msgs.msg import String
from tiago1.msg import Voice_rec
from tiago1.srv import robotstatedecision, robotstatedecisionRequest
import sys


class TaskManager:
    """
    Manages high-level task sequencing by interacting with the orchestration service.

    Attributes
    ----------
    server_client : rospy.ServiceProxy
        Client proxy for the '/robot_state_decision' service.
    state : str or None
        Most recent state received from '/feedback_acion'.
    m : str or None
        Most recent verified order message received from '/verif_T_manager'.
    """

    def __init__(self):
        """
        Initialize the TaskManager node.

        - Initialize ROS node 'task_manager_node'.
        - Wait for the '/robot_state_decision' service to become available.
        - Create a ServiceProxy to '/robot_state_decision'.
        - Subscribe to '/feedback_acion' for state feedback.
        - Subscribe to '/verif_T_manager' for verified orders.
        """
        self.robot_number = sys.argv[1]  # Now using argument for robot number
        rospy.init_node(f'{self.robot_number}_task_manager_node')

        # Wait for service to be ready
        rospy.wait_for_service(f'/{self.robot_number}/robot_state_decision')
        self.server_client = rospy.ServiceProxy(f'/{self.robot_number}/robot_state_decision', robotstatedecision)

        # Subscriber and Publisher
        rospy.Subscriber(f'/{self.robot_number}/feedback_acion', String, self.feed_callback_state)
        self.dialogue_pub = rospy.Publisher(f'/{self.robot_number}/speaker_channel', String, queue_size=10)

        # Internal state
        self.state_input = None

        rospy.loginfo(f"[TaskManager {self.robot_number}] Node initialized and ready.")

    def feed_callback_state(self, msg):
        """
        Callback for receiving action feedback.

        Parameters
        ----------
        msg : std_msgs.msg.String
            Message on '/feedback_acion' containing the current robot state.
        """
        rospy.loginfo(f"[TaskManager {self.robot_number}] Received feedback: {msg.data}")
        self.state_input = msg.data  # Just keep the string (not the full msg)

    def change_state(self):
        """
        Trigger a state-based service request using the latest state.
        """
        if self.state_input is not None:
            self.send_request()

    def send_request(self):
        """
        Call the '/robot_state_decision' service with the current state.

        Raises
        ------
        rospy.ServiceException
            If the service call fails.
        """
        try:
            request = robotstatedecisionRequest()
            request.state_input = self.state_input
            request.robot_id = self.robot_number
            print(f"[TaskManager {self.robot_number}] Sending request with state: {request.state_input} and robot_id: {request.robot_id}")

            response = self.server_client(request)
            if response is None:
                rospy.logerr(f"[TaskManager {self.robot_number}] Received None response from service.")
                return
            rospy.loginfo(f"[TaskManager {self.robot_number}] Server response: success={response.success}")

            # Prepare message depending on the server response
            mss = String()

            if response.state_output == "Busy":
                if not response.order:  # Empty order
                    mss.data = "Order obtained before, am busy getting to it!"
                else:
                    mss.data = f"I will get to {response.id_client}, whose order is {response.order}."

            elif response.state_output == "Wait":
                mss.data = "ok I will Wait, probably I've got better things to do."

            else:
                mss.data = f"Received unknown state output: {response.state_output}"

            self.dialogue_pub.publish(mss)
            rospy.loginfo(f"[TaskManager {self.robot_number}] Published dialogue message: {mss.data}")

        except rospy.ServiceException as e:
            rospy.logerr(f"[TaskManager {self.robot_number}] Service call failed: {e}")


if __name__ == "__main__":
    """
    Main entrypoint: instantiate TaskManager and continuously invoke change_state().
    """
    try:
        task_manager = TaskManager()
        rate = rospy.Rate(0.5)  # 0.5 Hz (every 2 seconds)
        while not rospy.is_shutdown():
            task_manager.change_state()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
