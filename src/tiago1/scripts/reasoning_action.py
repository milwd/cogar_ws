#!/usr/bin/env python
"""
reasoning_action.py

ROS node for high-level reasoning and action sequencing. Subscribes to planned
paths and table reasoning commands, then issues movement, arm, and gripper
actions via actionlib. Publishes task feedback messages upon completion.
"""

import rospy
import actionlib
from nav_msgs.msg import Path
from std_msgs.msg import String
from tiago1.msg import MovementControlAction, MovementControlGoal
from tiago1.msg import ArmControlAction, ArmControlGoal
from tiago1.msg import GripperControlAction, GripperControlGoal
import sys

class ReasoningAction:
    """
    Coordinates robot behaviors by receiving navigation paths and table commands,
    then dispatching appropriate actionlib goals.

    Attributes
    ----------
    path_received : bool
        Flag indicating a new Path message has arrived.
    path_msg : nav_msgs.msg.Path or None
        The latest received Path message.
    task_feedback_pub : rospy.Publisher
        Publisher for task completion feedback on 'task_feedback'.
    movement_client : actionlib.SimpleActionClient
        Client for MovementControlAction.
    arm_client : actionlib.SimpleActionClient
        Client for ArmControlAction.
    gripper_client : actionlib.SimpleActionClient
        Client for GripperControlAction.
    """

    def __init__(self):
        """
        Initialize the reasoning action node:

        - Initialize ROS node 'reasoning_action_node'.
        - Subscribe to '/planned_path' and '/table_reasoning_commands'.
        - Create publisher for task feedback.
        - Instantiate actionlib clients for movement, arm, and gripper.
        - Wait until all action servers are available.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_reasoning_action_node')

        self.path_received = False
        self.path_msg = None

        rospy.Subscriber(f'/{self.robot_number}/planned_path', Path, self.path_callback)
        rospy.Subscriber(f'/{self.robot_number}/table_reasoning_commands', String, self.table_command_callback)
        self.task_feedback_pub = rospy.Publisher(f'/{self.robot_number}/feedback_acion', String, queue_size=1)

        self.movement_client = actionlib.SimpleActionClient(f'/{self.robot_number}/movement_control', MovementControlAction)
        self.arm_client = actionlib.SimpleActionClient(f'/{self.robot_number}/arm_control', ArmControlAction)
        self.gripper_client = actionlib.SimpleActionClient(f'/{self.robot_number}/gripper_control', GripperControlAction)

        self.wait_for_servers()

    def wait_for_servers(self):
        """
        Poll each action server until all are up:

        - movement_control
        - arm_control
        - gripper_control

        Logs warnings while waiting and info when all are connected.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.movement_client.wait_for_server(rospy.Duration(1.0)):
                rospy.logwarn("Waiting for movement_control action server...")
                continue
            if not self.arm_client.wait_for_server(rospy.Duration(1.0)):
                rospy.logwarn("Waiting for arm_control action server...")
                continue
            if not self.gripper_client.wait_for_server(rospy.Duration(1.0)):
                rospy.logwarn("Waiting for gripper_control action server...")
                continue
            rospy.loginfo("[ReasoningAction] All action servers connected.")
            break
        rate.sleep()

    def path_callback(self, msg):
        """
        Callback when a Path message is received.

        Parameters
        ----------
        msg : nav_msgs.msg.Path
            The planned path to follow.
        """
        self.path_received = True
        self.path_msg = msg

    def perform_table_command(self, decision):
        """
        Execute a table reasoning command.

        Parameters
        ----------
        decision : str
            The command string from table reasoning, e.g. "PLACE", "CLEAR".
        """
        # TODO: implement specific arm/gripper motions based on decision
        pass

    def table_command_callback(self, msg):
        """
        Callback when a table reasoning command is received.

        Parameters
        ----------
        msg : std_msgs.msg.String
            Contains the decision string in msg.data.
        """
        decision = msg.data
        rospy.loginfo(f"[ReasoningAction] Received table command: {decision}")
        self.perform_table_command(decision)

    def loop(self):
        """
        Main loop: when a new path arrives, send movement goal and then
        conditionally send arm and gripper goals. Publish feedback on success
        or failure and reset the path flag.
        """
        rate = rospy.Rate(0.3)
        while not rospy.is_shutdown():
            if self.path_received:
                self.task_feedback_pub.publish("Busy")
                rospy.loginfo("[ReasoningAction] Received planned path, sending to ControlMovement.")

                move_goal = MovementControlGoal(path=self.path_msg)
                self.movement_client.send_goal(move_goal)
                self.movement_client.wait_for_result()
                move_result = self.movement_client.get_result()
                if move_result.success:
                    rospy.loginfo(
                        "[ReasoningAction] Movement succeeded. Sending commands to arm and gripper."
                    )
                    arm_goal = ArmControlGoal(degree=3)
                    self.arm_client.send_goal(arm_goal)
                    self.arm_client.wait_for_result()

                    gripper_goal = GripperControlGoal(gripnogrip=True)
                    self.gripper_client.send_goal(gripper_goal)
                    self.gripper_client.wait_for_result()

<<<<<<< HEAD
                    self.task_feedback_pub.publish("Free")
                else:
                    rospy.logwarn("[ReasoningAction] Movement failed, not sending arm/gripper commands.")
                    self.task_feedback_pub.publish("Free")

                self.path_received = False
            rate.sleep()


if __name__ == '__main__':
    """
    Main entrypoint: instantiate ReasoningAction and run its loop.
    """
    try:
        node = ReasoningAction()
        node.loop()
    except rospy.ROSInterruptException:
        pass
