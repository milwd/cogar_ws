#!/usr/bin/env python3
import rospy
import actionlib
from nav_msgs.msg import Path
from std_msgs.msg import String
from tiago1.msg import MovementControlAction, MovementControlGoal
from tiago1.msg import ArmControlAction, ArmControlGoal
from tiago1.msg import GripperControlAction, GripperControlGoal


class ReasoningAction:
    def __init__(self):
        rospy.init_node('reasoning_action_node')

        self.path_received = False
        self.path_msg = None

        rospy.Subscriber('/planned_path', Path, self.path_callback)
        rospy.Subscriber("/table_reasoning_commands", String, self.table_command_callback)
        self.task_feedback_pub = rospy.Publisher('feedback_acion', String, queue_size=1)

        self.movement_client = actionlib.SimpleActionClient('movement_control', MovementControlAction)
        self.arm_client = actionlib.SimpleActionClient('arm_control', ArmControlAction)
        self.gripper_client = actionlib.SimpleActionClient('gripper_control', GripperControlAction)

        self.wait_for_servers()

    def wait_for_servers(self):
        # polling for each server separately
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
        self.path_received = True
        self.path_msg = msg
    
    def perform_table_command(self, decision):
        pass

    def table_command_callback(self, msg):
        decision = msg.data
        rospy.loginfo(f"[ReasoningAction] Received table command: {decision}")
        self.perform_table_command(decision)

    def loop(self):
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
                    rospy.loginfo("[ReasoningAction] Movement succeeded. Sending commands to arm and gripper.")
                    arm_goal = ArmControlGoal(degree=3)
                    self.arm_client.send_goal(arm_goal)
                    self.arm_client.wait_for_result()

                    gripper_goal = GripperControlGoal(gripnogrip=True)
                    self.gripper_client.send_goal(gripper_goal)
                    self.gripper_client.wait_for_result()

                    self.task_feedback_pub.publish("Free")
                else:
                    rospy.logwarn("[ReasoningAction] Movement failed, not sending arm/gripper commands.")
                    self.task_feedback_pub.publish("Free")

                self.path_received = False 
            rate.sleep()


if __name__ == '__main__':
    try:
        node = ReasoningAction()
        node.loop()
    except rospy.ROSInterruptException:
        pass
