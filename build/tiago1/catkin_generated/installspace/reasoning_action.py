#!/usr/bin/env python3
import rospy
import actionlib
from nav_msgs.msg import Path
from tiago1.msg import MovementControlAction, MovementControlGoal
from tiago1.msg import ArmControlAction, ArmControlGoal
from tiago1.msg import GripperControlAction, GripperControlGoal


class ReasoningAction:
    def __init__(self):
        rospy.init_node('reasoning_action_node')

        rospy.Subscriber('/planned_path', Path, self.path_callback)

        self.movement_client = actionlib.SimpleActionClient('movement_control', MovementControlAction)
        self.arm_client = actionlib.SimpleActionClient('arm_control', ArmControlAction)
        self.gripper_client = actionlib.SimpleActionClient('gripper_control', GripperControlAction)

        rospy.loginfo("[ReasoningAction] Waiting for action servers...")
        self.movement_client.wait_for_server()
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        rospy.loginfo("[ReasoningAction] All action servers connected.")

    def path_callback(self, path_msg):
        rospy.loginfo("[ReasoningAction] Received planned path, sending to ControlMovement.")
        move_goal = MovementControlGoal(path=path_msg)
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
        else:
            rospy.logwarn("[ReasoningAction] Movement failed, not sending arm/gripper commands.")

if __name__ == '__main__':
    try:
        ReasoningAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
