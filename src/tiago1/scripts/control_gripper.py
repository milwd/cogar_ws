#!/usr/bin/env python
"""
control_gripper.py

ROS action server for operating the TIAGo robot’s gripper mechanism.
Receives grip/release goals via actionlib and publishes Boolean commands on
'/cmd_vel/gripper'. Provides feedback through the action interface.
"""

import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tiago1.msg import GripperControlAction, GripperControlFeedback, GripperControlResult
import time
import sys


class ControlGripperServer:
    """
    Action server that opens or closes the robot’s gripper.

    Attributes
    ----------
    cmd_pub : rospy.Publisher
        Publishes Bool messages to '/cmd_vel/gripper' (True = grip, False = release).
    server : actionlib.SimpleActionServer
        Handles GripperControlAction goals on the 'gripper_control' namespace.
    """

    def __init__(self):
        """
        Initialize the ControlGripperServer.

        - Advertise '/cmd_vel/gripper' for gripper commands.
        - Set up and start SimpleActionServer for GripperControlAction.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_control_gripper_node')
        self.cmd_pub = rospy.Publisher(f'/{self.robot_number}/cmd_vel/gripper', Bool, queue_size=10)

        self.server = actionlib.SimpleActionServer(
            f'/{self.robot_number}/gripper_control',
            GripperControlAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("[ControlGripper] Action server started")

    def execute_cb(self, gripnogrip):
        """
        Execute a gripper control goal.

        Parameters
        ----------
        gripnogrip : tiago1.msg.GripperControlGoal
            Contains the desired action: grip (True) or release (False).

        Process
        -------
        1. Check for preemption; if requested, set action to preempted.
        2. Publish the Bool command (True to grip, False to release).
        3. Publish feedback indicating the gripper status.
        4. Wait briefly to allow the gripper to actuate.
        5. Set result.success = True and mark action as succeeded.
        """
        feedback = GripperControlFeedback()
        result = GripperControlResult()

        rospy.loginfo("[ControlGripper] Received grip command")

        # control loop
        if self.server.is_preempt_requested():
            rospy.logwarn("[ControlGripper] Preempted")
            self.server.set_preempted()
            return

        # do control
        self.cmd_pub.publish(Bool(True))
        feedback.status = "Gripping"
        self.server.publish_feedback(feedback)
        time.sleep(2)  
        # end control loop

        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    ControlGripperServer()
    rospy.spin()
