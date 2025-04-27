#!/usr/bin/env python
"""
control_wheel.py

ROS action server for driving the TIAGo robot’s base. Receives a path goal and uses
a simple proportional controller to publish velocity commands on '/cmd_vel/wheel'.
"""

import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tiago1.msg import MovementControlAction, MovementControlFeedback, MovementControlResult
import time
import sys

class ControlMovementServer:
    """
    Action server that follows a given navigation path.

    Attributes
    ----------
    cmd_pub : rospy.Publisher
        Publishes Twist messages to '/cmd_vel/wheel' for base movement.
    server : actionlib.SimpleActionServer
        Handles MovementControlAction goals on the 'movement_control' namespace.
    """

    def __init__(self):
        """
        Initialize the movement control server.

        - Advertise '/cmd_vel/wheel' for velocity commands.
        - Set up and start the SimpleActionServer for MovementControlAction.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_control_movement_node')
        self.cmd_pub = rospy.Publisher(f'/{self.robot_number}/cmd_vel/wheel', Twist, queue_size=10)
        self.server = actionlib.SimpleActionServer(
            f'/{self.robot_number}/movement_control',
            MovementControlAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("[ControlMovement] Action server started")

    def execute_cb(self, path):
        """
        Execute a movement goal by following the first waypoint in the path.

        Parameters
        ----------
        path : tiago1.msg.MovementControlGoal
            Contains a nav_msgs/Path in path.path.poses to follow.

        Process
        -------
        1. Check for preemption; if requested, cancel and return.
        2. Create a Twist command using a proportional controller on the first waypoint:
           - linear.x and linear.y scaled from pose.position.x.
        3. Publish the command to cmd_pub.
        4. Publish feedback.status indicating progress.
        5. Sleep to allow the robot to move to the waypoint.
        6. On completion, set result.success = True and signal action succeeded.
        """
        feedback = MovementControlFeedback()
        result = MovementControlResult()

        rospy.loginfo("[ControlMovement] Received movement command")

        # Handle preemption
        if self.server.is_preempt_requested():
            rospy.logwarn("[ControlMovement] Preempted")
            self.server.set_preempted()
            return

        # Simple P-controller toward first waypoint
        cmd = Twist()
        cmd.linear.x = path.path.poses[0].pose.position.x * 0.01  
        cmd.linear.y = path.path.poses[0].pose.position.x * 0.01
        self.cmd_pub.publish(cmd)

        feedback.status = f"Moving to point 1/{len(path.path.poses)}"
        self.server.publish_feedback(feedback)
        rospy.sleep(2)  # time to reach each point
        time.sleep(3)

        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    """
    Main entrypoint: initialize ROS node and start ControlMovementServer.
    """
    ControlMovementServer()
    rospy.spin()
