#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tiago1.msg import MovementControlAction, MovementControlFeedback, MovementControlResult
import time
import sys

class ControlMovementServer:
    def __init__(self):
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
        feedback = MovementControlFeedback()
        result = MovementControlResult()

        rospy.loginfo(f"[ControlMovement] Received movement command")

        # should be a loop maybe
        if self.server.is_preempt_requested():
            rospy.logwarn("[ControlMovement] Preempted")
            self.server.set_preempted()
            return

        # silly p-controller
        cmd = Twist()
        cmd.linear.x = path.path.poses[0].pose.position.x * 0.01  
        cmd.linear.y = path.path.poses[0].pose.position.x * 0.01
        self.cmd_pub.publish(cmd)

        feedback.status = f"Moving to point {1}/{len(path.path.poses)}"
        self.server.publish_feedback(feedback)
        rospy.sleep(2)  # time to reach each point
        time.sleep(3)

        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    ControlMovementServer()
    rospy.spin()
