#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tiago1.msg import GripperControlAction, GripperControlFeedback, GripperControlResult

class ControlGripperServer:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel/gripper', Bool, queue_size=10)
        self.server = actionlib.SimpleActionServer(
            'gripper_control',
            GripperControlAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("[ControlGripper] Action server started")

    def execute_cb(self, gripnogrip):
        feedback = GripperControlFeedback()
        result = GripperControlResult()

        rospy.loginfo(f"[ControlGripper] Received grip command")

        # should be a loop maybe
        if self.server.is_preempt_requested():
            rospy.logwarn("[ControlGripper] Preempted")
            self.server.set_preempted()
            return

        self.cmd_pub.publish(Bool(True))

        feedback.status = "Gripping"
        self.server.publish_feedback(feedback)
        rospy.sleep(2)  # time to reach each point

        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('control_gripper_node')
    ControlGripperServer()
    rospy.spin()
