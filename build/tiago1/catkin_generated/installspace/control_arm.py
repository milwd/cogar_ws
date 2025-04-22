#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Twist
from tiago1.msg import ArmControlAction, ArmControlFeedback, ArmControlResult
from std_msgs.msg import Int32


class ControlArmServer:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.encoder_sub = rospy.Subscriber('/encoder_arm', Int32, self.encoder_callback)
        self.server = actionlib.SimpleActionServer(
            'arm_control',
            ArmControlAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("[ControlArm] Action server started")
        self.encoder_val = 0

    def encoder_callback(self, value):
        self.encoder_val = value.data

    def execute_cb(self, degree):
        feedback = ArmControlFeedback()
        result = ArmControlResult()

        rospy.loginfo(f"[ControlArm] Received path with {1} points")

        while (self.encoder_val != degree):
            if self.server.is_preempt_requested():
                rospy.logwarn("[ControlArm] Preempted")
                self.server.set_preempted()
                return

            # silly p-controller
            cmd = Twist()
            cmd.angular.z = degree * 0.01  
            self.cmd_pub.publish(cmd)

            feedback.status = f"Moving to angle {1}"
            self.server.publish_feedback(feedback)
            rospy.sleep(1)  # time to reach each point

        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('control_movement_node')
    ControlArmServer()
    rospy.spin()
