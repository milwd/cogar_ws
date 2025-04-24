#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Twist
from tiago1.msg import ArmControlAction, ArmControlFeedback, ArmControlResult
from std_msgs.msg import Int32
import time

class ControlArmServer:
    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel/arm', Twist, queue_size=10)
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

    def execute_cb(self, goal):
        feedback = ArmControlFeedback()
        result = ArmControlResult()
        target = goal.degree
        tolerance = 5
        timeout = 10  # seconds
        start_time = time.time()

        rospy.loginfo(f"[ControlArm] Target degree: {target}")

        rate = rospy.Rate(10)
        while abs(self.encoder_val - target) > tolerance:
            if rospy.is_shutdown():
                rospy.logwarn("[ControlArm] ROS shutdown detected")
                return

            if self.server.is_preempt_requested():
                rospy.logwarn("[ControlArm] Preempted")
                self.server.set_preempted()
                return

            if time.time() - start_time > timeout:
                rospy.logwarn("[ControlArm] Timeout reached")
                result.success = False
                self.server.set_aborted(result)
                return

            cmd = Twist()
            error = target - self.encoder_val
            cmd.angular.z = 0.01 * error
            self.cmd_pub.publish(cmd)

            feedback.status = f"Moving to angle {target}, current: {self.encoder_val}"
            self.server.publish_feedback(feedback)

            rate.sleep()

        rospy.loginfo("[ControlArm] Target reached")
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    try:
        rospy.init_node('control_arm_node')
        ControlArmServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
