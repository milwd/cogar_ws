#!/usr/bin/env python
"""
control_arm.py

ROS action server for controlling the TIAGo robot arm. Receives target angles,
monitors encoder feedback, and publishes velocity commands until the arm reaches
the desired position or a timeout/preemption occurs.
"""

import rospy
import actionlib
from geometry_msgs.msg import Twist
from tiago1.msg import ArmControlAction, ArmControlFeedback, ArmControlResult
from std_msgs.msg import Int32
import time
import sys

class ControlArmServer:
    """
    Action server that moves the robot arm to a specified angle.

    Attributes
    ----------
    cmd_pub : rospy.Publisher
        Publishes Twist commands to '/cmd_vel/arm' for arm movement.
    encoder_sub : rospy.Subscriber
        Subscribes to '/encoder_arm' to receive current encoder readings.
    server : actionlib.SimpleActionServer
        Handles ArmControlAction goals on the 'arm_control' namespace.
    encoder_val : int
        Latest encoder position (in degrees).
    """

    def __init__(self):
        """
        Initialize publishers, subscriber, and action server.

        - Advertise '/cmd_vel/arm' for sending velocity commands.
        - Subscribe to '/encoder_arm' for encoder feedback.
        - Set up and start SimpleActionServer for ArmControlAction.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_control_arm_node')
        self.cmd_pub = rospy.Publisher(f'/{self.robot_number}/cmd_vel/arm', Twist, queue_size=10)
        self.encoder_sub = rospy.Subscriber(f'/{self.robot_number}/encoder_arm', Int32, self.encoder_callback)
        
        self.server = actionlib.SimpleActionServer(
            f'/{self.robot_number}/arm_control',
            ArmControlAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("[ControlArm] Action server started")
        self.encoder_val = 0

    def encoder_callback(self, value):
        """
        Update latest encoder reading.

        Parameters
        ----------
        value : std_msgs.msg.Int32
            Current encoder position message.
        """
        self.encoder_val = value.data

    def execute_cb(self, goal):
        """
        Handle incoming ArmControlAction goals.

        Parameters
        ----------
        goal : tiago1.msg.ArmControlGoal
            Contains the 'degree' field specifying the target angle.

        Process
        -------
        1. Read target angle and set tolerance and timeout.
        2. Loop at 10 Hz until |encoder_val - target| <= tolerance:
           a. Break if ROS is shutting down.
           b. Preempt if requested.
           c. Abort on timeout.
           d. Compute angular velocity ∝ error and publish to cmd_pub.
           e. Publish feedback.status with current progress.
        3. On success, set result.success = True and call set_succeeded.
        4. On failure or preemption, set result.success = False and abort/preempt.
        """
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

            feedback.positions = [0.1]*7
            feedback.velocities = [0.1]*7
            feedback.efforts = [0.1]*7
            self.server.publish_feedback(feedback)

            rate.sleep()

        rospy.loginfo("[ControlArm] Target reached")
        result.success = True
        self.server.set_succeeded(result)

if __name__ == '__main__':
    """
    Main entrypoint: initialize ROS node and run ControlArmServer.
    """
    try:
        ControlArmServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
