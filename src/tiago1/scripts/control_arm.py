#!/usr/bin/env python
"""
control_arm.py
==============

Overview
--------
`control_arm.py` is a **closed-loop ActionLib server** that positions the TIAGo
manipulator to a target joint angle (in degrees).  It applies a proportional
control law on the arm’s angular velocity until the on-board encoder reports
the target is reached within tolerance, or a timeout/pre-emption occurs.

Design goals
------------
* **Precise control** – proportional feedback on encoder readings.  
* **Safety** – zero velocity published immediately on success, abort or pre-emption.  
* **Responsiveness** – honours ActionLib pre-emption so higher layers can interrupt.

Interfaces (strongly-typed, stateful)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 25 45

   * - Direction
     - Name / Topic
     - Type
     - Purpose
   * - **Subscribe**
     - ``/{robot}/encoder_arm``
     - ``std_msgs/Int32``
     - Current encoder position in degrees
   * - **Publish**
     - ``/{robot}/cmd_vel/arm``
     - ``geometry_msgs/Twist``
     - Desired angular velocity (`angular.z` only)
   * - **Action server**
     - ``/{robot}/arm_control``
     - ``tiago1/ArmControlAction``
     - Goal: `degree` (int)  
       Feedback: joint status lists  
       Result: `success` (bool)

Contract
--------
**Pre-conditions**  

• Encoder publishes valid degree readings on `/encoder_arm`.  
• Action server is started before accepting goals.

**Post-conditions**  

• On success: publishes zero velocity, sets Action to succeeded.  
• On timeout or encoder never reaches target: publishes zero velocity, sets aborted.  
• On pre-emption: stops control, sets pre-empted.  
• Feedback published at 10 Hz during execution.

**Invariants**  

• Loop frequency = 10 Hz ± ROS scheduler jitter.  
• Velocity obeys `ω = k_p · error` with `k_p` constant.

Tuning knobs
------------
* TOLERANCE = 5 °  
* TIMEOUT   = 10 s  
* Kₚ        = 0.01  


"""

import sys
import time
import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from tiago1.msg import ArmControlAction, ArmControlFeedback, ArmControlResult


class ControlArmServer:
    """
    ActionLib server exposing the `arm_control` namespace with a proportional
    joint controller.

    Variables
    ----------
    encoder_val : int
        Latest encoder reading (degrees).
    cmd_pub : rospy.Publisher
        Publishes `Twist` on `/cmd_vel/arm` to drive angular velocity.
    server : actionlib.SimpleActionServer
        Handles `ArmControlAction` goals, feedback, and results.
    """

    def __init__(self) -> None:
        """
        1. Read `robot_number` from CLI.  
        2. Initialise ROS node `<robot>_control_arm_node`.  
        3. Advertise `/cmd_vel/arm` and subscribe to `/encoder_arm`.  
        4. Set up and start the ActionLib server on `/arm_control`.
        """
        self.robot_number = sys.argv[1]
        rospy.init_node(f'{self.robot_number}_control_arm_node')

        # Publisher for angular velocity
        self.cmd_pub = rospy.Publisher(
            f'/{self.robot_number}/cmd_vel/arm',
            Twist,
            queue_size=10,
        )
        # Subscriber for encoder readings
        self.encoder_val = 0
        rospy.Subscriber(
            f'/{self.robot_number}/encoder_arm',
            Int32,
            self.encoder_callback,
            queue_size=10,
        )

        # ActionLib server setup (auto_start=False to configure before start)
        self.server = actionlib.SimpleActionServer(
            f'/{self.robot_number}/arm_control',
            ArmControlAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo(f"[ControlArm] Action server `/arm_control` ready.")

    def encoder_callback(self, msg: Int32) -> None:
        """
        Cache the most recent encoder position.

        Parameters
        ----------
        msg : std_msgs.msg.Int32
            Encoder reading in degrees.
        """
        self.encoder_val = msg.data

    def execute_cb(self, goal: ArmControlAction.Goal) -> None:
        """
        Execute a proportional control loop to reach `goal.degree`.

        Parameters
        ----------
        goal : ArmControlAction.Goal
            Contains `degree` target angle.
        """
        feedback = ArmControlFeedback()
        result = ArmControlResult()

        target = goal.degree
        tolerance = 5       # degrees
        timeout = 10.0      # seconds
        k_p = 0.01          # proportional gain

        start_time = time.time()
        rospy.loginfo(f"[ControlArm] New goal received: {target}°")

        rate = rospy.Rate(10)  # 10 Hz control loop
        while abs(self.encoder_val - target) > tolerance:
            if rospy.is_shutdown():
                return
            if self.server.is_preempt_requested():
                rospy.logwarn("[ControlArm] Goal pre-empted.")
                self.cmd_pub.publish(Twist())  # stop arm
                self.server.set_preempted()
                return
            if time.time() - start_time > timeout:
                rospy.logwarn("[ControlArm] Timeout reached.")
                result.success = False
                self.cmd_pub.publish(Twist())  # stop arm
                self.server.set_aborted(result)
                return

            # Proportional control law
            error = target - self.encoder_val
            cmd = Twist()
            cmd.angular.z = k_p * error
            self.cmd_pub.publish(cmd)

            # Populate simplistic feedback arrays (stub values)
            feedback.positions = [0.0] * 7
            feedback.velocities = [0.0] * 7
            feedback.efforts    = [0.0] * 7
            self.server.publish_feedback(feedback)

            rate.sleep()

        # Success: stop and report
        self.cmd_pub.publish(Twist())
        result.success = True
        self.server.set_succeeded(result)
        rospy.loginfo("[ControlArm] Target angle reached within tolerance.")

if __name__ == '__main__':
    try:
        ControlArmServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
