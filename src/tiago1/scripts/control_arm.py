#!/usr/bin/env python
"""
control_arm.py
==============

Closed-loop **ActionLib server** for positioning the TIAGo arm
--------------------------------------------------------------

This node listens for :rosmsg:`tiago1/ArmControlAction` goals that specify a
single **target joint angle** (in degrees).  It applies a proportional control
law on the arm’s **angular velocity** until the on-board encoder reports that
the target, within a configurable tolerance, has been reached or a timeout /  
pre-emption occurs.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Direction / type
     - Name
     - Purpose
   * - **publish** ``geometry_msgs/Twist``
     - ``/cmd_vel/arm``
     - Desired angular velocity about the arm’s primary axis  
       (only :code:`angular.z` is used).
   * - **subscribe** ``std_msgs/Int32``
     - ``/encoder_arm``
     - Current encoder position expressed in **degrees**.
   * - **action server** ``tiago1/ArmControlAction``
     - ``arm_control``
     - Goal: ``degree`` (int), Feedback: **status** (string),  
       Result: **success** (bool)

Control algorithm
-----------------
#. **Goal reception** – store target angle, define *tolerance* (°) and
   *timeout* (s).  
#. **10 Hz loop**  
   • compute error = *target – encoder*  
   • :math:`\omega_z = k_p \cdot error` with *k\_p = 0.01*  
   • publish velocity; send feedback string  
   • exit early if *|error| ≤ tolerance*, timeout expires, node is
   pre-empted, or ROS shuts down.  
#. **Outcome** – call :py:meth:`SimpleActionServer.set_succeeded`,
   :py:meth:`set_aborted` or :py:meth:`set_preempted` with
   :rosmsg:`tiago1/ArmControlResult`.

Tuning knobs
------------
*TOLERANCE* = 5 °  *TIMEOUT* = 10 s  *Kₚ* = 0.01 – tweak in
:py:meth:`execute_cb` to match your hardware.

Safety
------
The node publishes zero velocity immediately after success / abort to stop the
arm, and it honours standard ActionLib pre-emption so higher layers can
interrupt motion at any time.
"""

import time
import rospy
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time
import sys

class ControlArmServer:
    """
    Minimal proportional controller wrapped in an ActionLib server.

    Variables
    ------------------
    cmd_pub
        :pyclass:`rospy.Publisher` that streams velocity commands to
        ``/cmd_vel/arm``.
    encoder_sub
        :pyclass:`rospy.Subscriber` listening to ``/encoder_arm`` for position
        feedback.
    server
        :pyclass:`actionlib.SimpleActionServer` that exposes the
        **arm_control** action namespace.
    encoder_val
        Latest encoder reading in **degrees**; updated continuously by
        :py:meth:`encoder_callback`.
    """

    def __init__(self) -> None:
        """Wire publishers/subscribers, start the ActionLib server."""
        self.cmd_pub = rospy.Publisher('/cmd_vel/arm',
                                       Twist, queue_size=10)
        self.encoder_sub = rospy.Subscriber('/encoder_arm',
                                            Int32, self.encoder_callback)
        """
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
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[ControlArm] Action server ready.")

        self.encoder_val: int = 0

    # ------------------------------------------------------------------ #
    #                         encoder feedback                            #
    # ------------------------------------------------------------------ #
    def encoder_callback(self, value: Int32) -> None:
        """Cache the most recent encoder position."""
        self.encoder_val = value.data

    # ------------------------------------------------------------------ #
    #                           action logic                              #
    # ------------------------------------------------------------------ #
    def execute_cb(self, goal: ArmControlAction.Goal) -> None:
        """
        Main loop executed per goal.

        Publishes velocities at 10 Hz until the arm reaches *goal.degree*,
        is pre-empted, or times out.
        """
        feedback = ArmControlFeedback()
        result = ArmControlResult()

        target = goal.degree
        tolerance = 5          # degrees
        timeout = 10.0         # seconds
        k_p = 0.01             # proportional gain

        start_time = time.time()
        rospy.loginfo(f"[ControlArm] New goal → {target}°")

        rate = rospy.Rate(10)  # Hz
        while abs(self.encoder_val - target) > tolerance:
            # ---------- house-keeping checks --------------------------- #
            if rospy.is_shutdown():
                return
            if self.server.is_preempt_requested():
                rospy.logwarn("[ControlArm] Pre-empted.")
                self.server.set_preempted()
                return
            if time.time() - start_time > timeout:
                rospy.logwarn("[ControlArm] Timeout.")
                result.success = False
                self.cmd_pub.publish(Twist())      # stop arm
                self.server.set_aborted(result)
                return

            # ---------- control law ------------------------------------ #
            error = target - self.encoder_val
            cmd = Twist()
            cmd.angular.z = k_p * error
            self.cmd_pub.publish(cmd)

            feedback.positions = [0.1]*7
            feedback.velocities = [0.1]*7
            feedback.efforts = [0.1]*7
            self.server.publish_feedback(feedback)
            rate.sleep()

        # ---------- success path --------------------------------------- #
        self.cmd_pub.publish(Twist())  # zero velocity
        result.success = True
        self.server.set_succeeded(result)
        rospy.loginfo("[ControlArm] Target reached.")

# ---------------------------------------------------------------------- #
#                               bootstrap                                #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    """
    Main entrypoint: initialize ROS node and run ControlArmServer.
    """
    try:
        ControlArmServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass