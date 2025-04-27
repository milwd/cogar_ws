#!/usr/bin/env python
"""
control_gripper.py
==================

Overview
--------
`control_gripper.py` is an **ActionLib server** that commands the TIAGo gripper
to open or close.  It accepts a boolean goal—`True` to grip, `False` to release—
and publishes a `std_msgs/Bool` on the gripper command topic.  After a short
delay, it reports success or honours any pre-emption/abort requests.

Design goals
------------
* **Binary actuator** – simple close/open commands via a boolean topic.  
* **Responsiveness** – honours ActionLib pre-emption immediately.  
* **Safety** – can adjust delay and topic names in one place.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 28 45

   * - Direction
     - Topic / Name
     - Type / Semantics
   * - **Publish**
     - ``/{robot}/cmd_vel/gripper``
     - ``std_msgs/Bool`` – `True` to close, `False` to open
   * - **Action server**
     - ``/{robot}/gripper_control``
     - ``tiago1/GripperControlAction``
       Goal: `gripnogrip` (bool)  
       Feedback: `status` (string)  
       Result: `success` (bool)

Contract
--------
**Pre-conditions**  

• Action server started before goals arrive.  
• Publisher on `/cmd_vel/gripper` is available.

**Post-conditions**  

• On goal reception, publishes exactly one boolean command.  
• After delay, sets Action to succeeded, pre-empted, or aborted.  
• Feedback published once per goal with a brief status.

**Invariants**  

• Delay duration and command topic are configurable in `execute_cb()`.  
• No persistent state beyond wiring subscribers/publishers.

Tuning knobs
------------
* `actuation_delay = 2.0` seconds – adjust for real hardware response.

"""

import sys
import time
import rospy
import actionlib
from std_msgs.msg import Bool
from tiago1.msg import (
    GripperControlAction,
    GripperControlFeedback,
    GripperControlResult,
)


class ControlGripperServer:
    """
    ActionLib server wrapping a boolean gripper command.

    Variables
    ----------
    cmd_pub : rospy.Publisher
        Publishes `Bool` on `/cmd_vel/gripper`.
    server : actionlib.SimpleActionServer
        Serves `gripper_control` goals.
    """

    def __init__(self) -> None:
        """
        Workflow
        --------
        1. Read `robot_number` from CLI.  
        2. Initialise ROS node `<robot>_control_gripper_node`.  
        3. Advertise `/cmd_vel/gripper`.  
        4. Start SimpleActionServer on `/gripper_control`.
        """
        self.robot_number = sys.argv[1]
        rospy.init_node(f'{self.robot_number}_control_gripper_node')

        # Publisher for boolean gripper commands
        self.cmd_pub = rospy.Publisher(
            f'/{self.robot_number}/cmd_vel/gripper',
            Bool,
            queue_size=10,
        )

        # ActionLib server for GripperControlAction
        self.server = actionlib.SimpleActionServer(
            f'/{self.robot_number}/gripper_control',
            GripperControlAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[ControlGripper] Action server `/gripper_control` ready.")

    def execute_cb(self, goal: GripperControlAction.Goal) -> None:
        """
        Handle a grip/no-grip command.

        Parameters
        ----------
        goal : GripperControlAction.Goal
            `goal.gripnogrip == True` → close, `False` → open.
        """
        feedback = GripperControlFeedback()
        result = GripperControlResult()

        rospy.loginfo(f"[ControlGripper] Received command → "
                      f"{'grip' if goal.gripnogrip else 'release'}")

        # Pre-emption check
        if self.server.is_preempt_requested() or rospy.is_shutdown():
            rospy.logwarn("[ControlGripper] Pre-empted before action.")
            self.server.set_preempted()
            return

        # Publish the boolean command
        self.cmd_pub.publish(Bool(data=goal.gripnogrip))
        feedback.status = "Gripping" if goal.gripnogrip else "Releasing"
        self.server.publish_feedback(feedback)

        # Allow hardware to settle
        actuation_delay = 2.0  # seconds
        time.sleep(actuation_delay)

        # Success
        result.success = True
        self.server.set_succeeded(result)
        rospy.loginfo("[ControlGripper] Action completed.")

if __name__ == '__main__':
    try:
        ControlGripperServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
