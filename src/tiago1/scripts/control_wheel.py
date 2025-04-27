#!/usr/bin/env python
"""
control_wheel.py
================

Overview
--------
`control_wheel.py` is a **base controller** implemented as an ActionLib server.
It steers the TIAGo platform toward the **first waypoint** of a received
`nav_msgs/Path` using a simple proportional law, then reports success.  Replace
this stub’s single-shot behavior with a full trajectory tracker for production.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 45

   * - Direction
     - Topic / Name
     - Message type / Semantics
   * - **Publish**
     - ``/{robot}/cmd_vel/wheel``
     - ``geometry_msgs/Twist`` – linear x/y velocity commands
   * - **Action server**
     - ``/{robot}/movement_control``
     - ``tiago1/MovementControlAction`` –  
       **Goal**: `path` (`nav_msgs/Path`)  
       **Feedback**: `status` (string)  
       **Result**: `success` (bool)

Contract
--------
**Pre-conditions**  

• Path message contains at least one waypoint.  
• Action server is ready before accepting goals.

**Post-conditions**  

• Publishes exactly one `Twist` based on the first waypoint.  
• On exit, publishes a zero `Twist` to stop the robot.  
• Sets the Action state to succeeded, aborted (empty path), or preempted.

**Invariants**  

• Velocity = `k_p · error` with fixed `k_p`.  
• Execution is fire-and-forget: no looping over all poses.

Tuning knobs
------------
* `k_p = 0.01` – proportional gain  
* `travel_time = 2 s` – simulated actuation delay

"""

import sys
import time
import rospy
import actionlib
from geometry_msgs.msg import Twist
from tiago1.msg import (
    MovementControlAction,
    MovementControlFeedback,
    MovementControlResult,
)


class ControlMovementServer:
    """
    ActionLib server that drives the robot toward the first waypoint.

    Variables
    ----------
    cmd_pub : rospy.Publisher
        Publishes `Twist` on `/cmd_vel/wheel`.
    server : actionlib.SimpleActionServer
        Serves `movement_control` goals.
    """

    def __init__(self) -> None:
        """
        Workflow
        --------
        1. Read `robot_number` from CLI.
        2. Initialise `<robot>_control_movement_node`.
        3. Advertise `/cmd_vel/wheel`.
        4. Start SimpleActionServer on `/movement_control`.
        """
        self.robot_number = sys.argv[1]
        rospy.init_node(f'{self.robot_number}_control_movement_node')

        self.cmd_pub = rospy.Publisher(
            f'/{self.robot_number}/cmd_vel/wheel',
            Twist,
            queue_size=10,
        )

        self.server = actionlib.SimpleActionServer(
            f'/{self.robot_number}/movement_control',
            MovementControlAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[ControlMovement] Action server `/movement_control` ready.")

    def execute_cb(self, goal: MovementControlAction.Goal) -> None:
        """
        Execute one proportional command toward the first waypoint.

        Parameters
        ----------
        goal : MovementControlAction.Goal
            Contains `path` with a list of PoseStamped waypoints.
        """
        feedback = MovementControlFeedback()
        result = MovementControlResult()

        # Pre-emption check
        if rospy.is_shutdown() or self.server.is_preempt_requested():
            rospy.logwarn("[ControlMovement] Pre-empted before start.")
            self.server.set_preempted()
            return

        # Validate path
        if not goal.path.poses:
            rospy.logerr("[ControlMovement] Empty path – aborting.")
            result.success = False
            self.server.set_aborted(result)
            return

        # Extract first waypoint
        target = goal.path.poses[0].pose.position
        rospy.loginfo(f"[ControlMovement] First waypoint → ({target.x:.2f}, {target.y:.2f})")

        # Proportional command
        k_p = 0.01
        cmd = Twist()
        cmd.linear.x = k_p * target.x
        cmd.linear.y = k_p * target.y
        self.cmd_pub.publish(cmd)

        # Feedback
        feedback.status = f"Driving to first waypoint of {len(goal.path.poses)}"
        self.server.publish_feedback(feedback)

        # Simulate actuation delay
        rospy.sleep(2.0)

        # Stop the robot
        self.cmd_pub.publish(Twist())

        # Success
        result.success = True
        self.server.set_succeeded(result)
        rospy.loginfo("[ControlMovement] Goal succeeded.")


if __name__ == '__main__':
    try:
        ControlMovementServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
