#!/usr/bin/env python
"""
control_gripper.py
==================

Action-level **binary actuator** for the TIAGo gripper
-----------------------------------------------------

The node exposes a :ros:`tiago1/GripperControlAction` server that either
**closes** (grip) or **opens** (release) the end-effector.  
A single boolean goal, *True* → grip, *False* → release, is converted into a
:rosmsg:`std_msgs/Bool` command published on ``/cmd_vel/gripper``.  
After a short configurable delay the server replies with a *succeeded* or
*pre-empted / aborted* result.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Direction / type
     - Name
     - Semantics
   * - **publish** ``std_msgs/Bool``
     - ``/cmd_vel/gripper``
     - ``True`` → close, ``False`` → open
   * - **action server** ``tiago1/GripperControlAction``
     - ``gripper_control``
     - Goal → ``gripnogrip`` (bool)  
       Feedback → **status** (string)  
       Result → **success** (bool)

Execution logic
---------------
#. On goal reception the server checks for an immediate pre-emption request.  
#. The boolean command is forwarded to the *cmd* topic.  
#. A short wait (2 s by default) gives the hardware time to settle.  
#. A success result is returned; if the goal was pre-empted or a ROS shutdown
   occurred, the appropriate ActionLib state is set instead.

Timing, waiting strategy and topic names are easily changed in
:py:meth:`execute_cb`.
"""

import rospy
import actionlib
from std_msgs.msg import Bool
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tiago1.msg import GripperControlAction, GripperControlFeedback, GripperControlResult
import time
import sys


class ControlGripperServer:
    """
    Lightweight ActionLib wrapper around a boolean gripper command topic.

    Variables
    ------------------
    cmd_pub
        :pyclass:`rospy.Publisher` streaming ``std_msgs/Bool`` to
        ``/cmd_vel/gripper``.
    server
        :pyclass:`actionlib.SimpleActionServer` handling the
        **gripper_control** namespace.
    """

    def __init__(self) -> None:
        """Advertise publisher and start the ActionLib server."""
        self.cmd_pub = rospy.Publisher('/cmd_vel/gripper',
                                       Bool, queue_size=10)

        """
        - Advertise '/cmd_vel/gripper' for gripper commands.
        - Set up and start SimpleActionServer for GripperControlAction.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_control_gripper_node')
        self.cmd_pub = rospy.Publisher(f'/{self.robot_number}/cmd_vel/gripper', Bool, queue_size=10)
        self.server = actionlib.SimpleActionServer(
            f'/{self.robot_number}/gripper_control',
            GripperControlAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[ControlGripper] Action server ready.")

    # ------------------------------------------------------------------ #
    #                             callback                                #
    # ------------------------------------------------------------------ #
    def execute_cb(self,
                   goal: GripperControlAction.Goal) -> None:
        """
        Act on *goal.gripnogrip* and report the outcome to the Action client.
        """
        feedback = GripperControlFeedback()
        result = GripperControlResult()
        rospy.loginfo("[ControlGripper] Received grip command")

        # control loop
        if self.server.is_preempt_requested():
            rospy.logwarn("[ControlGripper] Preempted")
            self.server.set_preempted()
            return

        # do control
        self.cmd_pub.publish(Bool(True))
        feedback.status = "Gripping"
        self.server.publish_feedback(feedback)
        time.sleep(2)  
        # end control loop

        result.success = True
        self.server.set_succeeded(result)
        rospy.loginfo("[ControlGripper] Completed: "
                      f"{'grip' if goal.gripnogrip else 'release'}.")


# ---------------------------------------------------------------------- #
#                                bootstrap                               #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    ControlGripperServer()
    rospy.spin()
