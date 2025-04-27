#!/usr/bin/env python
"""
control_wheel.py
================

Action-level **base controller** for following a coarse navigation path
-----------------------------------------------------------------------

`ControlMovementServer` exposes the :ros:`tiago1/MovementControlAction`
namespace.  Each goal carries a complete :rosmsg:`nav_msgs/Path`; the server
steers the robot towards the **first waypoint only** with a proportional
controller and then reports *succeeded*.  Replace the placeholder loop with a
full trajectory tracker when integrating real localisation / odometry.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Direction / type
     - Name
     - Semantics
   * - **publish** ``geometry_msgs/Twist``
     - ``/cmd_vel/wheel``
     - Linear velocity in the map frame – only ``x``/``y`` are used
   * - **action server** ``tiago1/MovementControlAction``
     - ``movement_control``
     - Goal → ``path`` (``nav_msgs/Path``)  
       Feedback → **status** (string)  
       Result → **success** (bool)

Control logic
-------------
*Read first waypoint → compute planar error → send velocity*  

:math:`v_x = k_p \cdot \Delta x,\; v_y = k_p \cdot \Delta y` with
*kₚ = 0.01*.  
The controller runs **once** per goal (fire-and-forget stub).  Real deployment
should iterate over every pose, add angular control and stop conditions.

"""

import rospy
import actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tiago1.msg import MovementControlAction, MovementControlFeedback, MovementControlResult
import time
import sys

class ControlMovementServer:
    """
    Minimal ActionLib wrapper driving the robot base towards the first waypoint.

    Variables
    ------------------
    cmd_pub
        Publishes :rosmsg:`geometry_msgs/Twist` on ``/cmd_vel/wheel``.
    server
        Action server handling **movement_control** goals.
    """

    def __init__(self) -> None:
        """Advertise publisher and bring up the ActionLib server."""
        self.cmd_pub = rospy.Publisher('/cmd_vel/wheel',
                                       Twist, queue_size=10)

        """
        - Advertise '/cmd_vel/wheel' for velocity commands.
        - Set up and start the SimpleActionServer for MovementControlAction.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_control_movement_node')
        self.cmd_pub = rospy.Publisher(f'/{self.robot_number}/cmd_vel/wheel', Twist, queue_size=10)
        self.server = actionlib.SimpleActionServer(
            f'/{self.robot_number}/movement_control',
            MovementControlAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self.server.start()
        rospy.loginfo("[ControlMovement] Action server ready.")

    # ------------------------------------------------------------------ #
    #                          action logic                               #
    # ------------------------------------------------------------------ #
    def execute_cb(self, goal: MovementControlAction.Goal) -> None:
        """
        Drive once towards the first pose in :pyattr:`goal.path`.

        Succeeds after a fixed 2-second actuation delay.
        """
        feedback = MovementControlFeedback()
        result = MovementControlResult()

        # Immediate pre-emption check
        if self.server.is_preempt_requested() or rospy.is_shutdown():
            rospy.logwarn("[ControlMovement] Pre-empted before start.")
            self.server.set_preempted()
            return

        if not goal.path.poses:
            rospy.logerr("[ControlMovement] Empty path – aborting.")
            result.success = False
            self.server.set_aborted(result)
            return

        target_pose = goal.path.poses[0].pose.position
        rospy.loginfo(f"[ControlMovement] Target waypoint → "
                      f"({target_pose.x:.2f}, {target_pose.y:.2f})")

        # Proportional velocities
        k_p = 0.01
        cmd = Twist()
        cmd.linear.x = k_p * target_pose.x
        cmd.linear.y = k_p * target_pose.y
        self.cmd_pub.publish(cmd)

        feedback.status = (f"Driving to first waypoint "
                           f"of {len(goal.path.poses)}")
        self.server.publish_feedback(feedback)
        rospy.sleep(2)  # time to reach each point
        time.sleep(3)

        result.success = True
        self.server.set_succeeded(result)
        rospy.loginfo("[ControlMovement] Goal completed.")


# ---------------------------------------------------------------------- #
#                                bootstrap                               #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    """
    Main entrypoint: initialize ROS node and start ControlMovementServer.
    """
    ControlMovementServer()
    rospy.spin()
