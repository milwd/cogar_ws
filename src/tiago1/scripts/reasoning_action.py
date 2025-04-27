#!/usr/bin/env python
"""
reasoning_action.py
===================

High-level **task orchestrator** – turns symbolic plans into concrete ActionLib goals
------------------------------------------------------------------------------------

`ReasoningAction` sits at the interface between *planning* (which decides *what*
the robot should do) and *execution* (which knows *how* to move).  
It ingests a *navigation path* plus a *table-level command* and then drives the
three low-level ActionLib servers that move the base, position the arm and open
/ close the gripper.  When the whole sequence finishes – or when any part fails
– the node emits a single feedback string for downstream logging or GUI use.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Direction
     - Name
     - Type / semantics
   * - **subscribe**
     - ``/planned_path``
     - ``nav_msgs/Path`` – global or local path from the planner
   * - **subscribe**
     - ``/table_reasoning_commands``
     - ``std_msgs/String`` – symbolic action at table (*PLACE*, *CLEAR*, …)
   * - **publish**
     - ``task_feedback``
     - ``std_msgs/String`` – human-readable success / failure status
   * - **action client**
     - ``movement_control``
     - ``tiago1/MovementControlAction`` – follow the path
   * - **action client**
     - ``arm_control``
     - ``tiago1/ArmControlAction`` – position forearm / wrist
   * - **action client**
     - ``gripper_control``
     - ``tiago1/GripperControlAction`` – grasp or release the dish

Execution pipeline
------------------
#. **Path reception** – as soon as a new ``nav_msgs/Path`` arrives, flag
   :pyattr:`path_received` becomes *True*.  
#. **Movement phase** – a `MovementControlGoal` with the full path is sent to
   the base controller.  The node blocks until a result appears.  
#. **Conditional manipulation** – only if the base reports *success* does the
   node dispatch the arm and gripper goals, ensuring the robot is in position
   before manipulating objects.  
#. **Feedback** – a single sentence is published to ``task_feedback`` summarising
   the outcome.  UI widgets, loggers or higher-level reasoners can subscribe to
   that topic to update state machines or dashboards.

Customisation hooks
-------------------
*Override* :py:meth:`perform_table_command` to translate each symbolic table
command (PLACE, CLEAR, INSPECT…) into a concrete sequence of arm / gripper
actions.  The default implementation is a stub.
"""

import rospy
import actionlib
from nav_msgs.msg import Path
from std_msgs.msg import String
from tiago1.msg import MovementControlAction, MovementControlGoal
from tiago1.msg import ArmControlAction, ArmControlGoal
from tiago1.msg import GripperControlAction, GripperControlGoal
import sys

class ReasoningAction:
    """
    Coordinates navigation and manipulation through three ActionLib clients.

    Variables
    ----------
    path_received
        *True* as soon as a new path arrives; the flag is cleared after the
        path has been processed.
    path_msg
        Latest ``nav_msgs/Path`` received from the planner.
    task_feedback_pub
        Latched publisher used by GUIs or loggers to track high-level task
        completion.
    movement_client / arm_client / gripper_client
        Pre-connected ActionLib clients; connection is verified in
        :py:meth:`wait_for_servers`.
    """

    def __init__(self):
        """
        Bring up the node, connect to topics and block until all three action
        servers are online.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_reasoning_action_node')

        # ------------------- runtime state --------------------------------- #
        self.path_received: bool = False
        self.path_msg: Path | None = None

        rospy.Subscriber(f'/{self.robot_number}/planned_path', Path, self.path_callback)
        rospy.Subscriber(f'/{self.robot_number}/table_reasoning_commands', String, self.table_command_callback)
        self.task_feedback_pub = rospy.Publisher(f'/{self.robot_number}/feedback_acion', String, queue_size=1)

        self.movement_client = actionlib.SimpleActionClient(f'/{self.robot_number}/movement_control', MovementControlAction)
        self.arm_client = actionlib.SimpleActionClient(f'/{self.robot_number}/arm_control', ArmControlAction)
        self.gripper_client = actionlib.SimpleActionClient(f'/{self.robot_number}/gripper_control', GripperControlAction)

        self.wait_for_servers()

    # --------------------------------------------------------------------- #
    #                       connection helper                               #
    # --------------------------------------------------------------------- #
    def wait_for_servers(self) -> None:
        """
        Poll each ActionLib server at 1 Hz until the trio is ready.  Startup
        proceeds only when base, arm **and** gripper controllers are up; this
        prevents goal loss at boot.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.movement_client.wait_for_server(rospy.Duration(1.0)):
                rospy.logwarn("Waiting for movement_control action server…")
                continue
            if not self.arm_client.wait_for_server(rospy.Duration(1.0)):
                rospy.logwarn("Waiting for arm_control action server…")
                continue
            if not self.gripper_client.wait_for_server(rospy.Duration(1.0)):
                rospy.logwarn("Waiting for gripper_control action server…")
                continue
            rospy.loginfo("[ReasoningAction] All action servers connected.")
            break
        rate.sleep()

    # --------------------------------------------------------------------- #
    #                       subscriber callbacks                            #
    # --------------------------------------------------------------------- #
    def path_callback(self, msg: Path) -> None:
        """Store the incoming path for deferred execution in the main loop."""
        self.path_received = True
        self.path_msg = msg

    def table_command_callback(self, msg: String) -> None:
        """Forward the symbolic command to :py:meth:`perform_table_command`."""
        decision = msg.data
        rospy.loginfo(f"[ReasoningAction] Table command → {decision}")
        self.perform_table_command(decision)

    # --------------------------------------------------------------------- #
    #                      command execution hooks                          #
    # --------------------------------------------------------------------- #
    def perform_table_command(self, decision: str) -> None:
        """
        Translate *decision* into arm / gripper goals.

        Override this stub in subclasses to implement behaviour such as:

        * **PLACE** – lower arm to 3 cm above table, open gripper.
        * **CLEAR** – close gripper, raise arm to carry height.

        The default implementation does nothing so the example still compiles.
        """
        pass

    # --------------------------------------------------------------------- #
    #                               main loop                               #
    # --------------------------------------------------------------------- #
    def loop(self) -> None:
        """
        Run a 1 Hz cycle:

        * if a fresh path is queued, send it to the base controller and wait  
          synchronously for completion;  
        * if movement succeeds, execute arm and gripper macros;  
        * publish a one-line summary on ``task_feedback``.

        The method blocks until ROS shutdown.
        """
        rate = rospy.Rate(0.3)
        while not rospy.is_shutdown():
            if self.path_received:
                self.task_feedback_pub.publish("Busy")
                rospy.loginfo("[ReasoningAction] Received planned path, sending to ControlMovement.")
                move_goal = MovementControlGoal(path=self.path_msg)
                self.movement_client.send_goal(move_goal)
                self.movement_client.wait_for_result()
                move_result = self.movement_client.get_result()
                if move_result.success:
                    rospy.loginfo("[ReasoningAction] Movement succeeded. Sending commands to arm and gripper.")
                    arm_goal = ArmControlGoal(degree=3)
                    self.arm_client.send_goal(arm_goal)
                    self.arm_client.wait_for_result()

                    gripper_goal = GripperControlGoal(gripnogrip=True)
                    self.gripper_client.send_goal(gripper_goal)
                    self.gripper_client.wait_for_result()

                    self.task_feedback_pub.publish("Free")
                else:
                    rospy.logwarn("[ReasoningAction] Movement failed, not sending arm/gripper commands.")
                    self.task_feedback_pub.publish("Free")

                self.path_received = False
            rate.sleep()

    # --------------------------------------------------------------------- #
    #                       internal helper                                 #
    # --------------------------------------------------------------------- #
    def _run_manipulation_sequence(self) -> None:
        """
        Default manipulation macro: simple arm lower + gripper toggle.

        Adapt the goal parameters to match your real controller interface.
        """
        arm_goal = ArmControlGoal(degree=3)
        self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result()

        gripper_goal = GripperControlGoal(gripnogrip=True)
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result()


# ------------------------------------------------------------------------- #
#                                   entry                                   #
# ------------------------------------------------------------------------- #
if __name__ == '__main__':
    try:
        node = ReasoningAction()
        node.loop()
    except rospy.ROSInterruptException:
        pass
