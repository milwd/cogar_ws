#!/usr/bin/env python
"""
reasoning_action.py
===================

Overview
--------
`reasoning_action.py` is a **high-level task orchestrator** that converts
symbolic “table commands” (*PLACE*, *CLEAR*, …) and a navigation *path* into
three concrete ROS Action goals:

* **movement_control** – drives the mobile base along the path  
* **arm_control**      – moves the wrist / forearm to the working pose  
* **gripper_control**  – opens or closes the gripper

When the whole macro succeeds—or when **any** step fails—the node emits one
human-readable feedback message for GUIs, loggers, or supervisor FSMs.

Interfaces (strongly-typed, partially stateful)
-----------------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 26 27 55

   * - Direction
     - Name
     - Type
     - Notes
   * - **Required**
     - ``/{robot}/planned_path``
     - ``nav_msgs/Path``
     - Global or local path from the planner
   * - **Required**
     - ``/{robot}/table_reasoning_commands``
     - ``std_msgs/String``
     - Symbolic keyword (*PLACE*, *CLEAR*, …)
   * - **Provided**
     - ``/{robot}/feedback_acion``
     - ``std_msgs/String``
     - “Busy” → doing a task, “Free” → finished/failed
   * - **Action client**
     - ``/{robot}/movement_control``
     - ``tiago1/MovementControlAction``
     - Executes the navigation path
   * - **Action client**
     - ``/{robot}/arm_control``
     - ``tiago1/ArmControlAction``
     - Positions forearm / wrist
   * - **Action client**
     - ``/{robot}/gripper_control``
     - ``tiago1/GripperControlAction``
     - Grips or releases the dish

Contract
--------
*Pre-conditions*

• All three Action servers are running (checked at startup).  
• Path and command topics share the same robot namespace.

*Post-conditions*

• Exactly **one** feedback line per task: “Busy” at start, “Free” when done.  
• Arm & gripper goals are sent *only if* base motion succeeds.

*Statefulness*

The node keeps an internal flag ``path_received`` so each incoming path is
executed **once**; after completion the flag resets.

Customisation hooks
-------------------
Override :py:meth:`perform_table_command` (or the private
:meth:`_run_manipulation_sequence`) to map each symbolic keyword onto a custom
sequence of arm/gripper goals.

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
    Coordinates navigation + manipulation through three ActionLib clients.

    Variables
    ----------
    path_received : bool
        Becomes ``True`` when a new path topic arrives; cleared after use.
    path_msg : nav_msgs.msg.Path | None
        The most recent path waiting to be executed.
    task_feedback_pub : rospy.Publisher
        Latched channel for GUIs / loggers → “Busy” / “Free”.
    movement_client / arm_client / gripper_client : actionlib.SimpleActionClient
        Pre-connected clients; :py:meth:`wait_for_servers` blocks until ready.
    """

    # ------------------------------------------------------------------ #
    # initialise                                                         #
    # ------------------------------------------------------------------ #
    def __init__(self):
        self.robot_number = sys.argv[1]  # rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_reasoning_action_node')

        # ---------- runtime state ------------------------------------- #
        self.path_received: bool = False
        self.path_msg: Path | None = None

        # ---------- topic wiring -------------------------------------- #
        rospy.Subscriber(
            f'/{self.robot_number}/planned_path',
            Path,
            self.path_callback,
        )
        rospy.Subscriber(
            f'/{self.robot_number}/table_reasoning_commands',
            String,
            self.table_command_callback,
        )
        self.task_feedback_pub = rospy.Publisher(
            f'/{self.robot_number}/feedback_acion', String, queue_size=1, latch=True
        )

        # ---------- action clients ------------------------------------ #
        self.movement_client = actionlib.SimpleActionClient(
            f'/{self.robot_number}/movement_control',
            MovementControlAction,
        )
        self.arm_client = actionlib.SimpleActionClient(
            f'/{self.robot_number}/arm_control',
            ArmControlAction,
        )
        self.gripper_client = actionlib.SimpleActionClient(
            f'/{self.robot_number}/gripper_control',
            GripperControlAction,
        )
        self.wait_for_servers()

    # ------------------------------------------------------------------ #
    # helper: wait for all servers                                       #
    # ------------------------------------------------------------------ #
    def wait_for_servers(self) -> None:
        """
        Poll each ActionLib server at 1 Hz until base, arm **and** gripper
        controllers are ready.  Prevents goal loss at boot.
        """
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.movement_client.wait_for_server(rospy.Duration(1)):
                rospy.logwarn("Waiting for *movement_control* server…")
                continue
            if not self.arm_client.wait_for_server(rospy.Duration(1)):
                rospy.logwarn("Waiting for *arm_control* server…")
                continue
            if not self.gripper_client.wait_for_server(rospy.Duration(1)):
                rospy.logwarn("Waiting for *gripper_control* server…")
                continue
            rospy.loginfo("[ReasoningAction] All action servers connected.")
            break
        rate.sleep()

    # ------------------------------------------------------------------ #
    # subscriber callbacks                                               #
    # ------------------------------------------------------------------ #
    def path_callback(self, msg: Path) -> None:
        """Store the planner path for use in the main loop."""
        self.path_received = True
        self.path_msg = msg

    def table_command_callback(self, msg: String) -> None:
        """
        Forward symbolic keyword (*PLACE*, *CLEAR*, …) to
        :py:meth:`perform_table_command`.
        """
        self.perform_table_command(msg.data)

    # ------------------------------------------------------------------ #
    # override point                                                     #
    # ------------------------------------------------------------------ #
    def perform_table_command(self, decision: str) -> None:
        """
        Map symbolic command to arm/gripper macros.

        Stub does nothing so the example compiles; override in subclasses:

        * **PLACE** → lower arm, open gripper  
        * **CLEAR** → close gripper, raise arm
        """
        rospy.loginfo(f"[ReasoningAction] Table command received → {decision}")

    # ------------------------------------------------------------------ #
    # main loop                                                          #
    # ------------------------------------------------------------------ #
    def loop(self) -> None:
        """
        0.3 Hz cycle (≈ every 3 s):
            • if a new path is queued → send to base, wait blocking;  
            • on success → run arm & gripper macros;  
            • publish “Busy” / “Free” to feedback topic.
        """
        rate = rospy.Rate(0.3)  # ≈ 3-second iteration
        while not rospy.is_shutdown():
            if self.path_received:
                self.task_feedback_pub.publish("Busy")

                # ---------- 1. Move base -------------------------------- #
                move_goal = MovementControlGoal(path=self.path_msg)
                self.movement_client.send_goal(move_goal)
                self.movement_client.wait_for_result()
                move_ok = self.movement_client.get_result().success

                if move_ok:
                    rospy.loginfo("[ReasoningAction] Base reached goal.")
                    self._run_manipulation_sequence()
                else:
                    rospy.logwarn("[ReasoningAction] Base motion failed; skipping manipulation.")

                self.task_feedback_pub.publish("Free")
                self.path_received = False

            rate.sleep()

    # ------------------------------------------------------------------ #
    # default manipulation macro                                         #
    # ------------------------------------------------------------------ #
    def _run_manipulation_sequence(self) -> None:
        """
        Demo macro: **lower arm 3 deg** + **open gripper**.

        Adjust goal fields to match your real controllers.
        """
        arm_goal = ArmControlGoal(degree=3)
        self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result()

        gripper_goal = GripperControlGoal(gripnogrip=True)
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result()


# ---------------------------------------------------------------------- #
# entry                                                                  #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    try:
        node = ReasoningAction()
        node.loop()
    except rospy.ROSInterruptException:
        pass
