#!/usr/bin/env python
"""
reasoning_table_placement.py
============================

Node purpose
------------
*Bridge from cognition to manipulation.*  
It listens to high-level placement **decisions** coming from
``/placement_decision`` (produced by *distance_estimator*) and converts
them into low-level **commands** that the manipulation stack
understands.

Decision → Command mapping
^^^^^^^^^^^^^^^^^^^^^^^^^^

=========  ===============  ==========================================
Decision   Command sent     Rationale
=========  ===============  ==========================================
``PLACE``  ``PLACE_DISH``   There is room for one more plate on the table.
``CLEAR``  ``CLEAR_TABLE``  Table is finished; remove crockery.
*other*    ``NO_ACTION``    Either “FULL” or “IGNORE” → do nothing now.
=========  ===============  ==========================================

Why a dedicated mapper?
-----------------------
* **Decoupling** – you can swap in a smarter reasoning algorithm without
  touching the gripper driver, or vice-versa.
* **Future proof** – if the execution layer later moves from simple
  string commands to, e.g., action goals or behaviour-tree ticks, only
  *this* file needs editing.

ROS topic interface
-------------------
**Subscribes**

* ``/placement_decision``  : ``std_msgs/String``  
  Example payload → ``"Decision: PLACE, IGNORE"``

**Publishes**

* ``/table_reasoning_commands``  : ``std_msgs/String``  
  One of ``"PLACE_DISH"``, ``"CLEAR_TABLE"``, ``"NO_ACTION"``.


Implementation notes
--------------------
* A simple *substring* check keeps the logic robust against future
  additions to the decision string (timestamps, confidence scores …).
* The publisher queues **10** messages to absorb short bursts without
  back-pressure.
* All log lines are throttled or filtered to keep the ROS console tidy.
"""

import rospy
from std_msgs.msg import String
import sys



class ReasoningTablePlacement:
    """
    Runtime object that wires subscribers → callbacks → publisher.

    Public API consists solely of the constructor; the rest is handled
    internally until ROS shuts down.
    """

    # ------------------------------------------------------------------ #
    #                              SET-UP                                #
    # ------------------------------------------------------------------ #
    def __init__(self):
        """
        Launch the node and register its single pub/sub pair.

        Steps
        -----
        1. Initialise ROS with the node name
           ``reasoning_table_placement_node``.
        2. Subscribe to high-level decisions on ``/placement_decision``.
        3. Advertise low-level commands on ``/table_reasoning_commands``.
        4. Log a start-up banner so launch files can confirm health.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f"{self.robot_number}_reasoning_table_placement_node")

        self.decision_sub = rospy.Subscriber(f"/{self.robot_number}/placement_decision", String, self.decision_callback)
        self.command_pub = rospy.Publisher(f"/{self.robot_number}/table_reasoning_commands", String, queue_size=10)

        rospy.loginfo("[ReasoningTablePlacement] Node started.")

    # ------------------------------------------------------------------ #
    #                            CALLBACK                                #
    # ------------------------------------------------------------------ #
    def decision_callback(self, msg):
        """
        Translate *cognitive* decision → *motor* command.

        Parameters
        ----------
        msg : std_msgs.msg.String
            Expected payload examples::

                "Decision: PLACE, CLEAR"
                "Decision: FULL, IGNORE"


        Translation logic
        -----------------
        * If the string contains **``"PLACE"``** → emit ``"PLACE_DISH"``.  
        * Else if it contains **``"CLEAR"``** → emit ``"CLEAR_TABLE"``.  
        * Otherwise emit ``"NO_ACTION"`` so downstream nodes know the
          message was processed but no physical action is required.

        The simple *contains* check is intentionally robust: future
        reasoning nodes may prepend extra metadata (timestamps, scores,
        etc.) without breaking this mapper.
        """
        decision_raw = msg.data
        rospy.loginfo(f"[ReasoningTablePlacement] Received: {decision_raw}")

        if "PLACE" in decision_raw:
            command = "PLACE_DISH"
        elif "CLEAR" in decision_raw:
            command = "CLEAR_TABLE"
        else:
            command = "NO_ACTION"

        self.command_pub.publish(String(data=command))
        rospy.loginfo(f"[ReasoningTablePlacement] Published: {command}")


# ---------------------------------------------------------------------- #
#                               ENTRY POINT                              #
# ---------------------------------------------------------------------- #
def main():
    """
    Instantiate the node object and hand control to ROS.

    Only `rospy.ROSInterruptException` is swallowed; any other exception
    is allowed to propagate with a full traceback for debugging.
    """
    try:
        ReasoningTablePlacement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
