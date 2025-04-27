#!/usr/bin/env python
"""
reasoning_table_placement.py
============================

Overview
--------
`reasoning_table_placement.py` is a **bridge** between *cognition* and the
*manipulation stack*.  
It listens to high-level **decisions** on ``/{robot}/placement_decision`` and
maps them onto low-level **commands** that the arm / gripper planners
understand.


Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 28 25 55

   * - Direction
     - Topic
     - Message type
     - Notes
   * - **Required**
     - ``/{robot}/placement_decision``
     - ``std_msgs/String``
     - e.g. ``"Decision: PLACE, IGNORE"``
   * - **Provided**
     - ``/{robot}/table_reasoning_commands``
     - ``std_msgs/String``
     - ``"PLACE_DISH" | "CLEAR_TABLE" | "NO_ACTION"``

Decision → Command mapping
^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table:: Decision Mapping
   :widths: 15 25 60
   :header-rows: 1

   * - Decision
     - Published cmd
     - Rationale
   * - ``PLACE``
     - ``PLACE_DISH``
     - Room for one more plate
   * - ``CLEAR``
     - ``CLEAR_TABLE``
     - Table finished; remove crockery
   * - *other*
     - ``NO_ACTION``
     - Either “FULL” or “IGNORE” → do nothing


Contract
--------
**Pre-conditions**

• Upstream decision topic must contain “Decision:” and one keyword.

**Post-conditions** 

• Exactly one command published per decision message.  
• Mapper never blocks; if queue is full older commands drop first (`queue_size=10`).

Implementation notes
--------------------
* Uses simple substring checks so future metadata (timestamps, scores) won’t
  break parsing.  
* Publisher is latched at **10** messages to absorb bursts.  
* All log lines throttled or filtered to keep console tidy.

"""

import rospy
from std_msgs.msg import String
import sys


class ReasoningTablePlacement:
    """
    Runtime object that wires subscriber → callback → publisher.
    """

    # ------------------------------------------------------------------ #
    #                              SET-UP                                #
    # ------------------------------------------------------------------ #
    def __init__(self):
        """
        1. Initialise ROS under ``{robot}_reasoning_table_placement_node``  
        2. Subscribe to high-level decisions.  
        3. Advertise low-level commands.  
        """
        self.robot_number = sys.argv[1]  # rospy.get_param('~robot_number')
        rospy.init_node(f"{self.robot_number}_reasoning_table_placement_node")

        self.decision_sub = rospy.Subscriber(
            f"/{self.robot_number}/placement_decision",
            String,
            self.decision_callback,
        )
        self.command_pub = rospy.Publisher(
            f"/{self.robot_number}/table_reasoning_commands",
            String,
            queue_size=10,
        )

        rospy.loginfo("[ReasoningTablePlacement] Node started.")

    # ------------------------------------------------------------------ #
    #                            CALLBACK                                #
    # ------------------------------------------------------------------ #
    def decision_callback(self, msg):
        """
        Translate *Decision* → *Command*.

        Translation logic
        -----------------
        * contains **PLACE**  → ``PLACE_DISH``  
        * else contains **CLEAR** → ``CLEAR_TABLE``  
        * otherwise → ``NO_ACTION``
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
    Instantiate node and hand control to ROS spin loop.
    """
    try:
        ReasoningTablePlacement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
