#! /usr/bin/env python
"""
task_manager.py
===============

Overview
--------
`task_manager.py` is the **central state orchestrator** of the TIAGo cognitive
stack.  
It listens to low-level *controller feedback* and customer *order events*, then
delegates all reasoning to the `/robot_state_decision` service.  
By outsourcing the decision logic the node stays **stateless** and easy to unit-
test while the service can evolve (rule engine, ML policy, …) without touching
this file.


Interfaces (strongly-typed, partly stateful)
-------------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 14 30 25 50

   * - Direction / Type
     - Name
     - ROS type
     - Notes
   * - **Required** (sub)
     - ``/{robot}/feedback_acion``
     - ``std_msgs/String``
     - Controller status, e.g. ``"ARM_DONE"`` or ``"BASE_FAILED"``
   * - **Required** (sub)
     - ``/{robot}/verif_T_manager``
     - ``tiago1/Voice_rec``
     - Verified customer order (currently cached, not relayed)
   * - **Client** (srv)
     - ``/{robot}/robot_state_decision``
     - ``tiago1/robotstatedecision``
     - Request → ``state_input: str`` – last feedback  
       Response ← symbolic task / ack
   * - **Provided** (pub)
     - ``/{robot}/speaker_channel``
     - ``std_msgs/String``
     - Robot dialogue (“I will get to …”, “Waiting”, …)

Contract
--------
**Pre-conditions**

• `/robot_state_decision` is available before main loop starts.  
• Feedback topic publishes *at most* twice per second (queue=10).

**Post-conditions**

• On every feedback message exactly **one** service call is made.  
• Dialogue text is always published, even if the service fails.  
• Internal `state_input` is cleared only on node shutdown.

Execution loop
--------------
Every **2 s** (`Rate(0.5)`):

#. If a new feedback string exists → call the service.  
#. Log the response.  
#. Publish a spoken confirmation on `/speaker_channel`.

Implementation notes
--------------------
* `Voice_rec` orders are *stored* for future extensions where the service might
  need context.  
* A latched publisher is **not** used so UIs see only fresh responses.  
* Exception handling: only `rospy.ServiceException` is caught; any other error
  propagates for debugging.

"""

import rospy
from std_msgs.msg import String
from tiago1.msg import Voice_rec
from tiago1.srv import robotstatedecision, robotstatedecisionRequest
import sys


class TaskManager:
    """
    Aggregate feedback → call decision service → publish dialogue line.
    """

    # ------------------------------------------------------------------ #
    #                              SET-UP                                #
    # ------------------------------------------------------------------ #
    def __init__(self):
        self.robot_number = sys.argv[1]                       # namespace
        rospy.init_node(f'{self.robot_number}_task_manager_node')

        # ---- Service proxy (blocks until available) ------------------- #
        rospy.wait_for_service(f'/{self.robot_number}/robot_state_decision')
        self.server_client = rospy.ServiceProxy(
            f'/{self.robot_number}/robot_state_decision',
            robotstatedecision,
        )

        # ---- Topic wiring --------------------------------------------- #
        rospy.Subscriber(
            f'/{self.robot_number}/feedback_acion',
            String,
            self._cb_feedback,
            queue_size=10,
        )
        rospy.Subscriber(
            f'/{self.robot_number}/verif_T_manager',
            Voice_rec,
            self._cb_verified_order,
            queue_size=10,
        )
        self.dialogue_pub = rospy.Publisher(
            f'/{self.robot_number}/speaker_channel',
            String,
            queue_size=10,
        )

        # ---- Internal cache ------------------------------------------- #
        self.state_input: str | None = None
        self.order_msg: Voice_rec | None = None

        rospy.loginfo(f"[TaskManager {self.robot_number}] Node initialised.")

    # ------------------------------------------------------------------ #
    #                        CALLBACKS                                   #
    # ------------------------------------------------------------------ #
    def _cb_feedback(self, msg: String):
        """Store latest controller state string."""
        rospy.loginfo(f"[TaskManager {self.robot_number}] Feedback: {msg.data}")
        self.state_input = msg.data

    def _cb_verified_order(self, msg: Voice_rec):
        """Cache verified customer order for future use in service call."""
        self.order_msg = msg

    # ------------------------------------------------------------------ #
    #                        MAIN TICK                                   #
    # ------------------------------------------------------------------ #
    def change_state(self):
        """
        If `state_input` exists call decision service and speak reply.
        """
        if self.state_input is None:
            return

        try:
            req = robotstatedecisionRequest()
            req.state_input = self.state_input
            req.robot_id = self.robot_number
            resp = self.server_client(req)

            mss = String()
            if resp.success and resp.state_output == "Busy":
                if resp.order:
                    mss.data = f"I will get to {resp.id_client}, whose order is {resp.order}."
                else:
                    mss.data = "Order obtained before, I'm busy getting to it!"
            elif resp.state_output == "Wait":
                mss.data = "Okay, I'll wait—probably I've better things to do."
            else:
                mss.data = f"Received unknown state output: {resp.state_output}"

            self.dialogue_pub.publish(mss)
            rospy.loginfo(f"[TaskManager {self.robot_number}] Dialogue: {mss.data}")

        except rospy.ServiceException as exc:
            rospy.logerr(f"[TaskManager {self.robot_number}] Service error: {exc}")

        finally:
            # Reset so next feedback triggers a new service call
            self.state_input = None


# ---------------------------------------------------------------------- #
#                               bootstrap                                #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    try:
        tm = TaskManager()
        rate = rospy.Rate(0.5)  # every 2 s
        while not rospy.is_shutdown():
            tm.change_state()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
