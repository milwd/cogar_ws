#! /usr/bin/env python
"""
reasoning_order_verification.py
===============================

Overview
--------
`reasoning_order_verification.py` is the **natural-language gatekeeper** that
stands between automatic speech recognition (ASR) and the orchestration layer.

Workflow
~~~~~~~~
#. **ASR sentence** arrives on ``/{robot}/voice_recogn``.  
#. The node validates the text:

   * contains the polite fragment **“Can I have”**  
   * mentions at least one item from *food_list*

#. If valid → build a structured :pyclass:`tiago1.msg.Voice_rec` order, publish
   it to the task-manager **and** push the same order into the shared
   orchestration queue via the ``/robot_state_decision_add`` service.  
#. If invalid → emit an integer error code on the interaction error channel.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 25 55

   * - Direction
     - Name
     - ROS type
     - Notes
   * - **Required**
     - ``/{robot}/voice_recogn``
     - ``std_msgs/String``
     - Raw ASR text, e.g. ``"Can I have sushi please"``.
   * - **Provided**
     - ``/{robot}/verif_T_manager``
     - ``tiago1/Voice_rec``
     - Structured, validated order (id + list_of_orders).
   * - **Provided**
     - ``/{robot}/error_from_interaction``
     - ``std_msgs/Int32``
     - ``1`` → unknown dish  ``2`` → missing “Can I have”.
   * - **Service client**
     - ``/{robot}/robot_state_decision_add``
     - ``tiago1/send_order``
     - Appends the order to the orchestration FIFO.

Validation policy
-----------------
* **Syntax guard** – sentence *must* include “Can I have”.  
* **Menu match** – at least one substring from *food_list* must be present.  
* **Multi-dish support** – all matches packed into a single order message.

Error codes
-----------
``1`` – no recognised dish  

``2`` – polite fragment missing  
Both are recoverable; waiter simply prompts the customer to repeat.

"""

import rospy
from std_msgs.msg import String, Int32
from tiago1.msg import Voice_rec
from tiago1.srv import send_order, send_orderRequest
import sys


class ReasoningOrderVerification:
    """
    Parse spoken orders → validate → publish structured order or error code.

    Variables
    ----------
    server_client : rospy.ServiceProxy
        Link to ``/{robot}/robot_state_decision_add`` (FIFO enqueue).
    food_list : list[str]
        Canonical dish names; matching is **substring-based** (case-sensitive).
    msg : std_msgs.msg.String | None
        Latest ASR sentence waiting to be parsed.
    counter_id : int
        Auto-incremented ticket number for each accepted order.
    """

    # ------------------------------------------------------------------ #
    #                              set-up                                #
    # ------------------------------------------------------------------ #
    def __init__(self, server_robots: int, food_list: list[str]):
        self.robot_number = sys.argv[1]
        rospy.init_node(f'{self.robot_number}_reasoning_order_verification_node')

        self.server = server_robots           # placeholder – single queue now
        self.food_list = food_list
        self.msg: String | None = None
        self.counter_id = 0

        # --- ROS wiring -------------------------------------------------- #
        rospy.Subscriber(
            f'/{self.robot_number}/voice_recogn',
            String,
            self.string_callback,
            queue_size=10,
        )

        self.error_pub = rospy.Publisher(
            f'/{self.robot_number}/error_from_interaction',
            Int32,
            queue_size=10,
        )
        self.order_pub = rospy.Publisher(
            f'/{self.robot_number}/verif_T_manager',
            Voice_rec,
            queue_size=10,
        )

        rospy.wait_for_service(f'/{self.robot_number}/robot_state_decision_add')
        self.server_client = rospy.ServiceProxy(
            f'/{self.robot_number}/robot_state_decision_add',
            send_order,
        )

        rospy.loginfo("[OrderVerifier] Node initialised.")

    # ------------------------------------------------------------------ #
    #                         subscriber cb                              #
    # ------------------------------------------------------------------ #
    def string_callback(self, msg: String):
        """Cache latest ASR string for validation in main loop."""
        self.msg = msg

    # ------------------------------------------------------------------ #
    #                           main logic                               #
    # ------------------------------------------------------------------ #
    def parse_order(self):
        """
        Validate :pyattr:`self.msg`; publish order or error.

        Called once per second from the main loop.
        """
        if not self.msg:
            return                                   # nothing to do yet

        sentence = self.msg.data
        self.msg = None                              # mark consumed

        # ---------- rule 1: polite fragment -------------------------------- #
        if "Can I have" not in sentence:
            self._raise_error(2, "missing 'Can I have'")
            return

        # ---------- rule 2: at least one valid dish ------------------------ #
        dishes = [dish for dish in self.food_list if dish in sentence]
        if not dishes:
            self._raise_error(1, "unknown dish")
            return

        # ---------- build order message ----------------------------------- #
        order = Voice_rec()
        order.id_client = self.counter_id
        order.list_of_orders.extend(dishes)

        # Publish to task-manager topic
        self.order_pub.publish(order)
        rospy.loginfo(f"[OrderVerifier] Order accepted → {order}")

        # Push into orchestration FIFO
        self._push_to_service(order)

        self.counter_id += 1

    # ------------------------------------------------------------------ #
    #                           helpers                                  #
    # ------------------------------------------------------------------ #
    def _raise_error(self, code: int, note: str):
        """Publish error *code* and log a warning."""
        self.error_pub.publish(code)
        rospy.logwarn(f"[OrderVerifier] {note} (code {code})")

    def _push_to_service(self, order: Voice_rec):
        """Forward validated order to `/robot_state_decision_add` service."""
        try:
            req = send_orderRequest(order=order)
            resp = self.server_client(req)
            rospy.loginfo(f"[OrderVerifier] Service response: {resp.message}")
        except rospy.ServiceException as exc:
            rospy.logerr(f"[OrderVerifier] Service call failed: {exc}")


# ---------------------------------------------------------------------- #
#                                main                                    #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    try:
        dish_list = [
            "ragu", "sugo", "pasta", "kebab", "sushi",
            "riso", "frnach", "shit", "hamburger", "steak"
        ]
        server_robots = 1
        verifier = ReasoningOrderVerification(server_robots, dish_list)

        rate = rospy.Rate(1)          # 1 Hz
        while not rospy.is_shutdown():
            verifier.parse_order()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr("[OrderVerifier] Node interrupted – shutting down.")
