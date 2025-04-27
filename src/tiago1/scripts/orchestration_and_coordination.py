#!/usr/bin/env python
"""
orchestration_and_coordination.py
=================================

**Order queue + state gateway** for a fleet of sushi-waiter robots
-----------------------------------------------------------------

This node owns a *persistent* FIFO of customer orders (YAML on disk) and serves
as the single gateway where **each robot** asks:  

> “Given my current controller state, what do I do next?”

It exposes two services per robot namespace:

* ``/{robot}/robot_state_decision`` – FSM step (state → action).  
* ``/{robot}/robot_state_decision_add`` – append a freshly verified order.

Interfaces (strongly-typed, stateful)
-------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 32 28 55

   * - Kind
     - Name
     - ROS type
     - Notes
   * - **Service**
     - ``/{robot}/robot_state_decision``
     - ``tiago1/robotstatedecision``
     - Request → ``state_input: str``  
       Response ← ``state_output, order, success``
   * - **Service**
     - ``/{robot}/robot_state_decision_add``
     - ``tiago1/send_order``
     - Push order (``id_client, list_of_orders``) into queue

Orchestration System Integration KPIs
-------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 50 50

   * - **Metric**
     - **Target**
   * - Task assignment latency (call to `/robot_state_decision` → response)
     - ≤ 100 ms
   * - Throughput (orders assigned per second)
     - ≥ 20 orders/s
   * - Conflict rate (duplicate table assignments)
     - < 1 %


Persistent storage
------------------
*tiago_data.yaml* (beside this script) keeps the queue::

   orders:
     - id_client: 7
       food_list: [sushi, pasta]
     - id_client: 9
       food_list: [ramen]

File is re-loaded and re-saved on **every** service call → multiple nodes or
manual edits always stay in sync.

Decision rules
^^^^^^^^^^^^^^
#. Queue **empty** → ``state_output="Wait"`` , ``success=False``  
#. Queue non-empty **and** robot in *Free* or *Wait* → pop order,  
   return it with ``state_output="Busy"`` , ``success=True``  
#. Otherwise echo incoming state with ``success=False``

"""

import os
import yaml
import rospy
from tiago1.srv import (
    robotstatedecision, robotstatedecisionRequest, robotstatedecisionResponse,
    send_order,         send_orderResponse,
)

# --------------------------------------------------------------------------- #
#                               FSM STATES                                    #
# --------------------------------------------------------------------------- #
class State:
    """Abstract FSM state; subclasses must implement :meth:`handle`."""
    def handle(self, context, req: robotstatedecisionRequest) -> robotstatedecisionResponse:
        raise NotImplementedError


class FreeState(State):
    """Idle → try to grab the next order, else go to *Wait*."""
    def handle(self, context, req):
        order = context.obtain_order()
        robot_id = req.robot_id
        if order:
            context.set_state(robot_id, BusyState())
            return robotstatedecisionResponse(
                state_output="Busy",
                id_client=order["id_client"],
                order=order["food_list"],
                success=True,
            )
        context.set_state(robot_id, WaitState())
        return robotstatedecisionResponse(
            state_output="Wait", id_client=None, order=[], success=False
        )


class WaitState(State):
    """Waiting for an order; become *Busy* if one appears."""
    def handle(self, context, req):
        order = context.obtain_order()
        robot_id = req.robot_id
        if order:
            context.set_state(robot_id, BusyState())
            return robotstatedecisionResponse(
                state_output="Busy",
                id_client=order["id_client"],
                order=order["food_list"],
                success=True,
            )
        return robotstatedecisionResponse(
            state_output="Wait", id_client=None, order=[], success=False
        )


class BusyState(State):
    """Robot reports it has finished → flip back to *Free*."""
    def handle(self, context, req):
        robot_id = req.robot_id
        context.set_state(robot_id, FreeState())
        return robotstatedecisionResponse(
            state_output="Busy", id_client=None, order=[], success=True
        )


# --------------------------------------------------------------------------- #
#                ORCHESTRATION + COORDINATION  (main class)                   #
# --------------------------------------------------------------------------- #
class orchestration_and_coordination:
    """
    Persistent order queue + multi-robot FSM helper.

    Attributes
    ----------
    yaml_path : str
        Path to *tiago_data.yaml*.
    robot_states : dict[str, State]
        Per-robot FSM state objects.
    data : dict
        In-memory YAML contents (``orders`` list).
    """

    # --------------------------- INIT ----------------------------------- #
    def __init__(self):
        rospy.init_node("orchestration_and_coordination_node")

        # robot namespaces 1 … N
        self.number_of_robots = int(rospy.get_param("number_of_robots"))
        self.robot_list = [str(i) for i in range(1, self.number_of_robots + 1)]

        # YAML persistence
        self.yaml_path = os.path.join(os.path.dirname(__file__), "tiago_data.yaml")
        self.data = {"orders": []}
        self.load_data()

        # per-robot FSM
        self.robot_states = {rid: FreeState() for rid in self.robot_list}

        # advertise services per robot
        self.services = {}
        self.services_order = {}
        for rid in self.robot_list:
            self.services[rid] = rospy.Service(
                f"/{rid}/robot_state_decision", robotstatedecision, self.handle_request
            )
            self.services_order[rid] = rospy.Service(
                f"/{rid}/robot_state_decision_add", send_order, self.handle_request_new_order
            )

        rospy.loginfo("[Orchestration] Ready for robots %s", self.robot_list)

    # ---------------------- YAML helpers ------------------------------- #
    def load_data(self):
        """Load order queue from disk (create file if absent)."""
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, "r") as f:
                self.data = yaml.safe_load(f) or {"orders": []}
        else:
            with open(self.yaml_path, "w") as f:
                yaml.dump(self.data, f)

    def save_data(self):
        """Flush :pyattr:`data` back to YAML."""
        with open(self.yaml_path, "w") as f:
            yaml.dump(self.data, f)

    # ---------------------- queue ops ---------------------------------- #
    def obtain_order(self):
        """Pop and return oldest order or ``None`` if queue empty."""
        self.load_data()
        if self.data["orders"]:
            order = self.data["orders"].pop(0)
            self.save_data()
            rospy.loginfo(f"[Orchestration] Popped order: {order}")
            return order
        return None

    # ---------------------- FSM utility -------------------------------- #
    def set_state(self, robot_id: str, state: State):
        """Update FSM for *robot_id*."""
        rospy.loginfo(f"[Orchestration] {robot_id} → {state.__class__.__name__}")
        self.robot_states[robot_id] = state

    # ---------------------- service handlers --------------------------- #
    def handle_request(self, req: robotstatedecisionRequest):
        """Dispatch to the current FSM state."""
        return self.robot_states[req.robot_id].handle(self, req)

    def handle_request_new_order(self, req: send_order.Request) -> send_orderResponse:
        """Append new order from verifier into YAML queue."""
        new_order = {
            "id_client": req.order.id_client,
            "food_list": list(req.order.list_of_orders),
        }
        self.load_data()
        self.data["orders"].append(new_order)
        self.save_data()
        rospy.loginfo(f"[Orchestration] Added order: {new_order}")
        return send_orderResponse(message="Order successfully added")


# ---------------------------------------------------------------------- #
#                               bootstrap                                #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    try:
        orchestration_and_coordination()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
