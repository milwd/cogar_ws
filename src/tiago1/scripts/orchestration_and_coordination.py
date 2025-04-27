#!/usr/bin/env python
"""
orchestration_and_coordination.py
=================================

**Order queue + state gateway** for a fleet of sushi-waiter robots
------------------------------------------------------------------

The node owns a *persistent* FIFO of customer orders and exposes two ROS
services:

* ``/robot_state_decision`` – given the current **state** of a robot
  (``Free``, ``Busy``, ``Wait`` …) it returns the **next task** and updates the
  state machine.
* ``/robot_state_decision_add`` – appends a freshly verified customer order to
  the queue.

All orders survive restarts thanks to a YAML file on disk.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 25 35 40

   * - Type
     - Name
     - Semantics
   * - **service** ``tiago1/robotstatedecision``
     - ``/robot_state_decision``
     - Input  → **state_input** (str)  
       Output → **state_output** (str), **order** (string list), **success** (bool)
   * - **service** ``tiago1/send_order``
     - ``/robot_state_decision_add``
     - Input  → **order.id_client**, **order.list_of_orders**  
       Output → **message** (“order added”)

Persistent storage
------------------
Orders are stored in *tiago_data.yaml* next to this script::

   orders:
     - id_client: 7
       food_list: ['sushi', 'pasta']
     - id_client: 9
       food_list: ['ramen']

The file is loaded on every service call so multiple orchestrators (or manual
edits) stay in sync.

Decision rules
--------------
#. If the queue is **empty** → return ``state_output = "Wait"`` and
   ``success = False``.  
#. If *state_input* is ``Free`` or ``Wait`` and **an order exists** → pop the
   first order, return it with ``state_output = "Busy"`` and
   ``success = True``.  
#. Otherwise echo the original state and set ``success = False``.

"""

import os
import yaml
import rospy
from tiago1.srv import (
    robotstatedecision, robotstatedecisionResponse,
    send_order,         send_orderResponse,
)

# -- State Machine Base Classes --

class State:
    def handle(self, context, req):
        raise NotImplementedError("Each state must implement the handle method.")

class FreeState(State):
    def handle(self, context, req):
        order = context.obtain_order()
        robot_id = req.robot_id
        if order:
            context.set_state(robot_id, BusyState())
            return robotstatedecisionResponse(state_output="Busy", id_client=order["id_client"], order=order["food_list"], success=True)
        else:
            context.set_state(robot_id, WaitState())
            return robotstatedecisionResponse(state_output="Wait", id_client=None, order=[], success=False)

class WaitState(State):
    def handle(self, context, req):
        order = context.obtain_order()
        robot_id = req.robot_id
        if order:
            context.set_state(robot_id, BusyState())
            return robotstatedecisionResponse(state_output="Busy", id_client=order["id_client"], order=order["food_list"], success=True)
        else:
            return robotstatedecisionResponse(state_output="Wait", id_client=None, order=[], success=False)

class BusyState(State):
    def handle(self, context, req):
        # it is still a dummy implementation
        robot_id = req.robot_id
        context.set_state(robot_id, FreeState())
        return robotstatedecisionResponse(state_output="Busy", id_client=None, order=[], success=True)


class orchestration_and_coordination:
    """
    Persistent order queue + state-transition helper.
    Variables
    ----------
    yaml_path : str
        Filesystem path to the YAML file storing pending orders.
    data : dict
        In-memory representation of orders, with key "orders" mapping to a list.
    service : rospy.Service
        Service server for '/robot_state_decision'.
    service_order : rospy.Service
        Service server for '/robot_state_decision_add'.
    """
    def __init__(self):
        """
        Initialize the orchestration node:

        - Initialize ROS node 'orchestration_and_coordination_node'.
        - Determine YAML storage path for orders.
        - Advertise two services: robot_state_decision and robot_state_decision_add.
        - Load existing orders from disk.
        """
        rospy.init_node("orchestration_and_coordination_node")
        self.number_of_robots = int(rospy.get_param("number_of_robots"))

        self.yaml_path = os.path.join(os.path.dirname(__file__), "tiago_data.yaml")
        
        self.robot_list = [str(i) for i in range(1, self.number_of_robots + 1)]
        self.services       = {}
        self.services_order = {}
        self.robot_states = {robot_id: FreeState() for robot_id in self.robot_list}

        for robot_id in self.robot_list:
            self.services[robot_id] = rospy.Service(
                f"/{robot_id}/robot_state_decision",
                robotstatedecision,
                self.handle_request
            )
            self.services_order[robot_id] = rospy.Service(
                f"/{robot_id}/robot_state_decision_add",
                send_order,
                self.handle_request_new_order
            )

        print(self.robot_states)
        
        self.data = {"orders": []}
        self.load_data()
        
    def set_state(self, robot_id, state):
        rospy.loginfo(f"Transitioning to state: {state.__class__.__name__}")
        self.robot_states[robot_id] = state

    # ------------------------------------------------------------------ #
    #                        YAML persistence                             #
    # ------------------------------------------------------------------ #
    def load_data(self) -> None:
        """Load the order list from disk (creates an empty one if missing)."""
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as file:
                self.data = yaml.safe_load(file) or {"orders": []}
        else:
            self.data = {"orders": []}
            with open(self.yaml_path, 'w') as file:
                yaml.dump(self.data, file)

    def save_data(self) -> None:
        """Write :pyattr:`data` back to *tiago_data.yaml*."""
        with open(self.yaml_path, "w", encoding="utf-8") as f:
            yaml.dump(self.data, f)

    # ------------------------------------------------------------------ #
    #                        queue operations                             #
    # ------------------------------------------------------------------ #
    def obtain_order(self) -> dict | None:
        """
        Pop and return the *oldest* order, or *None* if the queue is empty.
        """
        self.load_data()
        if self.data["orders"]:
            first_order = self.data["orders"].pop(0)
            self.save_data()
            rospy.loginfo(f"Popped order: {first_order}")
            return first_order
        else:
            rospy.logwarn("No orders found.")
            return None

    def handle_request(self, req):
        """
        Parameters
        ----------
        req : tiago1.srv.robotstatedecisionRequest
            Incoming request containing 'state_input'.

        Returns
        -------
        robotstatedecisionResponse
            - state_output: "Busy" if Free/Wait and an order exists, else original state or "Wait".
            - order: list of food items for the robot to serve.
            - success: True on valid transition with order, False otherwise.
        """
        robot_id = req.robot_id
        return self.robot_states[robot_id].handle(self, req)


    def handle_request_new_order(self, req: send_order.Request
                                 ) -> send_orderResponse:
        """Append a new order sent by the order-verification node."""
        new_order = {
            "id_client": req.order.id_client,
            "food_list": list(req.order.list_of_orders),
        }
        self.load_data()
        self.data["orders"].append(new_order)
        self.save_data()
        rospy.loginfo(f"Added new order to YAML: {new_order}")
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
