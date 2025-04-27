#!/usr/bin/env python
"""
orchestration_and_coordination.py

ROS node for order orchestration and coordination in the sushi restaurant.
Offers two services:
 - /robot_state_decision: returns the next order and robot state based on current input state.
 - /robot_state_decision_add: adds a new order to the pending orders list.

Loads and saves orders to a YAML file alongside the node.
"""

import rospy
import yaml
import os
from tiago1.srv import robotstatedecision, robotstatedecisionResponse
from tiago1.srv import send_order, send_orderResponse

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

# -- Orchestration Node --

class orchestration_and_coordination:
    """
    Manages order queue and robot state transitions via ROS services.

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

    def load_data(self):
        """
        Load pending orders from the YAML file into memory.
        Creates an empty structure if the file does not exist.

        Raises
        ------
        IOError
            If the YAML file exists but cannot be read.
        """
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as file:
                self.data = yaml.safe_load(file) or {"orders": []}
        else:
            self.data = {"orders": []}
            with open(self.yaml_path, 'w') as file:
                yaml.dump(self.data, file)

    def save_data(self):
        """
        Persist the current in-memory orders list to the YAML file.
        Overwrites the existing file.
        """
        with open(self.yaml_path, 'w') as file:
            yaml.dump(self.data, file)

    def obtain_order(self):
        """
        Pop the first pending order from the queue.

        Returns
        -------
        dict or None
            The order dictionary with keys 'id_client' and 'food_list',
            or None if no orders are pending.
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
        Service callback for '/robot_state_decision'.

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

    def handle_request_new_order(self, req):
        """
        Service callback for '/robot_state_decision_add'.

        Parameters
        ----------
        req : tiago1.srv.send_orderRequest
            Incoming request containing 'order' with fields 'id_client' and 'list_of_orders'.

        Returns
        -------
        send_orderResponse
            - message: confirmation string ("order added").
        """
        new_order = {
            "id_client": req.order.id_client,
            "food_list": list(req.order.list_of_orders)
        }
        self.load_data()
        self.data["orders"].append(new_order)
        self.save_data()

        rospy.loginfo(f"Added new order to YAML: {new_order}")
        return send_orderResponse(message="Order successfully added")

if __name__ == "__main__":
    """
    Main entrypoint: instantiate the orchestration node and spin.
    """
    try:
        orchestration_and_coordination()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
