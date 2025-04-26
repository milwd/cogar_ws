#!/usr/bin/env python
import rospy
import yaml
import os
from tiago1.srv import robotstatedecision, robotstatedecisionResponse
from tiago1.srv import send_order, send_orderResponse

# --- State Machine Base Classes ---

class State:
    def handle(self, context, req):
        raise NotImplementedError("Each state must implement the handle method.")

class FreeState(State):
    def handle(self, context, req):
        order = context.obtain_order()
        if order:
            context.set_state(BusyState())
            return robotstatedecisionResponse(state_output="Busy", id_client=order["id_client"], order=order["food_list"], success=True)
        else:
            context.set_state(WaitState())
            return robotstatedecisionResponse(state_output="Wait", id_client=None, order=[], success=False)

class WaitState(State):
    def handle(self, context, req):
        order = context.obtain_order()
        if order:
            context.set_state(BusyState())
            return robotstatedecisionResponse(state_output="Busy", id_client=order["id_client"], order=order["food_list"], success=True)
        else:
            return robotstatedecisionResponse(state_output="Wait", id_client=None, order=[], success=False)

class BusyState(State):
    def handle(self, context, req):
        # Robot remains busy until external change
        context.set_state(FreeState())
        return robotstatedecisionResponse(state_output="Busy", id_client=None, order=[], success=True)

# --- Orchestration Node ---

class orchestration_and_coordination:
    def __init__(self):
        rospy.init_node("orchestration_and_coordination_node")
        
        self.yaml_path = os.path.join(os.path.dirname(__file__), "tiago_data.yaml")
        
        self.service = rospy.Service("/robot_state_decision", robotstatedecision, self.handle_request)
        self.service_order = rospy.Service("/robot_state_decision_add", send_order, self.handle_request_new_order)
        
        self.data = {"orders": []}
        self.load_data()
        
        self.state = FreeState()  # Start in Free state

    def set_state(self, state):
        rospy.loginfo(f"Transitioning to state: {state.__class__.__name__}")
        self.state = state

    def load_data(self):
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as file:
                self.data = yaml.safe_load(file) or {"orders": []}
        else:
            self.data = {"orders": []}
            with open(self.yaml_path, 'w') as file:
                yaml.dump(self.data, file)

    def save_data(self):
        with open(self.yaml_path, 'w') as file:
            yaml.dump(self.data, file)

    def obtain_order(self):
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
        return self.state.handle(self, req)

    def handle_request_new_order(self, req):
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
    try:
        orchestration_and_coordination()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
