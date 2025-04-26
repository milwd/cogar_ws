#!/usr/bin/env python3
import rospy
import yaml
import os
from tiago1.srv import robotstatedecision, robotstatedecisionResponse
from tiago1.srv import send_order, send_orderResponse

# --- State Machine Base Classes ---

class State:
    def handle(self, context, req, robot_id):
        raise NotImplementedError("Each state must implement the handle method.")

class FreeState(State):
    def handle(self, context, req, robot_id):
        order = context.obtain_order(robot_id)
        if order:
            context.set_state(robot_id, BusyState())
            return robotstatedecisionResponse(state_output="Busy", id_client=order["id_client"], order=order["food_list"], success=True)
        else:
            context.set_state(robot_id, WaitState())
            return robotstatedecisionResponse(state_output="Wait", id_client=None, order=[], success=False)

class WaitState(State):
    def handle(self, context, req, robot_id):
        order = context.obtain_order(robot_id)
        if order:
            context.set_state(robot_id, BusyState())
            return robotstatedecisionResponse(state_output="Busy", id_client=order["id_client"], order=order["food_list"], success=True)
        else:
            return robotstatedecisionResponse(state_output="Wait", id_client=None, order=[], success=False)

class BusyState(State):
    def handle(self, context, req, robot_id):
        # Robot remains busy until external change
        context.set_state(robot_id, FreeState())
        return robotstatedecisionResponse(state_output="Busy", id_client=None, order=[], success=True)

# --- Orchestration Node ---

class orchestration_and_coordination:
    def __init__(self):
        rospy.init_node("orchestration_and_coordination_node")
        
        self.robot_list = ["robot1", "robot2", "robot3", "robot4", "robot5"]
        self.states = {robot: FreeState() for robot in self.robot_list}
        
        self.yaml_path = os.path.join(os.path.dirname(__file__), "orchestration_data.yaml")
        
        self.service = rospy.Service("/robot_state_decision", robotstatedecision, self.handle_request)
        self.service_order = rospy.Service("/robot_state_decision_add", send_order, self.handle_request_new_order)
        
        self.data = {robot: {"orders": []} for robot in self.robot_list}
        self.load_data()

    def set_state(self, robot_id, state):
        rospy.loginfo(f"[{robot_id}] Transitioning to state: {state.__class__.__name__}")
        self.states[robot_id] = state

    def load_data(self):
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as file:
                self.data = yaml.safe_load(file) or {robot: {"orders": []} for robot in self.robot_list}
        else:
            self.data = {robot: {"orders": []} for robot in self.robot_list}
            self.save_data()

    def save_data(self):
        with open(self.yaml_path, 'w') as file:
            yaml.dump(self.data, file)

    def obtain_order(self, robot_id):
        self.load_data()
        if self.data[robot_id]["orders"]:
            first_order = self.data[robot_id]["orders"].pop(0)
            self.save_data()
            rospy.loginfo(f"[{robot_id}] Popped order: {first_order}")
            return first_order
        else:
            rospy.logwarn(f"[{robot_id}] No orders found.")
            return None

    def handle_request(self, req):
        robot_id = req.robot_id  # Assuming your service request includes robot_id
        if robot_id not in self.robot_list:
            rospy.logerr(f"Unknown robot_id: {robot_id}")
            return robotstatedecisionResponse(state_output="Error", id_client=None, order=[], success=False)
        return self.states[robot_id].handle(self, req, robot_id)

    def handle_request_new_order(self, req):
        robot_id = req.robot_id
        new_order = {
            "id_client": req.order.id_client,
            "food_list": list(req.order.list_of_orders)
        }
        self.load_data()
        if robot_id not in self.robot_list:
            rospy.logerr(f"Trying to add order for unknown robot: {robot_id}")
            return send_orderResponse(message="Failed: Unknown robot")
        
        self.data[robot_id]["orders"].append(new_order)
        self.save_data()

        rospy.loginfo(f"[{robot_id}] Added new order to YAML: {new_order}")
        return send_orderResponse(message="Order successfully added")

if __name__ == "__main__":
    try:
        orchestration_and_coordination()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# #!/usr/bin/env python
# import rospy
# import yaml
# import os
# from tiago1.srv import robotstatedecision, robotstatedecisionResponse
# from tiago1.srv import send_order, send_orderResponse

# # --- State Machine Base Classes ---

# class State:
#     def handle(self, context, req):
#         raise NotImplementedError("Each state must implement the handle method.")

# class FreeState(State):
#     def handle(self, context, req):
#         order = context.obtain_order()
#         if order:
#             context.set_state(BusyState())
#             return robotstatedecisionResponse(state_output="Busy", id_client=order["id_client"], order=order["food_list"], success=True)
#         else:
#             context.set_state(WaitState())
#             return robotstatedecisionResponse(state_output="Wait", id_client=None, order=[], success=False)

# class WaitState(State):
#     def handle(self, context, req):
#         order = context.obtain_order()
#         if order:
#             context.set_state(BusyState())
#             return robotstatedecisionResponse(state_output="Busy", id_client=order["id_client"], order=order["food_list"], success=True)
#         else:
#             return robotstatedecisionResponse(state_output="Wait", id_client=None, order=[], success=False)

# class BusyState(State):
#     def handle(self, context, req):
#         # Robot remains busy until external change
#         context.set_state(FreeState())
#         return robotstatedecisionResponse(state_output="Busy", id_client=None, order=[], success=True)

# # --- Orchestration Node ---

# class orchestration_and_coordination:
#     def __init__(self):
#         rospy.init_node("orchestration_and_coordination_node")
#         # self.a = rospy.get_param('~robot_number')
#         self.yaml_path = os.path.join(os.path.dirname(__file__), "tiago_data.yaml")
        
#         self.service = rospy.Service(f"/{1}/robot_state_decision", robotstatedecision, self.handle_request)
#         self.service_order = rospy.Service(f"/{1}/robot_state_decision_add", send_order, self.handle_request_new_order)
        
#         self.data = {"orders": []}
#         self.load_data()
        
#         self.state = FreeState()  # Start in Free state

#     def set_state(self, state):
#         rospy.loginfo(f"Transitioning to state: {state.__class__.__name__}")
#         self.state = state

#     def load_data(self):
#         if os.path.exists(self.yaml_path):
#             with open(self.yaml_path, 'r') as file:
#                 self.data = yaml.safe_load(file) or {"orders": []}
#         else:
#             self.data = {"orders": []}
#             with open(self.yaml_path, 'w') as file:
#                 yaml.dump(self.data, file)

#     def save_data(self):
#         with open(self.yaml_path, 'w') as file:
#             yaml.dump(self.data, file)

#     def obtain_order(self):
#         self.load_data()
#         if self.data["orders"]:
#             first_order = self.data["orders"].pop(0)
#             self.save_data()
#             rospy.loginfo(f"Popped order: {first_order}")
#             return first_order
#         else:
#             rospy.logwarn("No orders found.")
#             return None

#     def handle_request(self, req):
#         return self.state.handle(self, req)

#     def handle_request_new_order(self, req):
#         new_order = {
#             "id_client": req.order.id_client,
#             "food_list": list(req.order.list_of_orders)
#         }
#         self.load_data()
#         self.data["orders"].append(new_order)
#         self.save_data()

#         rospy.loginfo(f"Added new order to YAML: {new_order}")
#         return send_orderResponse(message="Order successfully added")

# if __name__ == "__main__":
#     try:
#         orchestration_and_coordination()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass
