# #!/usr/bin/env python
# import rospy
# import yaml
# import os
# from std_msgs.msg import String
# from tiago1.srv import robotstatedecision

# class orchestration_and_coordination():
#     def __init__(self):
#         rospy.init_node("orchestration_and_coordination_node")
#         self.orc_coor_pub = rospy.Subscrbe("/orc_coord_info")
#         self.yaml_pat = "tiago_data.yaml"
#         self.service = rospy.Service("/robot_state_decision", robotstatedecision, self.handle_request)
#         self.service_order = rospy.Service("/robot_state_decision", robotstatedecision, self.handle_request_new_order)
#         self.data = {"robot":[]}

#     def handle_request(self,req):
#         get_order = self.obtain_order()
#         self.data = None
#         # Risposte in base allo stato ricevuto
#         if get_order == -1:
#             return robotstatedecision(state_output= req.state_input ,order="",success =False)
#         elif get_order ==None:
#             return robotstatedecision(state_output="Wait" ,order="",success =True)
#         elif get_order is not None  and (req.state_input == "Free" or req.state_input == "Wait")  :
#             return robotstatedecision(state_output="Busy", order=get_order,success=True)
        
        
        

#     def obtain_order(self):
#         first_order = None
#         if "robots" in self.data and self.data["robots"]:
#             robot = self.data["robots"][0]
#             if "order" in robot and robot["order"]:
#                 first_order = robot["order"].pop(0)
#                 rospy.loginfo(f"Popped first order: {first_order}")
#                 with open(self.yaml_path, 'w') as file:
#                     yaml.dump(self.data, file)
#                 return first_order
#             else:
#                 rospy.logwarn("No orders available to pop for robot.")
#                 return None
#         else:
#             rospy.logwarn("No robots found in YAML.")
#             return -1
        
#     def handle_request_new_order(self, req):
#         order = req.order 
#         self.add_order_to_yaml(order)     
        
   
#     def add_order_to_yaml(self, order):
#         if "robots" in self.data and self.data["robots"]:
#             # Add to the first robot's order list
#             # robot = self.data["robots"][0]

#             if "order" not in robot or robot["order"] is None:
#                 robot["order"] = []

#             robot["order"].append(order)
#             rospy.loginfo(f"Added order '{order}' to robot {robot.get('name', 'unknown')}.")

#             # Save back to the file
#             with open(self.yaml_path, 'w') as file:
#                 yaml.dump(self.data, file)
#         else:
#             rospy.logwarn("No robots found in YAML. Cannot add order.")
        
#     # def add_order_to_yaml(self, client_id, food_items):
#     #     yaml_path = os.path.join(os.path.dirname(__file__), "tiago_data.yaml")

#     #     # Load existing data or create empty structure
#     #     if os.path.exists(yaml_path):
#     #         with open(yaml_path, 'r') as file:
#     #             data = yaml.safe_load(file) or {"robots": []}
#     #     else:
#     #         data = {"robots": []}

#     #     # Look for the robot with the given client_id
#     #     robot_found = False
#     #     for robot in data["robots"]:
#     #         if robot.get("id_client") == client_id:
#     #             robot_found = True
#     #             robot.setdefault("order", []).extend(food_items)
#     #             break

#     #     # If robot with client_id not found, create a new entry
#     #     if not robot_found:
#     #         new_robot = {
#     #             "order": food_items,
#     #             "id_client": client_id,
#     #         }
#     #         data["robots"].append(new_robot)

#     #     # Save updated data back to the YAML file
#     #     with open(yaml_path, 'w') as file:
#     #         yaml.safe_dump(data, file)

#     #     rospy.loginfo(f"Order for client {client_id} added: {food_items}")

# if __name__ == "__main__":
    
#     try:
#         orchestrationReasoner = orchestration_and_coordination()
#         while not rospy.is_shutdown():
#             orchestrationReasoner.process()
#     except rospy.ROSInterruptException:
#         pass
#!/usr/bin/env python
import rospy
import yaml
import os
from tiago1.srv import robotstatedecision, robotstatedecisionResponse
from tiago1.srv import send_order,send_orderResponse

class orchestration_and_coordination():
    def __init__(self):
        rospy.init_node("orchestration_and_coordination_node")
        
        self.yaml_path = os.path.join(os.path.dirname(__file__), "tiago_data.yaml")
        
        self.service = rospy.Service("/robot_state_decision", robotstatedecision, self.handle_request)
        self.service_order = rospy.Service("/robot_state_decision_add", send_order, self.handle_request_new_order)
        
        self.data = {"orders": []}
        self.load_data()

    def load_data(self):
        if os.path.exists(self.yaml_path):
            with open(self.yaml_path, 'r') as file:
                self.data = yaml.safe_load(file) or {"orders": []}

    def save_data(self):
        with open(self.yaml_path, 'w') as file:
            yaml.dump(self.data, file)

    def obtain_order(self):
        self.load_data()
        if "orders" in self.data and self.data["orders"]:
            first_order = self.data["orders"].pop(0)
            self.save_data()
            rospy.loginfo(f"Popped order: {first_order}")
            return first_order
        else:
            rospy.logwarn("No orders found.")
            return None

    def handle_request(self, req):
        order = self.obtain_order()
        if order is None:
            return robotstatedecisionResponse(state_output="Wait", order=[], success=False)
        
        if req.state_input in ["Free", "Wait"]:
            return robotstatedecisionResponse(state_output="Busy", order=order["food_list"], success=True)
        else:
            return robotstatedecisionResponse(state_output=req.state_input, order=[], success=False)

    def handle_request_new_order(self, req):
        new_order = {
            "id_client": req.id_client,
            "food_list": list(req.order)  # assume order is a list of strings
        }
        self.load_data()
        self.data["orders"].append(new_order)
        self.save_data()
        rospy.loginfo(f"Added new order: {new_order}")
        return send_orderResponse(message ="order added")

if __name__ == "__main__":
    try:
        orchestration_and_coordination()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
