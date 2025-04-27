#! /usr/bin/env python
"""
reasoning_order_verification.py
===============================

Natural–language **order gatekeeper** between speech recognition
and the orchestration layer
----------------------------------------------------------------

When a customer speaks, the voice-recognition pipeline converts the sentence
into plain text and publishes it on ``/voice_recogn``.  This node checks that
the sentence **1) actually requests something** and **2) mentions a dish that
exists**, then forwards the structured order to the task-manager while
reporting any problems on a dedicated error channel.

ROS interface
~~~~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 20 25 55

   * - Direction
     - Name
     - Type / semantics
   * - **subscribe**
     - ``/voice_recogn``
     - ``std_msgs/String`` – raw text captured by ASR
   * - **publish**
     - ``/verif_T_manager``
     - ``tiago1/Voice_rec`` – validated order, ready for planning
   * - **publish**
     - ``/error_from_interaction``
     - ``std_msgs/Int32`` – 1 = unknown dish, 2 = missing *“Can I have”*
   * - **service client**
     - ``/robot_state_decision_add``
     - ``tiago1/send_order`` – push the order into the orchestration FIFO

Validation policy
-----------------
#. **Syntax guard** – the sentence *must* contain the polite request  
   fragment *“Can I have”*.  Anything else is ignored to prevent false
   triggers such as background chatter.  
#. **Menu match** – at least one entry from *food_list* must appear in the
   remainder of the sentence.  The match is *substring-based* and therefore
   case-sensitive; tailor *food_list* accordingly (e.g. use lowercase if your
   ASR outputs lowercase).  
#. **Multi-dish support** – if the customer orders several items in one
   breath (“Can I have **sushi** and **pasta** please”), all matches are packed
   into a single ``Voice_rec`` message so that the kitchen receives a single
   ticket.

Error codes
-----------
* **1** – no recognised dish in the sentence  
* **2** – polite request fragment not found

Both errors are *recoverable*: the waiter robot simply prompts the customer to
repeat or rephrase the order.

"""

import rospy
from std_msgs.msg import String, Int32
from tiago1.msg import Voice_rec
from tiago1.srv import send_order,send_orderRequest
import sys



class ReasoningOrderVerification:
    """
    Parse spoken orders, validate them, publish structured messages and
    interact with the orchestration queue.

    Variables
    ----------

    server_robots : int
        Numerical identifier for the target orchestration queue (kept for
        future use – the current implementation always contacts the same
        service).
    food_list : list[str]
        Canonical dish names; matching is **substring-based** and therefore
        case-sensitive.

    Variables
    ----------

    msg : std_msgs.msg.String | None
        Buffer holding the most recent sentence received from the ASR.
    counter_id : int
        Auto-incrementing ticket number attached to every successful order.
    error_code : int
        Last error reported (0 = no error).
    """
    
    def __init__(self, server_robots,food_list):
        """
        Initialize the ReasoningOrderVerification node.

        Parameters
        ----------
        server_robots : int
            Placeholder for robot server identifier.
        food_list : list of str
            List of valid dishes to match in voice commands.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_reasoning_order_verification_node')
        self.msg=None
        self.server = server_robots
        self.food_list = food_list
        rospy.Subscriber(f'/{self.robot_number}/voice_recogn',String,self.string_callback)
        rospy.wait_for_service(f'/{self.robot_number}/robot_state_decision_add')  
        self.server_client = rospy.ServiceProxy(f'/{self.robot_number}/robot_state_decision_add', send_order)  
        self.error_notification = rospy.Publisher(f'/{self.robot_number}/error_from_interaction', Int32, queue_size=10)
        self.verific_taskmanager = rospy.Publisher(f'/{self.robot_number}/verif_T_manager', Voice_rec, queue_size = 10)
        self.counter_id = 0
        self.error_code = 0 # 0-> no error, 1-> no dish in order, 2-> order doesnt have "can I have"

               
    def string_callback(self,msg):
        """
        Callback invoked when a voice recognition message arrives.

        Parameters
        ----------
        msg : std_msgs.msg.String
            The raw voice recognition output.
        """
        self.msg=msg
        

            
    def parse_order(self):
        """
        Inspect :pyattr:`self.msg`; if valid, publish a ``Voice_rec`` order and
        forward it to the orchestration queue.  Otherwise emit an error code.
        """
        # rospy.loginfo(f"Received voice message: {self.msg}")
        if self.msg is not None and self.msg != '':
            # rospy.loginfo(f"Received voice message: {self.msg.data}")
            if "Can I have" in self.msg.data:
                found_items = [food for food in self.food_list if food in self.msg.data]
                # rospy.loginfo(f"Found items: {found_items}")
                if found_items:
                    order = Voice_rec()
                    order.id_client = self.counter_id
                    for item in found_items:
                        order.list_of_orders.append(item)
                    self.send_request(order)
                    self.verific_taskmanager.publish(order)
                    # rospy.loginfo(f"Published verified order: {order}")
                    self.counter_id += 1
                else:
                    self.error_code = 1
                    self.error_notification.publish(self.error_code)
                    # rospy.logwarn("No valid dish found in the order.")
                    self.error_code = 0
            else:
                self.error_code = 2
                self.error_notification.publish(self.error_code)
                # rospy.logwarn("The sentence doesn't contain 'Can I have'")

                self.error_code = 0

        sentence = self.msg.data
        self.msg = None  # mark as consumed

        # ---------- rule 1: polite request present? ----------------------- #
        if "Can I have" not in sentence:
            self._raise_error(2, "Missing polite request fragment.")
            return

        # ---------- rule 2: at least one valid dish? ---------------------- #
        found_items = [dish for dish in self.food_list if dish in sentence]
        if not found_items:
            self._raise_error(1, "No known dish mentioned.")
            return

        # ---------- build & publish structured order ---------------------- #
        order = Voice_rec()
        order.id_client = self.counter_id
        order.list_of_orders.extend(found_items)

        self.verific_taskmanager.publish(order)
        rospy.loginfo(f"[OrderVerifier] Order accepted → {order}")

        # Optional: push directly to orchestration queue
        self.send_request(order)

        self.counter_id += 1

    # --------------------------------------------------------------------- #
    #                              helpers                                  #
    # --------------------------------------------------------------------- #
    def _raise_error(self, code: int, log_msg: str) -> None:
        """Publish *code* to ``/error_from_interaction`` and log a warning."""
        self.error_notification.publish(code)
        rospy.logwarn(f"[OrderVerifier] {log_msg} (code {code})")

    def send_request(self, order: Voice_rec) -> None:
        """
        Forward the validated order to the orchestration service.

        The service appends the ticket to a FIFO consumed by the task manager,
        allowing multiple robots to share a common queue without race
        conditions.
        """
        try:
            req = send_orderRequest(order=order)
            resp = self.server_client(req)
            rospy.loginfo(f"[OrderVerifier] Service response: {resp.message}")
        except rospy.ServiceException as exc:
            rospy.logerr(f"[OrderVerifier] Service call failed: {exc}")


# ------------------------------------------------------------------------- #
#                                   main                                    #
# ------------------------------------------------------------------------- #
if __name__ == "__main__":
    """
    Main entrypoint: instantiate the verification node and run parse_order() at 1 Hz.
    """
    try:    
        dish_list = ["ragu", "sugo", "pasta", "kebab", "sushi","riso", "frnach", "shit", "hamburger","steak"]
        server_robots =1
        reasonerOrderVerification = ReasoningOrderVerification(server_robots,dish_list)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            verifier.parse_order()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.logerr("[OrderVerifier] Node interrupted – shutting down.")
