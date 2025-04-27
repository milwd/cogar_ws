#!/usr/bin/env python
import rospy
import threading
from tiago1.srv import GetNextId, GetNextIdResponse

class IdCounterServer:
    def __init__(self):
        rospy.init_node("id_counter_server")

        self.id_counter = rospy.get_param("id_client_counter", 0)

        self._id_lock = threading.Lock()

        self.id_service = rospy.Service("get_next_id", GetNextId, self.handle_get_next_id)

        rospy.loginfo("ID Counter Server is ready to provide unique IDs.")

    def handle_get_next_id(self, req):
        with self._id_lock:
            next_id = self.id_counter
            self.id_counter += 1

            rospy.set_param("id_client_counter", self.id_counter)

        return GetNextIdResponse(id=next_id)

if __name__ == "__main__":
    try:
        server = IdCounterServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
