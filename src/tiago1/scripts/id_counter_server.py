#!/usr/bin/env python
"""
id_counter_server.py
====================

Unique **ID Generation Service** – thread-safe counter persisted via ROS Parameter Server
----------------------------------------------------------------------------------------

This node provides a simple ROS service that hands out **globally unique integer IDs**  
to clients on demand. It stores the current counter in the ROS Parameter Server  
under `id_client_counter`, so the count survives node restarts, and uses a  
threading lock to serialize concurrent requests safely.

Interfaces (strongly-typed, stateful)
-------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 30  25  45

   * - Interface
     - Type
     - Semantics
   * - **Service**: ``/get_next_id``
     - ``tiago1/GetNextId``
     - **Stateful RPC** – returns the next integer in a persisted counter; the service  
       maintains and updates its internal state (the counter) on each call.

ROS Service
-----------

.. list-table::
   :header-rows: 1
   :widths: 30  25  45

   * - Service Name
     - Type
     - Description
   * - ``/get_next_id``
     - ``tiago1/GetNextId``
     - Returns the next unique integer ID in the sequence.

"""

import rospy
import threading
from tiago1.srv import GetNextId, GetNextIdResponse


class IdCounterServer:
    """
    Provides the `/get_next_id` service and manages the counter state.

    Variables
    ----------
    id_counter : int
        Current value of the ID counter (loaded from Parameter Server).
    _id_lock : threading.Lock
        Ensures only one thread can increment `id_counter` at a time.
    id_service : rospy.Service
        Service handle for `/get_next_id`.
    """

    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("id_counter_server")

        # Load or initialize the counter from the Parameter Server
        self.id_counter = rospy.get_param("id_client_counter", 0)

        # Mutex to protect concurrent access
        self._id_lock = threading.Lock()

        # Advertise the service
        self.id_service = rospy.Service(
            "get_next_id", GetNextId, self.handle_get_next_id
        )

        rospy.loginfo("ID Counter Server is ready to provide unique IDs.")

    def handle_get_next_id(self, req):
        """
        Service callback: safely return the next ID and update the counter.

        Parameters
        ----------
        req : GetNextIdRequest
            Empty request (no fields).

        Returns
        -------
        GetNextIdResponse
            Contains the allocated ID.
        """
        with self._id_lock:
            next_id = self.id_counter
            self.id_counter += 1
            # Persist the updated counter
            rospy.set_param("id_client_counter", self.id_counter)

        return GetNextIdResponse(id=next_id)


if __name__ == "__main__":
    try:
        server = IdCounterServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
