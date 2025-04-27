#!/usr/bin/env python
"""
object_detection.py
===================

Placeholder **vision node** that pretends to recognise objects in every frame
--------------------------------------------------------------------------

The node listens to the *already-preprocessed* RGB stream and emits a constant
string describing three items.  It runs at **1 Hz**, slow enough to be replaced
later by a real detector running on GPU/CPU accelerators without affecting
subscriber timing.

Topic contracts
---------------
==========================  =============  ======================================
Topic                       Type           Direction
--------------------------  -------------  --------------------------------------
``/camera_preprocessed``    ``sensor_msgs/Image``   **Subscribed** – cleaned RGB
``/camera_detections``      ``std_msgs/String``     **Published** – placeholder
==========================  =============  ======================================

The published text follows the simple format:

``Detected: <object1>, <object2>, <object3>``

Feel free to replace the hard-coded list or switch to structured messages
(e.g. ``vision_msgs/Detection2DArray``) once a real model is available.
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String  
import sys


class ObjectDetector:
    """
    Minimal façade around a future object-detection pipeline.

    Variables
    ----------
    image_sub : rospy.Subscriber
        Receives frames from ``/camera_preprocessed``.
    detect_pub : rospy.Publisher
        Emits detection results on ``/camera_detections``.
    """

    def __init__(self):
        """
        Register the subscriber and publisher.

        No additional state is required because the current implementation
        returns a fixed string regardless of the incoming image.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_object_detection_node')
        self.image_sub = rospy.Subscriber(f"/{self.robot_number}/camera_preprocessed", Image, self.image_callback)
        self.detect_pub = rospy.Publisher(f"/{self.robot_number}/camera_detections", String, queue_size=10)

    # ------------------------------------------------------------------ #
    # Callback                                                            #
    # ------------------------------------------------------------------ #
    def image_callback(self, _msg: Image):
        """
        Produce and publish a constant detection result.

        Parameters
        ----------
        _msg : sensor_msgs.msg.Image
            Ignored content of the pre-processed frame.


        Workflow
        --------
        1. Assemble the string ``"Detected: Table, Plate, Position"``. 

        2. Wrap it in :pyclass:`std_msgs.msg.String`.  

        3. Publish on ``/camera_detections``.
        """
        detection = String(data="Detected: Table, Plate, Position")
        self.detect_pub.publish(detection)


# -------------------------------------------------------------------- #
# Main                                                                  #
# -------------------------------------------------------------------- #
if __name__ == '__main__':
    """
    Initialise the node, instantiate :class:`ObjectDetector` and tick at 1 Hz.

    The explicit loop guarantees a steady publication rate even if images arrive
    faster; remove the loop once the real detector’s processing time naturally
    throttles the output.
    """
    detector = ObjectDetector()
    rate = rospy.Rate(1)  # 1 Hz

    try:
        while not rospy.is_shutdown():
            detector.image_callback(None)   # invoke manually to keep the rate
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object detection node terminated.")
