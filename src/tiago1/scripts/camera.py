#!/usr/bin/env python
"""
camera.py
=========

Synthetic **RGB + Depth** sensor for rapid, dependency-free testing
------------------------------------------------------------------

This script pretends to be a camera node even when no real hardware is
attached.  It continuously publishes **one static RGB frame** together with a
**mock depth map** so that downstream perception, fusion or visualisation
nodes can start development immediately.

Why a dummy camera?
    • *Deterministic output* – unit tests and integration pipelines do not rely  
      on external devices or recorded ROS bags.  
    • *Speed* – a single JPEG on disk avoids the overhead of connecting to  
      physical sensors.  
    • *Isolation* – failures in consumer nodes are never due to flaky hardware.

Published topics
----------------
=====================  =============  ==============================
Topic name             Type           Content
---------------------  -------------  ------------------------------
``/camera``            ``sensor_msgs/Image`` 24-bit BGR image read
                                      from *image.jpg*
``/depth``             ``sensor_msgs/Image`` 8-bit signed, all-ones  
                                      array the same resolution as the RGB
=====================  =============  ==============================

Both messages share *wall-clock* time stamps and are published at **10 Hz** so
that synchronisation filters (e.g. message_filters) treat them as a stereo
pair.

Directory layout
----------------
``camera.py`` expects the JPEG at  
``/root/Desktop/cogar_ass1/cogar_ws/src/tiago1/scripts/image.jpg``.  
Change the path if you move the file; no other configuration is required.

"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys


def publish():
    """
    Bootstrap the node and stream frames until ROS shuts down.

    Execution flow
    --------------
    1. **Node init** 
    – registers the process with the ROS master under the name ``camera``.
    
    2. **Publisher creation**

       * ``pub_rgb``  → ``/camera``  (BGR colour image)  

       * ``pub_depth`` → ``/depth``   (8-bit signed mono depth)
    3. **Main loop** (10 Hz):

       a. Read the *unchanging* JPEG from disk.  
          Reading from disk each cycle guarantees that edits to the file show up
          immediately, handy for quick visual experiments.  
       b. Convert the NumPy array to ``sensor_msgs/Image`` via **cv_bridge**.  
       c. Publish the RGB frame.  
       d. Allocate a depth array of identical resolution, fill it with ones
          (1 m in whatever synthetic unit you prefer), convert and publish.  
       e. Sleep the remaining time to keep the loop period at ~100 ms.
    4. On shutdown (Ctrl-C or ``rosnode kill``) the ``rospy.is_shutdown()``
       flag becomes *True* and the while-loop exits cleanly.

    Exceptions
    ----------
    Any exception raised inside the loop will abort the node unless you wrap
    the offending code in additional ``try/except`` blocks.  ROS-level
    interruptions (e.g. lost master) are already handled in ``main``.
    """
    robot_number = sys.argv[1]#rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_camera')
    # pub_rgb = rospy.Publisher('/camera', Image, queue_size=10)
    # pub_depth = rospy.Publisher('/depth', Image, queue_size=10)
    # rate = rospy.Rate(10)  
    # while not rospy.is_shutdown():
    #     original = cv2.imread('/root/Desktop/cogar_ass1/cogar_ws/src/tiago1/scripts/image.jpg')
    #     bridge = CvBridge()
    #     image = bridge.cv2_to_imgmsg(original, encoding="bgr8")
    #     image.header.stamp = rospy.Time.now()
    #     image.header.frame_id = 'camera_frame'
    #     pub_rgb.publish(image)
    #     depth_image = bridge.cv2_to_imgmsg(np.ones((original.shape[0], original.shape[1]), np.int8), encoding="8SC1")
    #     depth_image.header.stamp = rospy.Time.now()
    #     depth_image.header.frame_id = 'depth_frame'
    #     pub_depth.publish(depth_image)
    #     rate.sleep()


if __name__ == "__main__":
    """
    Entrypoint — start the publisher and swallow *only* ROS interruptions.

    Any other unexpected exception will propagate and leave a useful traceback
    in the ROS console, which is the preferred behaviour during development.
    """
    try:
        publish()
    except rospy.ROSInterruptException:
        # Expected when the node is killed gracefully.
        pass
