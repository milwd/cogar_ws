#!/usr/bin/env python3
"""
camera_preprocessing.py
=======================

Real-time image *sanitiser* that sits between the raw camera node and higher-level
perception modules.  It consumes **unprocessed RGB + Depth streams** and
publishes *cleaned* versions with reduced noise, corrected illumination and
normalised depth values.

Why preprocess?
    • *Noise robustness* – downstream CNNs and classical detectors converge
      faster on well-behaved input.  
    • *Bandwidth reduction* – optional resizing or compression can be added
      here without touching consumer nodes.  
    • *Separation of concerns* – hardware quirks are handled once in this node
      instead of scattered across the codebase.

ROS Topics
-------
.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * - Topic
     - Type
     - Role
   * - ``/camera``
     - ``sensor_msgs/Image``
     - Raw 24-bit BGR frames
   * - ``/depth``
     - ``sensor_msgs/Image``
     - Raw depth map (any encoding)
   * - ``/camera_processed``
     - ``sensor_msgs/Image``
     - Denoised / colour-corrected RGB
   * - ``/depth_processed``
     - ``sensor_msgs/Image``
     - Thresholded / hole-filled depth


All processed messages retain the **original header stamps** so that time-based
synchronisation in later stages remains intact.

Node life-cycle
---------------
1. Initialise ``rospy`` under the name **camera_preprocessing_node**.  
2. Instantiate a single **CvBridge** (expensive to create, cheap to reuse).  
3. Subscribe to raw RGB and depth topics.  
4. Advertise the corresponding “*_processed” topics.  
5. Process images inside the two callbacks; no polling loop is needed.  
6. Keep spinning until ROS shutdown or user interruption.

"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys


class CameraPreprocessing:
    """
    Perform in-place enhancement of RGB and depth streams.

    Variables
    ----------
    bridge : cv_bridge.CvBridge
        Zero-copy converter between ``sensor_msgs/Image`` and ``numpy.ndarray``.
    rgb_sub : rospy.Subscriber
        Subscribes to the raw RGB frames on ``/camera``.
    depth_sub : rospy.Subscriber
        Subscribes to the raw depth frames on ``/depth``.
    rgb_pub : rospy.Publisher
        Publishes cleaned RGB frames on ``/camera_processed``.
    depth_pub : rospy.Publisher
        Publishes cleaned depth maps on ``/depth_processed``.
    """

    def __init__(self):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')

        """
        Register the node, wire the pubs/subs and leave the callbacks running.

        No explicit main loop is necessary; ``rospy.spin()`` in *main* keeps the
        process alive and dispatches incoming messages to the callbacks.
        """

        rospy.init_node(f'{self.robot_number}_camera_preprocessing_node')
        self.bridge = CvBridge()

        self.rgb_sub = rospy.Subscriber(f"/{self.robot_number}/camera", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber(f"/{self.robot_number}/depth", Image, self.depth_callback)

        self.rgb_pub = rospy.Publisher(f"/{self.robot_number}/camera_processed", Image, queue_size=10)
        self.depth_pub = rospy.Publisher(f"/{self.robot_number}/depth_processed", Image, queue_size=10)


        # ---------- Publications -------------------------------------------
        self.rgb_pub = rospy.Publisher(
            "/camera_processed", Image, queue_size=10)
        self.depth_pub = rospy.Publisher(
            "/depth_processed", Image, queue_size=10)

    # --------------------------------------------------------------------- #
    # Callbacks                                                             #
    # --------------------------------------------------------------------- #
    def rgb_callback(self, msg: Image) -> None:
        """
        Denoise and forward an RGB frame.

        Parameters
        ----------
        msg : sensor_msgs.msg.Image
            Raw BGR frame from ``/camera``.


        Workflow
        --------
        1. Convert to ``numpy.ndarray`` using **cv_bridge**.  
        2. Apply a *Gaussian blur* (remove salt-and-pepper noise).  
        3. Convert back to ROS ``Image``.  
        4. Copy the *header* from the incoming message so that timestamps and
           frame IDs stay consistent.  
        5. Publish to ``/camera_processed``.
        """
        #1. ROS → OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #2. Example processing – replace with your own pipeline
        processed = cv2.GaussianBlur(cv_image, (5, 5), 0)

        #3 & 4. OpenCV → ROS, preserve metadata
        out_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
        out_msg.header = msg.header

        #5. Publish
        self.rgb_pub.publish(out_msg)

    def depth_callback(self, msg: Image) -> None:
        """
        Clean up a depth map and re-emit it.

        Parameters
        ----------
        msg : sensor_msgs.msg.Image
            Raw depth map from ``/depth`` (encoding is passed through).


        Workflow
        --------
        1. Convert to ``numpy.ndarray`` without altering the encoding.  
        2. Apply a simple *lower threshold* – any depth below 0.5 m is set to 0,
           everything else is kept (acts as a near-clipping plane).  
        3. Convert back to ROS ``Image`` and copy the header.  
        4. Publish to ``/depth_processed``.
        """
        depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')

        # Threshold example – replace or extend as required
        _, processed = cv2.threshold(
            depth_image, 0.5, 5.0, cv2.THRESH_TOZERO)

        out_msg = self.bridge.cv2_to_imgmsg(
            processed, encoding='passthrough')
        out_msg.header = msg.header

        self.depth_pub.publish(out_msg)


# ------------------------------------------------------------------------- #
# Main                                                                      #
# ------------------------------------------------------------------------- #
if __name__ == '__main__':
    """
    Spin the node until Ctrl-C or ``rosnode kill`` is invoked.

    Any non-ROS exception bubbles up, printing a full traceback – invaluable
    during early development.
    """
    try:
        CameraPreprocessing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
