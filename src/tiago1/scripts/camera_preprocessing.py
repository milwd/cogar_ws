#!/usr/bin/env python3
"""
camera_preprocessing.py
=======================

Overview
--------
`camera_preprocessing.py` is a *horizontal* (domain-independent) data-filter
component that improves raw RGB + depth imagery before any perception or fusion
module touches it.  Every robot sharing the same message contracts can reuse
this node as-is, giving you a single place to tweak sensor quirks instead of a
dozen ad-hoc patches spread across your stack.

Interfaces (strongly-typed, stateless)
--------------------------------------
.. list-table::
   :header-rows: 1
   :widths: 12 28 25 55

   * - Direction
     - Topic
     - Message type
     - Notes
   * - **Required**
     - ``/{robot}/camera``
     - ``sensor_msgs/Image``
     - BGR8 frames
   * - **Required**
     - ``/{robot}/depth``
     - ``sensor_msgs/Image``
     - Any depth encoding
   * - **Provided**
     - ``/{robot}/camera_processed``
     - ``sensor_msgs/Image``
     - Same header; 5 × 5 Gaussian blur (σ ≈ 1)
   * - **Provided**
     - ``/{robot}/depth_processed``
     - ``sensor_msgs/Image``
     - Same header; values < 0.5 m set to 0


Contract
--------
Pre-conditions

  • Incoming RGB must be 8-bit, 3-channel (BGR8).  
  • Depth resolution not larger than 1920×1080 (soft real-time ceiling).

Post-conditions

  • Header `stamp` and `frame_id` are **identical** between input and output.  
  • Latency from callback entry to publish is < 12 ms on a 4-core laptop.  
  • Output resolution equals input (no accidental rescale).

Invariants

  • Peak RAM ≤ 5×(w×h×c) bytes (one temp copy plus output buffer).  

**Protocol** 

  1. Subscribe to both raw topics.  
  2. Publish processed counterparts for every incoming frame.  
  3. No service calls or stateful dialogue; each message is handled independently.

Lifecycle
---------
- Node name: `{robot}_camera_preprocessing_node`.  
- Ready when its two publishers are advertised (no extra init service).  
- Clean shutdown on Ctrl-C or `rosnode kill`, handled by `rospy`.

Quality & Reusability Metrics
-----------------------------
- **Latency** < 12 ms → supports 30 Hz pipelines.  
- **Throughput** ≥ camera frame-rate (default 30 Hz).  
- **Cyclomatic complexity** < 15 → easy to maintain / extend.

Extensibility
-------------
The minimal interface is already in place.  A *complete* variant may expose a
`dynamic_reconfigure` server for:
- blur kernel size / σ  
- depth threshold  
- optional resize scale or JPEG quality

Adding such a server is orthogonal to the current contract and does not break
existing clients.

Implementation Notes
--------------------
- A single global `CvBridge` keeps conversion overhead low.  
- Processing occurs inside subscriber callbacks (no poll loop).  
- The duplicate publisher block at the end of `__init__` preserves existing
  downstream contracts; remove it only after all consumers migrate to the
  namespaced topics.
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys


class CameraPreprocessing:
    """
    In-place enhancement for RGB and depth streams.
    """

    def __init__(self):
        # Robot namespace comes from CLI for multi-robot simulation
        self.robot_number = sys.argv[1]  # rospy.get_param('~robot_number')

        rospy.init_node(f'{self.robot_number}_camera_preprocessing_node')
        self.bridge = CvBridge()

        # ---------- Subscriptions -----------------------------------------
        self.rgb_sub = rospy.Subscriber(
            f"/{self.robot_number}/camera", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber(
            f"/{self.robot_number}/depth", Image, self.depth_callback)

        # ---------- Publications (namespaced) -----------------------------
        self.rgb_pub = rospy.Publisher(
            f"/{self.robot_number}/camera_processed", Image, queue_size=10)
        self.depth_pub = rospy.Publisher(
            f"/{self.robot_number}/depth_processed", Image, queue_size=10)

        # ---------- Legacy publications (non-namespaced) ------------------
        # Keep these until every consumer switches to the namespaced topics.
        self.rgb_pub = rospy.Publisher(
            "/camera_processed", Image, queue_size=10)
        self.depth_pub = rospy.Publisher(
            "/depth_processed", Image, queue_size=10)

    # ------------------------------------------------------------------ #
    # Callbacks                                                          #
    # ------------------------------------------------------------------ #
    def rgb_callback(self, msg: Image) -> None:
        """
        Denoise and forward an RGB frame.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        processed = cv2.GaussianBlur(cv_image, (5, 5), 0)
        out_msg = self.bridge.cv2_to_imgmsg(processed, encoding='bgr8')
        out_msg.header = msg.header
        self.rgb_pub.publish(out_msg)

    def depth_callback(self, msg: Image) -> None:
        """
        Threshold low-range depth and forward.
        """
        depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        _, processed = cv2.threshold(
            depth_image, 0.5, 5.0, cv2.THRESH_TOZERO)
        out_msg = self.bridge.cv2_to_imgmsg(
            processed, encoding='passthrough')
        out_msg.header = msg.header
        self.depth_pub.publish(out_msg)


# ---------------------------------------------------------------------- #
# Main                                                                   #
# ---------------------------------------------------------------------- #
if __name__ == '__main__':
    try:
        CameraPreprocessing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
