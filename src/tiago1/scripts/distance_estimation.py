#!/usr/bin/env python
"""
distance_estimation.py
======================

3-in-1 node: **depth-aware object localisation + table-state reasoning**
-----------------------------------------------------------------------

The node fuses *semantic* information from an object-detection topic with
*metric* data from a depth image to output two things:

1. **/object_positions** – a human-readable list of XYZ positions for every
   object currently visible to the camera (single string message for easy
   logging or GUI display).

2. **/placement_decision** – a concise keyword that tells the waiter robot
   **what to do next at the table** ( *PLACE* a dish, *FULL* do nothing,
   *CLEAR* remove crockery, or *IGNORE*).

Both decisions are produced via the **Strategy pattern**, so swapping the
business logic requires only instantiating a different strategy class.

Why split placement vs clearing?
    • Placement reasoning only cares about *free space* (≤ 3 items).  
    • Clearing reasoning only cares about detecting *specific* objects
      (plate + cup).  
    Keeping them in separate interchangeable strategies lets you tune or
    replace each behaviour independently.

ROS I/O summary
---------------

========== ============================ =======================================
Direction  Topic name / Type            Description
========== ============================ =======================================
Subscribe  ``/camera_detections``       ``std_msgs/String`` – e.g.  
                                          ``"Detected: cup, plate"``
Subscribe  ``/depth_processed``         ``sensor_msgs/Image`` – 32-bit float
Publish    ``/object_positions``        ``std_msgs/String`` – formatted XYZ list
Publish    ``/placement_decision``      ``std_msgs/String`` –  
                                        ``"Decision: PLACE, CLEAR"`` etc.
========== ============================ =======================================

Assumptions & Limitations
-------------------------
* Exactly one RGB/Depth camera with **pinhole intrinsics** (fx, fy, cx, cy
  hard-coded below).
* Only the image-centre pixel is sampled for depth right now – replace that
  placeholder with real 2-D bounding-box → 3-D projection for production use.
* Object detection arrives as a simple string; in a real pipeline you would
  subscribe to a custom message carrying bounding boxes and class labels.

"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
from abc import ABC, abstractmethod
import sys


# --------------------------------------------------------------------------- #
#                       STRATEGY-PATTERN INTERFACES                           #
# --------------------------------------------------------------------------- #
class TableAnalysisStrategy(ABC):
    """
    Abstract contract for any *table-state* decision algorithm.

    Subclasses must override :py:meth:`analyze` and return **one of four**
    canonical keywords:

        * ``"PLACE"``  – safe to put another plate.  
        * ``"FULL"``   – table already crowded; wait.  
        * ``"CLEAR"``  – fetch dirty items.  
        * ``"IGNORE"`` – no action required.

    Design notes
    ------------
    • The strategy receives a **plain Python list** of object class‐names to
      keep the interface agnostic of the detector message format.  
    • The caller combines multiple strategies (placement + clearing) if both
      decisions are required.
    """

    @abstractmethod
    def analyze(self, objects):
        """
        Decide what to do given the current object list.

        Parameters
        ----------
        objects : list[str]
            Detected class names, lowercase.

        Returns
        -------
        str
            One of the four keywords defined above.
        """
        raise NotImplementedError


class FindPlacementStrategy(TableAnalysisStrategy):
    """
    *Minimal-viable* placement logic.

    Heuristic
    ---------
    If the table currently hosts **fewer than three** detected objects, there
    is space for another dish → return ``"PLACE"``.  Otherwise return
    ``"FULL"``.
    """

    def analyze(self, objects):
        return "PLACE" if len(objects) < 3 else "FULL"


class ClearingStrategy(TableAnalysisStrategy):
    """
    Simple clearing heuristic.

    Heuristic
    ---------
    If *both* ``'plate'`` **and** ``'cup'`` appear in the detection list the
    table is presumed finished → return ``"CLEAR"``.  Any other combination
    yields ``"IGNORE"``.
    """

    def analyze(self, objects):
        return "CLEAR" if {"plate", "cup"} <= set(objects) else "IGNORE"


# --------------------------------------------------------------------------- #
#                      MAIN NODE IMPLEMENTATION                               #
# --------------------------------------------------------------------------- #
class DistanceEstimator:
    """
    Combines perception and reasoning in one ROS node.

    Life-cycle
    ----------
    • **__init__** – set up pubs/subs, allocate strategy objects.  
    • **detection_callback** / **depth_callback** – store latest data and call
      :py:meth:`try_estimate_positions`.  
    • **try_estimate_positions** – when *both* modalities are present:
        1. Parse detection string → list.  
        2. For each class name, pick a proxy pixel (centre for now), read its
           depth, back-project to XYZ using pinhole equation.  
        3. Publish formatted XYZ list.  
        4. Run both strategies, merge their keywords, publish.  
        5. Clear stored detections to avoid duplicate processing.

    Public variables are intentionally *not* exposed; everything is internal
    to the node.
    """

    # ---- initialisation ---------------------------------------------------- #
    def __init__(self):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f"{self.robot_number}_distance_estimator_node")

        # ---------- ROS wiring --------------------------------------------- #
        self.detection_sub = rospy.Subscriber(f"/{self.robot_number}/camera_detections", String, self.detection_callback)
        self.depth_sub = rospy.Subscriber(f"/{self.robot_number}/depth_processed", Image, self.depth_callback)
        self.position_pub = rospy.Publisher(f"/{self.robot_number}/object_positions", String, queue_size=10)
        self.reasoning_pub = rospy.Publisher(f"/{self.robot_number}/placement_decision", String, queue_size=10)

        self.position_pub = rospy.Publisher(
            "/object_positions", String, queue_size=10)
        self.reasoning_pub = rospy.Publisher(
            "/placement_decision", String, queue_size=10)

        # ---------- state holders ------------------------------------------ #
        self.latest_detections = None              # raw string
        self.latest_depth = None                   # np.ndarray
        self.bridge = CvBridge()                   # reuse instance

        # ---------- strategy objects --------------------------------------- #
        self.placement_strategy = FindPlacementStrategy()
        self.clearing_strategy = ClearingStrategy()

    # ---- subscribers ------------------------------------------------------- #
    def detection_callback(self, msg):
        """
        Store latest detections and attempt full pipeline.

        Parameters
        ----------
        msg : std_msgs.String
            Example payload: ``"Detected: cup, plate"``
        """
        self.latest_detections = msg.data
        self.try_estimate_positions()

    def depth_callback(self, msg):
        """
        Convert depth image to NumPy and attempt full pipeline.

        Any conversion error is logged and ignored so the node keeps running.
        """
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough")
            self.try_estimate_positions()
        except Exception as exc:
            rospy.logerr(f"[distance_estimator] depth conversion failed: {exc}")

    # ---- core processing --------------------------------------------------- #
    def try_estimate_positions(self):
        """
        Run localisation + reasoning once *both* modalities are present.

        The method resets ``self.latest_detections`` afterwards so each detection
        batch is processed exactly once, regardless of depth FPS.
        """
        if self.latest_detections is None or self.latest_depth is None:
            return

        objects = self.parse_detections(self.latest_detections)
        xyz_entries = []

        # For each object class, sample the centre pixel as a placeholder --------
        # Replace with proper bounding-box iteration if available.
        u, v = 320, 240
        depth_val = float(self.latest_depth[v, u])
        x, y, z = self.project_to_robot_frame(u, v, depth_val)

        for obj_name in objects:
            xyz_entries.append(f"{obj_name} @ [{x:.2f}, {y:.2f}, {z:.2f}]")

        # ---------- publish positions -------------------------------------- #
        self.position_pub.publish(String(data="; ".join(xyz_entries)))
        rospy.loginfo_once("distance_estimator: publishing object positions.")

        # ---------- publish reasoning -------------------------------------- #
        place_kw = self.placement_strategy.analyze(objects)
        clear_kw = self.clearing_strategy.analyze(objects)
        decision_str = f"Decision: {place_kw}, {clear_kw}"
        self.reasoning_pub.publish(String(data=decision_str))
        rospy.loginfo(f"distance_estimator: {decision_str}")

        # Reset detection list to avoid re-use with stale depth
        self.latest_detections = None

    # ---- helpers ----------------------------------------------------------- #
    @staticmethod
    def parse_detections(raw):
        """
        Turn ``"Detected: cup, plate"`` → ``['cup', 'plate']``.

        Any parsing failure returns an *empty list* instead of throwing.
        """
        try:
            return raw.replace("Detected:", "").strip().split(", ")
        except Exception as exc:
            rospy.logwarn(f"parse_detections failed: {exc}")
            return []

    @staticmethod
    def project_to_robot_frame(u, v, depth):
        """
        Pinhole back-projection from (u,v,depth) to metric XYZ.

        Intrinsics here are **example values** for a 640×480 image; adjust to
        your real camera.

        Returns
        -------
        tuple[float, float, float]
            (x, y, z) in the camera coordinate frame.
        """
        fx = fy = 525.0
        cx, cy = 320, 240
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z


# --------------------------------------------------------------------------- #
#                                 MAIN                                        #
# --------------------------------------------------------------------------- #
def main():
    """
    Spin the node until ROS master shuts down.

    Any non-ROS exception bubbles up with a full traceback for easier debugging.
    """
    try:
        DistanceEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        # Expected on Ctrl-C or rosnode kill
        pass


if __name__ == "__main__":
    main()