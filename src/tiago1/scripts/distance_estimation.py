#!/usr/bin/env python
"""
distance_estimation.py
======================

Overview
--------
`distance_estimation.py` is a **vertical sensor-fusion component** that combines
semantic detections with depth imagery to deliver both *metric positions* and
a high-level *table-state* decision (place, clear, etc.).  Internally it uses
the **Strategy pattern** so placement and clearing logic can evolve
independently.

Why split placement vs clearing?
--------------------------------
• Placement cares only about *free space* (≤ 3 items).  
• Clearing cares only about *specific* dirty items (*plate + cup*).  
Two interchangeable strategies let you tune or replace each behaviour without
touching the other.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 30 25 58

   * - Direction
     - Topic
     - Message type
     - Notes
   * - **Required**
     - ``/{robot}/camera_detections``
     - ``std_msgs/String``
     - Example: ``"Detected: cup, plate"``
   * - **Required**
     - ``/{robot}/depth_processed``
     - ``sensor_msgs/Image``
     - 32-bit float depth (same resolution as RGB)
   * - **Provided**
     - ``/{robot}/object_positions``
     - ``std_msgs/String``
     - XYZ list – e.g. ``"cup @ [x,y,z]; plate @ [x,y,z]"``
   * - **Provided**
     - ``/{robot}/placement_decision``
     - ``std_msgs/String``
     - ``"Decision: PLACE, CLEAR"`` (keywords from both strategies)

Contract
--------
**Pre-conditions**  

• Depth topic and detection topic share the same resolution and optical frame.  

**Post-conditions**

• For every detection batch exactly **one** positions message and **one**
  decision string are published.  
• Each XYZ entry is based on the *centre pixel* (placeholder) until real
  bounding-box projection is implemented.

**Invariants**

• Publication latency (both topics present → decision publish) < 40 ms.  
• Strategy keywords are always one of: *PLACE, FULL, CLEAR, IGNORE*.

**Protocol**
  
1. Cache the latest depth and detection messages.  
2. When both are available, fuse → publish → reset detection cache.  
   (Depth can arrive faster; each batch of detections is used once.)

Assumptions & Limitations
-------------------------
* One RGB-D camera with pinhole intrinsics (fx, fy, cx, cy hard-coded).  
* Only the *centre pixel* depth is sampled – replace with bounding-box → 3-D
  projection for production.  
* Detections arrive as a simple string; switch to
  ``vision_msgs/Detection2DArray`` when a real detector is online.

"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from abc import ABC, abstractmethod
import sys


# --------------------------------------------------------------------------- #
#                           STRATEGY PATTERN                                  #
# --------------------------------------------------------------------------- #
class TableAnalysisStrategy(ABC):
    """
    Abstract base for table-state reasoning.

    ``analyze(objects: list[str]) -> str`` must return **one** of

    * ``"PLACE"`` – safe to put another plate  
    * ``"FULL"``  – table crowded; do nothing  
    * ``"CLEAR"`` – fetch dirty items  
    * ``"IGNORE"`` – no action needed
    """
    @abstractmethod
    def analyze(self, objects):  # pragma: no cover
        raise NotImplementedError


class FindPlacementStrategy(TableAnalysisStrategy):
    """Return *PLACE* when < 3 objects present, else *FULL*."""
    def analyze(self, objects):
        return "PLACE" if len(objects) < 3 else "FULL"


class ClearingStrategy(TableAnalysisStrategy):
    """Return *CLEAR* if both ``plate`` and ``cup`` present, else *IGNORE*."""
    def analyze(self, objects):
        return "CLEAR" if {"plate", "cup"} <= set(objects) else "IGNORE"


# --------------------------------------------------------------------------- #
#                               MAIN NODE                                     #
# --------------------------------------------------------------------------- #
class DistanceEstimator:
    """
    Fuse detections + depth into XYZ positions and table-state keywords.

    Internal State
    --------------
    latest_detections : str or None
        Raw detection string awaiting fusion.
    latest_depth : ndarray or None
        Most recent depth frame (32-bit float).
    """

    # ------------------------- initialisation ----------------------------- #
    def __init__(self):
        self.robot_number = sys.argv[1]              # namespace for multi-robot
        rospy.init_node(f"{self.robot_number}_distance_estimator_node")

        # -- ROS wiring ---------------------------------------------------- #
        self.detection_sub = rospy.Subscriber(
            f"/{self.robot_number}/camera_detections",
            String,
            self.detection_callback,
        )
        self.depth_sub = rospy.Subscriber(
            f"/{self.robot_number}/depth_processed",
            Image,
            self.depth_callback,
        )

        self.position_pub = rospy.Publisher(
            f"/{self.robot_number}/object_positions", String, queue_size=10)
        self.reasoning_pub = rospy.Publisher(
            f"/{self.robot_number}/placement_decision", String, queue_size=10)

        # (legacy non-namespaced publishers – remove when all consumers migrate)
        self.position_pub = rospy.Publisher("/object_positions", String, queue_size=10)
        self.reasoning_pub = rospy.Publisher("/placement_decision", String, queue_size=10)

        # -- state & helpers ----------------------------------------------- #
        self.latest_detections = None
        self.latest_depth = None
        self.bridge = CvBridge()

        # -- strategy objects ---------------------------------------------- #
        self.placement_strategy = FindPlacementStrategy()
        self.clearing_strategy = ClearingStrategy()

    # ------------------------- subscribers -------------------------------- #
    def detection_callback(self, msg):
        """Store detection string then attempt fusion."""
        self.latest_detections = msg.data
        self.try_estimate_positions()

    def depth_callback(self, msg):
        """Convert depth image → ndarray then attempt fusion."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            self.try_estimate_positions()
        except Exception as exc:
            rospy.logerr(f"[distance_estimator] depth conversion failed: {exc}")

    # ------------------------- core fusion -------------------------------- #
    def try_estimate_positions(self):
        """
        Run localisation + reasoning once both depth and detections exist.

        Steps
        -----
        1. Parse detection string → list of class names.  
        2. For each name, sample the *centre pixel* depth (placeholder) and
           back-project to XYZ.  
        3. Publish formatted XYZ list.  
        4. Run placement + clearing strategies, publish decision string.  
        5. Clear detections cache so each batch is processed exactly once.
        """
        if self.latest_detections is None or self.latest_depth is None:
            return

        objects = self.parse_detections(self.latest_detections)

        # Placeholder: one XYZ for all objects (centre pixel)
        u, v = 320, 240
        depth_val = float(self.latest_depth[v, u])
        x, y, z = self.project_to_robot_frame(u, v, depth_val)

        xyz_entries = [f"{obj} @ [{x:.2f}, {y:.2f}, {z:.2f}]" for obj in objects]
        self.position_pub.publish(String(data="; ".join(xyz_entries)))

        place_kw = self.placement_strategy.analyze(objects)
        clear_kw = self.clearing_strategy.analyze(objects)
        self.reasoning_pub.publish(String(data=f"Decision: {place_kw}, {clear_kw}"))

        # Reset so new depth won’t reuse stale detections
        self.latest_detections = None

    # ------------------------- helper funcs ------------------------------- #
    @staticmethod
    def parse_detections(raw):
        """
        Convert ``"Detected: cup, plate"`` → ``['cup', 'plate']``.

        Any parsing error returns an empty list so the node keeps running.
        """
        try:
            return raw.replace("Detected:", "").strip().split(", ")
        except Exception as exc:
            rospy.logwarn(f"parse_detections failed: {exc}")
            return []

    @staticmethod
    def project_to_robot_frame(u, v, depth):
        """
        Back-project pixel (u, v, depth) → metric XYZ in camera frame.

        Hard-coded intrinsics (fx, fy, cx, cy) assume a 640×480 pinhole model.

        Returns
        -------
        tuple[float, float, float]
            Coordinates (x, y, z) in metres.
        """
        fx = fy = 525.0
        cx, cy = 320, 240
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z


# --------------------------------------------------------------------------- #
#                                    MAIN                                     #
# --------------------------------------------------------------------------- #
def main():
    """Register node and spin until shutdown."""
    try:
        DistanceEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()