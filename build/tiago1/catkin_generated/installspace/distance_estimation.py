#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2
from abc import ABC, abstractmethod
import sys


class TableAnalysisStrategy(ABC):
    @abstractmethod
    def analyze(self, objects):
        pass


class FindPlacementStrategy(TableAnalysisStrategy):
    def analyze(self, objects):
        if len(objects) < 3:
            return "PLACE"
        return "FULL"


class ClearingStrategy(TableAnalysisStrategy):
    def analyze(self, objects):
        if "plate" in objects and "cup" in objects:
            return "CLEAR"
        return "IGNORE"


class DistanceEstimator:
    def __init__(self):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f"{self.robot_number}_distance_estimator_node")

        self.detection_sub = rospy.Subscriber(f"/{self.robot_number}/camera_detections", String, self.detection_callback)
        self.depth_sub = rospy.Subscriber(f"/{self.robot_number}/depth_processed", Image, self.depth_callback)
        self.position_pub = rospy.Publisher(f"/{self.robot_number}/object_positions", String, queue_size=10)
        self.reasoning_pub = rospy.Publisher(f"/{self.robot_number}/placement_decision", String, queue_size=10)

        self.latest_detections = None
        self.latest_depth = None
        self.bridge = CvBridge()

        self.placement_strategy = FindPlacementStrategy()
        self.clearing_strategy = ClearingStrategy()

    def detection_callback(self, msg):
        self.latest_detections = msg.data
        self.try_estimate_positions()

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.try_estimate_positions()
        except Exception as e:
            rospy.logerr(f"Depth image conversion failed: {e}")

    def try_estimate_positions(self):
        if self.latest_detections is None or self.latest_depth is None:
            return

        objects = self.parse_detections(self.latest_detections)
        result = []

        for obj_name in objects:
            u, v = 320, 240
            depth = self.latest_depth[v, u]
            x, y, z = self.project_to_robot_frame(u, v, depth)
            result.append(f"{obj_name} @ [{x:.2f}, {y:.2f}, {z:.2f}]")

        msg_out = String(data="; ".join(result))
        self.position_pub.publish(msg_out)
        rospy.loginfo("Published object positions.")

        placement_decision = self.placement_strategy.analyze(objects)
        clearing_decision = self.clearing_strategy.analyze(objects)

        decision = f"Decision: {placement_decision}, {clearing_decision}"
        self.reasoning_pub.publish(String(data=decision))
        rospy.loginfo(f"Published decision to reasoning_action: {decision}")

        self.latest_detections = None

    def parse_detections(self, data):
        try:
            return data.replace("Detected:", "").strip().split(", ")
        except Exception as e:
            rospy.logwarn(f"Failed to parse detections: {e}")
            return []

    def project_to_robot_frame(self, u, v, depth):
        fx, fy = 525.0, 525.0
        cx, cy = 320, 240
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z


def main():
    try:
        estimator = DistanceEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()

