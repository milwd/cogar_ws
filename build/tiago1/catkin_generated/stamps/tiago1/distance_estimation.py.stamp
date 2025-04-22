#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

class DistanceEstimator:
    def __init__(self):
        rospy.init_node("distance_estimator_node")

        # Subscribers
        self.detection_sub = rospy.Subscriber("/camera_detections", String, self.detection_callback)
        self.depth_sub = rospy.Subscriber("/depth_processed", Image, self.depth_callback)

        # Publisher
        self.position_pub = rospy.Publisher("/object_positions", String, queue_size=10)

        # State
        self.latest_detections = None
        self.latest_depth = None
        self.bridge = CvBridge()

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
            return  # Wait until both are available

        objects = self.parse_detections(self.latest_detections)
        result = []
        for obj_name in objects:
            # Dummy location estimation logic
            u, v = 320, 240  # center of image
            depth = self.latest_depth[v, u]  # depth at center
            x, y, z = self.project_to_robot_frame(u, v, depth)
            result.append(f"{obj_name} @ [{x:.2f}, {y:.2f}, {z:.2f}]")

        # Publish result
        msg_out = String(data="; ".join(result))
        self.position_pub.publish(msg_out)
        rospy.loginfo("Published object positions.")

        # Clear only detection, keep depth (simulate streaming camera)
        self.latest_detections = None

    def parse_detections(self, data):
        try:
            return data.replace("Detected:", "").strip().split(", ")
        except Exception as e:
            rospy.logwarn(f"Failed to parse detections: {e}")
            return []

    def project_to_robot_frame(self, u, v, depth):
        # Simplified pinhole projection (mock)
        fx, fy = 525.0, 525.0  # focal length
        cx, cy = 320, 240  # optical center
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

if __name__ == "__main__":
    try:
        DistanceEstimator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
