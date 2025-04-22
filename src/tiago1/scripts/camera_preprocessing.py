#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPreprocessing:
    def __init__(self):
        rospy.init_node('camera_preprocessing_node')
        self.bridge = CvBridge()

        self.rgb_sub = rospy.Subscriber("/camera", Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber("/depth", Image, self.depth_callback)

        self.rgb_pub = rospy.Publisher("/camera_processed", Image, queue_size=10)
        self.depth_pub = rospy.Publisher("/depth_processed", Image, queue_size=10)

    def rgb_callback(self, msg):
        try:
            # RGB preprocessing
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            processed = cv2.GaussianBlur(cv_image, (5, 5), 0) 
            ros_image = self.bridge.cv2_to_imgmsg(processed, encoding="bgr8")
            ros_image.header = msg.header 
            self.rgb_pub.publish(ros_image)
        except Exception as e:
            rospy.logerr(f"RGB processing failed: {e}")

    def depth_callback(self, msg):
        try:
            # depth preprocessing
            cv_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            processed = cv_depth 
            ros_depth = self.bridge.cv2_to_imgmsg(processed, encoding="8SC1")
            ros_depth.header = msg.header
            self.depth_pub.publish(ros_depth)
        except Exception as e:
            rospy.logerr(f"Depth processing failed: {e}")

if __name__ == '__main__':
    try:
        CameraPreprocessing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# This script subscribes to RGB and depth camera images, processes them, and publishes the filtered images.
# The processing logic is currently a placeholder and should be replaced with actual image processing code.
# The script uses ROS to handle the camera data and publish the results.
# The script is designed to run as a ROS node and will keep running until it is interrupted.
# The script uses rospy to handle the ROS communication and threading.
# The script uses the Image message type from the sensor_msgs package to handle the camera data.

