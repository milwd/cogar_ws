#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def publish():
    rospy.init_node('camera')
    pub_rgb = rospy.Publisher('/camera', Image, queue_size=10)
    pub_depth = rospy.Publisher('/depth', Image, queue_size=10)
    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
        original = cv2.imread('/root/Desktop/cogar_ass1/cogar_ws/src/tiago1/scripts/image.jpg')
        bridge = CvBridge()
        image = bridge.cv2_to_imgmsg(original, encoding="bgr8")
        image.header.stamp = rospy.Time.now()
        image.header.frame_id = 'camera_frame'
        pub_rgb.publish(image)
        depth_image = bridge.cv2_to_imgmsg(np.ones((original.shape[0], original.shape[1]), np.int8), encoding="8SC1")
        depth_image.header.stamp = rospy.Time.now()
        depth_image.header.frame_id = 'depth_frame'
        pub_depth.publish(depth_image)
        rate.sleep()


if __name__ == "__main__":    
    try:
        publish()
    except rospy.ROSInterruptException:
        pass

