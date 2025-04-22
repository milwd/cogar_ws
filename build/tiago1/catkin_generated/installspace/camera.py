#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def publish():
    rospy.init_node('camera')
    pub = rospy.Publisher('/camera', Image, queue_size=10)
    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
        bridge = CvBridge()
        image = bridge.cv2_to_imgmsg(cv2.imread('/root/cogar_ws/src/tiago1/scripts/image.jpg'), encoding="bgr8")
        image.header.stamp = rospy.Time.now()
        image.header.frame_id = 'camera_frame'
        pub.publish(image)
        rate.sleep()


if __name__ == "__main__":    
    try:
        publish()
    except rospy.ROSInterruptException:
        pass

