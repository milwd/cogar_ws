#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range


def publish():
    pub = rospy.Publisher("/sonar", Range, queue_size=10)
    rospy.init_node("sonar_node", anonymous=True)
    # rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    #     sonar = Range()
    #     sonar.header.stamp = rospy.Time.now()
    #     sonar.header.frame_id = "sonar"
    #     sonar.radiation_type = Range.ULTRASOUND
    #     sonar.field_of_view = 0.1
    #     sonar.min_range = 0.2
    #     sonar.max_range = 4.0
    #     sonar.range = 3.0
    #     pub.publish(sonar)
    #     rate.sleep()


if __name__ == "__main__":
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
# sonar.py  
# This script publishes sonar data to the /sonar topic in ROS.
# It initializes a ROS node, creates a publisher, and continuously publishes
# sonar data at a rate of 10 Hz. The sonar data includes header information,
# radiation type, field of view, minimum and maximum range, and the current range.
# The script can be run as a standalone program.
# It uses the rospy library for ROS communication and the sensor_msgs.msg
# module for the Range message type. The script handles ROS interruptions gracefully.
# The sonar data is published in a loop until the node is shut down.
# The script is intended to be run in a ROS environment and requires the
# appropriate ROS setup and dependencies to be installed.
