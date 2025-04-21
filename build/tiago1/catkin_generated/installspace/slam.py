#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

class SLAM:
    def __init__(self):
        rospy.init_node('slam_node')
        rospy.Subscriber('/fused_scan', LaserScan, self.callback)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)

    def callback(self, data):
        map_msg = OccupancyGrid()
        odom_msg = Odometry()
        self.map_pub.publish(map_msg)
        self.odom_pub.publish(odom_msg)

if __name__ == '__main__':
    SLAM()
    rospy.spin()
