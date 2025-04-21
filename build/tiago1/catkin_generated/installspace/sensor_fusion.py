#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan, Range


class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_fusion_node')
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)
        self.sonar_sub = rospy.Subscriber('/sonar', Range, self.sonar_callback)
        self.pub = rospy.Publisher('/fused_scan', LaserScan, queue_size=10)
        self.lidar_data = None
        self.sonar_data = None

    def lidar_callback(self, data):
        self.lidar_data = data

    def sonar_callback(self, data):
        self.sonar_data = data
        self.publish_fused()
    
    def merge_data(self):
        return [self.lidar_data.ranges[i] + self.adapted_sonar[i] for i in range(len(self.lidar_data.ranges))]

    def publish_fused(self):
        if self.lidar_data and self.sonar_data:
            fused = LaserScan()
            self.adapted_sonar = Adapter()(self.sonar_data.range, len(self.lidar_data.ranges))
            fused.ranges = self.merge_data()
            self.pub.publish(fused)
# This class is responsible for the sensor fusion process.

class Adapter:
    def __call__(self, sonar_data, length=0):
        new_data = [0] * length
        new_data[length // 2] = sonar_data.range
        return new_data
# This adapter takes sonar data and creates a new list of zeros with the length specified.
# It places the sonar range value in the middle of the list, effectively
# adapting the sonar data to fit into a format that can be used with the LIDAR data.
        

if __name__ == '__main__':
    SensorFusionNode()
    rospy.spin()
