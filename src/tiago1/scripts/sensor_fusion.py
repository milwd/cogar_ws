#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, Range
import sys


class SensorFusionNode:
    def __init__(self):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_sensor_fusion_node')
        self.lidar_sub = rospy.Subscriber(f'/{self.robot_number}/lidar', LaserScan, self.lidar_callback)
        self.sonar_sub = rospy.Subscriber(f'/{self.robot_number}/sonar', Range, self.sonar_callback)
        self.pub = rospy.Publisher(f'/{self.robot_number}/fused_scan', LaserScan, queue_size=10)
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
# It subscribes to both LIDAR and sonar data, processes the data,
# and publishes the fused data to a new topic.
# The `merge_data` method combines the LIDAR and sonar data,
# while the `publish_fused` method publishes the fused data.
# The `lidar_callback` and `sonar_callback` methods handle incoming data
# from the respective sensors and trigger the fusion process.
# The `Adapter` class is used to adapt the sonar data to fit into the LIDAR data format.
# The `publish_fused` method checks if both LIDAR and sonar data are available
# before proceeding with the fusion and publishing process.
# The `merge_data` method combines the LIDAR and sonar data by adding
# the sonar data to the LIDAR data at each corresponding index.

class Adapter:
    def __call__(self, sonar_data, length=0):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        new_data = [0] * length
        new_data[length // 2] = sonar_data
        return new_data
# This adapter takes sonar data and creates a new list of zeros with the length specified.
# It places the sonar range value in the middle of the list, effectively
# adapting the sonar data to fit into a format that can be used with the LIDAR data.
        

if __name__ == '__main__':
    fused = SensorFusionNode()
    while not rospy.is_shutdown():
        fused.publish_fused()
        rospy.Rate(10).sleep()

