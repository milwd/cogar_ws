#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

class SLAM:
    def __init__(self):
        rospy.init_node('slam_node')
        rospy.Subscriber('/fused_scan', LaserScan, self.callback)
        rospy.Subscriber('/odom_proc', Odometry, self.callback_odom)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom_slam', Odometry, queue_size=1)

    def callback_odom(self, data):
        self.odom_pub.publish(data)

    def callback(self, data):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = rospy.Time.now()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = 0.05
        map_msg.info.width = 10
        map_msg.info.height = 10
        map_msg.info.origin.position.x = -2.5
        map_msg.info.origin.position.y = -2.5
        map_msg.info.origin.position.z = 0
        map_msg.info.origin.orientation.x = 0
        map_msg.info.origin.orientation.y = 0
        map_msg.info.origin.orientation.z = 0
        map_msg.info.origin.orientation.w = 1
        map_msg.data = [0] * (map_msg.info.width * map_msg.info.height)
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = 0
        odom_msg.pose.pose.position.y = 0
        odom_msg.pose.pose.position.z = 0
        odom_msg.pose.pose.orientation.x = 0
        odom_msg.pose.pose.orientation.y = 0
        odom_msg.pose.pose.orientation.z = 0
        odom_msg.pose.pose.orientation.w = 1
        odom_msg.twist.twist.linear.x = 0
        odom_msg.twist.twist.linear.y = 0   
        odom_msg.twist.twist.linear.z = 0
        odom_msg.twist.twist.angular.x = 0
        odom_msg.twist.twist.angular.y = 0
        odom_msg.twist.twist.angular.z = 0
        # In a real implementation, one would use the data to update the map and odometry
        # Here we just publish the empty map and odometry messages
        # Would replace this with your SLAM algorithm
        # and update the map_msg.data and odom_msg with actual values:
        # map_msg.data = self.update_map(data)
        # odom_msg.pose.pose.position.x = self.update_position(data)
        # odom_msg.pose.pose.orientation.z = self.update_orientation(data)

        self.map_pub.publish(map_msg)

if __name__ == '__main__':
    slam = SLAM()
    try:
        while not rospy.is_shutdown():
            slam.callback(2)
            rospy.Rate(1).sleep() 
    except rospy.ROSInterruptException:
        pass
