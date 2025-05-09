#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import sys


class PathPlanner:
    def __init__(self):
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_path_planning_node')

        self.map = None
        self.odom = None

        rospy.Subscriber(f'/{self.robot_number}/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber(f'/{self.robot_number}/odom_slam', Odometry, self.odom_callback)

        self.path_pub = rospy.Publisher(f'/{self.robot_number}/planned_path', Path, queue_size=1)

    def map_callback(self, msg):
        self.map = msg

    def odom_callback(self, msg):
        self.odom = msg

    def plan_path(self):
        if self.map and self.odom:
            path = Path()
            pose = PoseStamped()
            pose.pose.position.x = 1.0  # dummy goal
            pose.pose.position.y = 1.0
            path.poses.append(pose)

            rospy.loginfo("[PathPlanner] Publishing path to reasoning_action.")
            self.path_pub.publish(path)

    def loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.plan_path()
            rate.sleep()

if __name__ == '__main__':
    try:
        pp = PathPlanner()
        pp.loop()
    except rospy.ROSInterruptException:
        pass
