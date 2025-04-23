#!/usr/bin/env python3
import rospy
import actionlib
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tiago1.msg import MovementControlAction, MovementControlGoal


class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planning_node')

        self.map = None
        self.odom = None

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
<<<<<<< HEAD
        # rospy.Timer(rospy.Duration(5), self.plan_path)
=======

        self.client = actionlib.SimpleActionClient('movement_control', MovementControlAction)
        rospy.loginfo("[PathPlanner] Waiting for movement_control action server...")
        self.client.wait_for_server()
        rospy.loginfo("[PathPlanner] Connected to movement_control action server.")

        self.goal_sent = False
>>>>>>> 12be1fcab963b329af0649c4075d9d1835a9ee03

    def map_callback(self, msg):
        self.map = msg

    def odom_callback(self, msg):
        self.odom = msg

    def plan_path(self):
        if self.map and self.odom and not self.goal_sent:
            path = Path()
            pose = PoseStamped()
            pose.pose.position.x = 1.0  # dummy work
            pose.pose.position.y = 1.0
            path.poses.append(pose)

            self.path_pub.publish(path)
            goal = MovementControlGoal()
            goal.path = path

            rospy.loginfo("[PathPlanner] Sending goal to movement_control...")
            self.client.send_goal(goal, feedback_cb=self.feedback_cb)
            self.goal_sent = True

    def feedback_cb(self, feedback):
        rospy.loginfo(f"[PathPlanner] Feedback: {feedback.status}")

    def loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.plan_path()

            if self.goal_sent:
                state = self.client.get_state()
                if state in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                    rospy.loginfo(f"[PathPlanner] Movement goal completed with state: {state}")
                    self.goal_sent = False

            rate.sleep()



if __name__ == '__main__':
    try:
        pp = PathPlanner()
        pp.loop()
    except rospy.ROSInterruptException:
        pass

