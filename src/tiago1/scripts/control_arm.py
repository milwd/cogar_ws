#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Int32


class ControlArm:
    def __init__(self):
        self.pub = rospy.Publisher('/arm_motor', Int32, queue_size=10)
        rospy.Subscriber('/arm_cmd', Float32, self.cmd_callback)
        self.SCALE = 10  # fake scale: 1 cm = 10 ticks

    def cmd_callback(self, msg):
        target_x = msg.data
        motor_ticks = int(target_x * self.SCALE)
        rospy.loginfo(f"[ControlArm] Received x={target_x:.2f} cm -> Motor = {motor_ticks} ticks")
        self.pub.publish(Int32(motor_ticks))


if __name__ == '__main__':
    rospy.init_node('control_arm_node')
    ControlArm()
    rospy.spin()
