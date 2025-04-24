#!/usr/bin/env python
import rospy
from std_msgs.msg import String

class ReasoningTablePlacement:
    def __init__(self):
        rospy.init_node("reasoning_table_placement_node")

        self.decision_sub = rospy.Subscriber("/placement_decision", String, self.decision_callback)
        self.command_pub = rospy.Publisher("/table_reasoning_commands", String, queue_size=10)

        rospy.loginfo("[ReasoningTablePlacement] Node started.")

    def decision_callback(self, msg):
        decision = msg.data
        rospy.loginfo(f"[ReasoningTablePlacement] Received decision: {decision}")

        if "PLACE" in decision:
            command = "PLACE_DISH"
        elif "CLEAR" in decision:
            command = "CLEAR_TABLE"
        else:
            command = "NO_ACTION"

        self.command_pub.publish(String(data=command))
        rospy.loginfo(f"[ReasoningTablePlacement] Sent command: {command}")

def main():
    try:
        ReasoningTablePlacement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
