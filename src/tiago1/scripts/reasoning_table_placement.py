#!/usr/bin/env python
"""
reasoning_table_placement.py

ROS node that listens to placement decisions and publishes table
commands accordingly. Converts high-level decisions ("PLACE", "CLEAR")
into low-level commands ("PLACE_DISH", "CLEAR_TABLE") for execution.
"""

import rospy
from std_msgs.msg import String
import sys


class ReasoningTablePlacement:
    """
    Subscribes to placement decisions and publishes table placement commands.

    Attributes
    ----------
    decision_sub : rospy.Subscriber
        Subscriber to '/placement_decision' for receiving decisions.
    command_pub : rospy.Publisher
        Publisher to '/table_reasoning_commands' for sending commands.
    """

    def __init__(self):
        """
        Initialize the reasoning_table_placement node.

        - Initialize ROS node 'reasoning_table_placement_node'.
        - Subscribe to '/placement_decision' to receive decisions.
        - Advertise '/table_reasoning_commands' to send commands.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f"{self.robot_number}_reasoning_table_placement_node")

        self.decision_sub = rospy.Subscriber(f"/{self.robot_number}/placement_decision", String, self.decision_callback)
        self.command_pub = rospy.Publisher(f"/{self.robot_number}/table_reasoning_commands", String, queue_size=10)

        rospy.loginfo("[ReasoningTablePlacement] Node started.")

    def decision_callback(self, msg):
        """
        Process incoming placement decisions and publish corresponding commands.

        Parameters
        ----------
        msg : std_msgs.msg.String
            The incoming decision message containing decision keywords.

        Process
        -------
        - Parse msg.data for keywords "PLACE" or "CLEAR".
        - Map decisions to commands: "PLACE_DISH", "CLEAR_TABLE", or "NO_ACTION".
        - Publish the appropriate command on '/table_reasoning_commands'.
        """
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
    """
    Main entry point: instantiate ReasoningTablePlacement and spin ROS.
    """
    try:
        ReasoningTablePlacement()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
