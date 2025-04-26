#!/usr/bin/env python3
import unittest, rospy, rostest, rosnode
from std_msgs.msg import Int32
from nav_msgs.msg import Path

class SmokeTest(unittest.TestCase):
    NODE_NAMES = {
        '/camera', '/camera_preproc', '/object_detection',
        '/distance_estimation', '/sensor_fusion',
        '/slam', '/path_planning',
        '/reasoning_action', '/task_manager',
        '/control_arm', '/control_gripper', '/control_wheel',
        '/encoder_arm', '/encoder_gripper', '/encoder_wheel'
    }

    def setUp(self):
        rospy.init_node('smoke_test', anonymous=True)

        self.enc_seen  = False
        self.path_seen = False

        rospy.Subscriber('/encoder_arm', Int32,
                         lambda _: setattr(self, 'enc_seen', True))
        rospy.Subscriber('/path', Path,
                         lambda _: setattr(self, 'path_seen', True))

    # ---------------------------------------------------------------------
    def test_full_pipeline(self):
        t_end = rospy.Time.now() + rospy.Duration(20.0)
        missing = self.NODE_NAMES.copy()
        while missing and rospy.Time.now() < t_end and not rospy.is_shutdown():
            missing = {n for n in self.NODE_NAMES if n not in rosnode.get_node_names()}
            rospy.sleep(0.2)
        self.assertFalse(missing, f'nodes missing after 20 s: {sorted(missing)}')
        self._wait_true(lambda: self.enc_seen, 5.0, 'no /encoder_arm data within 5 s')
        self._wait_true(lambda: self.path_seen, 10.0, 'no /path messages within 10 s')

    # ---------------------------------------------------------------------
    def _wait_true(self, predicate, timeout, msg):
        t_end = rospy.Time.now() + rospy.Duration(timeout)
        while not predicate() and rospy.Time.now() < t_end and not rospy.is_shutdown():
            rospy.sleep(0.05)
        self.assertTrue(predicate(), msg)

if __name__ == '__main__':
    rostest.rosrun('tiago1', 'pipeline_smoke', SmokeTest)
