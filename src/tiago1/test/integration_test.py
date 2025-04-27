import subprocess
import time
import unittest

class TestIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        print("[INFO] Waiting 10 seconds for system to start...")
        time.sleep(10)
        print("[INFO] Collecting initial rosnode list...")
        cls.nodes = subprocess.check_output(["rosnode", "list"]).decode().split()

    def test_node_count(self):
        expected_node_count = 109 
        print(f"[TEST] Expected nodes: {expected_node_count}, Found nodes: {len(self.nodes)}")
        self.assertEqual(len(self.nodes), expected_node_count)

    def test_all_nodes_respond(self):
        print("[TEST] Checking if all nodes respond...")
        for node in self.nodes:
            try:
                subprocess.check_output(["rosnode", "ping", "-c", "1", node])
                print(f"[OK] Node {node} responds.")
            except subprocess.CalledProcessError:
                self.fail(f"Node {node} did not respond to ping!")

    @classmethod
    def tearDownClass(cls):
        print("[INFO] Integration tests complete.")

if __name__ == '__main__':
    unittest.main()
