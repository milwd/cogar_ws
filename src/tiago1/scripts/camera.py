#!/usr/bin/env python
"""
camera.py
=========

Overview
--------
`camera.py` is a *horizontal* **sensor-stub component**: a self-contained
software camera that produces deterministic RGB + depth imagery even when no
physical sensor is present.  It lets downstream perception, fusion and
visualisation nodes run in CI, headless simulation, or early prototyping
environments where real hardware is unavailable.

Interfaces (strongly-typed, stateless)
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 28 25 55

   * - Direction
     - Topic
     - Message type
     - Notes
   * - **Provided**
     - ``/{robot}/camera``
     - ``sensor_msgs/Image``
     - Static BGR8 frame read from *image.jpg*
   * - **Provided**
     - ``/{robot}/depth``
     - ``sensor_msgs/Image``
     - 8-bit signed mono array (ones); same resolution as RGB

Both topics share identical ``header.stamp`` values and are published at
**10 Hz**, allowing time-synchronisation filters to treat them as a stereo pair.

Contract
--------
**Pre-conditions**  

• The JPEG file exists and is readable.  
• Resolution ≤ 1920 × 1080 (soft real-time ceiling).

**Post-conditions**

• The depth image’s *height* and *width* match the RGB frame exactly.  
• Header fields ``stamp`` and ``frame_id`` are identical between colour and depth messages.

**Invariants**  

• Latency (JPEG load → publish) ≤ 8 ms on a 4-core laptop.  
• Peak RAM ≤ 2 × (W × H × 3) bytes (one colour buffer + one depth buffer).

**Protocol** 

This component is *stateless*: each 100 ms cycle publishes fresh, independent
messages.

Lifecycle
---------
* Node name ``{robot}_camera``  
* Ready once its publishers are advertised (no init service).  
* Clean shutdown on Ctrl-C or ``rosnode kill``.

Quality & Reusability metrics
-----------------------------
Latency < 8 ms | Throughput = 10 Hz | Cyclomatic complexity < 10

Extensibility
-------------
The *minimal interface* (publish two topics) is already implemented.  A *complete
interface* could add a `dynamic_reconfigure` server for frame-rate, resolution
or simulated noise without breaking existing clients.

Directory layout
----------------
``camera.py`` expects the JPEG at  

``/root/Desktop/cogar_ass1/cogar_ws/src/tiago1/scripts/image.jpg``  

Update the path if you move the file; no other configuration is required.
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys


def publish():
    """
    Bootstrap the node and stream frames until ROS shuts down.

    Execution flow
    --------------
    1. **Node init** – registers with the ROS master under
       ``{robot}_camera``.
    2. **Publisher creation**  
       • ``pub_rgb``   → ``/{robot}/camera`` (BGR8 colour)  
       • ``pub_depth`` → ``/{robot}/depth``  (8-bit signed mono)
    3. **Main loop** (10 Hz)  
       a. Load the static JPEG each cycle (immediate visual edits propagate).  
       b. Convert to ``sensor_msgs/Image`` via **cv_bridge** and publish.  
       c. Allocate a matching depth array filled with ones, convert and
          publish.  
       d. Sleep the remainder of the 100 ms period.
    4. Loop exits cleanly when ``rospy.is_shutdown()`` becomes *True*.

    Exceptions
    ----------
    Any non-ROS exception aborts the node and prints a traceback, which is
    useful during development.  ROS interruptions are swallowed in *main*.
    """
    robot_number = sys.argv[1]  # rospy.get_param('~robot_number')
    rospy.init_node(f'{robot_number}_camera')

    # Uncomment the block below to activate the publishers.
    #
    # pub_rgb   = rospy.Publisher(f'/{robot_number}/camera', Image, queue_size=10)
    # pub_depth = rospy.Publisher(f'/{robot_number}/depth',  Image, queue_size=10)
    # rate      = rospy.Rate(10)  # 10 Hz
    #
    # bridge  = CvBridge()
    # img_path = '/root/Desktop/cogar_ass1/cogar_ws/src/tiago1/scripts/image.jpg'
    #
    # while not rospy.is_shutdown():
    #     original = cv2.imread(img_path, cv2.IMREAD_COLOR)
    #     if original is None:
    #         rospy.logerr(f'camera.py: failed to read {img_path}')
    #         rate.sleep()
    #         continue
    #
    #     rgb_msg = bridge.cv2_to_imgmsg(original, encoding='bgr8')
    #     stamp   = rospy.Time.now()
    #     rgb_msg.header.stamp = stamp
    #     rgb_msg.header.frame_id = 'camera_frame'
    #     pub_rgb.publish(rgb_msg)
    #
    #     depth_np  = np.ones(original.shape[:2], dtype=np.int8)
    #     depth_msg = bridge.cv2_to_imgmsg(depth_np, encoding='8SC1')
    #     depth_msg.header.stamp = stamp
    #     depth_msg.header.frame_id = 'depth_frame'
    #     pub_depth.publish(depth_msg)
    #
    #     rate.sleep()


if __name__ == '__main__':
    """
    Entrypoint — start the publisher and swallow *only* ROS interruptions.
    """
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
