#!/usr/bin/env python
"""
speech_generator.py
===================

Text-to-Speech **relay node** for restaurant announcements
----------------------------------------------------------

The cognitive layer of the sushi-waiter architecture thinks in sentences:
*“Table 3, your Dragon Roll is here”*, *“Apologies for the delay”*, *“Order
cancelled, returning to station”*.  Something has to turn those strings into
audible sound.  **speech_generator.py** fills that gap while staying totally
agnostic about *how* the audio is produced.

Design philosophy
-----------------
* **Decoupling** – upstream reasoning nodes push text to a *single* topic
  (``/task_speech_command``).  Downstream, you may connect *any* TTS driver
  (Festival, `sound_play`, cloud API, proprietary speaker), as long as it
  listens to ``/speaker_channel``.  
* **State buffering** – the latest line is cached so that late-joining
  subscribers (e.g. the speaker driver restarting) still receive the current
  sentence.  
* **Rate control** – messages are re-emitted at **1 Hz** to avoid flooding the
  audio backend while giving it enough chances to catch up.

ROS topics
~~~~~~~~~~
.. list-table:: Node interface
   :header-rows: 1
   :widths: 15 25 60

   * - Direction
     - Topic
     - Type / semantics
   * - **subscribe**
     - ``/task_speech_command``
     - ``std_msgs/String`` – UTF-8 input from the cognitive layer
   * - **publish**
     - ``/speaker_channel``
     - ``std_msgs/String`` – identical text, forwarded once per second


"""

import rospy
from std_msgs.msg import String
import sys



class SpeechGenerator:
    """
    Thin, timer-driven **republisher**.

    Workflow
    --------
    1. A subscriber callback stores the newest incoming sentence
       in :pyattr:`self.msg`.
    2. The main loop publishes that stored message once every second.
       If no message has been received yet, it publishes nothing,
       thereby avoiding empty utterances.

    The class holds no additional state – perfect for hot-reloading or
    replacement in unit tests.
    """

    def __init__(self):
        """
        Initialize the SpeechGenerator node:

        - Initialize ROS node 'speech_gen_node'.
        - Subscribe to '/task_speech_command' for speech commands.
        - Advertise '/speaker_channel' for speaking output.
        """
        self.robot_number = sys.argv[1]#rospy.get_param('~robot_number')
        rospy.init_node(f'{self.robot_number}_speech_gen_node')
        self.msg = None
        rospy.Subscriber(f'/{self.robot_number}/task_speech_command', String, self.speaker_callback)
        self.pub_speak = rospy.Publisher(f'/{self.robot_number}/speaker_channel', String, queue_size=10)
        
    def speaker_callback(self, msg):
        """
        Store the newest sentence.

        Parameters
        ----------
        msg : std_msgs.msg.String
            UTF-8 text coming from the cognitive layer.
        """
        self.msg = msg
        rospy.loginfo(f"[SpeechGenerator] Buffered: {msg.data!r}")

    # ------------------------------------------------------------------ #
    #                         periodic publisher                          #
    # ------------------------------------------------------------------ #
    def publish_msg(self) -> None:
        """
        Republish the buffered sentence (if any) at the current loop tick.
        """
        if self.msg is not None:
            self.pub_speak.publish(self.msg)


# ---------------------------------------------------------------------- #
#                               main loop                                #
# ---------------------------------------------------------------------- #
if __name__ == "__main__":
    """
    Spin until ROS is shut down by Ctrl-C, master failure, or ``rosnode kill``.
    Any non-ROS exception will propagate and show a traceback, which is useful
    during development.
    """
    try:
        bridge = SpeechGenerator()
        rate = rospy.Rate(1)  # one publish attempt per second
        while not rospy.is_shutdown():
            bridge.publish_msg()
            rate.sleep()
    except rospy.ROSInterruptException:
        # Expected when the node is stopped gracefully – no special handling.
        pass
