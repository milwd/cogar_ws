=========================
Subsystem Control
=========================

Mission
-------
The Control layer takes high-level goals from the Brain and produces
**low-level actuation commands** while streaming sensor feedback for closed-loop
correction.

* **Wheel controller** – drives the base along a velocity set-point.
* **Arm controller** – positions the 7-DoF arm in joint space.
* **Gripper controller** – binary open/close with feedback.
* **Encoders & Force** – publish joint ticks and contact force so all upper
  layers can run on desktop CI.

.. image:: images/control_subsystem.png
   :alt: Component diagram of the Control subsystem
   :align: center
   :width: 90%

ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 30 20 50

   * - Topic / Action / Service
     - Type
     - KPI / Note
   * - ``/cmd_vel/wheel`` ← *control_wheel.py*
     - ``geometry_msgs/Twist``
     - 10 Hz, max |v| 0.7 m/s, latency < 50 ms
   * - ``/cmd_vel/arm``  ← *control_arm.py*
     - ``geometry_msgs/Twist``
     - |ω| < 0.4 rad/s, settles within ± 5°
   * - ``/cmd_vel/gripper`` ← *control_gripper.py*
     - ``std_msgs/Bool``
     - Open/close < 2 s, confirmed feedback
   * - ``/encoder_wheel``  ← *encoder_wheel.py*
     - ``std_msgs/Int32``
     - 10 Hz, lost < 0.1 %
   * - ``/encoder_arm``    ← *encoder_arm.py*
     - ``std_msgs/Int32``
     - 10 Hz, jitter < ±10 ms
   * - ``/encoder_gripper`` ← *encoder_gripper.py*
     - ``std_msgs/Int32``
     - 10 Hz, range 0–100 mm
   * - ``/force`` ← *force.py*
     - ``std_msgs/Float32``
     - 0–50 N, noise < 0.5 N RMS
   * - ``arm_control`` action
     - ``tiago1/ArmControlAction``
     - Goal latency < 100 ms, success ≥ 99 %
   * - ``gripper_control`` action
     - ``tiago1/GripperControlAction``
     - Goal latency < 100 ms, success ≥ 99 %
   * - ``movement_control`` action
     - ``tiago1/MovementControlAction``
     - Path repro < 5 cm RMSE

Implementation modules
----------------------
Click a component for its full API documentation.

.. toctree::
   :maxdepth: 1
   :caption: Control Components

   control_modules/control_wheel
   control_modules/control_arm
   control_modules/control_gripper
   control_modules/encoder_wheel
   control_modules/encoder_arm
   control_modules/encoder_gripper
   control_modules/force
