=========================
Subsystem Control
=========================

What the Control subsystem does
--------------------------------
The Control layer is responsible for turning high-level goals from the Brain into precise actuator commands and streaming sensor feedback for closed-loop correction.  Its workflow is:

1. **Base motion**  
   **Control Movement** receives a `nav_msgs/Path` goal, applies a proportional controller to drive the base toward the first waypoint, and reports success or failure via ActionLib.

2. **Arm positioning**  
   **Control Arm** listens for `ArmControlAction` goals, runs a 10 Hz proportional joint‐space controller until the target angle is reached (or times out/preempts), and publishes zero velocity on completion.

3. **Gripper actuation**  
   **Control Gripper** exposes a binary open/close ActionLib server that publishes a `std_msgs/Bool` command and waits a fixed delay for mechanical settling.

4. **Feedback streams**  
   **Encoder & Force** nodes continuously publish wheel ticks, joint angles and end-effector force so that controllers and higher layers can run in simulation or CI without real hardware.

.. image:: images/control_subsystem.png
   :alt: Component diagram of the Control subsystem
   :align: center
   :width: 90%

Design Patterns
---------------

Command
~~~~~~~
Each controller—**Wheel**, **Arm** and **Gripper**—exposes an ActionLib server where goals are commands (`MovementControlGoal`, `ArmControlGoal`, `GripperControlGoal`).  This allows the Brain to queue, pre-empt or retry actions uniformly, without needing to know the details of the underlying motion or actuator interface.

Observer
~~~~~~~~  
All controllers and the Force sensor subscribe to encoder and force topics; as soon as new sensor data arrives, control loops use that feedback immediately for error computation or state reporting.  This push-based design avoids polling and ensures **tight closed-loop latency**.

Template Method
~~~~~~~~~~~~~~~
All ActionLib servers share a common execution flow—initialize, loop until goal reached/pre-empted/timeout, publish feedback, set result—while each controller overrides just the core `compute_command()` or `handle_completion()` hooks.  This enforces a consistent server structure and simplifies adding new controllers.

Singleton
~~~~~~~~~
Each controller node uses a single ROS node handle and shared publisher/subscriber objects; by treating the ROS handle as a singleton, we can avoid accidental re-initialization and ensure **consistent QoS and parameters** across callbacks.

Component roles
---------------
- **control_wheel.py**  
  - Proportional base controller (`MovementControlAction`) that steers toward the first waypoint, publishes feedback and sets result on completion.

- **control_arm.py**  
  - ActionLib server (`ArmControlAction`) implementing a 10 Hz proportional joint controller with configurable tolerance and timeout.  Publishes `geometry_msgs/Twist` on `/cmd_vel/arm`.

- **control_gripper.py**  
  - Binary actuator server (`GripperControlAction`) that sends a `std_msgs/Bool` grip/release command and confirms completion after a delay.

- **encoder_wheel.py**, **encoder_arm.py**, **encoder_gripper.py**  
  - Synthetic encoders publishing `std_msgs/Int32` at 10 Hz for base, arm and gripper joints.

- **force.py**  
  - Fake force sensor publishing `std_msgs/Float32` at 10 Hz for end-effector contact force.


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
