==============================
Subsystem Brain
==============================

What the “Brain” does
---------------------
This layer converts *intent* (orders, table context, fleet status) into a
**sequence of concrete robot actions**.  It reasons about where to place a
dish, when to speak, when to move and when to wait for another robot.

.. image:: images/brain_subsystem.png
   :alt: Component diagram of the Brain subsystem
   :align: center
   :width: 90%

Component roles
~~~~~~~~~~~~~~~
* **Reasoning Action** – transforms a symbolic task into ordered
  ActionLib goals (base → arm → gripper).  
* **Reasoning Table Placement** – analyses the RGB-D map of a table and picks a
  free, stable coordinate for the plate or for clearing.  
* **Task Manager** – aggregates feedback, queries the
  ``/robot_state_decision`` service and pushes the next task to the correct
  robot.  
* **Reasoning Speech Generation** – decides *when* and *what* to announce to
  customers, then hands the sentence to the interaction layer.

ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 33 22 45

   * - Topic / Service
     - Type
     - KPI / Note
   * - ``/planned_path`` → *Reasoning Action*
     - ``nav_msgs/Path``
     - First waypoint < 0.5 m from target; update ≤ 500 ms
   * - ``/table_reasoning_commands`` ← *Reasoning Table Placement*
     - ``std_msgs/String``
     - Commands **PLACE**, **CLEAR**, **INSPECT**
   * - ``/task_feedback`` (out)
     - ``std_msgs/String``
     - State change ≤ 200 ms after controller result
   * - ``/robot_state_decision`` (srv)
     - ``tiago1/robotstatedecision``
     - Response ≤ 100 ms, 99.9 % success
   * - ``/speaker_channel`` (out)
     - ``std_msgs/String``
     - ≤ 1 s after customer confirmed

Implementation modules
----------------------
Detailed API docs for each Brain component can be found below.

.. toctree::
   :maxdepth: 1
   :caption: Brain Components

   brain_modules/reasoning_action
   brain_modules/reasoning_table_placement
   brain_modules/task_manager
   brain_modules/reasoning_speech_generation
