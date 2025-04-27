.. _index:

Assignment 1 Documentation
==========================

Overview
--------
This project implements **Topic 2 – Autonomous Waiters in a Sushi Restaurant**.  
A fleet of five PAL-Robotics TIAGo robots cooperates to serve dishes, clear tables and return to the station. :contentReference[oaicite:0]{index=0}&#8203;:contentReference[oaicite:1]{index=1}

Key software components
^^^^^^^^^^^^^^^^^^^^^^^
1. **Orchestration System**  
   • Dynamically allocates each order to the best-placed robot.  
   • Prevents table congestion by queuing robots if necessary.  

2. **Reasoning about Food Placement**  
   • Analyses the table layout with camera + depth data.  
   • Chooses a free, stable spot and adapts if the table is cluttered.  

3. **Order Verification & Error Handling**  
   • Announces the dish, waits for customer confirmation.  
   • Registers disputes and notifies human staff. :contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}

Installation
------------
Clone the repository into your catkin workspace, build, then source the setup file:

.. code-block:: bash

   cd ~/catkin_ws/src
   git clone https://github.com/<your-org>/cogar_ws.git
   cd ..
   catkin_make
   source devel/setup.bash

Make sure the required ROS Noetic dependencies are installed:

.. code-block:: bash

   sudo apt-get install \
       ros-noetic-actionlib \
       ros-noetic-geometry-msgs \
       ros-noetic-std-msgs \
       ros-noetic-std-srvs \
       ros-noetic-nav-msgs

How to Launch
-------------
**Step 1 – bring up the whole stack**

.. code-block:: bash

   roslaunch cogar_ws sushi_waiter.launch

**Step 2 – assign an order manually (example)**

.. code-block:: none

   rosservice call /assign_order "table_id: 4
   dish_name: 'Dragon Roll'"

**Step 3 – verify / cancel**

.. code-block:: bash

   rostopic echo /order_verification
   rosservice call /cancel_order "table_id: 4"

Proof of Work
-------------
.. image:: static/robot_serving.gif
   :alt: Demo – robot serving a table

.. image:: static/table_clearing.gif
   :alt: Demo – table clearing sequence


.. toctree::
   :maxdepth: 2
   :caption: Contents

   perception
   reasoning
   actuation
   interaction
   orchestration
