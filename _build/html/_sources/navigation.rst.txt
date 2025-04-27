========================
Subsystem Navigation
========================

Mission
-------
The Navigation stack answers two questions:

1. **Where am I?** – `slam.py` builds and updates a 3-D map while estimating
   the robot pose.
2. **How do I reach the goal?** – `path_planning.py` turns the pose plus a
   target into a collision-free trajectory.

To feed those core functions the subsystem fuses range data from **LiDAR** and
**SONAR**, produces a dense cost-map and continuously replans if new obstacles
appear.

.. image:: images/navigation_subsystem.png
   :alt: Component diagram of the Navigation subsystem
   :align: center
   :width: 90%

ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 33 22 45

   * - Topic / Service
     - Type
     - KPI / Note
   * - ``/scan`` ← *lidar.py*
     - ``sensor_msgs/LaserScan``
     - 10 Hz, 360°, ≤ 50 ms latency
   * - ``/sonar_cloud`` ← *sonar.py*
     - ``sensor_msgs/PointCloud2``
     - 5 Hz, 120° front arc
   * - ``/costmap`` ← *sensor_fusion.py*
     - ``nav_msgs/OccupancyGrid``
     - Update ≤ 150 ms after new scan
   * - ``/slam_out_pose`` ← *slam.py*
     - ``geometry_msgs/PoseWithCovarianceStamped``
     - Drift ≤ 0.5 % of distance
   * - ``/planned_path`` ← *path_planning.py*
     - ``nav_msgs/Path``
     - First waypoint ≤ 0.5 m from robot, replans ≤ 300 ms after cost-map change

Implementation modules
----------------------
Click a component to view its API and internal documentation.

.. toctree::
   :maxdepth: 1
   :caption: Navigation Components

   navigation_modules/sonar
   navigation_modules/lidar
   navigation_modules/sensor_fusion
   navigation_modules/slam
   navigation_modules/path_planning
