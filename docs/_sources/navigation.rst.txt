========================
Subsystem Navigation
========================

What the Navigation subsystem does
----------------------------------
The Navigation stack answers two fundamental questions for each TIAGo robot:

1. **Where am I?**  
   The **SLAM** node builds and updates a map (via `/map`) and estimates the robot’s pose (`/odom_slam`), ensuring downstream components know the robot’s location in a global frame.

2. **How do I reach my goal?**  
   The **Path Planner** consumes the current map and odometry to generate a collision-free trajectory (`/planned_path`), handing off waypoints to the motion controllers.

To support those core functions, the subsystem also:

- **Scans the environment** with synthetic **LiDAR** and **Sonar** sensors.
- **Fuses** their data into a coherent cost-map (`/costmap` or fused scan) for obstacle avoidance.
- **Continuously replans** if new obstacles appear or the goal moves.

.. image:: images/navigation_subsystem.png
   :alt: Component diagram of the Navigation subsystem
   :align: center
   :width: 90%

Design Patterns
---------------

Strategy
~~~~~~~~
In our Navigation subsystem, **Path Planning** uses Strategy so we can replace the dummy one-waypoint planner with A*, D*, RRT or any other planning algorithm simply by supplying a new implementation of the same interface.  This keeps planning logic **extensible** and **decoupled** from the rest of the navigation flow.

Observer
~~~~~~~~
**Sensor Fusion** subscribes to both `/lidar` and `/sonar` topics and triggers its fusion logic whenever new data arrives.  This event-driven design ensures that cost-map updates and fused scans happen exactly when fresh sensor measurements are available, without expensive polling loops.

Adapter
~~~~~~~ 
In **Sensor Fusion**, the `Adapter` class transforms the sonar’s single scalar range into a pseudo-LiDAR scan of equal length.  By encapsulating this conversion in a dedicated adapter, the fusion logic and downstream consumers remain **agnostic** to the sonar’s original message format.

Template Method
~~~~~~~~~~~~~~~
Our **SLAM** node stub implements a fixed callback flow—receive fused scan, build blank occupancy grid, publish map, publish identity odometry—while leaving the `update_map(...)` and `update_pose(...)` hooks for a real SLAM backend to override.  This ensures a consistent processing pipeline and simplifies integration of a full SLAM solution later.

Facade
~~~~~~
Our **SLAM** node is exactly Facade: it hides all the real mapping, pose-estimation and TF bookkeeping behind a simple interface (`/map`, `/odom_slam`).  Downstream code never needs to know whether we're running RTAB-Map, GMapping or our blank stub—just publish a scan, and a map & odometry appear.

Factory
~~~~~~~ 
In each Navigation node (e.g. **lidar.py**, **sonar.py**, **path_planning.py**), we factor the setup of ROS publishers, subscribers and message constructors into small helper functions.  That way, if we need a different message type or QoS settings, we change only the factory, not every callback.

Chain of Responsibility (Pipeline)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Our **Sensor Fusion** logic is effectively a fusion pipeline—LiDAR → Adapter → merge → publish—where each stage handles part of the work.  We can insert new fusion strategies (e.g. a Kalman filter) into the chain without rewriting the entire node.


Component roles
---------------
- **lidar.py**  
  - Synthesises a `sensor_msgs/LaserScan` at 10 Hz over 180°; acts as a stand-in LiDAR sensor.

- **sonar.py**  
  - Publishes `sensor_msgs/Range` (ultrasound) at 10 Hz; used for short-range obstacle detection.

- **sensor_fusion.py**  
  - Observes `/lidar` and `/sonar`, adapts sonar via the Adapter pattern, merges the two scans element-wise, and republishes a fused `LaserScan` for downstream cost-map consumers.

- **slam.py**  
  - Facade/stub for SLAM: subscribes to fused scans, generates a blank `nav_msgs/OccupancyGrid` and an identity `nav_msgs/Odometry`, and publishes both on `/map` and `/odom_slam`.  Hooks (`update_map`, `update_pose`) are provided for real mapping and localization.

- **path_planning.py**  
  - Implements a dummy global planner using the Strategy pattern: when both map and odometry are available, publishes a one-waypoint `nav_msgs/Path` at 1 Hz on `/planned_path`.

ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 33 22 45

   * - Topic / Service
     - Type
     - KPI / Note
   * - **`/lidar`**  
     - `sensor_msgs/LaserScan`  
     - 10 Hz, 180° sweep; ≤ 50 ms sensor latency  
   * - **`/sonar`**  
     - `sensor_msgs/Range`  
     - 10 Hz, single-beam; near-range obstacle detection  
   * - **`/fused_scan`**  
     - `sensor_msgs/LaserScan`  
     - Fusion ≤ 100 ms after new sonar or LiDAR input  
   * - **`/map`, `/odom_slam`**  
     - `nav_msgs/OccupancyGrid`, `nav_msgs/Odometry`  
     - Map update and odometry publish ≤ 200 ms after fused scan (stub meets immediately)  
   * - **`/planned_path`**  
     - `nav_msgs/Path`  
     - First waypoint ≤ 0.5 m from robot; replanning ≤ 300 ms after cost-map change  

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
