=======================
Subsystem Vision
=======================

What the Vision subsystem does
------------------------------
The Vision layer captures raw RGB‐D streams and transforms them into a sparse, actionable **Object Map**.  
It is responsible for acquiring sensor data, cleaning it, detecting relevant entities (plates, hands, obstacles)  
and estimating their 3D positions for downstream planning and manipulation.

.. image:: images/vision_subsystem.png
   :alt: Component diagram of the Vision subsystem
   :align: center
   :width: 90%

Component roles
~~~~~~~~~~~~~~~
- **Camera**  
  - Publishes synchronized **RGB** and **Depth** frames on `/camera` and `/depth` at **1,280×720** @ **10 Hz**.  
- **Camera Preprocessing**  
  - Subscribes to raw images, performs undistortion, colour‐balance and 5×5 Gaussian denoising, then republishes on `/camera_processed` and `/depth_processed`.  
- **Object Detection**  
  - Listens to `/camera_processed` and emits a `vision_msgs/Detection2DArray` of detected plates, hands and obstacles at **1 Hz**.  
- **Distance Estimation**  
  - Consumes `/object_hypotheses` and `/depth_processed`, back‐projects centre pixels (or bounding‐box centres) into metric XYZ, and outputs a `cogar_msgs/ObjectMap`.

ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 30 25 45

   * - Topic / Service
     - Type
     - KPI / Note
   * - **`/camera`, `/depth`**  
     - `sensor_msgs/Image`  
     - 10 Hz, end-to-end sensor latency ≤ 200 ms  
   * - **`/camera_processed`, `/depth_processed`**  
     - `sensor_msgs/Image`  
     - Preprocessing adds < 25 ms; depth completeness ≥ 99 %  
   * - **`/object_hypotheses`**  
     - `vision_msgs/Detection2DArray`  
     - Detection mAP ≥ 0.75 (IoU 0.5) on plate set  
   * - **`/object_map`**  
     - `cogar_msgs/ObjectMap`  
     - Published ≤ 300 ms after frame capture  

Implementation modules
----------------------
Detailed API docs for each Vision component are linked below:

.. toctree::
   :maxdepth: 1
   :caption: Vision Components

   vision_modules/camera
   vision_modules/camera_preprocessing
   vision_modules/object_detection
   vision_modules/distance_estimation
