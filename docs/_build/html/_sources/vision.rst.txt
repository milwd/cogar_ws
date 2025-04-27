=======================
Subsystem Vision
=======================

What the Vision subsystem does
------------------------------
The Vision layer is responsible for turning raw RGB-D sensor streams into a concise **Object Map** that downstream planners and manipulators can use.  Its end-to-end workflow is:

  1. **Capture**  
     The **Camera** node streams synchronized color and depth frames.

  2. **Clean & Normalize**  
     **Camera Preprocessing** applies lens undistortion, colour-balance and denoising filters in a reusable pipeline.

  3. **Perceive**  
     **Object Detection** locates plates, hands and obstacles in each frame at a steady 1 Hz, producing bounding-box hypotheses.

  4. **Localize**  
     **Distance Estimation** fuses those bounding boxes with depth pixels to compute metric XYZ coordinates, emitting a final `cogar_msgs/ObjectMap`.

.. image:: images/vision_subsystem.png
   :alt: Component diagram of the Vision subsystem
   :align: center
   :width: 90%

Design Patterns
---------------

Strategy
~~~~~~~~
In our Vision subsystem, **Object Detection** uses Strategy to swap between different detection implementations—whether a placeholder stub, a classical CV method or a deep-learning model—without touching the rest of the pipeline.  This keeps detection logic **extensible** and **decoupled** from downstream consumers.

Observer
~~~~~~~~
**Camera Preprocessing** subscribes to `/camera` and `/depth` and processes each frame as soon as it arrives.  By reacting to new-image events rather than polling, we guarantee that every frame is handled exactly once and with minimal latency.

Adapter
~~~~~~~
**Camera** uses `CvBridge` to adapt ROS `sensor_msgs/Image` messages into OpenCV `numpy` arrays and back.  This isolates ROS message formats from CV code, making sensor-to-image integration seamless and maintainable.

Decorator
~~~~~~~~~
**Camera Preprocessing** chains filters—undistort → colour-balance → 5×5 Gaussian blur—by wrapping each output in the next filter.  We can add, remove or reorder filters without modifying the subscription logic.

Template Method
~~~~~~~~~~~~~~~ 
**Distance Estimation** provides a fixed flow—parse detection results, sample depth, back-project to XYZ, publish—while allowing future variants to customize parsing or projection without rewriting the overall pipeline.  

Component roles
---------------
- **Camera**  
  - Publishes `/camera` (`sensor_msgs/Image`) and `/depth` (`sensor_msgs/Image`) at **1,280×720 @ 10 Hz**.  
  - Implements the **Adapter** pattern via `CvBridge` for ROS↔OpenCV conversions.

- **Camera Preprocessing**  
  - Subscribes to raw streams, applies an undistort + Gaussian blur + colour-balance **pipeline** (Decorator pattern), and republishes on `/camera_processed` and `/depth_processed`.  

- **Object Detection**  
  - Listens to `/camera_processed`, applies a detection algorithm chosen by the **Strategy** pattern, and publishes `vision_msgs/Detection2DArray` on `/object_hypotheses` at **1 Hz**.  

- **Distance Estimation**  
  - Subscribes to `/object_hypotheses` and `/depth_processed`, uses a **Template Method**–style base class to parse, project via pinhole intrinsics and publish a `cogar_msgs/ObjectMap` on `/object_map`.  


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
     - 10 Hz capture rate; end-to-end acquisition latency ≤ 200 ms  
   * - **`/camera_processed`, `/depth_processed`**  
     - `sensor_msgs/Image`  
     - Preprocessing adds < 25 ms latency; depth completeness ≥ 99 %  
   * - **`/object_hypotheses`**  
     - `vision_msgs/Detection2DArray`  
     - Detection mAP ≥ 0.75 (IoU 0.5) on plate set, published ≤ 1 s after frame  
   * - **`/object_map`**  
     - `cogar_msgs/ObjectMap`  
     - Metric positions published ≤ 300 ms after detections  

Implementation modules
----------------------
Detailed API docs for each Vision component:

.. toctree::
   :maxdepth: 1
   :caption: Vision Components

   vision_modules/camera
   vision_modules/camera_preprocessing
   vision_modules/object_detection
   vision_modules/distance_estimation
