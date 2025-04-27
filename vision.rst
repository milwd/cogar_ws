=======================
Subsystem Vision
=======================

Overview
--------
The **Vision** subsystem converts raw RGB-D data into a sparse *Object Map* for
down-stream planners.  It comprises four components:

* **Camera** – publishes RGB and depth frames.  
* **Camera Preprocessing** – undistorts, colour-balances and denoises.  
* **Object Detection / Segmentation** – detects plates, hands and obstacles.  
* **Distance Estimation** – fuses masks with depth to output metric distances.

.. image:: images/vision_subsystem.png
   :alt: Component diagram of the Vision subsystem
   :align: center
   :width: 90%

ROS interfaces & KPIs
---------------------
.. list-table::
   :header-rows: 1
   :widths: 32 23 45

   * - Topic / Service
     - Type
     - KPI / Note
   * - ``/camera``  
       ``/depth``
     - ``sensor_msgs/Image``
     - 1280×720 @ 10 Hz, ≤ 200 ms latency
   * - ``/img_processed``
     - ``sensor_msgs/Image``
     - Undistorted RGB, < 25 ms extra latency
   * - ``/object_hypotheses``
     - ``vision_msgs/Detection2DArray``
     - ≥ 0.75 mAP (IoU 0.5) on sushi-dish set
   * - ``/depth_processed``
     - ``sensor_msgs/Image``
     - Hole-filled depth, < 1 % invalid pixels
   * - ``/object_map``
     - ``cogar_msgs/ObjectMap``
     - Published ≤ 300 ms after frame start

Implementation modules
----------------------
The pages below document every public class, function and parameter of each
Vision component.

.. toctree::
   :maxdepth: 1
   :caption: Vision Components

   vision_modules/camera
   vision_modules/camera_preprocessing
   vision_modules/object_detection
   vision_modules/distance_estimation
