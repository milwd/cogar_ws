# Autonomous Waiters

**Team Name:** ACM Robatics Solution  
**Contributors:** Arian Tavousi, Milad Rabiei, Christian Negri Ravera

## 1. Introduction

This project deploys a fleet of five PAL Robotics TIAGo robots as autonomous waiters in a restaurant, handling order delivery, table clearing, customer interactions and many more.
(Visit the website for more in depth documentation)

## 2. System Overview

The software follows a classic **perception → reasoning → actuation** pipeline,  
with a thin orchestration layer balancing the fleet.

## 3. Workflow

1. **Customer speaks** – simulated ASR (`voice_recognition.py`) publishes “Can I have sushi”  
2. **Order verification** – `reasoning_order_verification.py` parses dish names, enqueues via `send_order` service  
3. **Orchestrator** – a free robot polls `/robot_state_decision`, receives the “sushi” task  
4. **ReasoningAction** – `reasoning_action.py` turns the plan + table command into ActionLib goals (move, arm, gripper)  
5. **Execution** – `control_wheel.py`, `control_arm.py`, `control_gripper.py` carry out the motions  
6. **Feedback loop** – upon completion each controller publishes status; `task_manager.py` feeds back into the orchestrator  
7. **Table placement** – after delivery `reasoning_table_placement.py` issues PLACE_DISH or CLEAR_TABLE commands  
8. **Interaction** – `speech_generator.py` & `speaker.py` announce actions to customers

## 4. Architectural Diagrams

Only the component diagram is included here due to size restrictions. You can view all diagrams in full size using the following link:  
[View Diagrams](https://drive.google.com/file/d/1yGJAAkXFYum7aQUJqAF_S_iSz2Qtz7F8/view?usp=sharing)

## 5. Codebase Tour

Each script is grouped by its pipeline layer. Click through the API docs for details.

### Vision
- **camera.py** – static RGB + synthetic depth generator  
- **camera_preprocessing.py** – colour-correction & noise reduction  
- **object_detection.py** – placeholder object detector (1 Hz)   
- **distance_estimation.py** – depth-powered object localization  

### Brain
- **task_manager.py** – event aggregator, polls `/robot_state_decision`  
- **reasoning_action.py** – translates symbolic plans into ActionLib goals  
- **reasoning_table_placement.py** – maps PLACE/CLEAR decisions to commands  
- **reasoning_speech_generation.py** – throttles and republishes speech commands at 1 Hz  

### Navigation
- **path_planning.py** – dummy global planner (1 waypoint)  
- **slam.py** – blank map + identity odom stub  
- **sensor_fusion.py** – fuse LiDAR + sonar into a single scan  
- **lidar.py**, **sonar.py** – synthetic LiDAR/sonar streams  

### Control
- **control_wheel.py** – base controller (first waypoint)  
- **control_arm.py** – proportional joint controller (ActionLib)  
- **control_gripper.py** – binary gripper actuator (ActionLib)  
- **encoder_wheel.py**, **encoder_arm.py**, **encoder_gripper.py** – synthetic encoders  
- **force.py** – fake force sensor for manipulator testing  

### Interaction
- **speech_generator.py** – buffers and throttles TTS commands  
- **speaker.py** – last-mile relay to TTS backend  
- **microphone.py**, **voice_recognition.py** – synthetic ASR pipeline  
- **reasoning_order_verification.py** – validates “Can I have …” orders  

### Server
- **orchestration_and_coordination.py** – YAML-backed order queue + state gateway  

## 6. Running the Project

You need to add the number of Robots you want to launch (the default is 2) in order to have sufficient number of nodes created.  
For this, you can edit ROS params, then use the following to run the project:
```bash
roslaunch cogar_ws tiago_launch.launch
```
## 7. Running the Test

For running the test, open the test folder and then run:
```bash
python integration_test.py
```
