roslaunch tiago1 tiago_launch.launch
rosbag play src/tiago1/rosbag/arm_jointstate.bag & 
rosbag play src/tiago1/rosbag/depth_raw.bag & 
rosbag play src/tiago1/rosbag/force_sensor.bag &
rosbag play src/tiago1/rosbag/odometry.bag &
rosbag play src/tiago1/rosbag/rgb_raw.bag & 
rosbag play src/tiago1/rosbag/scan_lidar.bag &
rosbag play src/tiago1/rosbag/sonar.bag
