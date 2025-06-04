#!/bin/bash

# Source ROS 2 and your workspace
echo "Sourcing ROS 2 and workspace..."
source /opt/ros/humble/setup.bash
source /home/cam_ws/install/setup.bash

echo "Starting v4l2_camera_node on /dev/video8..."
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args \
  -p video_device:=/dev/video8 \
  -p camera_info_url:=file:///home/cam_ws/camera_calib3.yaml \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info &

sleep 2

echo "Starting AprilTag node..."
ros2 run apriltag_ros apriltag_node \
  --ros-args \
  -r image_rect:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml &
sleep 2

echo "Starting Matrix Calc..."
ros2 run apriltag_detection apriltag_detector
