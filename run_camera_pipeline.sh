#!/bin/bash

# Source ROS 2 and your workspace
echo "Sourcing ROS 2 and workspace..."
source /opt/ros/humble/setup.bash
source /home/cam_ws/install/setup.bash

echo "Starting v4l2_camera_node on /dev/video8..."
#ros2 run v4l2_camera v4l2_camera_node --ros-args \
#  -p video_device:=/dev/video8 \
#  -p camera_info_url:=file:///home/cam_ws/camera_calib3.yaml \
#  -p auto_exposure:=1 \
#  -p exposure_time_absolute:=330 \
#  -p image_size:=[960,600] \
#  -p time_per_frame:=[1,15] \
#  -r image_raw:=/camera/image_raw \
#  -r camera_info:=/camera/camera_info

ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video8 \
  -p camera_info_url:=file:///home/cam_ws/camera_calib3.yaml \
  -p autoexposure:=false \
  -p exposure:=330 \
  -p image_width:=960 \
  -p image_height:=600 \
  -p framerate:=80.0 \
  -r image_raw:=/camera/image_raw \
  -r camera_info:=/camera/camera_info &

exit 1
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
