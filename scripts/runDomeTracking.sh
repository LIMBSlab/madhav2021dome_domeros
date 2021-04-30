#!/bin/bash

gnome-terminal -x roslaunch pointgrey_camera_driver camera.launch
sleep 2
gnome-terminal -x rosrun ros_marker_tracking tracker_realtime D
sleep 2
gnome-terminal -x rqt_image_view
sleep 2
rosrun ros_limbs_dome_sensor_fusion angle_only /tracking_results /back_tracking_results /rat_head_angle /rat_body_angle
