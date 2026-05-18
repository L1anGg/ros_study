#!/bin/bash

# 仿真环境启动脚本 (Ubuntu 18.04 + ROS Melodic)
#--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; rosrun xunfei_py_ros iat_node.py; exec bash"' \

gnome-terminal --window -e 'bash -c "sleep 2; source ~/abot_ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/abot_ws/devel/setup.bash; roslaunch robot_slam navigation_shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/abot_ws/devel/setup.bash; rosrun tf tf_echo /map /base_footprint; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/abot_ws/devel/setup.bash; rosrun rqt_robot_steering rqt_robot_steering; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/abot_ws/devel/setup.bash; rosrun robot_slam shoot_pid_debug.py; exec bash"'