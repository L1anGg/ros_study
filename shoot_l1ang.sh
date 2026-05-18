#!/bin/bash

# 仿真环境启动脚本 (Ubuntu 18.04 + ROS Melodic)

gnome-terminal --window -e 'bash -c "sleep 1; source ~/abot_ws/devel/setup.bash; roslaunch gazebo_pkg race.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; source ~/abot_ws/devel/setup.bash; roslaunch robot_slam navigation_shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; source ~/abot_ws/devel/setup.bash; roslaunch find_object_2d find_object_2d_shoot.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; rosrun xunfei_py_ros iat_node.py; exec bash"' \
--tab -e 'bash -c "sleep 6; source ~/abot_ws/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; source ~/abot_ws/devel/setup.bash; rosrun robot_slam shoot_pid.py; exec bash"'