# ros学习
个人ros学习包


启动机器人底盘
roslaunch abot_bringup robot_with_imu.launch

启动脚本注释
###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \  
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \    （启动小车底盘）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch abot_bringup shoot.launch; exec bash"' \        （射击模块初始化）
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; roslaunch robot_slam navigation_shoot.launch; exec bash"' \   （导航模块）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \  （启动摄像头）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \    （二维码识别功能包）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch find_object_2d find_object_2d_shoot.launch; exec bash"' \   （模板匹配功能包）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; rosrun TTS_audio TTS.py; exec bash"' \  （语音播报）
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; rosrun robot_slam 2026_shoot_target.py; exec bash"' \   （中间传输）
--tab -e 'bash -c "cd /home/abot/abot_ws/src/robot_slam/scripts/; python3 2026_shoot_demo.py; exec bash"' \   （转写文件）
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; roslaunch robot_slam multi_goal_shoot_2025.launch; exec bash"' \   （点位文件,修改目标点）


