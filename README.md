# 目标射击ros包
个人ros学习包


启动机器人底盘
roslaunch abot_bringup robot_with_imu.launch

启动建图
roslaunch robot_slam gmapping.launch

启动脚本注释
###gmapping with abot###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \  
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch abot_bringup robot_with_imu.launch; exec bash"' \    （启动小车底盘）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch abot_bringup shoot.launch; exec bash"' \        （射击模块初始化）
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; roslaunch robot_slam navigation_shoot.launch; exec bash"' \   （导航模块，真机要先启动底盘）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch track_tag usb_cam_with_calibration.launch; exec bash"' \  （启动摄像头）（usb_cam没有这个包就手动安装）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch track_tag ar_track_camera.launch; exec bash"' \    （二维码识别功能包）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; roslaunch find_object_2d find_object_2d_shoot.launch; exec bash"' \   （模板匹配功能包）
--tab -e 'bash -c "sleep 3; source ~/abot_ws/devel/setup.bash; rosrun TTS_audio TTS.py; exec bash"' \  （启动tts系统，安装补全：pip2 install websocket-client，还要兼容py2.....）
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; rosrun robot_slam 2026_shoot_target.py; exec bash"' \   （中间传输，接收语音信息：旋转靶和移动靶ID，并播报相应信息,安装sudo apt install mplayer）
--tab -e 'bash -c "cd /home/abot/abot_ws/src/robot_slam/scripts/; python3 2026_shoot_demo.py; exec bash"' \   （转写文件，播放音频，比赛开始、提示音，调用录音以及识别）
--tab -e 'bash -c "sleep 4; source ~/abot_ws/devel/setup.bash; roslaunch robot_slam multi_goal_shoot_2025.launch; exec bash"' \   （点位文件,修改目标点）





语音识别问题。。：
具体安装的依赖按照报错来
先启动miniconda
source /home/l1ang/miniconda3/bin/activate
再启动虚拟环境
conda activate py39
环境变更
conda remove -n ros_asr_py3 --all -y  
conda create -n py39 python=3.9 -y
安装启动桥接
sudo apt install ros-melodic-rosbridge-server -y
roslaunch rosbridge_server rosbridge_websocket.launch
python3 2026_shoot_demo.py



仿真环境启动顺序
仿真包
roslaunch gazebo_pkg race.launch
导航包
roslaunch robot_slam navigation_shoot.launch
固定靶识别包
roslaunch find_object_2d find_object_2d_shoot.launch
语音识别
rosrun xunfei_py_ros iat_node.py
ar码识别包
roslaunch track_tag ar_track_camera.launch
主程序逻辑py文件
rosrun robot_slam shoot_pid.py



实车（删除小车滤波雷达话题发布节点和camera_info发布节点，实车自己发布）
启动小车底盘
roslaunch abot_bringup robot_with_imu.launch
射击模块启动
roslaunch abot_bringup shoot.launch
启动摄像头
roslaunch track_tag usb_cam_with_calibration.launch
导航包
roslaunch robot_slam navigation_shoot.launch
固定靶识别包
roslaunch find_object_2d find_object_2d_shoot.launch
语音识别
rosrun xunfei_py_ros iat_node.py
ar码识别包
roslaunch track_tag ar_track_camera.launch
主程序逻辑py文件
rosrun robot_slam shoot_pid.py
