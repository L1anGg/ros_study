#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@功能说明：ROS移动机器人自动导航+视觉瞄准+串口射击控制节点
@核心能力：
1. 基于move_base实现机器人定点导航，支持多目标点序列巡航
2. 订阅物体识别话题，实现目标视觉对准与自动射击
3. 订阅AR码识别话题，实现旋转靶标精准瞄准与自动射击
4. 通过串口指令控制射击机构的启停
@运行环境：ROS1 + Python2.7
"""
# 导入ROS核心Python接口
import rospy
# 导入数学计算库，用于角度、坐标计算
import math
# 导入ROS动作库，用于move_base导航的异步动作调用
import actionlib
# 导入串口通信库，用于与射击机构硬件通信
#import serial
# 导入系统时间库（注意：原代码全局变量time覆盖了该模块，已修正命名）
import time as sys_time
# 导入ROS标准字符串消息类型
from std_msgs.msg import String
# ---------------------- 串口硬件配置 ----------------------
# 射击机构串口设备路径
serialPort = "/dev/shoot"
# 串口波特率
baudRate = 9600
# 初始化串口实例，配置串口参数：无校验、8位数据位、1位停止位
#ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)
# ---------------------- 导航与动作相关消息导入 ----------------------
# 导入动作状态消息，用于判断导航执行结果
from actionlib_msgs.msg import *
# 导入move_base导航动作的目标与动作定义
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# 导入路径消息类型
from nav_msgs.msg import Path
# 导入带协方差的位姿消息，用于设置机器人初始位姿
from geometry_msgs.msg import PoseWithCovarianceStamped
# 导入TF坐标变换库，用于欧拉角与四元数的转换
from tf_conversions import transformations
# 导入圆周率常量
from math import pi
# 导入ROS速度控制消息，用于机器人底盘运动控制
from geometry_msgs.msg import Twist
# 导入三维点消息，用于接收物体识别的坐标结果
from geometry_msgs.msg  import Point
# 导入AR码识别消息类型，用于接收Alvar AR标签的识别结果
from ar_track_alvar_msgs.msg import AlvarMarkers
from ar_track_alvar_msgs.msg import AlvarMarker

from TTS_audio.srv import StringService

# ---------------------- Python2编码与系统配置 ----------------------
import sys
# 重载sys模块，设置Python2默认编码为UTF-8，解决中文乱码问题
reload(sys)
sys.setdefaultencoding('utf-8')
import os

# ---------------------- 全局变量定义 ----------------------
target_id_rotating_mapping = {
    "零":0,
    "一": 1,
    "二": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    '0':0,
    '1': 1,
    '2': 2,
    '3': 3,
    '4': 4,
    '5': 5
}
rotating_id = 10

target_id_moving_mapping = {
    "零":0,
    "六": 6,
    "七": 7,
    "八": 8,
    '0':0,
    '6': 6,
    '7': 7,
    '8': 8
}
moving_id = 10

# 音频文件路径（预留语音播报功能）
music_path="~/'07.mp3'"
# AR码ID全局变量，存储当前识别到的AR标签ID
id = 255
# 物体识别X轴偏移量标志
flog0 = 255
# 物体识别X轴偏移绝对值标志
flog1 = 255
# 射击完成状态标志，防止重复射击
flog2 = 255
# 预留状态标志位
flog3 = 255
flog4 = 255
# 计数变量
count = 0
# 后退动作计时变量（修正原变量名，避免覆盖time模块）
back_time = 0
# 机器人运动状态标志
move_flog = 0
# 瞄准偏航角阈值：AR码X轴偏移小于该值，判定为对准
Yaw_th = 0.40
# AR码Y轴坐标有效范围下限
Min_y = -0.26 #0.36
# AR码Y轴坐标有效范围上限
Max_y = -0.10 #0.30
# AR码识别状态标志
ar_flog=255
# ---------------------- 核心状态机变量 ----------------------
# 机器人整体运行状态机：0=初始瞄准阶段 1=1号靶位瞄准 2=2号靶位瞄准 3=终点阶段
case = 255
# 预留状态机子变量
case1 = 255
case2 = 255
case3 = 255


# ---------------------- 精准停靠PID控制器（无电机死区） ----------------------
class DockPID:
    """
    机器人精准停靠PID控制器
    功能：基于AMCL位姿反馈，实现map坐标系下x/y/yaw精准停靠
    无电机死区补偿，纯位姿纠偏
    """
    def __init__(self, cmd_vel_pub):
        """
        初始化PID控制器
        :param cmd_vel_pub: 外部传入的/cmd_vel发布者（复用原有发布者）
        """
        # 复用外部的速度发布者
        self.cmd_vel_pub = cmd_vel_pub
        
        # PID参数配置（和你原有参数完全一致）
        self.kp_x = 0.4    # x轴比例系数
        self.ki_x = 0.01   # x轴积分系数
        self.kd_x = 0.1    # x轴微分系数
        
        self.kp_y = 0.4    # y轴比例系数
        self.ki_y = 0.01   # y轴积分系数
        self.kd_y = 0.1    # y轴微分系数
        
        self.kp_yaw = 0.8  # 偏航角比例系数
        self.ki_yaw = 0.01 # 偏航角积分系数
        self.kd_yaw = 0.1  # 偏航角微分系数
        
        # PID输出限幅（和你原有参数一致）
        self.max_x = 0.3   # x轴最大速度
        self.min_x = -0.3
        self.max_y = 0.2   # y轴最大速度
        self.min_y = -0.2
        self.max_yaw = 0.5 # 最大角速度
        self.min_yaw = -0.5
        
        # PID内部状态变量
        self._reset_pid_state()
        
        # 机器人当前位姿
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # 停靠误差阈值（和你原有参数一致）
        self.x_th = 0.07    # x轴误差<2cm
        self.y_th = 0.07    # y轴误差<2cm
        self.yaw_th = 0.07  # 偏航角误差<0.57°

    def _reset_pid_state(self):
        """重置PID内部状态（每次停靠前自动调用）"""
        # X轴PID状态
        self.last_error_x = 0.0
        self.integral_x = 0.0
        self.last_time_x = sys_time.time()
        
        # Y轴PID状态
        self.last_error_y = 0.0
        self.integral_y = 0.0
        self.last_time_y = sys_time.time()
        
        # 偏航角PID状态
        self.last_error_yaw = 0.0
        self.integral_yaw = 0.0
        self.last_time_yaw = sys_time.time()

    def update_pose(self, x, y, yaw):
        """
        外部传入AMCL位姿（复用原有amcl_pose_cb的结果）
        :param x: 当前x坐标
        :param y: 当前y坐标
        :param yaw: 当前偏航角（弧度制）
        """
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw

    def _pid_compute(self, target, current, kp, ki, kd, min_out, max_out, 
                     last_error, integral, last_time):
        """通用PID计算函数"""
        current_time = sys_time.time()
        dt = current_time - last_time
        if dt <= 0:
            return 0.0, last_error, integral, last_time
        
        error = target - current
        
        # 比例项
        p_term = kp * error
        
        # 积分项（限幅防止饱和）
        integral += error * dt
        integral = max(min(integral, 1.0), -1.0)
        i_term = ki * integral
        
        # 微分项
        d_term = kd * (error - last_error) / dt
        
        # 总输出+限幅
        output = p_term + i_term + d_term
        output = max(min(output, max_out), min_out)
        
        return output, error, integral, current_time

    def precise_dock(self, target_x, target_y, target_yaw_deg, timeout=6.0):
        """
        核心停靠函数
        :param target_x: 目标x坐标（map坐标系）
        :param target_y: 目标y坐标（map坐标系）
        :param target_yaw_deg: 目标偏航角（角度制）
        :param timeout: 最大停靠超时时间（秒）
        :return: 停靠成功返回True
        """
        # 角度转弧度
        target_yaw_rad = math.radians(target_yaw_deg)
        # 重置PID状态
        self._reset_pid_state()
        
        # 20Hz控制频率
        max_iter = int(timeout * 20)
        rate = rospy.Rate(20)
        
        rospy.loginfo("开始PID精准停靠，目标：(%.2f, %.2f), 朝向%.1f°" % (target_x, target_y, target_yaw_deg))
        
        for _ in range(max_iter):
            # 计算误差
            error_x = target_x - self.current_x
            error_y = target_y - self.current_y
            error_yaw = target_yaw_rad - self.current_yaw
            
            # 误差达标，停靠完成
            if abs(error_x) < self.x_th and abs(error_y) < self.y_th and abs(error_yaw) < self.yaw_th:
                self.stop()
                rospy.loginfo("精准停靠完成！")
                return True
            
            # 分段减速（保留你原有的逻辑）
            if abs(error_x) < 0.05:
                current_max_x = 0.25
            else:
                current_max_x = self.max_x
            
            # PID计算各轴输出
            vel_x, self.last_error_x, self.integral_x, self.last_time_x = self._pid_compute(
                target_x, self.current_x, self.kp_x, self.ki_x, self.kd_x,
                self.min_x, current_max_x, self.last_error_x, self.integral_x, self.last_time_x
            )
            
            vel_y, self.last_error_y, self.integral_y, self.last_time_y = self._pid_compute(
                target_y, self.current_y, self.kp_y, self.ki_y, self.kd_y,
                self.min_y, self.max_y, self.last_error_y, self.integral_y, self.last_time_y
            )
            
            vel_yaw, self.last_error_yaw, self.integral_yaw, self.last_time_yaw = self._pid_compute(
                target_yaw_rad, self.current_yaw, self.kp_yaw, self.ki_yaw, self.kd_yaw,
                self.min_yaw, self.max_yaw, self.last_error_yaw, self.integral_yaw, self.last_time_yaw
            )
            
            # 发布速度指令
            twist = Twist()
            twist.linear.x = vel_x
            twist.linear.y = vel_y
            twist.angular.z = vel_yaw
            self.cmd_vel_pub.publish(twist)
            
            rate.sleep()
        
        # 超时停车
        self.stop()
        rospy.logerr("精准停靠超时！")
        return False

    def stop(self):
        """发布停车指令"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)




# ---------------------- 导航与射击核心类 ----------------------
class navigation_demo:
    # 类初始化函数：ROS节点初始化、话题发布订阅、动作客户端初始化
    def __init__(self):
        # 发布者：发布机器人初始位姿到/initialpose话题，用于地图定位初始化
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        # 发布者：发布到达目标点的语音播报内容到/voiceWords话题
        self.arrive_pub = rospy.Publisher('/voiceWords',String,queue_size=10)
        # 订阅者：订阅物体识别节点输出的目标坐标/object_position，回调函数为find_cb
        self.find_sub = rospy.Subscriber('/object_position', Point, self.find_cb)
        # 订阅者：订阅AR码识别节点输出的标签结果/ar_pose_marker，回调函数为ar_cb
        self.ar_sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.ar_cb)
        # 动作客户端：连接move_base导航动作服务器，实现定点导航
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # 等待导航动作服务器上线，超时时间60秒
        self.move_base.wait_for_server(rospy.Duration(60))
        # 发布者：发布底盘速度控制指令到/cmd_vel话题，直接控制机器人运动
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1000)

        # ========== 初始化精准停靠PID ==========
        # 复用原有的/cmd_vel发布者
        self.dock_pid = DockPID(self.pub)
        # 订阅AMCL实时位姿（保留原有订阅，给DockPID传数据）
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_cb)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0



    # 实时获取小车当前坐标和朝向，并更新给DockPID
    def amcl_pose_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # 四元数转欧拉角，获取偏航角
        q = msg.pose.pose.orientation
        r, p, yaw = transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        # 把最新位姿传给DockPID
        self.dock_pid.update_pose(self.current_x, self.current_y, self.current_yaw)




    #x方向上的速度发布函数
    def goto_x(self,set):
            msg_s = Twist()
            msg_s.angular.z = 0.0
            self.pub.publish(msg_s)
            msg = Twist()
            # X轴线速度
            if set == 'on':
                msg.linear.x = 0.25
            elif set == 'down':
                msg.linear.x = -0.25
            msg.linear.y = 0
            back_time = 0
            while(back_time <= 37):
                self.pub.publish(msg)
                 # 休眠0.1秒，控制发布频率
                rospy.sleep(0.1)
                back_time = back_time + 1

            msg.linear.y = 0
            msg.linear.x = 0
            self.pub.publish(msg)

    #y方向上的速度发布函数
    def goto_y(self,set):
            msg_s = Twist()
            msg_s.angular.z = 0.0
            self.pub.publish(msg_s)
            msg = Twist()
            # Y轴线速度
            if set == 'on':
                msg.linear.y = 0.38
            elif set == 'down':
                msg.linear.y = -0.38
            # Z轴线速度：置0
            msg.linear.z = 0.0
            # 三个轴的角速度全部置0，仅平移运动
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            # 循环发布速度指令，总时长15*0.1=1.5秒
            back_time = 0
            while(back_time <= 30):
                self.pub.publish(msg)
                # 休眠0.1秒，控制发布频率
                rospy.sleep(0.1)
                back_time = back_time + 1

            msg.linear.y = 0
            msg.linear.x = 0
            self.pub.publish(msg)
            
    #简易前往目标点函数
    def pid_goto(self,pid_g):
        global case
        #直接发布速度话题到达目标点1
        if pid_g == 1:
            print('start pidgoto')
            global back_time
            # 初始化速度控制消息
            msg = Twist()

            msg_s = Twist()
            msg_s.angular.z = 0.0
            # 发布速度指令
            self.pub.publish(msg_s)

            # Y轴线速度
            msg.linear.y = -0.4
            # Z轴线速度：置0
            msg.linear.z = 0.0
            # 三个轴的角速度全部置0，仅平移运动
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            # 循环发布速度指令，总时长15*0.1=1.5秒
            while(back_time <= 10):
                self.pub.publish(msg)
                # 休眠0.1秒，控制发布频率
                rospy.sleep(0.1)
                back_time = back_time + 1
        
            # X轴线速度
            msg.linear.x = 0.25
            msg.linear.y = 0
            back_time = 0
            while(back_time <= 44):
                self.pub.publish(msg)
                # 休眠0.1秒，控制发布频率
                rospy.sleep(0.1)
                back_time = back_time + 1

            msg.linear.y = 0
            msg.linear.x = 0
            self.pub.publish(msg)
            print(self.current_x, self.current_y, self.current_yaw)
            self.dock_pid.precise_dock(target_x=1.0, target_y=-0.45, target_yaw_deg=0)
            case = 0
            #return True


        #pid_g=2
        #直接发布速度话题到达目标点2
        if pid_g == 2:
            print('start pidgoto')
            self.goto_x('down')
            self.goto_y('down')
            self.goto_x('on')
            print(self.current_x, self.current_y, self.current_yaw)
            self.dock_pid.precise_dock(target_x=1.00, target_y=-1.6, target_yaw_deg=0.00)
            case = 1
            #return True

        #pid_g=3
        if pid_g == 3:
            print('start pidgoto')
            self.goto_x('down')
            self.goto_y('down')
            self.goto_x('on')
            print(self.current_x, self.current_y, self.current_yaw)
            self.dock_pid.precise_dock(target_x=1.00, target_y=-2.75, target_yaw_deg=0.00)
            case = 2
            #return True

        #self.end()


    def yaw_zero(self):
        print('start yaw_zero')
        rate = rospy.Rate(20)
        twist = Twist()
        while not rospy.is_shutdown():
            # 1. 计算误差+归一化（自动最短路径）
            err = math.atan2(math.sin(-self.current_yaw), math.cos(-self.current_yaw))
            print('yaw_err',err)
            # 2. 误差<1度停止
            if abs(err) < 0.03:
                twist.angular.z = 0.0
                self.pub.publish(twist)
                print('yaw_zero completed')
                break
            # 3. 纯比例控制+限幅，发速度
            twist.angular.z = max(-0.4, min(0.4, 1 * err))
            self.pub.publish(twist)
            rate.sleep()


    # 终点后退函数：任务完成后，控制机器人匀速后退指定时长
    def end(self):
        print('start end')
        global back_time
        back_time = 0
        # 初始化速度控制消息
        msg = Twist()

        msg_s = Twist()
        msg_s.angular.z = 0.0
        # 发布速度指令
        self.pub.publish(msg_s)

        # Y轴线速度：横向移动-0.3m/s
        msg.linear.y = -0.25
        # Z轴线速度：置0
        msg.linear.z = 0.0
        # 三个轴的角速度全部置0，仅平移运动
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # 循环发布速度指令，总时长15*0.1=1.5秒
        while(back_time <= 17.5):
            self.pub.publish(msg)
            # 休眠0.1秒，控制发布频率
            rospy.sleep(0.1)
            back_time = back_time + 1
        # X轴线速度：后退0.3m/s
        msg.linear.x = -0.3
        msg.linear.y = 0
        back_time = 0
        while(back_time <= 36):
            self.pub.publish(msg)
            # 休眠0.1秒，控制发布频率
            rospy.sleep(0.1)
            back_time = back_time + 1

        msg.linear.y = 0
        msg.linear.x = 0
        self.pub.publish(msg)
      
    # AR码识别回调函数：核心瞄准逻辑，根据AR码坐标调整机器人姿态，对准后执行射击
    def ar_cb(self, data):
        global id, flog, qr_x, msg, ar_flog, ar_x, point_msg, ar_x_abs, flog4, ar_x_0_abs, ar_y_0, Min_y, Max_y, case
        
        # 获取AR码识别结果数据
        ar_markers = data
        # 遍历所有识别到的AR标签
        for marker in data.markers:
            #print(marker)
            # ---------------------- 状态1：1号靶位瞄准射击 ----------------------
            if marker.id == rotating_id and case == 1 :
                # 获取AR码在机器人坐标系下的X、Y坐标（左右、前后）
                ar_x_0 = marker.pose.pose.position.x
                ar_y_0 = marker.pose.pose.position.y
                # 计算X轴偏移绝对值，判断是否对准
                ar_x_0_abs = abs(ar_x_0)
                
                # 偏移量大于阈值，未对准，调整角速度
                if ar_x_0_abs >= Yaw_th :
                    # 初始化速度消息
                    msg = Twist()
                    # 角速度与X偏移量成反比，实现闭环对准（偏移越大，转得越快）
                    msg.angular.z = max(-0.3, min(0.3, -0.5 * ar_x_0))
                    # 发布速度指令，控制机器人旋转
                    self.pub.publish(msg)
                    print('瞄准中[马了]')
                    print(ar_x_0)
                
                # 对准完成，且Y轴坐标在有效射击范围内，执行射击
                if ar_y_0 <= Max_y and ar_y_0 >= Min_y :
                    # 串口发送射击启动指令（硬件协议指令）
                    #ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print("发射[好枪兄弟]")
                    # 等待射击机构启动
                    rospy.sleep(0.08)
                    # 串口发送射击停止指令
                    #ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                    # 射击后等待2秒，避免机构抖动
                    
                    # 状态机切换到2号靶位状态
                    #case = 2

                    msg_s1 = Twist()
                    msg_s1.angular.z = 0.0
                    # 发布速度指令
                    self.pub.publish(msg_s1)
                    rospy.sleep(2)
                    self.yaw_zero()
                    rospy.sleep(2)
                    print(case)
                    # 导航到3号目标点
                    self.pid_goto(pid_g=3)
                    print('导航到3号目标点')    
                    rospy.sleep(2)
            
            # ---------------------- 状态2：2号靶位瞄准射击 ----------------------
            if marker.id == moving_id and case == 2 :
                # 获取AR码X轴坐标
                ar_x_0 = marker.pose.pose.position.x
                # 计算X轴偏移绝对值
                ar_x_0_abs = abs(ar_x_0)
                
                # 未对准，调整旋转角速度
                if ar_x_0_abs >= Yaw_th :
                    msg_2 = Twist()
                    msg_2.angular.z = max(-0.3, min(0.3, -0.5 * ar_x_0))
                    print(msg_2.angular.z)
                    print(ar_x_0_abs)
                    self.pub.publish(msg_2)
                    print('瞄准中[马了]')

                # 对准完成，执行射击
                elif ar_x_0_abs < Yaw_th :
                    # 串口发送射击启动指令
                    #ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print("发射[好枪兄弟]")
                    rospy.sleep(0.07)
                    # 串口发送射击停止指令
                    #ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                    # 状态机切换到终点状态
                    case = 3   
                    msg_s = Twist()
                    msg_s.angular.z = 0.0
                    # 发布速度指令
                    self.pub.publish(msg_s)
                    rospy.sleep(2)
                    self.yaw_zero()
                    # 导航到3号终点目标点
                    #self.goto(goals[3])
                    rospy.sleep(1)
                    # 执行终点后退动作
                    self.end()
                    #self.goto(goals[3])
                    print('执行终点后退动作')

    # 物体识别回调函数：根据视觉识别的目标坐标，执行对准与射击
    def find_cb(self, data):
        
        global id, flog0, flog1, flog2, count, move_flog, point_msg, case
        # 初始化AR码ID为无效值
        id =255
        # 获取物体识别的坐标结果
        point_msg = data
        # 计算目标在画面X轴与中心(320像素)的偏移量
        flog0 = point_msg.x -320
        # 计算偏移绝对值
        flog1 = abs(flog0)
        '''
        print('target:',point_msg.z)
        print('flog1',flog1)
        print('flog2:',flog2)
        print('case:',case)
        '''

        # ---------------------- 初始状态：物体瞄准射击 ----------------------
        # 偏移量大于0.5像素，目标ID为51，未射击过，状态机为初始状态0
        if abs(flog1) > 2 and point_msg.z == 51 and flog2 >= 255 and case == 0:
            print('瞄准中[马了]')
            # 初始化速度消息
            msg = Twist()
            # 角速度与偏移量成正比，闭环调整机器人朝向，对准目标
            msg.angular.z = max(-0.3, min(0.3, -0.02 * flog0))
            # 发布速度指令
            self.pub.publish(msg)
            rospy.sleep(0.1)
        
        # 对准完成：偏移量小于0.5像素，目标ID正确，未射击过
        elif abs(flog1) <= 2 and point_msg.z == 51 and flog2 >= 255:
            # 串口发送射击启动指令
            #ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
            print("发射[好枪兄弟]")
            rospy.sleep(0.08)
            # 串口发送射击停止指令
            # ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')

            msg_end = Twist()
            msg_end.angular.z = 0.0
            # 发布速度指令
            self.pub.publish(msg_end)

            self.yaw_zero()

            rospy.sleep(3)


            # 射击完成，导航到2号目标点
            #self.goto(goals[1])
            self.pid_goto(pid_g=2)
            print('导航到2号目标点')

            rospy.sleep(4)

            # 射击标志位减1，防止重复射击
            flog2 = flog2 - 1

            #self.goto(goals[2])
            #print('导航到3号目标点')
            #rospy.sleep(4)
            # 状态机切换到1号靶位状态
            #case = 4
            #self.end()
            #print('执行终点后退动作')


    # 设置机器人初始位姿函数：在地图中初始化机器人的位置和朝向
    def set_pose(self, p):
        # 导航客户端未初始化，直接返回
        if self.move_base is None:
            return False
        # 解析输入的位姿参数：x坐标、y坐标、yaw偏航角(角度制)
        x, y, th = p
        # 初始化带协方差的位姿消息
        pose = PoseWithCovarianceStamped()
        # 设置消息时间戳
        pose.header.stamp = rospy.Time.now()
        # 设置坐标系为map地图坐标系
        pose.header.frame_id = 'map'
        # 赋值位置坐标
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        # 欧拉角转四元数：ROS中姿态用四元数表示，输入为弧度制
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        # 赋值四元数姿态
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        # 发布初始位姿消息
        self.set_pose_pub.publish(pose)
        return True



# ---------------------- 程序主入口 ----------------------
if __name__ == "__main__":
    # 初始化ROS节点，节点名navigation_demo，匿名模式避免重名
    rospy.init_node('navigation_demo',anonymous=True)
    
    # 从ROS参数服务器获取导航目标点参数，设置默认值
    # 目标点X坐标列表，逗号分隔
    #goalListX = rospy.get_param('~goalListX', '1.1,1.1,1.1,0.02')
    # 目标点Y坐标列表，逗号分隔
    #goalListY = rospy.get_param('~goalListY', '-0.4, -1.62,-2.85,-3.2')
    # 目标点偏航角列表(角度制)，逗号分隔
    #goalListYaw = rospy.get_param('~goalListYaw', '0, 0, 0, 0')
    
    # 解析参数，拼接成目标点列表，每个元素为[x, y, yaw]
    #goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","),goalListY.split(","),goalListYaw.split(","))]
    
    # 等待用户输入确认，启动程序
    print('输入1开始: ')
    input = raw_input()
    # 打印解析后的目标点列表
    #print(goals)
    
    # 设置循环频率1Hz
    r = rospy.Rate(1)
    r.sleep()


    # 替换为你地图中实际的初始位置（x, y, yaw角度）
    initial_pose = (0.0, 0.0, 0.0)  # 示例：地图原点，朝向0度


    
    
    def publish_audio():
        # 创建发布者对象，发布到 audio_topic 主题，消息类型为 String
        publisher = rospy.Publisher('audio_topic', String, queue_size=10)
        # 消息发布次数
        publish_count = 2
        # 循环直到发布次数达到设定值
        while not rospy.is_shutdown() and publish_count > 0:
            # 创建要发布的字符串消息
            audio_data = "audio message"  # 这里可以替换成您要发布的消息内容
            # 发布消息
            publisher.publish(audio_data)
            # 打印消息发布状态
            rospy.loginfo("Published: %s", audio_data)
            # 减少发布次数
            publish_count -= 1
            # 保持循环频率
            rate = rospy.Rate(1)  # 确保 rate 在循环内部定义，以便每次循环都能更新频率
            rate.sleep()

    def listen_callback(msg):
        global rotating_id, moving_id

        text = msg.data.strip()
        rospy.loginfo("Received voice command: %s", text)

        for i in target_id_rotating_mapping:
            if i in text:
                rotating_id = target_id_rotating_mapping[i]
                rospy.loginfo("Set rotating target ID to: %d", rotating_id)
                break  # 找到匹配后退出循环，避免重复设置
            
        for i in target_id_moving_mapping:
            if i in text:
                moving_id = target_id_moving_mapping[i]
                rospy.loginfo("Set moving target ID to: %d", moving_id)
                break
        print('识别到旋转靶id:'+str(rotating_id)+', 识别到移动靶id:'+str(moving_id))


    # 实例化导航类
    navi = navigation_demo()
    navi.set_pose(initial_pose)
    rospy.sleep(1)  # 等待位姿生效
    if input == '1':
        # 语音识别打击目标
        
        publish_audio()
        rospy.Subscriber("recognized_text", String, listen_callback)
        rospy.sleep(10)
        
        # 打击环形靶
        print('pidgoto')
        navi.pid_goto(pid_g=1)
        rospy.sleep(5)

        # 初始化状态机为0号初始状态
        #case = 0


        # ROS主循环，持续接收回调，直到节点关闭
    while not rospy.is_shutdown():
        r.sleep()