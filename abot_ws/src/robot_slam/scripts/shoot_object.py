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
import serial
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
ser = serial.Serial(port=serialPort, baudrate=baudRate, parity="N", bytesize=8, stopbits=1)
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
# ---------------------- Python2编码与系统配置 ----------------------
import sys
# 重载sys模块，设置Python2默认编码为UTF-8，解决中文乱码问题
reload(sys)
sys.setdefaultencoding('utf-8')
import os

# ---------------------- 全局变量定义 ----------------------
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
Yaw_th = 0.0040
# AR码Y轴坐标有效范围下限
Min_y = -0.36
# AR码Y轴坐标有效范围上限
Max_y = -0.30
# AR码识别状态标志
ar_flog=255
# ---------------------- 核心状态机变量 ----------------------
# 机器人整体运行状态机：0=初始瞄准阶段 1=1号靶位瞄准 2=2号靶位瞄准 3=终点阶段
case = 255
# 预留状态机子变量
case1 = 255
case2 = 255
case3 = 255

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
    
    # 终点后退函数：任务完成后，控制机器人匀速后退指定时长
    def end(self):
        global back_time
        # 初始化速度控制消息
        msg = Twist()
        # X轴线速度：后退0.3m/s
        msg.linear.x = -0.3
        # Y轴线速度：横向移动-0.3m/s
        msg.linear.y = -0.3
        # Z轴线速度：置0
        msg.linear.z = 0.0
        # 三个轴的角速度全部置0，仅平移运动
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        # 循环发布速度指令，总时长15*0.1=1.5秒
        while(back_time <= 15):
            self.pub.publish(msg)
            # 休眠0.1秒，控制发布频率
            rospy.sleep(0.1)
            back_time = back_time + 1
            
    # AR码识别回调函数：核心瞄准逻辑，根据AR码坐标调整机器人姿态，对准后执行射击
    def ar_cb(self, data):
        global id, flog, qr_x, msg, ar_flog, ar_x, point_msg, ar_x_abs, flog4, ar_x_0_abs, ar_y_0, Min_y, Max_y, case
        
        # 获取AR码识别结果数据
        ar_markers = data
        # 遍历所有识别到的AR标签
        for marker in data.markers:
            # ---------------------- 状态1：1号靶位瞄准射击 ----------------------
            if marker.id ==0 and case == 1 :
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
                    msg.angular.z = -1 * ar_x_0
                    # 发布速度指令，控制机器人旋转
                    self.pub.publish(msg)
                
                # 对准完成，且Y轴坐标在有效射击范围内，执行射击
                if ar_y_0 <= Max_y and ar_y_0 >= Min_y :
                    # 串口发送射击启动指令（硬件协议指令）
                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print("shoot")
                    # 等待射击机构启动
                    rospy.sleep(0.08)
                    # 串口发送射击停止指令
                    ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                    # 射击后等待2秒，避免机构抖动
                    rospy.sleep(2)
                    # 状态机切换到2号靶位状态
                    case = 2
                    print(case)
                    # 导航到2号目标点
                    self.goto(goals[2])    
                    rospy.sleep(2)
            
            # ---------------------- 状态2：2号靶位瞄准射击 ----------------------
            if marker.id ==0 and case == 2 :
                # 获取AR码X轴坐标
                ar_x_0 = marker.pose.pose.position.x
                # 计算X轴偏移绝对值
                ar_x_0_abs = abs(ar_x_0)
                
                # 未对准，调整旋转角速度
                if ar_x_0_abs >= Yaw_th :
                    msg = Twist()
                    msg.angular.z = -1 * ar_x_0
                    self.pub.publish(msg)
                # 对准完成，执行射击
                elif ar_x_0_abs < Yaw_th :
                    # 串口发送射击启动指令
                    ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
                    print("shoot")
                    rospy.sleep(0.07)
                    # 串口发送射击停止指令
                    ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
                    # 状态机切换到终点状态
                    case = 3   
                    # 导航到3号终点目标点
                    self.goto(goals[3])
                    rospy.sleep(2)
                    # 执行终点后退动作
                    self.end()
    
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

        # ---------------------- 初始状态：物体瞄准射击 ----------------------
        # 偏移量大于0.5像素，目标ID为34，未射击过，状态机为初始状态0
        if abs(flog1) > 0.5 and point_msg.z == 34 and flog2 >= 255 and case == 0:
            # 初始化速度消息
            msg = Twist()
            # 角速度与偏移量成正比，闭环调整机器人朝向，对准目标
            msg.angular.z = -0.01 * flog0
            # 发布速度指令
            self.pub.publish(msg)
            print(flog0 * 0.01)
        
        # 对准完成：偏移量小于0.5像素，目标ID正确，未射击过
        elif abs(flog1) <= 0.5 and point_msg.z == 34 and flog2 >= 255:
            # 串口发送射击启动指令
            ser.write(b'\x55\x01\x12\x00\x00\x00\x01\x69')
            print("shoot")
            rospy.sleep(0.08)
            # 串口发送射击停止指令
            ser.write(b'\x55\x01\x11\x00\x00\x00\x01\x68')
            # 射击完成，导航到1号目标点
            self.goto(goals[1])
            rospy.sleep(2)
            # 状态机切换到1号靶位状态
            case = 1
            # 射击标志位减1，防止重复射击
            flog2 = flog2 - 1

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

    # ---------------------- move_base动作回调函数 ----------------------
    # 导航完成回调：导航结束时触发，打印导航状态与结果
    def _done_cb(self, status, result):
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))
        # 拼接到达目标点的播报字符串
        arrive_str = "arrived to traget point"
        # 发布语音播报消息
        self.arrive_pub.publish(arrive_str)

    # 导航激活回调：导航任务开始执行时触发
    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    # 导航反馈回调：导航过程中持续触发，返回实时导航状态
    def _feedback_cb(self, feedback):
        msg = feedback
        # 注释掉的日志，调试时可打开查看实时反馈
        #rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    # ---------------------- 核心导航函数 ----------------------
    # 导航到指定目标点，封装move_base动作调用全流程
    def goto(self, p):
        rospy.loginfo("[Navi] goto %s"%p)
        # 初始化导航目标消息
        goal = MoveBaseGoal()
        # 设置目标点坐标系为map地图坐标系
        goal.target_pose.header.frame_id = 'map'
        # 设置消息时间戳
        goal.target_pose.header.stamp = rospy.Time.now()
        # 赋值目标点X、Y坐标
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        # 欧拉角(角度制)转四元数，赋值目标点朝向
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        
        # 发送导航目标，绑定三个回调函数
        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        # 等待导航结果，超时时间60秒
        result = self.move_base.wait_for_result(rospy.Duration(60))
        
        # 超时未到达，取消导航任务
        if not result:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        # 导航完成，判断是否成功到达
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("reach goal %s succeeded!"%p)
        return True

    # 取消所有导航任务函数
    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

# ---------------------- 程序主入口 ----------------------
if __name__ == "__main__":
    # 初始化ROS节点，节点名navigation_demo，匿名模式避免重名
    rospy.init_node('navigation_demo',anonymous=True)
    
    # 从ROS参数服务器获取导航目标点参数，设置默认值
    # 目标点X坐标列表，逗号分隔
    goalListX = rospy.get_param('~goalListX', '2.0, 2.0,2.0')
    # 目标点Y坐标列表，逗号分隔
    goalListY = rospy.get_param('~goalListY', '2.0, 4.0,2.0')
    # 目标点偏航角列表(角度制)，逗号分隔
    goalListYaw = rospy.get_param('~goalListYaw', '0, 90.0,2.0')
    
    # 解析参数，拼接成目标点列表，每个元素为[x, y, yaw]
    goals = [[float(x), float(y), float(yaw)] for (x, y, yaw) in zip(goalListX.split(","),goalListY.split(","),goalListYaw.split(","))]
    
    # 等待用户输入确认，启动程序
    print('Please 1 to continue: ')
    input = raw_input()
    # 打印解析后的目标点列表
    print(goals)
    
    # 设置循环频率1Hz
    r = rospy.Rate(1)
    r.sleep()
    
    # 实例化导航类
    navi = navigation_demo()
    # 第一步：导航到0号初始目标点
    navi.goto(goals[0])
    rospy.sleep(2)
    # 初始化状态机为0号初始状态
    case = 0   
    
    # ROS主循环，持续接收回调，直到节点关闭
    while not rospy.is_shutdown():
        r.sleep()
