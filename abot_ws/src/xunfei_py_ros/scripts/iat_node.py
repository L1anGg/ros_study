#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 指明Python解释器路径 + 文件编码为UTF-8，适配Python2.7

# 导入ROS核心库
import rospy
# 导入音频采集库，用于获取麦克风数据
import pyaudio
# 导入WebSocket库，用于和科大讯飞服务器建立长连接
import websocket
# 导入线程库，实现录音与数据发送的异步处理
import threading
# 导入时间库，用于时间戳、延时操作
import time
# 导入JSON库，解析讯飞服务器返回的识别结果
import json
# 导入哈希加密库，用于鉴权签名计算
import hashlib
# 导入Base64编码库，音频数据与签名需要Base64编码
import base64
# 导入HMAC加密库，科大讯飞鉴权核心加密算法
import hmac
# 导入日期时间库，生成鉴权所需的GMT时间
import datetime
# 导入Python2的URL处理库，用于参数编码
import urllib
# 导入ROS字符串消息类型，用于发布语音识别结果
from std_msgs.msg import String

import os


music_path = "/home/l1ang/abot_ws/src/robot_slam/scripts/比赛开始.mp3"
music1_path = "/home/l1ang/abot_ws/src/robot_slam/scripts/提示音.mp3"
model_path = "/home/l1ang/abot_ws/src/robot_slam/scripts/paraformer-zh"

# ===================== 科大讯飞接口核心配置（已填写你的有效参数）=====================
# 讯飞开放平台应用ID
APPID = "14fcfcdc"
# 讯飞开放平台API Key
APIKey = "133e7b41728ff8884027b0e72e78b15f"
# 讯飞开放平台API Secret
APISecret = "YjU5ZWU4YTllMDY0ZWZkMzJmZTZhOWRm"

# ===================== 音频采集参数（讯飞官方要求固定配置）=====================
# 音频格式：16位整型
FORMAT = pyaudio.paInt16
# 音频通道数：单声道
CHANNELS = 1
# 采样率：16000Hz（讯飞语音听写必填）
RATE = 16000
# 每次读取的音频帧数
CHUNK = 1280

# 全局变量：ROS识别结果发布器
result_pub = None

# ===================== Python2.7 编码修复（必须加，否则中文报错）=====================
import sys
reload(sys)
sys.setdefaultencoding('utf8')

def audio_callback(msg):
    """收到音频触发消息时，开始一次语音识别"""
    rospy.loginfo("🔔 收到触发消息，开始语音识别...")
    
    # 🔥 只有收到消息时才调用识别方法
    try:
        iat_recognizer.start()
    except Exception as e:
        rospy.logerr("识别失败")


# ===================== 科大讯飞语音识别核心类 =====================
class XunfeiIAT:
    """
    科大讯飞流式语音识别（IAT）封装类
    功能：麦克风录音 → WebSocket发送音频 → 接收识别结果 → 发布ROS话题
    """
    def __init__(self):
        """类初始化函数，配置讯飞识别基础参数"""
        # 公共参数：传入APPID
        self.common_args = {"app_id": APPID}
        # 业务参数：配置识别领域、语言、口音、静音断连时间
        self.business_args = {
            "domain": "iat",          # 识别领域：iat=语音听写
            "language": "zh_cn",      # 识别语言：中文
            "accent": "mandarin",     # 口音：普通话
            "vad_eos": 1000           # 静音超时：1秒无声音自动结束识别
        }
        # 录音状态标记：False=未录音，True=正在录音
        self.is_recording = False

    def create_url(self):
        """
        旧版鉴权URL生成函数（当前代码未使用，保留兼容）
        功能：生成旧版接口的鉴权URL，包含时间戳、签名、URL安全处理
        """
        # 生成Unix时间戳
        ts = str(int(time.time()))
        # MD5加密：APIKey + 时间戳
        base_str = hashlib.md5(APIKey + ts).hexdigest()
        # HMAC-SHA1加密生成签名
        signa = hmac.new(APISecret, base_str, hashlib.sha1).digest()
        # Base64编码签名
        signa = base64.b64encode(signa)
        # 签名转URL安全格式：替换+、/，去除=
        signa = signa.replace('+', '-').replace('/', '_').rstrip('=')
        # 对签名做URL编码
        signa = urllib.quote(signa)
        # 拼接最终URL
        url = "wss://ws-api.xfyun.cn/v2/iat"
        url += "?appid=" + APPID
        url += "&api_key=" + APIKey
        url += "&x_date=" + ts
        url += "&authorization=" + signa
        return url

    def on_message(self, ws, message):
        
        """
        WebSocket消息接收回调函数
        功能：接收讯飞服务器返回的识别结果，解析并发布ROS话题
        :param ws: WebSocket连接对象
        :param message: 服务器返回的JSON字符串
        """
        try:
            # 将JSON字符串转为字典
            msg = json.loads(message)
            # 判断识别是否成功（code=0为成功）
            if msg.get("code") != 0:
                rospy.logerr("识别错误：错误码 %d，信息：%s", msg.get("code"), msg.get("message"))
                return

            # 提取识别数据体
            data = msg["data"]
            # 提取识别结果
            result = data["result"]
            text = ""
            # 遍历识别结果片段，拼接完整文本
            for item in result["ws"]:
                text += item["cw"][0]["w"]

            # status=2 表示识别结束，输出最终结果
            if data["status"] == 2:
                rospy.loginfo("✅ 识别结果：%s", text)
                # 将识别结果发布到ROS话题 /recognized_text
                result_pub.publish(String(text))
                # 结束录音状态
                self.is_recording = False

        except Exception as e:
            # 解析结果异常处理
            rospy.logerr("解析结果失败：%s", str(e))

    def on_error(self, ws, error):
        """WebSocket连接错误回调函数"""
        rospy.logerr("连接错误：%s", str(error))

    def on_close(self, ws, *args):
        """WebSocket连接关闭回调函数"""
        rospy.loginfo("语音识别连接已关闭")

    def on_open(self, ws):
        
        """
        WebSocket连接成功回调函数
        功能：连接成功后，开启子线程进行录音并发送音频数据
        :param ws: WebSocket连接对象
        """
        def run():
            # 第一步：发送识别初始化帧（告知服务器开始识别）
            ws.send(json.dumps({
                "common": self.common_args,
                "business": self.business_args,
                "data": {"status": 0, "format": "audio/L16;rate=16000", "encoding": "raw"}
            }))

            
            # 初始化PyAudio，打开麦克风
            p = pyaudio.PyAudio()
            stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
            os.system('mplayer %s' % music_path)
            rospy.loginfo("🎤 请开始说话...")
            
            # 标记正在录音
            self.is_recording = True

            # 循环采集音频并发送
            while self.is_recording and not rospy.is_shutdown():
                # 读取一帧音频数据
                audio_data = stream.read(CHUNK)
                # 第二步：发送音频数据帧（status=1 表示中间音频帧）
                ws.send(json.dumps({
                    "data": {
                        "status": 1,
                        "format": "audio/L16;rate=16000",
                        "encoding": "raw",
                        "audio": base64.b64encode(audio_data)
                    }
                }))

            # 第三步：发送结束帧（告知服务器识别结束）
            ws.send(json.dumps({"data": {"status": 2, "format": "audio/L16;rate=16000", "encoding": "raw", "audio": ""}}))
            os.system('mplayer %s' % music1_path)
            
            # 关闭音频流与PyAudio
            stream.stop_stream()
            stream.close()
            p.terminate()

        

            

        # 开启子线程执行录音发送，不阻塞主线程
        threading.Thread(target=run).start()

    def get_headers(self):
        """
        新版鉴权请求头生成函数（当前代码实际使用的有效鉴权）
        功能：按照讯飞2023后新接口规范，生成HMAC-SHA256鉴权请求头
        :return: 鉴权请求头字典
        """
        # 生成GMT格式时间（鉴权必填）
        date = datetime.datetime.utcnow().strftime('%a, %d %b %Y %H:%M:%S GMT')
        # 拼接签名原文
        signature_origin = ("host: ws-api.xfyun.cn\n"
                            "date: " + date + "\n"
                            "GET /v2/iat HTTP/1.1")
        # HMAC-SHA256加密生成签名
        signature_sha = hmac.new(
            APISecret.encode('utf-8'),
            signature_origin.encode('utf-8'),
            digestmod=hashlib.sha256
        ).digest()
        # Base64编码签名
        signature = base64.b64encode(signature_sha).decode('utf-8')
        # 拼接Authorization鉴权字段
        authorization = (
            'api_key="{}", algorithm="hmac-sha256", '
            'headers="host date request-line", signature="{}"'
        ).format(APIKey, signature)
        # 返回完整请求头
        return {
            "Authorization": authorization,
            "Date": date,
            "Host": "ws-api.xfyun.cn"
        }

    def start(self):
        """启动语音识别：生成鉴权头 → 建立WebSocket连接"""
        # 获取鉴权请求头
        headers = self.get_headers()
        # 拼接WebSocket连接URL
        url = "wss://ws-api.xfyun.cn/v2/iat?appid=" + APPID
        # 创建WebSocket客户端，绑定回调函数
        ws = websocket.WebSocketApp(
            url,
            header=headers,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        # 绑定连接成功回调
        ws.on_open = self.on_open
        # 启动WebSocket长连接
        ws.run_forever()

# ===================== ROS节点主函数 =====================
def main():
    """ROS节点入口函数"""
    global result_pub, iat_recognizer
    # 初始化ROS节点，节点名：xunfei_voice_recognizer
    rospy.init_node('xunfei_voice_recognizer')
    # 创建ROS发布器：话题名/recognized_text，消息类型String，队列长度10
    result_pub = rospy.Publisher('/recognized_text', String, queue_size=10)
    # 打印启动日志
    rospy.loginfo("✅ 科大讯飞语音识别节点启动成功")


    # 🔥 实例化识别对象
    iat_recognizer = XunfeiIAT()
    
    
    rospy.Subscriber("/audio_topic", String, audio_callback)
    # 实例化识别类并启动
    #XunfeiIAT().start()
    # ROS阻塞循环，保持节点运行
    rospy.spin()

# ===================== 程序入口 =====================
if __name__ == '__main__':
    try:
        # 执行主函数
        main()
    except rospy.ROSInterruptException:
        # 处理ROS关闭异常
        pass