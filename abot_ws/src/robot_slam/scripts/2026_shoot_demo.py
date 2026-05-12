#!/home/l1ang/miniconda3/envs/py39/bin/python 
# -*- coding: utf-8 -*-

'''
识别语音内容，并将音频内容发布到chinese_topic话题
'''



import roslibpy
import pyaudio
import wave
import os
from funasr import AutoModel
import soundfile
import time as t

# 配置路径（根据实际情况修改）
music_path = "/home/l1ang/abot_ws/src/robot_slam/scripts/比赛开始.mp3"
music1_path = "/home/l1ang/abot_ws/src/robot_slam/scripts/提示音.mp3"
model_path = "/home/l1ang/abot_ws/src/robot_slam/scripts/paraformer-zh"
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

# 初始化 ROS 连接（替代 rospy.init_node）
ros = roslibpy.Ros(host='localhost', port=9090)
pub1 = roslibpy.Topic(ros, '/chinese_topic', 'std_msgs/String')

def audio_callback(message):
    print("Received audio message, starting recording and recognition")
    start_audio()
    print("Recording and recognition completed")

def start_audio(time=5, save_file="test1.wav"):
    global model
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 16000
    RECORD_SECONDS = time
    WAVE_OUTPUT_FILENAME = save_file

    p = pyaudio.PyAudio()
    print("ON")
    os.system('mplayer %s' % music_path)
    
    if os.path.exists(save_file):
        os.remove(save_file)

    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    frames_per_buffer=CHUNK)
    frames = []
    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)
    print("OFF")
    os.system('mplayer %s' % music1_path)

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

    print("Starting recognition")
    res = model.generate(input='/home/l1ang/abot_ws/src/robot_slam/scripts/test1.wav')
    result = res[0].get('text', '默认值')
    print("Recognition result:", result)

    # 发布识别结果到 ROS 话题
    print("Publishing message:", result)
    pub1.publish(roslibpy.Message({'data': result}))
    t.sleep(0.5)  # 确保消息发送完成

def audio_subscriber():
    # 订阅 ROS 话题（替代 rospy.Subscriber）
    subscriber = roslibpy.Topic(ros, '/audio_topic', 'std_msgs/String')
    subscriber.subscribe(audio_callback)
    
    print("Audio subscriber node started, waiting for messages...")
    ros.run_forever()  # 替代 rospy.spin()

if __name__ == '__main__':
    # 初始化语音识别模型
    model = AutoModel(model=model_path, disable_update=True)
    '''
    model = AutoModel(
        model="iic/speech_paraformer-large_asr_nat-zh-cn-16k-common-vocab8404-pytorch",
        disable_update=True,
        trust_remote_code=False,
        device="cpu"
    )
    '''
    # 连接 ROS 主节点并启动订阅
    try:
        #ros.run()  # 先连接 ROS
        audio_subscriber()
    except KeyboardInterrupt:
        print("Shutting down...")
        pub1.unadvertise()
        ros.terminate()