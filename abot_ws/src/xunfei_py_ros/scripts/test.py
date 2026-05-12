#!/usr/bin/env python
# -*- coding: utf-8 -*-
import hashlib
import hmac
import base64
import time
import websocket
import json
import datetime

# 你的正确参数
APPID = "14fcfcdc"
APIKey = "133e7b41728ff8884027b0e72e78b15f"
APISecret = "YjU5ZWU4YTllMDY0ZWZkMzJmZTZhOWRm"

def create_url():
    # 🔴 新算法1：生成RFC1123格式的GMT时间（不是Unix时间戳！）
    date = datetime.datetime.utcnow().strftime("%a, %d %b %Y %H:%M:%S GMT")
    print "当前GMT时间:", date
    
    # 🔴 新算法2：拼接签名原文
    signature_origin = "date: " + date
    
    # 🔴 新算法3：HMAC-SHA256加密（不是SHA1！）
    signature_sha = hmac.new(APISecret, signature_origin, hashlib.sha256).digest()
    signature = base64.b64encode(signature_sha)
    print "签名结果:", signature
    
    # 🔴 新算法4：生成标准Authorization头
    authorization = 'api_key="%s", algorithm="hmac-sha256", headers="date", signature="%s"' % (
        APIKey, signature
    )
    print "Authorization头:", authorization
    
    # 拼接最终URL
    url = "wss://ws-api.xfyun.cn/v2/iat?appid=%s" % APPID
    return url, date, authorization

def on_message(ws, message):
    print "✅ 收到服务器响应:", message
    ws.close()

def on_error(ws, error):
    print "❌ 错误:", error

def on_close(ws, *args):
    print "连接关闭"

def on_open(ws):
    print "✅ 鉴权成功！已连接到科大讯飞服务器"
    # 发送测试帧
    ws.send(json.dumps({
        "common": {"app_id": APPID},
        "business": {"domain": "iat", "language": "zh_cn", "accent": "mandarin"},
        "data": {"status": 0, "format": "audio/L16;rate=16000", "encoding": "raw"}
    }))

if __name__ == "__main__":
    print "正在测试科大讯飞最新鉴权算法..."
    url, date, authorization = create_url()
    
    # 🔴 新算法5：在WebSocket请求头中携带鉴权信息
    headers = [
        "Date: %s" % date,
        "Authorization: %s" % authorization
    ]
    
    ws = websocket.WebSocketApp(
        url,
        header=headers,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close
    )
    ws.on_open = on_open
    ws.run_forever()