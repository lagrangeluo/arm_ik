#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试视觉检测HTTP API的脚本
仿照现有接口格式: http://127.0.0.1:56322/rpc/aimdk.protocol.McActionService/GetColorResult
"""

import requests
import json
import time

def test_get_action():
    """测试GetColorResult API - 仿照现有接口格式"""
    url = 'http://127.0.0.1:56322/rpc/aimdk.protocol.McActionService/GetColorResult'
    headers = {'Content-Type': 'application/json'}
    
    # 请求体（可以为空，但保持格式一致）
    request_data = {}
    
    try:
        print(f"正在测试视觉检测API: {url}")
        print(f"请求头: {headers}")
        print(f"请求体: {json.dumps(request_data, ensure_ascii=False)}")
        print("-" * 50)
        
        response = requests.post(url, headers=headers, json=request_data)
        
        print(f"响应状态码: {response.status_code}")
        print(f"响应头: {dict(response.headers)}")
        print("-" * 50)
        
        if response.status_code == 200:
            result = response.json()
            print("响应内容:")
            print(json.dumps(result, indent=2, ensure_ascii=False))
            
            # 解析响应数据
            if result.get('code') == 200:
                detection_data = result.get('data')
                if detection_data:
                    print(f"\n检测结果解析:")
                    print(f"  颜色: {detection_data.get('color', 'N/A')}")
                    print(f"  位置: x={detection_data.get('position', {}).get('x', 'N/A')}, y={detection_data.get('position', {}).get('y', 'N/A')}")
                    print(f"  面积: {detection_data.get('area', 'N/A')}")
                    print(f"  到图像中心距离: {detection_data.get('distance_to_center', 'N/A'):.2f} 像素")
                    print(f"  置信度: {detection_data.get('confidence', 'N/A')}")
                else:
                    print("  当前没有检测到颜色块")
            else:
                print(f"  错误: {result.get('message', 'Unknown error')}")
        else:
            print(f"错误响应: {response.text}")
            
    except requests.exceptions.ConnectionError:
        print("连接失败：请确保视觉检测HTTP服务器正在运行")
        print("启动命令: python3 scripts/color_detect_ros2_web.py")
    except Exception as e:
        print(f"请求失败: {str(e)}")

def test_cors():
    """测试CORS支持"""
    url = 'http://127.0.0.1:56322/rpc/aimdk.protocol.McActionService/GetColorResult'
    headers = {
        'Content-Type': 'application/json',
        'Origin': 'http://localhost:3000'
    }
    
    try:
        print("\n正在测试CORS支持...")
        response = requests.options(url, headers=headers)
        
        print(f"OPTIONS状态码: {response.status_code}")
        print(f"CORS头: {dict(response.headers)}")
        
    except Exception as e:
        print(f"CORS测试失败: {str(e)}")

def show_usage_example():
    """显示使用示例"""
    print("\n" + "="*60)
    print("使用示例:")
    print("="*60)
    print("""
# Python客户端调用示例:
import requests

url = 'http://127.0.0.1:56322/rpc/aimdk.protocol.McActionService/GetColorResult'
headers = {'Content-Type': 'application/json'}
response = requests.post(url, headers=headers, json={})

if response.status_code == 200:
    result = response.json()
    if result['code'] == 200:
        detection_data = result['data']
        if detection_data:
            color = detection_data['color']
            x = detection_data['position']['x']
            y = detection_data['position']['y']
            area = detection_data['area']
            distance = detection_data['distance_to_center']
            confidence = detection_data['confidence']
            print(f"检测到{color}色块，位置({x}, {y})，面积{area}，距离中心{distance:.2f}像素，置信度{confidence}")
        else:
            print("未检测到颜色块")
    else:
        print(f"错误: {result['message']}")

# 响应格式:
{
  "code": 200,
  "message": "success", 
  "data": {
    "color": "red",
    "position": {"x": 100, "y": 200},
    "area": 5000,
    "confidence": 0.95,
    "distance_to_center": 223.61
  },
  "timestamp": 1234567890123
}
""")

if __name__ == "__main__":
    print("视觉检测HTTP API测试")
    print("="*60)
    
    # 测试CORS
    test_cors()
    
    # 测试GetColorResult API
    test_get_action()
    
    # 显示使用示例
    #show_usage_example()
    
    print("\n测试完成！") 