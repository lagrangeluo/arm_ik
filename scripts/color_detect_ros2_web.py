#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
from typing import Tuple, List, Optional
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
from functools import lru_cache
import threading
from concurrent.futures import ThreadPoolExecutor
from http.server import HTTPServer, BaseHTTPRequestHandler
import urllib.parse


# 配置参数
CONFIG = {
    'min_area': 3000,  # 最小面积阈值
    'rect_ratio': 0.85,  # 矩形度阈值
    'resize_width': 640,  # 处理图像的宽度
    'blur_kernel_size': 5,  # 中值滤波核大小
    'morph_kernel': np.ones((5, 5), np.uint8),  # 预计算形态学操作的内核
    'run_mode':'debug',
    'http_server': {
        'host': '127.0.0.1',  # 这个将在类初始化时从参数获取
        'port': 56322,
        'enable': True
    },
    'colors': {
        'red': {
            'ranges': [
                {'lower': np.array([0, 120, 50]), 'upper': np.array([10, 255, 255])},
                {'lower': np.array([170, 120, 50]), 'upper': np.array([180, 255, 255])}
            ],
            'bgr': (0, 0, 255)
        },
        'green': {
            'ranges': [
                {'lower': np.array([35, 120, 50]), 'upper': np.array([85, 255, 255])}
            ],
            'bgr': (0, 255, 0)
        }
    },
    'image_total_area': 640*480,
    'visualization_dir': 'visualization_steps'  # 可视化结果保存目录
}

FONT = cv2.FONT_HERSHEY_SIMPLEX

def preprocess_image(img: np.ndarray) -> Tuple[np.ndarray]:
    """图像预处理"""
    try:
        # 调整图像大小以提高处理速度
        h, w = img.shape[:2]

        # 更新总面积信息
        CONFIG['image_total_area'] = w * h
        # 使用均值滤波代替双边滤波，速度更快
        img_blur = cv2.medianBlur(img, 5)

        return img_blur
    except Exception as e:
        print(f"图像预处理失败: {str(e)}")
        return img

def create_color_mask(hsv_img: np.ndarray, color_ranges: List[dict]) -> np.ndarray:
    """创建颜色掩码，优化版本"""
    # 仅处理一个颜色范围的常见情况
    if len(color_ranges) == 1:
        return cv2.inRange(hsv_img, color_ranges[0]['lower'], color_ranges[0]['upper'])
    
    # 处理多颜色范围的情况
    mask = cv2.inRange(hsv_img, color_ranges[0]['lower'], color_ranges[0]['upper'])
    for range_dict in color_ranges[1:]:
        mask |= cv2.inRange(hsv_img, range_dict['lower'], range_dict['upper'])
    return mask


@lru_cache(maxsize=1024)
def calculate_area_percentage(area: int) -> float:
    """计算色块面积占总图像的百分比，使用缓存提高性能"""
    total_area = CONFIG['image_total_area']
    percentage = (area / total_area) * 100
    return round(percentage, 2)


def process_contours(contours: List[np.ndarray], color_name: str, result_image: np.ndarray) -> Tuple[List[Tuple[int, int]], Optional[float]]:
    """处理轮廓并返回中心点列表和面积"""
    if not contours:  # 快速检查是否有轮廓
        return [], None
        
    centers = []
    color_config = CONFIG['colors'][color_name]
    bgr_color = color_config['bgr']

    # 无需创建副本，直接在结果图像上绘制
    img_height, img_width = result_image.shape[:2]
    img_center_x = img_width // 2
    img_center_y = img_height // 2
    
    max_area = 0
    max_contour_info = None
    
    # 查找最大轮廓
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > CONFIG['min_area'] and area > max_area:
            # 仅对超过阈值的大轮廓进行多边形逼近
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            
            # 只处理矩形（顶点数为4的形状）
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(contour)
                max_area = area
                max_contour_info = (contour, area, x, y, w, h)
    
    # 如果找到符合条件的轮廓
    if max_contour_info:
        contour, area, x, y, w, h = max_contour_info
        
        # 计算面积百分比
        area_percentage = calculate_area_percentage(int(area))
        
        # 计算以图像中心为原点的坐标
        cx = x + w / 2 - img_center_x
        cy = img_center_y - (y + h / 2)  # 注意y轴方向翻转
        centers.append((cx, cy))
        
        # 在原始图像上绘制轮廓 - 比绘制矩形更精确且资源消耗更少
        cv2.drawContours(result_image, [contour], -1, bgr_color, 2)
        
        # 绘制中心点
        screen_cx = x + w // 2
        screen_cy = y + h // 2
        cv2.circle(result_image, (screen_cx, screen_cy), 5, bgr_color, -1)
        
        # 绘制颜色和坐标信息 - 仅绘制必要的文本
        cv2.putText(result_image, f"{color_name}: ({cx}, {cy})", 
                  (screen_cx - 20, screen_cy - 20), FONT, 0.5, bgr_color, 2)
        
        # 添加面积百分比
        cv2.putText(result_image, f"percentage:{area_percentage}", 
                  (10, 35), FONT, 1.5, (0, 0, 255), 2)
                  
        return centers, area
    
    return [], None

def detect_color_blocks(img: np.ndarray, frame_count: int) -> Tuple[dict, np.ndarray]:
    """检测颜色块并返回检测结果"""
    if img is None:
        raise ValueError("输入图像为空")

    try:
        # 图像预处理
        processed_img = preprocess_image(img)

        result_image = img

        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(processed_img, cv2.COLOR_BGR2HSV)

        result_json = {}
        detected_objects = []
        
        # 获取图像中心点
        img_height, img_width = result_image.shape[:2]
        img_center_x = img_width // 2
        img_center_y = img_height // 2
        
        for color_name, color_config in CONFIG['colors'].items():
            # 创建颜色掩码
            mask = create_color_mask(hsv, color_config['ranges'])

            # 形态学操作改善掩码质量
            kernel = CONFIG['morph_kernel']            
            # 将两个操作合并，减少临时数组的创建
            mask = cv2.morphologyEx(
                cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel),
                cv2.MORPH_CLOSE, kernel
            )
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 处理轮廓
            centers, area = process_contours(contours, color_name, result_image)
            
            # 构建检测结果
            if centers:     # 构建 JSON 结果
                for cx, cy in centers:
                    # 计算到图像中心的欧式距离
                    distance_to_center = np.sqrt(cx**2 + cy**2)
                    
                    detected_objects.append({
                        "color": color_name,
                        "position": {"x": int(cx), "y": int(cy)},
                        "area": int(area) if area else 0,
                        "confidence": 0.95,  # 可以添加置信度
                        "distance_to_center": float(distance_to_center)  # 添加距离信息
                    })

        # 按距离排序：距离图像中心越近的越靠前
        if detected_objects:
            detected_objects.sort(key=lambda obj: obj['distance_to_center'])
            # 返回距离最近的对象
            result_json = detected_objects[0]
        else:
            result_json = {}

        return result_json, result_image

    except Exception as e:
        print(f"颜色检测失败: {str(e)}")
        return {}, img

class ColorDetectorHTTPHandler(BaseHTTPRequestHandler):
    """HTTP请求处理器"""
    
    def __init__(self, *args, detector=None, **kwargs):
        self.detector = detector
        print(f"DEBUG: ColorDetectorHTTPHandler.__init__ called, detector = {detector}")
        super().__init__(*args, **kwargs)
    
    def do_POST(self):
        """处理POST请求"""
        try:
            # 解析请求路径
            parsed_path = urllib.parse.urlparse(self.path)
            
            # 检查是否是目标路径
            if parsed_path.path == '/rpc/aimdk.protocol.McActionService/GetColorResult':
                self.handle_get_color_result()
            else:
                self.send_error(404, "Not Found")
                
        except Exception as e:
            self.send_error(500, f"Internal Server Error: {str(e)}")
    
    def handle_get_color_result(self):
        """处理GetColorResult请求"""
        try:
            # 添加调试信息
            print(f"DEBUG: self.detector = {self.detector}")
            print(f"DEBUG: type(self.detector) = {type(self.detector)}")
            
            # 检查detector是否可用
            if not self.detector:
                raise Exception("Detector实例不可用")
            
            # 读取请求体
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length)
            
            # 解析JSON请求体
            request_data = json.loads(post_data.decode('utf-8')) if post_data else {}
            
            # 获取最新的检测结果
            latest_result = self.detector.get_latest_detection_result()
            
            # 构建响应 - 仿照现有接口格式
            response_data = {
                "code": 200,
                "message": "success",
                "data": latest_result if latest_result else None,
                "timestamp": int(time.time() * 1000)  # 毫秒时间戳
            }
            
            # 发送响应
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')
            self.end_headers()
            
            response_json = json.dumps(response_data, ensure_ascii=False)
            self.wfile.write(response_json.encode('utf-8'))
            
        except Exception as e:
            # 错误响应格式
            error_response = {
                "code": 500,
                "message": f"Error: {str(e)}",
                "data": None,
                "timestamp": int(time.time() * 1000)
            }
            
            self.send_response(500)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            error_json = json.dumps(error_response, ensure_ascii=False)
            self.wfile.write(error_json.encode('utf-8'))
    
    def do_OPTIONS(self):
        """处理OPTIONS请求（CORS预检）"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()
    
    def log_message(self, format, *args):
        """重写日志方法，使用ROS2日志"""
        if self.detector:
            self.detector.get_logger().info(f"HTTP: {format % args}")
        else:
            print(f"HTTP: {format % args}")

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detect')

        self.bridge = CvBridge()
        self.frame_count = 0
        
        # 创建发布者
        self.result_pub = self.create_publisher(String, '/detect_result', 10)
        self.image_pub = self.create_publisher(Image, '/camera_right/camera/color/image_detected', 1)

        # 创建订阅者
        self.image_sub = self.create_subscription(
            Image, 
            '/camera_right/camera/color/image_rect_raw', 
            self.image_callback, 
            10
        )

        # 添加关闭标志
        self.is_shutdown = False

         # 添加频率控制
        self.target_period = 0.1  # 10Hz等于每帧0.1秒
        self.last_callback_time = self.get_clock().now()

        # HTTP服务器相关
        self.http_server = None
        self.http_thread = None
        self.latest_detection_result = {}
        
        # 初始化HTTP服务器
        if CONFIG['http_server']['enable']:
            self.init_http_server()

        self.get_logger().info("Color detector node initialized")

    def init_http_server(self):
        """初始化HTTP服务器"""
        try:
            # 在新线程中运行HTTP服务器
            self.http_thread = threading.Thread(target=self.run_http_server, daemon=True)
            self.http_thread.start()
            self.get_logger().info(f"HTTP服务器已启动在 http://{CONFIG['http_server']['host']}:{CONFIG['http_server']['port']}")
        except Exception as e:
            self.get_logger().error(f"HTTP服务器启动失败: {str(e)}")

    def run_http_server(self):
        """在新线程中运行HTTP服务器"""
        try:
            # 保存detector实例的引用
            detector_instance = self
            
            # 创建自定义的请求处理器类
            class CustomHandler(BaseHTTPRequestHandler):
                def __init__(self, *args, **kwargs):
                    # 设置detector实例
                    self.detector = detector_instance
                    print(f"DEBUG: CustomHandler.__init__ called, detector = {self.detector}")
                    super().__init__(*args, **kwargs)
                
                def do_POST(self):
                    """处理POST请求"""
                    try:
                        # 解析请求路径
                        parsed_path = urllib.parse.urlparse(self.path)
                        
                        # 检查是否是目标路径
                        if parsed_path.path == '/rpc/aimdk.protocol.McActionService/GetColorResult':
                            self.handle_get_color_result()
                        else:
                            self.send_error(404, "Not Found")
                            
                    except Exception as e:
                        self.send_error(500, f"Internal Server Error: {str(e)}")
                
                def handle_get_color_result(self):
                    """处理GetColorResult请求"""
                    try:
                        # 添加调试信息
                        print(f"DEBUG: self.detector = {self.detector}")
                        print(f"DEBUG: type(self.detector) = {type(self.detector)}")
                        
                        # 检查detector是否可用
                        if not self.detector:
                            raise Exception("Detector实例不可用")
                        
                        # 读取请求体
                        content_length = int(self.headers.get('Content-Length', 0))
                        post_data = self.rfile.read(content_length)
                        
                        # 解析JSON请求体
                        request_data = json.loads(post_data.decode('utf-8')) if post_data else {}
                        
                        # 获取最新的检测结果
                        latest_result = self.detector.get_latest_detection_result()
                        
                        # 构建响应 - 仿照现有接口格式
                        response_data = {
                            "code": 200,
                            "message": "success",
                            "data": latest_result if latest_result else None,
                            "timestamp": int(time.time() * 1000)  # 毫秒时间戳
                        }
                        
                        # 发送响应
                        self.send_response(200)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
                        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                        self.end_headers()
                        
                        response_json = json.dumps(response_data, ensure_ascii=False)
                        self.wfile.write(response_json.encode('utf-8'))
                        
                    except Exception as e:
                        # 错误响应格式
                        error_response = {
                            "code": 500,
                            "message": f"Error: {str(e)}",
                            "data": None,
                            "timestamp": int(time.time() * 1000)
                        }
                        
                        self.send_response(500)
                        self.send_header('Content-Type', 'application/json')
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.end_headers()
                        
                        error_json = json.dumps(error_response, ensure_ascii=False)
                        self.wfile.write(error_json.encode('utf-8'))
                
                def do_OPTIONS(self):
                    """处理OPTIONS请求（CORS预检）"""
                    self.send_response(200)
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Access-Control-Allow-Methods', 'POST, OPTIONS')
                    self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                    self.end_headers()
                
                def log_message(self, format, *args):
                    """重写日志方法，使用ROS2日志"""
                    if self.detector:
                        self.detector.get_logger().info(f"HTTP: {format % args}")
                    else:
                        print(f"HTTP: {format % args}")
            
            # 创建HTTP服务器
            self.http_server = HTTPServer(
                (CONFIG['http_server']['host'], CONFIG['http_server']['port']), 
                CustomHandler
            )
            
            self.get_logger().info(f"HTTP服务器正在监听 {CONFIG['http_server']['host']}:{CONFIG['http_server']['port']}")
            self.http_server.serve_forever()
        except Exception as e:
            self.get_logger().error(f"HTTP服务器运行错误: {str(e)}")
        finally:
            # 关闭HTTP服务器
            if self.http_server:
                self.http_server.server_close()
                self.get_logger().info("HTTP服务器已关闭")

    def image_callback(self, data):
        """处理图像回调，控制为10Hz频率"""
        # 计算自上次处理以来的时间
        now = self.get_clock().now()
        time_since_last = (now - self.last_callback_time).nanoseconds / 1e9
        
        # 如果距离上次处理的时间不足目标周期，则跳过这一帧
        if time_since_last < self.target_period:
            return
        
        try:
            # 更新上次处理时间
            self.last_callback_time = now
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 检测颜色块
            result_json, result_image = detect_color_blocks(cv_image, self.frame_count)
            
            # 只在有结果时发布消息
            if result_json:
                # 发布检测结果 - 预先序列化JSON
                result_msg = String()
                result_msg.data = json.dumps(result_json)
                self.result_pub.publish(result_msg)
                
                # 发布标注后的图像 - 避免不必要的图像转换
                if self.image_pub.get_subscription_count() > 0:
                    ros_image = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
                    self.image_pub.publish(ros_image)
            
            # 更新最新的检测结果
            self.latest_detection_result = result_json
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def shutdown(self):
        """安全关闭节点"""
        self.is_shutdown = True
        self.get_logger().info("正在安全关闭检测器...")
        
        # 关闭HTTP服务器
        if self.http_server:
            try:
                self.http_server.shutdown()
                if self.http_thread and self.http_thread.is_alive():
                    self.http_thread.join(timeout=3)
                self.get_logger().info("HTTP服务器已关闭")
            except Exception as e:
                self.get_logger().error(f"关闭HTTP服务器时出错: {str(e)}")
        
        # 关闭所有OpenCV窗口
        cv2.destroyAllWindows()
        
        self.get_logger().info("检测器已安全关闭")

    def get_latest_detection_result(self):
        """获取最新的检测结果"""
        return self.latest_detection_result


def main(args=None):
    rclpy.init(args=args)
    
    try:
        detector = ColorDetector()
        detector.get_logger().info("色块检测节点已启动，按Ctrl+C可安全退出")
        
        # 使用rclpy.spin()替代自定义循环
        rclpy.spin(detector)
        
    except KeyboardInterrupt:
        detector.get_logger().info("接收到键盘中断")
    except Exception as e:
        detector.get_logger().error(f"意外错误: {str(e)}")
    finally:
        # 清理资源
        detector.shutdown()
        detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

