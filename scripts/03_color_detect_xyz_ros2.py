"""
模块化绿色色块检测和机械臂控制系统
优化版本 - 提高可读性、改进状态机逻辑、删除时间检查
"""

import pyrealsense2 as rs
import numpy as np
import cv2
import time
from typing import Optional, Tuple, List
from enum import Enum
from dobot_api import DobotApiFeedBack, DobotApiDashboard
import rclpy
from rclpy.node import Node
from rm_interfaces.msg import GripperCommand
from std_srvs.srv import Trigger
import logging

from logging_config import setup_development_logging, get_logger


# ==================== 常量定义 ====================
class Constants:
    """系统常量定义"""
    
    # 相机配置
    CAMERA_WIDTH = 848
    CAMERA_HEIGHT = 480
    CAMERA_FPS = 30
    
    # 绿色HSV范围
    GREEN_LOWER = np.array([48, 80, 50])
    GREEN_UPPER = np.array([80, 255, 255])
    
    # 图像处理参数
    MIN_CONTOUR_AREA = 800
    MORPHOLOGY_KERNEL_SIZE = 5
    
    # 运动控制参数
    PIXEL_THRESHOLD = 1  # 像素对准阈值，1px
    STABLE_THRESHOLD = 3  # 稳定帧数阈值，稳定3帧才移动
    MOTION_SCALE = 1.0   # 运动缩放因子，没用到
    
    # TCP偏移参数
    TCP_OFFSET_Y = 0.05  # 5cm摄像头到末端中心偏移
    
    # 机械臂参数
    ROBOT_IP = "192.168.5.1"
    DASHBOARD_PORT = 29999
    FEED_PORT = 30004
    USER_COORD = 0
    TOOL_COORD = 1
    
    # 距离参数（毫米）
    TARGET_DEPTH_MM = 90    #摄像头到夹爪尖端的距离
    PICKUP_HEIGHT_MM = 100  #夹取后往后拉的距离
    ROTATION_ANGLE = 90     #旋转角度
    
    # 夹爪参数
    GRIPPER_CLOSE_POSITION = 70.0
    GRIPPER_SPEED = 30.0
    GRIPPER_FORCE = 50.0
    GRIPPER_CLOSE_WAIT = 3.0
    GRIPPER_OPEN_WAIT = 1.0


class TaskState(Enum):
    """任务状态枚举"""
    DETECTING = "detecting"           # 检测绿色色块
    ALIGNING_XZ = "aligning_xz"      # XZ方向对准
    MOVING_Y = "moving_y"            # Y方向移动到目标深度
    PICKING = "picking"              # 执行抓取序列
    COMPLETED = "completed"          # 任务完成
    RESETTING = "resetting"          # 重置状态


# ==================== 色块检测模块 ====================
class ColorBlockDetector:
    """
    绿色色块检测模块
    
    功能：
    - 初始化RealSense相机
    - 检测绿色色块并计算位置
    - 提供像素坐标到机械臂坐标的转换
    - 判断目标稳定性
    """
    
    def __init__(self) -> None:
        """初始化色块检测器"""
        self.logger = logging.getLogger(__name__ + '.ColorBlockDetector')
        
        # 相机相关属性
        self.pipeline: Optional[rs.pipeline] = None
        self.config: Optional[rs.config] = None
        self.profile: Optional[rs.profile] = None
        self.intrinsics: Optional[rs.intrinsics] = None
        self.depth_scale: Optional[float] = None
        self.align: Optional[rs.align] = None
        
        # 图像中心参数（考虑TCP偏移）
        self.img_center_x: Optional[float] = None
        self.img_center_y: Optional[float] = None
        self.pixel_offset_y: float = 0.0
        
        # 相机坐标系到机械臂坐标系的转换矩阵
        self.camera_to_robot_rotation = np.array([
            [-1,  0,  0],  # 相机X -> 机械臂-X
            [ 0,  0, -1],  # 相机Y -> 机械臂-Z  
            [ 0, -1,  0]   # 相机Z -> 机械臂-Y
        ])
        
        # 目标稳定性检测参数
        self.stable_count: int = 0
        self.last_target_pos: Optional[Tuple[int, int]] = None
        
        # 初始化相机
        self._initialize_camera()
        self.logger.info("ColorBlockDetector 初始化完成")
    
    def _initialize_camera(self) -> None:
        """初始化RealSense相机"""
        try:
            self.logger.info("开始初始化RealSense相机...")
            
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 配置RGB和深度流
            self.config.enable_stream(
                rs.stream.color, 
                Constants.CAMERA_WIDTH, 
                Constants.CAMERA_HEIGHT, 
                rs.format.bgr8, 
                Constants.CAMERA_FPS
            )
            self.config.enable_stream(
                rs.stream.depth, 
                Constants.CAMERA_WIDTH, 
                Constants.CAMERA_HEIGHT, 
                rs.format.z16, 
                Constants.CAMERA_FPS
            )
            
            # 启动管道
            self.profile = self.pipeline.start(self.config)
            
            # 获取相机内参
            color_profile = self.profile.get_stream(rs.stream.color)
            self.intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
            
            # 获取深度传感器的深度比例
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            # 创建对齐对象（将深度图对齐到彩色图）
            self.align = rs.align(rs.stream.color)
            
            # 设置初始图像中心点
            self.img_center_x = self.intrinsics.ppx
            self.img_center_y = self.intrinsics.ppy
            
            self.logger.info("相机初始化成功!")
            self.logger.info(f"相机内参: fx={self.intrinsics.fx:.2f}, fy={self.intrinsics.fy:.2f}")
            self.logger.info(f"主点: cx={self.intrinsics.ppx:.2f}, cy={self.intrinsics.ppy:.2f}")
            self.logger.info(f"深度比例: {self.depth_scale}")
            
        except Exception as e:
            self.logger.error(f"相机初始化失败: {e}")
            raise RuntimeError(f"相机初始化失败: {e}")

    def update_image_center_with_tcp_offset(self, distance: float) -> None:
        """
        根据距离更新图像中心点，由于近大远小，图像中心点不是固定的，需要根据相机内参实时更新中心点
        考虑TCP偏移
        
        Args:
            distance: 目标距离（米）
        """
        if distance > 0 and self.intrinsics is not None:
            # 计算TCP偏移在当前深度下对应的像素偏移
            self.pixel_offset_y = (self.intrinsics.fy * Constants.TCP_OFFSET_Y) / distance
            
            # 更新图像中心点（X方向无偏移，Y方向考虑TCP偏移）
            self.img_center_x = self.intrinsics.ppx
            self.img_center_y = self.intrinsics.ppy + self.pixel_offset_y
            
            self.logger.debug(
                f"距离: {distance:.3f}m, 像素偏移: {self.pixel_offset_y:.2f}, "
                f"更新后中心: ({self.img_center_x:.2f}, {self.img_center_y:.2f})"
            )

    def pixel_to_camera_coordinates(self, u: float, v: float, depth: float) -> np.ndarray:
        """
        像素坐标转相机坐标
        
        Args:
            u, v: 像素坐标
            depth: 深度值（米）
            
        Returns:
            相机坐标系下的3D坐标 [X, Y, Z]
        """
        X = (u - self.intrinsics.ppx) * depth / self.intrinsics.fx
        Y = (v - self.intrinsics.ppy) * depth / self.intrinsics.fy
        Z = depth
        return np.array([X, Y, Z])
    
    def camera_to_robot_coordinates(self, camera_coords: np.ndarray) -> np.ndarray:
        """
        相机坐标转机械臂坐标
        
        Args:
            camera_coords: 相机坐标系下的3D坐标
            
        Returns:
            机械臂坐标系下的3D坐标
        """
        return self.camera_to_robot_rotation @ camera_coords
    
    def calculate_robot_movement(self, center_x: int, center_y: int, depth: float) -> Tuple[float, float]:
        """
        计算机械臂移动量
        
        Args:
            center_x, center_y: 目标像素坐标
            depth: 目标深度（米）
            
        Returns:
            机械臂X、Z方向的移动量（毫米）
        """
        # 计算图像中心点和目标点的相机坐标
        center_camera_coords = self.pixel_to_camera_coordinates(
            self.img_center_x, self.img_center_y, depth
        )
        target_camera_coords = self.pixel_to_camera_coordinates(
            center_x, center_y, depth
        )
        
        # 计算相机坐标系下的偏移
        camera_offset = target_camera_coords - center_camera_coords
        
        # 转换到机械臂坐标系
        robot_offset = self.camera_to_robot_coordinates(camera_offset)
        
        # 转换为毫米并应用缩放因子
        delta_x = robot_offset[0] * 1000 * Constants.MOTION_SCALE
        delta_z = robot_offset[2] * 1000 * Constants.MOTION_SCALE

        self.logger.debug(
            f"移动计算: 像素({center_x}, {center_y}) -> 机械臂移动({delta_x:.2f}, {delta_z:.2f})mm"
        )

        return delta_x, delta_z
    
    def is_target_stable(self, center_x: int, center_y: int, threshold: int = 5) -> bool:
        """
        检查目标位置是否稳定
        
        Args:
            center_x, center_y: 当前目标位置
            threshold: 稳定性阈值（像素）
            
        Returns:
            目标是否稳定
        """
        current_pos = (center_x, center_y)
        
        if self.last_target_pos is None:
            self.last_target_pos = current_pos
            self.stable_count = 0
            return False
        
        # 计算位置变化
        dx = abs(current_pos[0] - self.last_target_pos[0])
        dy = abs(current_pos[1] - self.last_target_pos[1])
        
        if dx < threshold and dy < threshold:
            self.stable_count += 1
        else:
            self.stable_count = 0
        
        self.last_target_pos = current_pos
        return self.stable_count >= Constants.STABLE_THRESHOLD
    
    def detect_green_block(self, show_debug: bool = True) -> Optional[Tuple[int, int, float, float]]:
        """
        检测绿色色块
        
        Args:
            show_debug: 是否显示调试信息
            
        Returns:
            检测结果: (center_x, center_y, distance, area) 或 None
        """
        try:
            # 检查pipeline状态
            if self.pipeline is None:
                self.logger.error("Pipeline未初始化")
                return None
            
            # 获取相机帧
            try:
                frames = self.pipeline.wait_for_frames()
            except Exception as e:
                self.logger.error(f"获取相机帧失败: {e}")
                self._restart_pipeline()
                return None
            
            # 对齐深度图和彩色图
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None
            
            # 转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 颜色空间转换和阈值处理
            hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, Constants.GREEN_LOWER, Constants.GREEN_UPPER)
            
            # 形态学操作去噪
            kernel = np.ones((Constants.MORPHOLOGY_KERNEL_SIZE, Constants.MORPHOLOGY_KERNEL_SIZE), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 找到最大轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                # 检查面积是否符合要求
                if area > Constants.MIN_CONTOUR_AREA:
                    # 计算轮廓中心
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                        
                        # 获取深度信息
                        depth_value = depth_frame.get_distance(center_x, center_y)
                        distance = depth_value
                        
                        if show_debug:
                            self._draw_debug_info(
                                color_image, mask, largest_contour, 
                                center_x, center_y, distance, area
                            )
                        
                        return center_x, center_y, distance, area
            
            # 没有检测到有效目标时仍显示图像
            if show_debug:
                cv2.imshow('Green Block Detection', color_image)
                cv2.imshow('Mask', mask)
            return None
            
        except Exception as e:
            self.logger.error(f"检测过程中发生错误: {e}")
            return None
    
    def _restart_pipeline(self) -> None:
        """重启相机pipeline"""
        try:
            self.logger.info("尝试重新启动相机pipeline...")
            if self.pipeline:
                self.pipeline.stop()
            self.pipeline.start(self.config)
            self.logger.info("相机pipeline重新启动成功")
        except Exception as e:
            self.logger.error(f"重新启动相机失败: {e}")
    
    def _draw_debug_info(self, color_image: np.ndarray, mask: np.ndarray, 
                        contour: np.ndarray, center_x: int, center_y: int, 
                        distance: float, area: float) -> None:
        """
        绘制调试信息
        
        Args:
            color_image: 彩色图像
            mask: 掩码图像
            contour: 检测到的轮廓
            center_x, center_y: 目标中心
            distance: 目标距离
            area: 轮廓面积
        """
        try:
            debug_image = color_image.copy()
            
            # 绘制轮廓和中心点
            cv2.drawContours(debug_image, [contour], -1, (0, 255, 0), 2)
            cv2.circle(debug_image, (center_x, center_y), 5, (0, 0, 255), -1)
            
            # 绘制边界框
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(debug_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            
            if distance > 0:
                delta_x, delta_z = self.calculate_robot_movement(center_x, center_y, distance)
                
                # 显示状态信息
                info_texts = [
                    f"Target: ({center_x}, {center_y})",
                    f"Center: ({self.img_center_x:.1f}, {self.img_center_y:.1f})",
                    f"Offset: ({center_x - self.img_center_x:.1f}, {center_y - self.img_center_y:.1f})",
                    f"Distance: {distance:.3f}m ({distance*1000:.1f}mm)",
                    f"Area: {area:.0f}",
                    f"Move: X={delta_x:.1f}mm, Z={delta_z:.1f}mm",
                    f"TCP Offset Y: {self.pixel_offset_y:.1f}px",
                    f"Stable: {self.stable_count}/{Constants.STABLE_THRESHOLD}"
                ]
                
                for i, text in enumerate(info_texts):
                    cv2.putText(debug_image, text, (10, 25 + i*25), 
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 显示当前的图像中心点（黄色十字）
            self._draw_crosshair(debug_image, int(self.img_center_x), int(self.img_center_y), 
                               (0, 255, 255), 10, 2)
            
            # 显示相机主点（绿色十字）
            self._draw_crosshair(debug_image, int(self.intrinsics.ppx), int(self.intrinsics.ppy), 
                               (0, 255, 0), 5, 1)
            
            cv2.imshow('Green Block Detection', debug_image)
            cv2.imshow('Mask', mask)
            cv2.imwrite("result.jpg", debug_image)

        except Exception as e:
            self.logger.warning(f"绘制调试信息失败: {e}")
    
    def _draw_crosshair(self, image: np.ndarray, x: int, y: int, 
                       color: Tuple[int, int, int], size: int, thickness: int) -> None:
        """绘制十字线"""
        cv2.line(image, (x, y - size), (x, y + size), color, thickness)
        cv2.line(image, (x - size, y), (x + size, y), color, thickness)
    
    def reset_stability_tracking(self) -> None:
        """重置稳定性跟踪"""
        self.stable_count = 0
        self.last_target_pos = None
        self.logger.debug("稳定性跟踪已重置")
    
    def cleanup(self) -> None:
        """清理资源"""
        self.logger.info("正在清理相机资源...")
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
        cv2.destroyAllWindows()
        self.logger.info("相机资源清理完成")


# ==================== 机械臂控制模块 ====================
class RobotController:
    """
    机械臂控制模块
    
    功能：
    - 初始化Dobot机械臂连接
    - 提供各种运动控制方法
    - 监控机械臂状态
    - 管理初始位姿
    """
    
    def __init__(self) -> None:
        """初始化机械臂控制器"""
        self.logger = logging.getLogger(__name__ + '.RobotController')
        
        # 连接参数
        self.dashboard: Optional[DobotApiDashboard] = None
        self.feed_four: Optional[DobotApiFeedBack] = None
        
        # 位姿管理
        self.initial_pose: Optional[List[float]] = None
        
        # 初始化机械臂
        self._initialize_robot()
        self.logger.info("RobotController 初始化完成")
    
    def _initialize_robot(self) -> None:
        """初始化机械臂连接"""
        try:
            self.logger.info("开始初始化机械臂连接...")
            
            # 建立连接
            self.dashboard = DobotApiDashboard(Constants.ROBOT_IP, Constants.DASHBOARD_PORT)
            self.feed_four = DobotApiFeedBack(Constants.ROBOT_IP, Constants.FEED_PORT)
            
            # 机械臂配置
            self.logger.info(f"请求控制权: {self.dashboard.RequestControl()}")
            self.logger.info(f"使能机械臂: {self.dashboard.EnableRobot()}")
            self.logger.info(f"设置工具坐标系: {self.dashboard.Tool(Constants.TOOL_COORD)}")

            # 记录初始位姿
            #! 可用可不用，暂不启用
            # self._record_initial_pose()
            
            self.logger.info("机械臂初始化成功")
            
        except Exception as e:
            self.logger.error(f"机械臂初始化失败: {e}")
            raise RuntimeError(f"机械臂初始化失败: {e}")
    
    def _record_initial_pose(self) -> None:
        """记录初始位姿"""
        try:
            self.initial_pose = self.get_current_pose()
            if self.initial_pose:
                self.logger.info(f"初始位姿已记录: {self.initial_pose}")
            else:
                self.logger.warning("获取初始位姿失败")
        except Exception as e:
            self.logger.error(f"记录初始位姿失败: {e}")
    
    def get_current_pose(self) -> Optional[List[float]]:
        """
        获取当前位姿
        
        Returns:
            当前位姿 [x, y, z, rx, ry, rz] 或 None
        """
        try:
            pose_response = self.dashboard.GetPose(Constants.USER_COORD, Constants.TOOL_COORD)
            if pose_response is None:
                self.logger.warning("获取位姿失败")
                return None
            
            # 解析位姿字符串
            start = pose_response.find("{")
            end = pose_response.find("}")
            
            if start == -1 or end == -1 or end <= start:
                self.logger.error(f"位姿字符串格式错误: {pose_response}")
                return None
            
            pose_str = pose_response[start+1:end]
            pose_values = [float(x.strip()) for x in pose_str.split(",") if x.strip()]
            
            self.logger.debug(f"当前位姿: {pose_values}")
            return pose_values
            
        except Exception as e:
            self.logger.error(f"获取位姿失败: {e}")
            return None
    
    def is_robot_ready(self) -> bool:
        """
        检查机械臂是否就绪（不在运动中）
        
        Returns:
            机械臂是否就绪
        """
        try:
            robot_mode = self.dashboard.RobotMode()
            # 模式7表示机械臂正在运动
            # 只要不是7就是True
            is_ready = robot_mode is not None and "7" not in robot_mode
            self.logger.debug(f"机械臂状态: {robot_mode}, 就绪: {is_ready}")
            return is_ready
        except Exception as e:
            self.logger.warning(f"检查机械臂状态失败: {e}")
            return False
    
    def can_execute_movement(self, center_x: int, center_y: int, detector: ColorBlockDetector) -> Tuple[bool, str]:
        """
        判断是否可以执行运动（只检查稳定性和运动状态）
        
        Args:
            center_x, center_y: 目标像素坐标
            detector: 检测器实例
            
        Returns:
            (是否可以移动, 原因说明)
        """
        # 检查机械臂是否正在运动
        if not self.is_robot_ready():
            return False, "机械臂正在运动中"
        
        # 检查目标是否稳定
        if not detector.is_target_stable(center_x, center_y, threshold=5):
            return False, "目标位置不稳定"
        
        return True, "可以移动"
    
    def move_xz(self, target_x: float, target_z: float, current_pose: List[float]) -> bool:
        """
        执行XZ方向移动，平面移动（图像坐标系中的XY,参考坐标系定义）
        
        Args:
            target_x: 目标X坐标
            target_z: 目标Z坐标
            current_pose: 当前位姿
            
        Returns:
            移动是否成功
        """
        try:
            x, y, z, rx, ry, rz = current_pose
            
            self.logger.info(f"执行XZ移动: X={target_x:.2f}, Z={target_z:.2f}")
            self.dashboard.MovJ(target_x, y, target_z, rx, ry, rz, 0)
            
            # 等待运动开始
            time.sleep(0.1)
            
            self.logger.info("XZ移动指令已发送")
            return True
            
        except Exception as e:
            self.logger.error(f"XZ移动失败: {e}")
            return False
    
    def move_y_to_target_depth(self, depth_distance: float) -> Tuple[bool, str]:
        """
        执行Y方向移动到目标深度
        
        Args:
            depth_distance: 当前深度距离（毫米）
            
        Returns:
            (移动是否成功, 状态信息)
        """
        try:
            # 计算需要移动的距离
            depth_offset = depth_distance - Constants.TARGET_DEPTH_MM
            
            self.logger.info(
                f"执行Y方向移动: 当前深度={depth_distance:.1f}mm, "
                f"目标深度={Constants.TARGET_DEPTH_MM}mm, 偏移={depth_offset:.1f}mm"
            )
            
            # 使用工具坐标系进行相对移动
            self.dashboard.RelMovLTool(
                0, 0, depth_offset, 0, 0, 0, 
                user=Constants.USER_COORD, 
                tool=Constants.TOOL_COORD, 
                a=10, v=20, cp=100
            )
            
            # 等待运动开始
            time.sleep(0.1)
            
            self.logger.info("Y方向移动指令已发送")
            return True, "success"
            
        except Exception as e:
            error_msg = f"Y方向移动失败: {e}"
            self.logger.error(error_msg)
            return False, error_msg
    
    def move_down(self, distance: float = Constants.PICKUP_HEIGHT_MM) -> bool:
        """
        移动指定距离（根据距离伸进去夹取）
        
        Args:
            distance: 向里移动距离（毫米）
            
        Returns:
            移动是否成功
        """
        try:
            self.logger.info(f"移动: {distance}mm")
            self.dashboard.RelMovLTool(
                0, 0, -distance, 0, 0, 0, 
                user=Constants.USER_COORD, 
                tool=Constants.TOOL_COORD, 
                a=10, v=30
            )
            
            # 等待运动开始
            time.sleep(0.1)
            
            self.logger.info("向里移动指令已发送")
            return True
            
        except Exception as e:
            self.logger.error(f"向里移动失败: {e}")
            return False

    def rotate(self, angle: float = Constants.ROTATION_ANGLE) -> bool:
        """
        绕Z轴旋转指定角度
        
        Args:
            angle: 旋转角度（度）
            
        Returns:
            旋转是否成功
        """
        try:
            self.logger.info(f"旋转: {angle}度")
            self.dashboard.RelMovJTool(
                0, 0, 0, 0, 0, angle, 
                user=Constants.USER_COORD, 
                tool=Constants.TOOL_COORD, 
                a=-1, v=-1, cp=-1
            )
            
            # 等待运动开始
            time.sleep(0.1)
            
            self.logger.info("旋转指令已发送")
            return True
            
        except Exception as e:
            self.logger.error(f"旋转失败: {e}")
            return False

    def move_to_preview_pose(self) -> bool:
        """移动到预瞄位置"""
        try:
            self.logger.info("移动到预瞄位置")
            self.dashboard.MovJ(-90, 5.1677, -115.1287, 104.0244, 90, 0, 1)
            return True
        except Exception as e:
            self.logger.error(f"移动到预瞄位置失败: {e}")
            return False
    

    def clear_errors(self) -> None:
        """清除机械臂错误"""
        try:
            self.dashboard.ClearError()
            self.logger.debug("机械臂错误已清除")
        except Exception as e:
            self.logger.error(f"清除错误失败: {e}")
    
    def cleanup(self) -> None:
        """清理资源"""
        try:
            self.logger.info("正在清理机械臂资源...")
            if self.dashboard:
                self.clear_errors()
                self.move_to_preview_pose()
            self.logger.info("机械臂资源清理完成")
        except Exception as e:
            self.logger.warning(f"机械臂清理过程中出现错误: {e}")


# ==================== 夹爪控制模块 ====================
class GripperController(Node):
    """
    夹爪控制模块（使用ROS2接口）
    
    功能：
    - 通过ROS2话题控制夹爪闭合
    - 通过ROS2服务控制夹爪张开
    - 提供统一的夹爪控制接口
    """
    
    def __init__(self) -> None:
        """初始化夹爪控制器"""
        super().__init__('gripper_controller')
        self.python_logger = logging.getLogger(__name__ + '.GripperController')
        
        # 创建夹爪控制发布者（用于闭合）
        self.gripper_publisher = self.create_publisher(
            GripperCommand, 
            '/gripper_command', 
            10
        )
        
        # 创建服务客户端（用于张开/归零）
        self.go_home_client = self.create_client(Trigger, '/go_home')
        
        # 等待服务上线
        self._wait_for_services()
        
        self.python_logger.info("GripperController 初始化完成")
    
    def _wait_for_services(self) -> None:
        """等待ROS2服务上线"""
        self.python_logger.info("等待夹爪服务上线...")
        while not self.go_home_client.wait_for_service(timeout_sec=1.0):
            self.python_logger.info("'/go_home' 服务不可用，继续等待...")
        self.python_logger.info("夹爪服务已连接")
    
    def control_gripper(self, action: str) -> bool:
        """
        控制夹爪动作
        
        Args:
            action: 'close' 闭合 或 'open' 张开
            
        Returns:
            控制是否成功
        """
        if action == 'close':
            return self._close_gripper()
        elif action == 'open':
            return self._open_gripper()
        else:
            self.python_logger.warning(f"无效的夹爪动作: '{action}'。只接受 'open' 或 'close'")
            return False
    
    def _close_gripper(self) -> bool:
        """闭合夹爪"""
        try:
            gripper_msg = GripperCommand()
            gripper_msg.speed = Constants.GRIPPER_SPEED
            gripper_msg.force = Constants.GRIPPER_FORCE
            gripper_msg.position = Constants.GRIPPER_CLOSE_POSITION
            
            self.python_logger.info(f"发送夹爪闭合指令: position={gripper_msg.position}")
            self.gripper_publisher.publish(gripper_msg)
            
            self.python_logger.info(f"等待 {Constants.GRIPPER_CLOSE_WAIT}s 确保夹爪闭合...")
            time.sleep(Constants.GRIPPER_CLOSE_WAIT)
            return True

        except Exception as e:
            self.python_logger.error(f"夹爪闭合失败: {e}")
            return False
    
    def _open_gripper(self) -> bool:
        """张开夹爪"""
        try:
            self.python_logger.info("调用 '/go_home' 服务使夹爪张开...")
            
            # 构造服务请求
            request = Trigger.Request()
            
            # 异步调用服务
            future = self.go_home_client.call_async(request)
            
            # 等待服务响应
            time.sleep(2)
            self.python_logger.info("夹爪张开指令已发送")
            return True
            
        except Exception as e:
            self.python_logger.error(f"夹爪张开失败: {e}")
            return False


# ==================== 主控制器 ====================
class MainController:
    """
    主控制器 - 整合所有模块，实现完整的视觉抓取流程
    
    功能：
    - 管理系统状态机
    - 协调各个模块的工作
    - 执行完整的抓取序列
    """
    
    def __init__(self) -> None:
        """初始化主控制器"""
        self.logger = logging.getLogger(__name__ + '.MainController')
        
        # 初始化各个模块
        self.logger.info("开始初始化系统模块...")
        
        try:
            self.logger.info("初始化色块检测器...")
            self.detector = ColorBlockDetector()
            
            self.logger.info("初始化机械臂控制器...")
            self.robot = RobotController()
            
            self.logger.info("初始化夹爪控制器...")
            self.gripper = GripperController()
            
            # 任务状态管理
            self.task_state = TaskState.DETECTING
            
            self.logger.info("主控制器初始化完成")
            
        except Exception as e:
            self.logger.error(f"主控制器初始化失败: {e}")
            self._cleanup_on_error()
            raise
    
    def _cleanup_on_error(self) -> None:
        """初始化失败时的清理工作"""
        if hasattr(self, 'detector'):
            try:
                self.detector.cleanup()
            except:
                pass
        if hasattr(self, 'robot'):
            try:
                self.robot.cleanup()
            except:
                pass
    
    def run_continuous_detection(self) -> None:
        """运行连续检测和控制循环"""
        self.logger.info("开始连续检测和控制...")
        
        # 移动到预瞄位置
        self.robot.move_to_preview_pose()
        time.sleep(1)
        
        try:
            while True:
                # 处理ROS2回调
                rclpy.spin_once(self.gripper, timeout_sec=0.001)
                
                # 根据当前状态执行对应逻辑
                if self.task_state == TaskState.COMPLETED:
                    self._handle_completed_state()
                elif self.task_state == TaskState.RESETTING:
                    self._handle_resetting_state()
                else:
                    self._handle_active_detection()
                
                # 检查退出条件
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        except KeyboardInterrupt:
            self.logger.warning("\n检测被用户中断")
        finally:
            self.cleanup()
    
    def _handle_completed_state(self) -> None:
        """处理任务完成状态"""
        # 显示检测图像但不执行控制
        self.detector.detect_green_block(show_debug=True)
        time.sleep(2)
        
        # 自动重置任务
        self._reset_task()
    
    def _handle_resetting_state(self) -> None:
        """处理重置状态"""
        self.logger.info("正在重置任务状态...")
        self.task_state = TaskState.DETECTING
        time.sleep(0.5)
    
    def _handle_active_detection(self) -> None:
        """处理活跃检测状态"""
        result = self.detector.detect_green_block(show_debug=True)
        
        if result:
            self._process_detection_result(result)
    
    def _process_detection_result(self, result: Tuple[int, int, float, float]) -> None:
        """
        处理检测结果并根据状态机执行相应动作
        
        Args:
            result: 检测结果 (center_x, center_y, distance, area)
        """
        center_x, center_y, distance, area = result
        
        # 获取当前机械臂位姿
        current_pose = self.robot.get_current_pose()
        if current_pose is None:
            self.logger.error("获取机械臂位姿失败")
            return
        
        self.logger.debug(
            f"[{self.task_state.value}] 检测到绿色色块: "
            f"中心({center_x}, {center_y}), 距离{distance:.3f}m, 面积{area:.0f}"
        )
        
        if distance <= 0:
            return
        
        # 更新图像中心点（考虑TCP偏移）
        self.detector.update_image_center_with_tcp_offset(distance)
        
        # 根据当前状态执行相应逻辑
        if self.task_state == TaskState.DETECTING or self.task_state == TaskState.ALIGNING_XZ:
            self._handle_alignment_phase(center_x, center_y, distance, current_pose)
        elif self.task_state == TaskState.MOVING_Y:
            self._handle_y_movement_phase(distance)
        elif self.task_state == TaskState.PICKING:
            self._handle_picking_phase()
    
    def _handle_alignment_phase(self, center_x: int, center_y: int, 
                              distance: float, current_pose: List[float]) -> None:
        """处理XZ对准阶段"""
        # 检查是否需要XZ方向调整
        offset_x = abs(center_x - self.detector.img_center_x)
        offset_y = abs(center_y - self.detector.img_center_y)
        
        need_move_x = offset_x >= Constants.PIXEL_THRESHOLD
        need_move_z = offset_y >= Constants.PIXEL_THRESHOLD
        
        if need_move_x or need_move_z:
            # 更新状态为对准阶段
            if self.task_state == TaskState.DETECTING:
                self.task_state = TaskState.ALIGNING_XZ
                self.logger.info("进入XZ对准阶段")
            
            # 检查是否可以移动
            can_move, reason = self.robot.can_execute_movement(center_x, center_y, self.detector)
            
            if can_move:
                self._execute_xz_alignment(center_x, center_y, distance, current_pose)
            else:
                self.logger.debug(f"等待移动条件满足: {reason}")
        else:
            # XZ对准完成，进入Y方向移动阶段
            self.logger.info("XZ对准完成，进入Y方向移动阶段")
            self.task_state = TaskState.MOVING_Y
    
    def _execute_xz_alignment(self, center_x: int, center_y: int, 
                            distance: float, current_pose: List[float]) -> None:
        """执行XZ方向对准移动"""
        x, y, z, rx, ry, rz = current_pose
        
        # 计算移动量
        delta_x, delta_z = self.detector.calculate_robot_movement(center_x, center_y, distance)
        
        # 计算目标位置
        target_x = x + delta_x 
        target_z = z + delta_z 
        
        self.logger.info(
            f"执行XZ对准: 当前({x:.1f}, {z:.1f}) -> "
            f"目标({target_x:.1f}, {target_z:.1f}), "
            f"偏移({delta_x:.1f}, {delta_z:.1f})"
        )
        
        if self.robot.move_xz(target_x, target_z, current_pose):
            # 重置稳定性跟踪，等待下一次检测
            self.detector.reset_stability_tracking()
        else:
            self.logger.error("XZ对准移动失败")
    
    def _handle_y_movement_phase(self, distance: float) -> None:
        """处理Y方向移动阶段"""
        self.logger.info("执行Y方向移动到目标深度")
        depth_distance = distance * 1000  # 转换为毫米
        
        success, message = self.robot.move_y_to_target_depth(depth_distance)
        
        if success:
            self.logger.info("Y方向移动成功，进入抓取阶段")
            self.task_state = TaskState.PICKING
        else:
            self.logger.error(f"Y方向移动失败: {message}")
            self.task_state = TaskState.DETECTING  # 回到检测状态
    
    def _handle_picking_phase(self) -> None:
        """处理抓取阶段"""
        self.logger.info("开始执行抓取序列")
        
        if self._execute_pick_sequence():
            self.logger.info("抓取序列完成！")
            self.task_state = TaskState.COMPLETED
        else:
            self.logger.error("抓取序列失败")
            self.task_state = TaskState.DETECTING  # 回到检测状态
    
    def _execute_pick_sequence(self) -> bool:
        """
        执行完整的抓取序列
        
        Returns:
            抓取序列是否成功
        """
        self.logger.info("=" * 50)
        self.logger.info("开始执行抓取序列...")
        
        try:
            # 1. 等待Y方向移动完成
            self.logger.info("等待机械臂到达目标位置...")
            self._wait_for_robot_ready()
            
            # 2. 闭合夹爪抓取物体
            self.logger.info("闭合夹爪抓取物体...")
            if not self.gripper.control_gripper('close'):
                self.logger.error("夹爪闭合失败")
                return False
            
            # 3. 向上提升物体
            self.logger.info(f"向上提升物体 {Constants.PICKUP_HEIGHT_MM}mm...")
            if not self.robot.move_down(Constants.PICKUP_HEIGHT_MM):
                self.logger.error("向上移动失败")
                return False
            
            # 等待上升完成
            self._wait_for_robot_ready()
            
            # 4. 旋转到放置位置
            self.logger.info(f"旋转 {Constants.ROTATION_ANGLE}度 到放置位置...")
            if not self.robot.rotate(Constants.ROTATION_ANGLE):
                self.logger.error("旋转失败")
                return False
            
            # 等待旋转完成
            time.sleep(0.5)
            self._wait_for_robot_ready()
            
            # 5. 张开夹爪放下物体
            self.logger.info("张开夹爪放下物体...")
            if not self.gripper.control_gripper('open'):
                self.logger.error("夹爪张开失败")
                return False
            
            time.sleep(0.5)
            
            # 6. 返回初始位姿
            self.logger.info("返回初始位姿...")
            if not self.robot.move_to_preview_pose():
                self.logger.error("返回初始位姿失败")
                return False
            
            self.logger.info("抓取序列完成！")
            self.logger.info("=" * 50)
            return True
            
        except Exception as e:
            self.logger.error(f"抓取序列执行过程中发生错误: {e}")
            return False
    
    def _wait_for_robot_ready(self, timeout: float = 10.0) -> bool:
        """
        等待机械臂就绪
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            是否在超时时间内就绪
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.robot.is_robot_ready():
                return True
            time.sleep(0.1)
        
        self.logger.warning(f"等待机械臂就绪超时 ({timeout}s)")
        return False
    
    def _reset_task(self) -> None:
        """重置任务状态"""
        self.logger.info("重置任务状态...")
        self.task_state = TaskState.RESETTING
        self.detector.reset_stability_tracking()
        self.logger.info("任务状态已重置")
    
    def cleanup(self) -> None:
        """清理所有资源"""
        self.logger.info("正在清理系统资源...")
        
        # 清理各个模块
        self.detector.cleanup()
        self.robot.cleanup()
        
        # 确保夹爪张开
        try:
            self.gripper.control_gripper('open')
        except:
            pass
        
        self.logger.info("系统资源清理完成")


# ==================== 主函数 ====================
def main():
    """主函数 - 系统入口点"""
    # 设置日志系统
    setup_development_logging()
    logger = get_logger(__name__ + '.main')
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        logger.info("=" * 60)
        
        # 创建主控制器
        controller = MainController()
        
        # 运行检测和控制循环
        controller.run_continuous_detection()
        
        logger.info("系统正常退出")
            
    except Exception as e:
        logger.error(f"程序运行错误: {e}", exc_info=True)
    finally:
        # 确保ROS2被正确关闭
        if rclpy.ok():
            rclpy.shutdown()
        logger.info("ROS2已关闭")
        logger.info("=" * 60)


if __name__ == "__main__":
    main()