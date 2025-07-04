#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import yaml
import os
from typing import Dict, Any, Optional
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np


@dataclass
class NetworkConfig:
    """网络配置"""
    ip: str = "192.168.0.162"
    port: int = 56322
    hand_port: int = 56422


@dataclass
class JointConfig:
    """关节配置"""
    name: str
    limit: tuple
    default: float = 0.0


@dataclass
class ArmJointsConfig:
    """机械臂关节配置"""
    left_arm: list[JointConfig] = field(default_factory=list)
    right_arm: list[JointConfig] = field(default_factory=list)
    head: list[JointConfig] = field(default_factory=list)
    waist_lift: list[JointConfig] = field(default_factory=list)
    waist_pitch: list[JointConfig] = field(default_factory=list)


@dataclass
class MotionConfig:
    """运动配置"""
    timer_period: float = 0.03
    max_velocity: float = 1.0
    acceleration: float = 0.5


@dataclass
class GripperConfig:
    """夹爪配置"""
    default_position: list[int] = field(default_factory=lambda: [0, 0])
    max_position: int = 100
    min_position: int = 0


@dataclass
class ColorRange:
    """颜色范围配置"""
    lower: list[int]
    upper: list[int]


@dataclass
class ColorConfig:
    """颜色配置"""
    ranges: list[ColorRange]
    bgr: list[int]


@dataclass
class ColorDetectionConfig:
    """颜色检测配置"""
    min_area: int = 3000
    rect_ratio: float = 0.85
    resize_width: int = 640
    blur_kernel_size: int = 5
    morph_kernel_size: int = 5
    colors: Dict[str, ColorConfig] = field(default_factory=dict)
    rect_filter: Dict[str, list[int]] = field(default_factory=dict)
    save: Dict[str, str] = field(default_factory=dict)


@dataclass
class SystemConfig:
    """系统配置"""
    debug_mode: bool = True
    log_level: str = "INFO"
    data_dir: str = "./data"
    visualization_dir: str = "visualization_steps"


class ConfigManager:
    """配置管理器"""
    
    def __init__(self, config_path: str = "config/arm_config.yaml"):
        self.config_path = Path(config_path)
        self._config_data: Dict[str, Any] = {}
        self._load_config()
    
    def _load_config(self):
        """加载配置文件"""
        if not self.config_path.exists():
            raise FileNotFoundError(f"配置文件不存在: {self.config_path}")
        
        with open(self.config_path, 'r', encoding='utf-8') as f:
            self._config_data = yaml.safe_load(f)
    
    def reload_config(self):
        """重新加载配置"""
        self._load_config()
    
    def get_network_config(self) -> NetworkConfig:
        """获取网络配置"""
        network_data = self._config_data.get('arm', {}).get('network', {})
        return NetworkConfig(**network_data)
    
    def get_arm_joints_config(self) -> ArmJointsConfig:
        """获取机械臂关节配置"""
        joints_data = self._config_data.get('arm', {}).get('joints', {})
        
        def create_joint_configs(joint_list):
            return [JointConfig(**joint) for joint in joint_list]
        
        return ArmJointsConfig(
            left_arm=create_joint_configs(joints_data.get('left_arm', [])),
            right_arm=create_joint_configs(joints_data.get('right_arm', [])),
            head=create_joint_configs(joints_data.get('head', [])),
            waist_lift=create_joint_configs(joints_data.get('waist', {}).get('lift', [])),
            waist_pitch=create_joint_configs(joints_data.get('waist', {}).get('pitch', []))
        )
    
    def get_motion_config(self) -> MotionConfig:
        """获取运动配置"""
        motion_data = self._config_data.get('arm', {}).get('motion', {})
        return MotionConfig(**motion_data)
    
    def get_gripper_config(self) -> GripperConfig:
        """获取夹爪配置"""
        gripper_data = self._config_data.get('arm', {}).get('gripper', {})
        return GripperConfig(**gripper_data)
    
    def get_color_detection_config(self) -> ColorDetectionConfig:
        """获取颜色检测配置"""
        color_data = self._config_data.get('color_detection', {})
        
        # 处理颜色配置
        colors = {}
        for color_name, color_info in color_data.get('colors', {}).items():
            ranges = [ColorRange(**r) for r in color_info.get('ranges', [])]
            colors[color_name] = ColorConfig(
                ranges=ranges,
                bgr=color_info.get('bgr', [0, 0, 0])
            )
        
        return ColorDetectionConfig(
            min_area=color_data.get('min_area', 3000),
            rect_ratio=color_data.get('rect_ratio', 0.85),
            resize_width=color_data.get('resize_width', 640),
            blur_kernel_size=color_data.get('blur_kernel_size', 5),
            morph_kernel_size=color_data.get('morph_kernel_size', 5),
            colors=colors,
            rect_filter=color_data.get('rect_filter', {}),
            save=color_data.get('save', {})
        )
    
    def get_system_config(self) -> SystemConfig:
        """获取系统配置"""
        system_data = self._config_data.get('system', {})
        return SystemConfig(**system_data)
    
    def get_raw_config(self) -> Dict[str, Any]:
        """获取原始配置数据"""
        return self._config_data.copy()
    
    def get_numpy_morph_kernel(self) -> np.ndarray:
        """获取numpy形态学核"""
        size = self.get_color_detection_config().morph_kernel_size
        return np.ones((size, size), np.uint8)
    
    def get_numpy_color_ranges(self) -> Dict[str, Dict[str, np.ndarray]]:
        """获取numpy格式的颜色范围"""
        color_config = self.get_color_detection_config()
        numpy_ranges = {}
        
        for color_name, color_info in color_config.colors.items():
            ranges = []
            for range_info in color_info.ranges:
                ranges.append({
                    'lower': np.array(range_info.lower, dtype=np.uint8),
                    'upper': np.array(range_info.upper, dtype=np.uint8)
                })
            numpy_ranges[color_name] = {
                'ranges': ranges,
                'bgr': tuple(color_info.bgr)
            }
        
        return numpy_ranges


# 全局配置实例
config_manager = ConfigManager()


def get_config() -> ConfigManager:
    """获取全局配置管理器"""
    return config_manager


# 便捷函数
def get_network_config() -> NetworkConfig:
    return config_manager.get_network_config()


def get_arm_joints_config() -> ArmJointsConfig:
    return config_manager.get_arm_joints_config()


def get_motion_config() -> MotionConfig:
    return config_manager.get_motion_config()


def get_gripper_config() -> GripperConfig:
    return config_manager.get_gripper_config()


def get_color_detection_config() -> ColorDetectionConfig:
    return config_manager.get_color_detection_config()


def get_system_config() -> SystemConfig:
    return config_manager.get_system_config() 