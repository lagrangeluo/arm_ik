#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('arm_ik')
    
    # 声明launch参数
    robot_description_file_arg = DeclareLaunchArgument(
        'robot_description_file',
        default_value=os.path.join(pkg_share, 'urdf', 'arx5_description_ik.urdf'),
        description='Path to robot description URDF file'
    )
    
    robot_description_semantic_file_arg = DeclareLaunchArgument(
        'robot_description_semantic_file',
        default_value=os.path.join(pkg_share, 'urdf', 'arx5_description.srdf'),
        description='Path to robot description semantic SRDF file'
    )
    
    # 颜色检测节点
    color_detect_node = Node(
        package='arm_ik',
        executable='color_detect_ros2.py',
        name='color_detect',
        output='screen',
        parameters=[{
            'min_area': 3000,
            'rect_ratio': 0.85,
            'resize_width': 640,
            'blur_kernel_size': 5,
            'run_mode': 'debug'
        }]
    )
    
    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': LaunchConfiguration('robot_description_file'),
            'robot_description_semantic': LaunchConfiguration('robot_description_semantic_file')
        }]
    )
    
    # 手眼标定变换发布器（右臂）
    hand_eye_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hand_eye_tf',
        arguments=['-0.02', '0.254', '0.69', '-0.624', '0.671', '-0.281', '0.284', 'base_link', 'camera_link']
    )
    
    # 手眼标定变换发布器（左臂）
    left_hand_eye_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_hand_eye_tf',
        arguments=['-0.01', '0.58', '0.02', '0', '0', '0', 'base_link', 'base_link_l']
    )
    
    # 绿色到红色变换发布器
    green_to_red_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='green_to_red_tf',
        arguments=['0.0', '-0.09', '0.0', '0.0', '0.0', '0.0', 'grab_link', 'grab_red_link']
    )
    
    return LaunchDescription([
        robot_description_file_arg,
        robot_description_semantic_file_arg,
        color_detect_node,
        robot_state_publisher_node,
        hand_eye_tf_node,
        left_hand_eye_tf_node,
        green_to_red_tf_node
    ]) 