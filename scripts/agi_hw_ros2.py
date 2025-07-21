#!/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String, Float64MultiArray, Int32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import time
import requests
from datetime import datetime
import json
import copy
import argparse
#flag to avoid tow thread 
from scipy.spatial.transform import Rotation

#**************** tools *****************#
def quaternion_to_rpy_scipy(x, y, z, w):
    """使用 scipy 转换四元数到 RPY (弧度)"""
    rot = Rotation.from_quat([x, y, z, w])  # 注意顺序: [x, y, z, w]
    roll, pitch, yaw = rot.as_euler('xyz', degrees=False)  # 'xyz' 对应 RPY
    return roll, pitch, yaw

def rpy_to_quaternion(roll, pitch, yaw, degrees=False):
    """
    将欧拉角 (Roll, Pitch, Yaw) 转换为四元数 [x, y, z, w]
    
    参数:
        roll (float): X轴旋转角度
        pitch (float): Y轴旋转角度
        yaw (float): Z轴旋转角度
        degrees (bool): 输入是否为角度（默认弧度）
    返回:
        np.ndarray: 四元数 [x, y, z, w]
    """
    # 创建旋转对象（欧拉角顺序为 'xyz'，对应RPY）
    rot = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=degrees)
    # 转换为四元数（顺序: x, y, z, w）
    return rot.as_quat()

def check_rclpy_init():
    if not rclpy.ok():
        rclpy.init()  # 如果未初始化，则调用初始化
        print("rclpy.init() called")
    else:
        print("rclpy already initialized")
#--------------- tools ----------------#

class AGI_CONFIG():
  def __init__(self):
    self.target_action = "PLANNING_MOVE"
    
    self.left_arm_joints = [
        {"name": "idx12_left_arm_joint1", "limit": (-6.28, 6.28)},
        {"name": "idx13_left_arm_joint2", "limit": (-1.570, 0.785)},
        {"name": "idx14_left_arm_joint3", "limit": (-6.28, 6.28)},
        {"name": "idx15_left_arm_joint4", "limit": (-2.530, 0.523)},
        {"name": "idx16_left_arm_joint5", "limit": (-6.283, 6.283)},
        {"name": "idx17_left_arm_joint6", "limit": (-1.570, 1.570)},
        {"name": "idx18_left_arm_joint7", "limit": (-6.283, 6.283)}
    ]

    self.right_arm_joints = [
        {"name": "idx19_right_arm_joint1", "limit": (-6.28, 6.28)},
        {"name": "idx20_right_arm_joint2", "limit": (-1.570, 0.785)},
        {"name": "idx21_right_arm_joint3", "limit": (-6.28, 6.28)},
        {"name": "idx22_right_arm_joint4", "limit": (-2.530, 0.523)},
        {"name": "idx23_right_arm_joint5", "limit": (-6.283, 6.283)},
        {"name": "idx24_right_arm_joint6", "limit": (-1.570, 1.570)},
        {"name": "idx25_right_arm_joint7", "limit": (-6.283, 6.283)}
    ]

    self.head_joints = [
        {"name": "idx11_head_joint", "limit": (-1.02, 1.02)}
    ]

    self.waist_lift_joints = [
        {"name": "idx09_lift_joint", "limit": (-0.265, 0.265)}
    ]

    self.waist_pitch_joints = [
        {"name": "idx10_waist_joint", "limit": (0.053, 1.55)}
    ]

    self.left_init_tcp =  {"pos": {"x": 0.8, "y": 0.21, "z": 1.18},\
                           "ori": {"roll": -np.pi/2, "pitch": 0, "yaw": -np.pi/2}}
    self.right_init_tcp =  {"pos": {"x": 0.8005, "y": -0.244, "z": 1.1775},\
                           "ori": {"roll": np.pi, "pitch": -np.pi/2, "yaw": 0}}
    # x y z qx qy qz qw
    self.left_tool_tcp = [0, 0, 0.2, 0, 0, -0.707, 0.707]
    self.right_tool_tcp = [0, 0, 0.2, 0, 0, 0.707, 0.707]

class arm_hw(Node):
  def __init__(self,name="agi_arm_hw"):
    super().__init__(name)

    parser = argparse.ArgumentParser()
    parser.add_argument("--collect_data", type=bool, default=False, help="collect data mode")
    parser.add_argument("--tcp_in_joint", type=bool, default=False, help="tcp merge into joint_state mode")
    parser.add_argument("--arm_state_enable", type=bool, default=True, help="turn on or turn off arm state feedback")
    self.args = parser.parse_args()

    # 网络配置 - 从配置文件或环境变量获取
    self.server_ip = "127.0.0.1"
    self.server_port = 56322
    self.hand_port = 56422
    self.config = AGI_CONFIG()
    
    # ros publisher and subscriber
    self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
    # TCP坐标发布者
    self.left_tcp_pub = self.create_publisher(PoseStamped, '/agi_hw/left_arm_current_pose', 10)
    self.right_tcp_pub = self.create_publisher(PoseStamped, '/agi_hw/right_arm_current_pose', 10)
    # self.cmd_right_pub = self.create_publisher('/master/joint_right', JointState, queue_size=10)
    # self.cmd_left_pub = self.create_publisher('/master/joint_left', JointState, queue_size=10)
    
    # store the current joint state
    self.current_arm_state=[]
    self.arm_state_update_flag=False
    
    # 末端夹爪数值
    self.gripper_value = [0,0]

    # 初始化机械臂运动状态，如果是在
    if not self.args.collect_data:
      self.check_and_set_action(self.config.target_action)

    # 定时器：0.5秒周期调用
    if self.args.arm_state_enable:
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

    self.arm_running_flag = False

    # 订阅左臂和右臂的坐标控制指令
    # 使用标准的ROS2 PoseStamped消息类型
    self.left_arm_pose_sub = self.create_subscription(
        PoseStamped,
        '/agi_hw/left_arm_target_pose',
        self.left_arm_pose_callback,
        10
    )
    
    self.right_arm_pose_sub = self.create_subscription(
        PoseStamped,
        '/agi_hw/right_arm_target_pose',
        self.right_arm_pose_callback,
        10
    )

    # 新增：订阅左臂和右臂的关节空间控制指令
    self.left_arm_joint_sub = self.create_subscription(
        Float64MultiArray,
        '/agi_hw/left_arm_joint_cmd',
        self.left_arm_joint_callback,
        10
    )
    self.right_arm_joint_sub = self.create_subscription(
        Float64MultiArray,
        '/agi_hw/right_arm_joint_cmd',
        self.right_arm_joint_callback,
        10
    )

    self.waist_lift_head_sub = self.create_subscription(
        Float64MultiArray,
        '/agi_hw/waist_lift_head_cmd',
        self.waist_lift_head_callback,
        10
    )

    # 创建夹爪控制话题订阅
    self.gripper_control_sub = self.create_subscription(
        Int32MultiArray,
        '/agi_hw/gripper_control',
        self.gripper_control_callback,
        10
    )

    # 打印网络配置信息
    self.get_logger().info(f"机械臂服务器配置: {self.server_ip}:{self.server_port}")
    self.get_logger().info(f"夹爪服务器配置: {self.server_ip}:{self.hand_port}")

    # 打印 self.args 中所有参数
    if hasattr(self, 'args'):
        self.get_logger().info("启动参数列表：")
        for k, v in vars(self.args).items():
            self.get_logger().info(f"  {k}: {v}")
    else:
        self.get_logger().info("未检测到 self.args")

    #延时等待ros服务初始化完毕
    time.sleep(0.5)
    
  #************** 机械臂控制应用层 **************#
  # 主循环函数
  def timer_callback(self):
    self.publish_joint_state_ros2()
    if not self.args.tcp_in_joint:
        self.publish_tcp_poses_ros2()  # 发布TCP坐标
    #self.get_current_gripper()
  
  # 初始化机械臂
  def init_arm(self):
    left_pos = self.config.left_init_tcp["pos"]
    left_ori = self.config.left_init_tcp["ori"]
    # 转换为四元数
    left_quat = rpy_to_quaternion(left_ori["roll"], left_ori["pitch"], left_ori["yaw"], degrees=False)
    left_quat = {"x": left_quat[0], "y": left_quat[1], "z": left_quat[2], "w": left_quat[3]}
    # 调用linear_move控制左臂
    resp = self.linear_move('left', left_pos, left_quat, None, None)

    if resp.status_code == 200:
        self.get_logger().info(f'左臂运动指令执行成功: 位置({left_pos["x"]:.3f}, {left_pos["y"]:.3f}, {left_pos["z"]:.3f}, qx:{left_quat["x"]}, qy:{left_quat["y"]}, qz:{left_quat["z"]}, qw:{left_quat["w"]})')
    else:
        self.get_logger().error(f'左臂运动指令执行失败: HTTP {resp.status_code}')

    # right_pos = self.config.right_init_tcp["pos"]
    # right_ori = self.config.right_init_tcp["ori"]
    # # 转换为四元数
    # right_quat = rpy_to_quaternion(right_ori["roll"], right_ori["pitch"], right_ori["yaw"], degrees=False)
    # # right_quat = {"x": right_quat[0], "y": right_quat[1], "z": right_quat[2], "w": right_quat[3]}
    # right_quat = {"x": -0.5, "y": 0.5, "z": -0.5, "w": 0.5}
    # # 调用linear_move控制左臂
    # resp = self.linear_move('right', None, None, right_pos, right_quat)

    # if resp.status_code == 200:
    #     self.get_logger().info(f'右臂运动指令执行成功: 位置({right_pos["x"]:.3f}, {right_pos["y"]:.3f}, {right_pos["z"]:.3f}, qx:{right_quat["x"]}, qy:{right_quat["y"]}, qz:{right_quat["z"]}, qw:{right_quat["w"]})')
    # else:
    #     self.get_logger().error(f'右臂运动指令执行失败: HTTP {resp.status_code}')

    self.move_arm(0.8, -0.24, 1.1)
    return

  def move_arm(self,cmd_x,cmd_y,cmd_z):
    # 移动右臂
    right_pose = PoseStamped()
    right_pose.header.stamp = self.get_clock().now().to_msg()
    right_pose.header.frame_id = "base_link"
    right_pose.pose.position.x = cmd_x
    right_pose.pose.position.y = cmd_y
    right_pose.pose.position.z = cmd_z
    right_pose.pose.orientation.x = -0.5
    right_pose.pose.orientation.y = 0.5
    right_pose.pose.orientation.z = -0.5
    right_pose.pose.orientation.w = 0.5
    # right_pose.pose.orientation.x = 0.7
    # right_pose.pose.orientation.y = 0.0
    # right_pose.pose.orientation.z = 0.7
    # right_pose.pose.orientation.w = 0.0
    self.right_arm_pose_callback(right_pose)
    self.get_logger().info(f"move_arm: cmd_x:{cmd_x}, cmd_y:{cmd_y}, cmd_z:{cmd_z}")

  def rotate_gripper(self):
    left_tcp, right_tcp = self.get_arm_flange()
    right_pose = PoseStamped()
    right_pose.header.stamp = self.get_clock().now().to_msg()
    right_pose.header.frame_id = "base_link"
    right_pose.pose.position.x = right_tcp[0]
    right_pose.pose.position.y = right_tcp[1]
    right_pose.pose.position.z = right_tcp[2]
    # right_pose.pose.orientation.x = -0.5
    # right_pose.pose.orientation.y = 0.5
    # right_pose.pose.orientation.z = -0.5
    # right_pose.pose.orientation.w = 0.5
    right_pose.pose.orientation.x = 0.707
    right_pose.pose.orientation.y = 0.0
    right_pose.pose.orientation.z = 0.707
    right_pose.pose.orientation.w = 0.0
    self.right_arm_pose_callback(right_pose)
    self.get_logger().info(f"move_arm: cmd_x:{right_pose.pose.position.x}, cmd_y:{right_pose.pose.position.y}, cmd_z:{right_pose.pose.position.z}")
    return

  # 发布关节状态到ros2
  def publish_joint_state_ros2(self):

    joint_state = self.get_current_joint_state()

    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()

    # 提取关节名称和角度
    msg.name = joint_state[0]  # 获取所有关节名称
    msg.position = joint_state[1]  # 获取所有关节角度 

    # 添加gripper状态
    gripper_state = self.get_current_gripper()
    if gripper_state[0] is not None and gripper_state[1] is not None:
      msg.name.append("idx26_left_arm_gripper")
      msg.name.append("idx27_right_arm_gripper")
      msg.position.append(gripper_state[0])
      msg.position.append(gripper_state[1])
    else:
      self.get_logger().error("AGI_ARM_HW: get_current_gripper failed!")

    # 添加tcp状态
    if self.args.tcp_in_joint:
        left_tcp, right_tcp = self.get_arm_flange()
        # 名称前缀和分量
        arms = [("left", left_tcp), ("right", right_tcp)]
        fields = ["x", "y", "z", "qx", "qy", "qz", "qw"]

        for idx, (arm_name, tcp) in enumerate(arms):
            base = 28 + idx * 7  # left:28, right:35
            msg.name.extend([f"idx{base+i}_{arm_name}_arm_tcp_{field}" for i, field in enumerate(fields)])
            msg.position.extend(tcp)

    # 发布消息
    self.joint_pub.publish(msg)

  # 发布TCP坐标到ros2
  def publish_tcp_poses_ros2(self):
    try:
        # 获取两臂法兰TCP坐标
        left_tcp_pos,left_tcp_ori,right_tcp_pos,right_tcp_ori = self.get_arm_tcp()

        if left_tcp_pos is None:
            self.get_logger().warn("Publish tcp poses Failed")
            return
      # --- 左臂 ---
        left_msg = PoseStamped()
        left_msg.header.stamp = self.get_clock().now().to_msg()
        left_msg.header.frame_id = "base_link"
        left_msg.pose.position.x = left_tcp_pos[0]
        left_msg.pose.position.y = left_tcp_pos[1]
        left_msg.pose.position.z = left_tcp_pos[2]
        left_msg.pose.orientation.x = left_tcp_ori[0]
        left_msg.pose.orientation.y = left_tcp_ori[1]
        left_msg.pose.orientation.z = left_tcp_ori[2]
        left_msg.pose.orientation.w = left_tcp_ori[3]
        self.left_tcp_pub.publish(left_msg)
      # --- 右臂 ---
        right_msg = PoseStamped()
        right_msg.header.stamp = self.get_clock().now().to_msg()
        right_msg.header.frame_id = "base_link"
        right_msg.pose.position.x = right_tcp_pos[0]
        right_msg.pose.position.y = right_tcp_pos[1]
        right_msg.pose.position.z = right_tcp_pos[2]
        right_msg.pose.orientation.x = right_tcp_ori[0]
        right_msg.pose.orientation.y = right_tcp_ori[1]
        right_msg.pose.orientation.z = right_tcp_ori[2]
        right_msg.pose.orientation.w = right_tcp_ori[3]
        self.right_tcp_pub.publish(right_msg)

    except Exception as e:
      self.get_logger().error(f"发布工具TCP坐标失败: {str(e)}")

  def get_arm_tcp(self):
      # 获取两臂法兰TCP坐标
      left_tcp, right_tcp = self.get_arm_flange()
      # 工具相对法兰的变换
      left_tool = list(self.config.left_tool_tcp)
      right_tool = list(self.config.right_tool_tcp)

      if left_tcp is None or right_tcp is None:
        self.get_logger().warn("Get arm flange Failed!")
        return None,None,None,None

      # --- 左臂 ---
      if left_tcp is not None:
        # 法兰四元数
        flange_rot = Rotation.from_quat(left_tcp[3:7])
        # 工具四元数
        tool_rot = Rotation.from_quat(left_tool[3:7])
        # 合成四元数
        left_tool_rot_abs = flange_rot * tool_rot
        # 工具位置（先旋转再平移）
        left_tool_pos = np.array(left_tcp[0:3]) + flange_rot.apply(left_tool[0:3])
        left_q = left_tool_rot_abs.as_quat()  # x, y, z, w


      # --- 右臂 ---
      if right_tcp is not None:
        flange_rot = Rotation.from_quat(right_tcp[3:7])
        tool_rot = Rotation.from_quat(right_tool[3:7])
        right_tool_rot_abs = flange_rot * tool_rot

        right_tool_pos = np.array(right_tcp[0:3]) + flange_rot.apply(right_tool[0:3])
        right_q = right_tool_rot_abs.as_quat()

        return left_tool_pos,left_q,right_tool_pos,right_q


  def get_arm_flange(self):
    left_tcp = self.get_transformation("left_arm_link07")
    right_tcp = self.get_transformation("right_arm_link07")
    return self.extract_pose(left_tcp), self.extract_pose(right_tcp)

  #------------- 机械臂控制应用层 -------------#


  #************** http网络请求 **************#
  def get_current_joint_state(self):
    url = f'http://{self.server_ip}:{self.server_port}/rpc/aimdk.protocol.McDataService/GetJointAngle'
    headers = {'Content-Type': 'application/json'}
    response = requests.post(url, headers=headers, json={})
    if response.status_code == 200:
      data = response.json()
      joints = data.get('joints', [])
      if not joints:
        return None

    joint_name = [joint['name'] for joint in joints]  # 获取所有关节名称
    joint_angle = [float(joint['position']) for joint in joints]  # 获取所有关节角度

    return joint_name,joint_angle
  
  # 获取夹爪状态
  def get_current_gripper(self):
    # curl -i \
    # -H 'content-type:application/json' \
    # -X POST  'http://127.0.0.1:56422/rpc/aimdk.protocol.HalHandService/GetHandState' \
    # -d '{"header":{"timestamp":{"seconds":"0","nanos":0,"ms_since_epoch":"0"}}}'
    url = f'http://{self.server_ip}:{self.hand_port}/rpc/aimdk.protocol.HalHandService/GetHandState'
    headers = {'Content-Type': 'application/json'}
    response = requests.post(url, headers=headers, json={})
    if response.status_code == 200:
      data = response.json()
      data_info = data.get('data', {})
      left_pos = data_info.get('left', {}).get('agi_claw_state', {}).get('pos', 0)
      right_pos = data_info.get('right', {}).get('agi_claw_state', {}).get('pos', 0)
      return left_pos, right_pos
    return None, None  # 如果请求失败，返回None
  
  # http网络请求获取agibot a2w获取当前action状态
  # 返回：字符串
  def get_current_action(self):
      url = f'http://{self.server_ip}:{self.server_port}/rpc/aimdk.protocol.McActionService/GetAction'
      headers = {'Content-Type': 'application/json'}
      response = requests.post(url, headers=headers, json={})
      if response.status_code == 200:
          data = response.json()
          info = data.get('info', {})
          action = info.get('ext_action', '')
          return action,data
      return "UNKNOWN"

  def get_transformation(self,source_frame):
    url = f"http://192.168.100.110:50100/rpc/aimdk.protocol.TransFormService/GetTransFormation"

    headers = {
        "content-type": "application/json"
    }
    data = {
        "target_frame": "base_link",
        "source_frame": source_frame
    }
    try:
        response = requests.post(url, headers=headers, json=data, timeout=1)
        if response.status_code == 200:
            return response.json()
        else:
            print(f"Request failed: {response.status_code}, {response.text}")
    except requests.exceptions.RequestException as e:
        print(f"Exception during request: {e}")
    return None

  # 创建头信息
  def create_header(self):
      now = datetime.utcnow()
      return {
          "timestamp": {
              "seconds": int(now.timestamp()),
              "nanos": now.microsecond * 1000,
              "ms_since_epoch": int(now.timestamp() * 1000),
          },
          "control_source": "ControlSource_MANUAL"
      }

  # 设置action运动状态
  def set_action(self, selected_action):
      url = f'http://{self.server_ip}:{self.server_port}/rpc/aimdk.protocol.McActionService/SetAction'
      headers = {'Content-Type': 'application/json'}
      payload = {
          "header": self.create_header(),
          "command": {
              "action": "McAction_USE_EXT_CMD",
              "ext_action": selected_action
          }
      }
      return requests.post(url, headers=headers, json=payload)

  # 控制机械臂前必须要设置，类似使能
  def check_and_set_action(self, selected_action='PLANNING_MOVE'):
      while True:
          current_action = self.get_current_action()[0]
          if current_action == selected_action:
              break
          self.set_action(selected_action)
          print("Setting action:", selected_action)
          time.sleep(0.5)

  # 控制机械臂运动，可单独控制左臂或者右臂，以及同时控制双臂
  # 示例：
  # left_position = {"x": 0.8005, "y": 0.216, "z": 1.1775}
  # left_orientation = {"x": 0.5, "y": -0.5, "z": 0.5, "w": -0.5}
  # check_and_set_action(target_action)
  # res = linear_move('right',left_position,left_orientation,right_position,right_orientation)  
  def linear_move(self, arm_choice, left_pos, left_ori, right_pos, right_ori, ip=None, port=None):
      # 使用类成员变量作为默认值
      if ip is None:
          ip = self.server_ip
      if port is None:
          port = self.server_port
          
      url = f"http://{ip}:{port}/rpc/aimdk.protocol.McMotionService/LinearMove"
      headers = {"Content-Type": "application/json"}

      if arm_choice == 'left':
          payload = {
              "group": "McPlanningGroup_LEFT_ARM",
              "target": {
                  "left": {
                      "position": left_pos,
                      "orientation": left_ori
                  }
              }
          }
      elif arm_choice == 'right':
          payload = {
              "group": "McPlanningGroup_RIGHT_ARM",
              "target": {
              "right": {
                  "position": right_pos,
                  "orientation": right_ori
                  }
              }
          }
      elif arm_choice == 'dual':
          payload = {
          "group": "McPlanningGroup_DUAL_ARM",
          "target": {
              "left": {
                  "position": left_pos,
                  "orientation": left_ori
              },
              "right": {
                  "position": right_pos,
                  "orientation": right_ori
              }
          }
      }
      else:
          print("wrong arm cmd!")

      response = requests.post(url, headers=headers, json=payload)
      return response

  def planning_move(self, arm_choice, left_pos, left_ori, right_pos, right_ori, ip=None, port=None):
      # 使用类成员变量作为默认值
      if ip is None:
          ip = self.server_ip
      if port is None:
          port = self.server_port
        
      url = f"http://{ip}:{port}/rpc/aimdk.protocol.McMotionService/PlanningMove"
      headers = {"Content-Type": "application/json"}

      if arm_choice == 'left':
          payload = {
          "group": "McPlanningGroup_LEFT_ARM",
          "target": {
                "type": "SE3",
                "left": {
                    "position": left_pos,
                    "orientation": left_ori
                }
              }
          }
      elif arm_choice == 'right':
          payload = {
          "group": "McPlanningGroup_RIGHT_ARM",
          "target": {
              "type": "SE3",
              "right": {
                  "position": right_pos,
                  "orientation": right_ori
                  }
              }
          }
      elif arm_choice == 'dual':
          payload = {
          "group": "McPlanningGroup_DUAL_ARM",
          "target": {
              "type": "SE3",
              "left": {
                  "position": left_pos,
                  "orientation": left_ori
              },
              "right": {
                  "position": right_pos,
                  "orientation": right_ori
              }
          }
      }
      else:
          print("wrong arm cmd!")
      response = requests.post(url, headers=headers, json=payload)
      return response

  # 控制腰关节和升降关节运动
  # 示例：
  # move_waist_lift_head(lift_angle=0.05, pitch_angle=0.2, head_angle=0.3)
  # 限位：
  # {"name": "idx09_lift_joint", "limit": (-0.265, 0.265)}
  # {"name": "idx10_waist_joint", "limit": (0.053, 1.55)}
  # {"name": "idx11_head_joint", "limit": (-1.02, 1.02)}
  def move_waist_lift_head(self,
      lift_angle: float,
      pitch_angle: float,
      head_angle: float,
      *,
      timeout: float = 1.0
  ) -> None:

      url = f'http://{self.server_ip}:{self.server_port}/rpc/aimdk.protocol.McMotionService/JointMove'

      # 1. 限位检查
      if not (self.config.waist_lift_joints[0]['limit'][0] <= lift_angle <= self.config.waist_lift_joints[0]['limit'][1]):
          raise ValueError(f"lift_angle {lift_angle} 超出限位 {self.config.waist_lift_joints[0]['limit']}")
      if not (self.config.waist_pitch_joints[0]['limit'][0] <= pitch_angle <= self.config.waist_pitch_joints[0]['limit'][1]):
          raise ValueError(f"pitch_angle {pitch_angle} 超出限位 {self.config.waist_pitch_joints[0]['limit']}")
      if not (self.config.head_joints[0]['limit'][0] <= head_angle <= self.config.head_joints[0]['limit'][1]):
          raise ValueError(f"head_angle {head_angle} 超出限位 {self.config.head_joints[0]['limit']}")
      # 2. 共用时间戳的 header
      header = self.create_header()

      # 3. 拼装两条 payload
      payloads = [
          {
              "header": copy.deepcopy(header),
              "group": "McPlanningGroup_WAIST_LIFT",
              "angles": [lift_angle],
          },
          {
              "header": copy.deepcopy(header),
              "group": "McPlanningGroup_WAIST_PITCH",
              "angles": [pitch_angle],
          },
          {
              "header": copy.deepcopy(header),
              "group": "McPlanningGroup_HEAD",
              "angles": [head_angle],
          },
      ]

      #url = f"{host}{service}"
      for p in payloads:
          resp = requests.post(url, json=p, timeout=timeout)
          resp.raise_for_status()          # 出错直接抛异常更安全
          # 实测 5~10 ms 就够，取决于控制器循环周期；这里保守留   20 ms
          time.sleep(0.02)

  def joint_move(self, group: str, angles: list, velocity_scale: float = 0.5, acceleration_scale: float = 0.9) -> bool:
        """执行关节运动

        Args:
            group: 规划组 ("McPlanningGroup_LEFT_ARM", "McPlanningGroup_RIGHT_ARM", "McPlanningGroup_DUAL_ARM")
            angles: 关节角度列表 (弧度)
            velocity_scale: 速度比例 (0-1)
            acceleration_scale: 加速度比例 (0-1)

        Returns:
            bool: 执行是否成功
        """
        url = f"http://{self.server_ip}:{self.server_port}/rpc/aimdk.protocol.McMotionService/JointMove"
        headers = {"Content-Type": "application/json"}

        payload = {
            "group": group,
            "param": {
                "velocity_scale": velocity_scale,
                "acceleration_scale": acceleration_scale
            },
            "angles": angles
        }
        try:
            response = requests.post(url, headers=headers, json=payload, timeout=5)

            if response.status_code == 200:
                result = response.json()
                return True
            else:
                print(f"关节运动请求失败: {response.status_code}, {response.text}")
                return False

        except Exception as e:
            print(f"关节运动执行异常: {e}")
            return False
  # 设置夹爪状态
  def set_gripper_value(self, left_pos=100, right_pos=100, left_vel=100, right_vel=100, 
                       left_force=60, right_force=80, left_cmd=0, right_cmd=0,
                       left_clamp_method=0, right_clamp_method=0, 
                       left_finger_pos=0, right_finger_pos=0):
    """
    设置夹爪状态
    
    参数:
    - left_pos/right_pos: 左/右夹爪行程，即夹爪张开的行程 (0-100)
    - left_vel/right_vel: 左/右夹爪速度 (0-100)
    - left_force/right_force: 左/右夹爪力度 (0-100)
    - left_cmd/right_cmd: 左/右夹爪命令 (0: TODO, 1: TODO, 2: TODO)默认为0
    - left_clamp_method/right_clamp_method: 左/右夹爪控制指令 (0: 复位, 1: 抓, 2: 放)
    - left_finger_pos/right_finger_pos: 左/右夹爪手指位置:TODO:默认为零
    """
    url = f'http://{self.server_ip}:{self.hand_port}/rpc/aimdk.protocol.HalHandService/SetHandCommand'
    headers = {'Content-Type': 'application/json'}
    
    payload = {
        "header": self.create_header(),
        "data": {
            "left": {
                "agi_claw_cmd": {
                    "cmd": left_cmd,
                    "pos": left_pos,
                    "vel": left_vel,
                    "force": left_force,
                    "clamp_method": left_clamp_method,
                    "finger_pos": left_finger_pos
                }
            },
            "right": {
                "agi_claw_cmd": {
                    "cmd": right_cmd,
                    "pos": right_pos,
                    "vel": right_vel,
                    "force": right_force,
                    "clamp_method": right_clamp_method,
                    "finger_pos": right_finger_pos
                }
            }
        }
    }
    
    try:
        response = requests.post(url, headers=headers, json=payload)
        if response.status_code == 200:
            self.get_logger().info(f'夹爪控制指令执行成功: 左夹爪位置={left_pos}, 右夹爪位置={right_pos}')
            return True
        else:
            self.get_logger().error(f'夹爪控制指令执行失败: HTTP {response.status_code}')
            return False
    except Exception as e:
        self.get_logger().error(f'夹爪控制请求失败: {str(e)}')
        return False
    
  #-------------  http网络请求 -------------#
  
  # get current arm joint angle
  def get_current_arm_state(self):
    if self.current_arm_state:
      return self.current_arm_state
    else:
      return None
          
  def gripper_control(self, left_cmd, right_cmd):
    """
    夹爪控制函数
    
    参数:
    - left_cmd: 左夹爪 (0: 放, 1: 抓)
    - right_cmd: 右夹爪 (0: 放, 1: 抓)

    - left_cmd: 原始左夹爪控制指令 (0: 复位, 1: 抓, 2: 放)
    - right_cmd: 原始右夹爪控制指令 (0: 复位, 1: 抓, 2: 放)
    """

    # 逻辑变换
    if left_cmd == 0:
        left_cmd = 2
    else:
        self.get_logger().warn(f'夹爪控制cmd指令非法! left_cmd:{left_cmd},right_cmd:{right_cmd}')
        return

    try:
        success = self.set_gripper_value(left_clamp_method=left_cmd, right_clamp_method=right_cmd)
        if success:
            self.get_logger().info(f'夹爪控制成功: 左夹爪={left_cmd}, 右夹爪={right_cmd}')
        return success
    except Exception as e:
        self.get_logger().error(f'夹爪控制失败: {str(e)}')
        return False 

  def motor_add_control(self,joint,angle,puppet):
    return
      
  def aim_arm(self,gripper,puppet):
    cmd = JointState()
    cmd.header.stamp = self.get_clock().now().to_msg()
    cmd.name = ['joint0', 'joint1', 'joint2', 'joint3','joint4' ,'joint5','joint6']
    if puppet=="right":
      cmd.position = [1.329, 1.772, 0.83, 1.077, 1.367, 0]
    elif puppet=="left":
      cmd.position = [-1.329, 1.772, 0.83, 1.077, -1.367, 0]


  def fold_arm(self,gripper,puppet):
    return
  
  def list1_sub_list2(self,list1,list2):
    if len(list1) != len(list2):
      self.get_logger().error(f"list1:{list1}")
      self.get_logger().error(f"list2:{list2}")
      raise ValueError("List1 sub List2 failed!")
    result = [a - b for a,b in zip(list1,list2)]
    return result

  # 左臂坐标控制回调
  def left_arm_pose_callback(self, msg):
    start_time = time.time()  # 记录开始时间
    if self.arm_running_flag == True:
        self.get_logger().warn(f'abord left linear_move api')
        return
    self.arm_running_flag = True
    try:
        
        # 从PoseStamped消息中提取位置和姿态
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        # 转换为linear_move需要的格式
        left_pos = {"x": pos.x, "y": pos.y, "z": pos.z}
        left_ori = {"x": ori.x, "y": ori.y, "z": ori.z, "w": ori.w}
        # 获取roll pitch yaw角
        roll, pitch, yaw = quaternion_to_rpy_scipy(ori.x, ori.y, ori.z, ori.w)

        # # 初始化机械臂运动状态
        # self.check_and_set_action(self.config.target_action)
        
        # 调用linear_move控制左臂
        resp = self.planning_move('left', left_pos, left_ori, None, None)
        
        if resp.status_code == 200:
            self.get_logger().info(f'左臂运动指令执行成功: 位置({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}), roll:{roll},pitch:{pitch},yaw:{yaw}, ori:{left_ori}')
        else:
            self.get_logger().error(f'左臂运动指令执行失败: HTTP {resp.status_code}')
        self.arm_running_flag = False
            
    except Exception as e:
        self.get_logger().error(f'左臂运动指令处理失败: {str(e)}')
        self.arm_running_flag = False
    end_time = time.time()  # 记录结束时间
    elapsed_ms = (end_time - start_time) * 1000  # 转为毫秒
    self.get_logger().info(f"left_arm_pose_callback耗时: {elapsed_ms:.2f} ms")

  # 右臂坐标控制回调
  def right_arm_pose_callback(self, msg):
    start_time = time.time()  # 记录开始时间
    if self.arm_running_flag == True:
        self.get_logger().info(f'abord right linear_move api')
        return
    try:
        self.arm_running_flag = True
        # 从PoseStamped消息中提取位置和姿态
        pos = msg.pose.position
        ori = msg.pose.orientation
        
        # 转换为linear_move需要的格式
        right_pos = {"x": pos.x, "y": pos.y, "z": pos.z}
        right_ori = {"x": ori.x, "y": ori.y, "z": ori.z, "w": ori.w}
        
        # 获取roll pitch yaw角
        #roll, pitch, yaw = quaternion_to_rpy_scipy(ori.x, ori.y, ori.z, ori.w)

        # roll = np.pi
        # pitch = -np.pi/2
        # yaw = 0
        # right_quat = rpy_to_quaternion(roll, pitch, yaw, degrees=False)

        # right_ori = {"x": right_quat[0] ,"y": right_quat[1] ,"z": right_quat[2] ,"w": right_quat[3]}

        # # 初始化机械臂运动状态
        # self.check_and_set_action(self.config.target_action)
        
        # 调用linear_move控制右臂
        resp = self.planning_move('right', None, None, right_pos, right_ori)
        
        if resp.status_code == 200:
            self.get_logger().info(f'右臂运动指令执行成功: 位置({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})')
        else:
            self.get_logger().error(f'右臂运动指令执行失败: HTTP {resp.status_code}')
        self.arm_running_flag = False
            
    except Exception as e:
        self.get_logger().error(f'右臂运动指令处理失败: {str(e)}')
        self.arm_running_flag = False
    end_time = time.time()  # 记录结束时间
    elapsed_ms = (end_time - start_time) * 1000  # 转为毫秒
    self.get_logger().info(f"right_arm_pose_callback耗时: {elapsed_ms:.2f} ms")

  # 回调函数
  def waist_lift_head_callback(self, msg):
      try:
          if len(msg.data) != 3:
              self.get_logger().error('腰部升降头部控制指令长度必须为3')
              return
          lift_angle, pitch_angle, head_angle = msg.data
          self.move_waist_lift_head(lift_angle, pitch_angle, head_angle)
          self.get_logger().info(f'腰部升降头部控制: 升降={lift_angle:.3f}, 俯仰={pitch_angle:.3f}, 头部={head_angle:.3f}')
      except Exception as e:
          self.get_logger().error(f'腰部升降头部控制失败: {str(e)}')

  # 夹爪控制话题回调函数
  def gripper_control_callback(self, msg):
      """
      夹爪控制话题回调函数
      消息格式: [left_cmd, right_cmd, left_pos, right_pos, left_vel, right_vel, left_force, right_force]
      - left_cmd/right_cmd: 左/右夹爪控制指令 (0: 复位, 1: 抓, 2: 放)
      - left_pos/right_pos: 左/右夹爪行程 (0-100)
      - left_vel/right_vel: 左/右夹爪速度 (0-100)
      - left_force/right_force: 左/右夹爪力度 (0-100)
      """
      try:
          if len(msg.data) < 2:
              self.get_logger().error('夹爪控制指令长度至少为2 [left_cmd, right_cmd]')
              return
          
          # 提取基本参数
          left_cmd = msg.data[0]
          right_cmd = msg.data[1]
          
          # 设置默认值
          left_pos = 100 if len(msg.data) <= 2 else msg.data[2]
          right_pos = 100 if len(msg.data) <= 3 else msg.data[3]
          left_vel = 100 if len(msg.data) <= 4 else msg.data[4]
          right_vel = 100 if len(msg.data) <= 5 else msg.data[5]
          left_force = 60 if len(msg.data) <= 6 else msg.data[6]
          right_force = 60 if len(msg.data) <= 7 else msg.data[7]
          
          # 调用夹爪控制函数
          success = self.set_gripper_value(
              left_pos=left_pos,
              right_pos=right_pos,
              left_vel=left_vel,
              right_vel=right_vel,
              left_force=left_force,
              right_force=right_force,
              left_cmd=0,  # 默认命令类型
              right_cmd=0,  # 默认命令类型
              left_clamp_method=left_cmd,
              right_clamp_method=right_cmd,
              left_finger_pos=0,
              right_finger_pos=0
          )
          
          if success:
              self.get_logger().info(f'夹爪控制成功: 左夹爪({left_cmd}, {left_pos}), 右夹爪({right_cmd}, {right_pos})')
          else:
              self.get_logger().error('夹爪控制失败')
              
      except Exception as e:
          self.get_logger().error(f'夹爪控制话题处理失败: {str(e)}')

          
  def extract_pose(self,tcp_json):
      try:
          t = tcp_json["transform_stamped"]["transform"]
          p = t["position"]
          q = t["orientation"]
          return [p["x"], p["y"], p["z"], q["x"], q["y"], q["z"], q["w"]]
      except Exception as e:
          print(f"Failed to extract TCP: {e}")
          return None

  # 新增：左臂关节空间控制回调
  def left_arm_joint_callback(self, msg):
      if self.arm_running_flag == True:
        self.get_logger().info(f'abord right linear_move api')
        return True

      """左臂关节空间控制回调"""
      start_time = time.time()  # 记录开始时间

      if len(msg.data) != 7:
          self.get_logger().error(f"左臂关节角度数量错误，应为7，实际为{len(msg.data)}")
          return
      success = self.joint_move(
          group="McPlanningGroup_LEFT_ARM",
          angles=list(msg.data)
      )
      if success:
          self.get_logger().info(f"左臂关节运动指令已下发: {msg.data}")
      else:
          self.get_logger().error("左臂关节运动指令执行失败")

      end_time = time.time()  # 记录结束时间
      elapsed_ms = (end_time - start_time) * 1000  # 转为毫秒
      self.get_logger().info(f"left_arm_joint_callback耗时: {elapsed_ms:.2f} ms")

      self.arm_running_flag = False

  # 新增：右臂关节空间控制回调
  def right_arm_joint_callback(self, msg):
      if self.arm_running_flag == True:
        self.get_logger().info(f'abord right linear_move api')
        return True

      """右臂关节空间控制回调"""
      start_time = time.time()  # 记录开始时间

      if len(msg.data) != 7:
          self.get_logger().error(f"右臂关节角度数量错误，应为7，实际为{len(msg.data)}")
          return
      success = self.joint_move(
          group="McPlanningGroup_RIGHT_ARM",
          angles=list(msg.data)
      )
      if success:
          self.get_logger().info(f"右臂关节运动指令已下发: {msg.data}")
      else:
          self.get_logger().error("右臂关节运动指令执行失败")

      end_time = time.time()  # 记录结束时间
      elapsed_ms = (end_time - start_time) * 1000  # 转为毫秒
      self.get_logger().info(f"right_arm_joint_callback耗时: {elapsed_ms:.2f} ms")

      self.arm_running_flag = False


if __name__=='__main__' :
  rclpy.init()
  node = arm_hw()
#   node.init_arm()
#   time.sleep(1)
#   node.rotate_gripper()
  #node.move_arm(0.75, -0.2, 0.9)
  #time.sleep(0.1)
  #node.move_arm(0.8, -0.24, 1.1)
#   node.move_arm(0.8, -0.24, 1.1)
  rclpy.spin(node)
  rclpy.shutdown()
