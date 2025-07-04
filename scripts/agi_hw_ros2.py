#!/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
import time
import requests
from datetime import datetime
import json
import copy
#flag to avoid tow thread 

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


class arm_hw(Node):
  def __init__(self,name):
    super().__init__(name)
    
    # 网络配置 - 从配置文件或环境变量获取
    self.server_ip = "127.0.0.1"
    self.server_port = 56322
    self.hand_port = 56422
    self.config = AGI_CONFIG()
    
    # ros publisher and subscriber
    self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
    # self.cmd_right_pub = self.create_publisher('/master/joint_right', JointState, queue_size=10)
    # self.cmd_left_pub = self.create_publisher('/master/joint_left', JointState, queue_size=10)
    
    # store the current joint state
    self.current_arm_state=[]
    self.arm_state_update_flag=False
    
    # 末端夹爪数值
    self.gripper_value = [0,0]

    # 定时器：0.5秒周期调用
    timer_period = 0.03
    self.timer = self.create_timer(timer_period, self.timer_callback)

    # 初始化a2w
    self.check_and_set_action(self.config.target_action)
    
    # 打印网络配置信息
    self.get_logger().info(f"机械臂服务器配置: {self.server_ip}:{self.server_port}")
    self.get_logger().info(f"夹爪服务器配置: {self.server_ip}:{self.hand_port}")

    # 订阅linear_move请求，发布linear_move响应
    self.linear_move_req_sub = self.create_subscription(
        String,
        '/agi_hw/linear_move',
        self.linear_move_callback,
        10
    )
    # self.linear_move_resp_pub = self.create_publisher(
    #     String,
    #     '/arm_hw/linear_move/response',
    #     10
    # )

  #************** 机械臂控制应用层 **************#
  # 初始化机械臂
  def init_arm(self):
    return

  # 发布关节状态到ros2
  def publish_joint_state_ros2(self):

    joint_state = self.get_current_joint_state()

    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()

    # 提取关节名称和角度
    msg.name = joint_state[0]  # 获取所有关节名称
    msg.position = joint_state[1]  # 获取所有关节角度 

    gripper_state = self.get_current_gripper()
    if gripper_state[0] is not None and gripper_state[1] is not None:
      msg.name.append("idx26_left_arm_gripper")
      msg.name.append("idx27_right_arm_gripper")
      msg.position.append(gripper_state[0])
      msg.position.append(gripper_state[1])
    else:
      self.get_logger().error("AGI_ARM_HW: get_current_gripper failed!")

    # 发布消息
    self.joint_pub.publish(msg)
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
  
  # 控制腰关节和升降关节运动
  # 示例：
  # move_waist_and_lift(lift_angle=0.05, pitch_angle=0.2)
  # 限位：
  # {"name": "idx09_lift_joint", "limit": (-0.265, 0.265)}
  # {"name": "idx10_waist_joint", "limit": (0.053, 1.55)}
  def move_waist_and_lift(self,
      lift_angle: float,
      pitch_angle: float,
      *,
      host: str = "http://{self.server_ip}:{self.server_port}",
      service: str = "/rpc/aimdk.protocol.McMotionService/JointMove",
      timeout: float = 1.0
  ) -> None:

      # 1. 限位检查
      if not (self.config.waist_lift_joints[0]['limit'][0] <= lift_angle <= self.config.waist_lift_joints[0]['limit'][1]):
          raise ValueError(f"lift_angle {lift_angle} 超出限位 {self.config.waist_lift_joints[0]['limit']}")
      if not (self.config.waist_pitch_joints[0]['limit'][0] <= pitch_angle <= self.config.waist_pitch_joints[0]['limit'][1]):
          raise ValueError(f"pitch_angle {pitch_angle} 超出限位 {self.config.waist_pitch_joints[0]['limit']}")

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
      ]

      url = f"{host}{service}"
      for p in payloads:
          resp = requests.post(url, json=p, timeout=timeout)
          resp.raise_for_status()          # 出错直接抛异常更安全
          # 实测 5~10 ms 就够，取决于控制器循环周期；这里保守留   20 ms
          time.sleep(0.02)

  # TODO: 设置夹爪状态
  def set_gripper_value(self):
    return None
    
  #-------------  http网络请求 -------------#

  def timer_callback(self):
    self.publish_joint_state_ros2()
    self.get_current_gripper()
  
  # get current arm joint angle
  def get_current_arm_state(self):
    if self.current_arm_state:
      return self.current_arm_state
    else:
      return None
          
  def gripper_control(self,gripper,puppet="right"):
    return 

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

  # json请求格式示例
  #{
  #"arm_choice": "left",
  #"left_pos": {"x": 0.8, "y": 0.2, "z": 1.1},
  #"left_ori": {"x": 0.5, "y": -0.5, "z": 0.5, "w": -0.5},
  #"right_pos": null,
  #"right_ori": null
  #}
  def linear_move_callback(self, msg):
    try:
        params = json.loads(msg.data)
        arm_choice = params.get('arm_choice')
        left_pos = params.get('left_pos')
        left_ori = params.get('left_ori')
        right_pos = params.get('right_pos')
        right_ori = params.get('right_ori')
        ip = params.get('ip', None)
        port = params.get('port', None)
        resp = self.linear_move(arm_choice, left_pos, left_ori, right_pos, right_ori, ip, port)
        result = {
            'status': 'success',
            'http_status': resp.status_code,
            'text': resp.text
        }
    except Exception as e:
        result = {
            'status': 'error',
            'error': str(e)
        }
    #resp_msg = String()
    #resp_msg.data = json.dumps(result, ensure_ascii=False)
    #self.linear_move_resp_pub.publish(resp_msg)

if __name__=='__main__' :
  rclpy.init()
  node = arm_hw("agi_arm_hw")
  rclpy.spin(node)
  rclpy.shutdown()
