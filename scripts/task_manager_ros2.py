#!/bin/python3
import sys
import argparse
import rclpy
from rclpy.node import Node
import time
from threading import Thread,Lock,Event
import signal
import json
import math

from std_msgs.msg import String
from ik import ik_caculator
# from arm_hw import arm_hw
#from yolo_detect import map1
from arm_ik.srv import ColorDetector
from agit_hw_ros2 import arm_hw

class task_manager(Node):
    def __init__(self,args):
        self.args = args
        # 初始化机械臂硬件控制器
        self.arm_hw = arm_hw("arm_hw")

        self.cmd_sub = self.create_subscription(String, "/task_cmd", self.task_cmd_callback, 10)
        #self.aim_sub = self.create_subscription(String, "/detect_result", self.aim_callback, 10)
        self.state_pub = self.create_publisher(String, '/task_states', 10)
        self.aim_client = self.create_client(ColorDetector, '/detect_color')
        self.cmd_type = None
        self.aim_dic = {'green':[],'red':[]}
        
        self.task_thread = Thread(target=self.run_task)
        self.task_states_thread = Thread(target=self.pub_state)
        self.update_thread_stop_event = Event()
        self.shutdown = False
        self.task_state = "idle"
    
    def pub_state(self):

        while not self.is_shutdown():
            msg = String()
            msg.data = self.task_state
            self.state_pub.publish(msg)
            time.sleep(0.2)

    def run_task(self):

        while not self.is_shutdown():
            log_flag = False
            # 等待新任务进来
            while not self.is_shutdown() and self.cmd_type is None:
                if log_flag == False:
                    self.get_logger().warn("waiting for cmd topic to start task")
                    self.task_state = "idle"
                    log_flag = True
                time.sleep(0.1)
            
            if self.is_shutdown():
                return
            
            self.get_logger().info(f"cmd_type: {self.cmd_type}")

            if self.cmd_type == "init_left":
                # 机械臂归位
                self.get_logger().info("Start init task")
                self.task_state = "run_init_left"
                self.arm_hw.init_arm(puppet="left")
                self.cmd_type = None
                self.get_logger().info("******* end *******")
                continue
            elif self.cmd_type == "init_right":
                # 机械臂归位
                self.get_logger().info("Start init task")
                self.task_state = "run_init_right"
                self.arm_hw.init_arm(puppet="right")
                self.cmd_type = None
                self.get_logger().info("******* end *******")
                continue
            elif self.cmd_type == "right_green" or self.cmd_type == "right_grab_red" or self.cmd_type == "right_put_red" \
                or self.cmd_type == "left_green" or self.cmd_type == "left_grab_red" or self.cmd_type == "left_put_red" or self.cmd_type == "test":
                #result,class_id = map1.start_caculate(detect_count=4)
                
            else:
                self.get_logger().error(f"cmd type: {self.cmd_type} is invalid")
                continue
                            
            self.get_logger().info("-------start--------")
            
            # right arm 
            if self.cmd_type == "right_green":
                if class_id==1:
                    self.task_state = "run_grab_green"
                    self.grab(color="green",puppet="right")
                if class_id==0:
                    self.task_state = "run_put_green"
                    self.put(color="green",puppet="right")

            elif self.cmd_type == "right_grab_red":
                self.task_state = "run_grab_red"
                self.grab(color="red",puppet="right")
            elif self.cmd_type == "right_put_red":
                self.task_state = "run_put_red"
                self.put(color="red",puppet="right")
            
            # left arm
            if self.cmd_type == "left_green":
                if class_id==1:
                    self.task_state = "run_grab_green"
                    self.grab(color="green",puppet="left")
                if class_id==0:
                    self.task_state = "run_put_green"
                    self.put(color="green",puppet="left")

            elif self.cmd_type == "left_grab_red":
                self.task_state = "run_grab_red"
                self.grab(color="red",puppet="left")
            elif self.cmd_type == "left_put_red":
                self.task_state = "run_put_red"
                self.put(color="red",puppet="left")

            elif self.cmd_type == "test":
                self.test()
            
            # 机械臂归零
            self.ik_caculator.init_arm()

            # clear cmd_type
            self.cmd_type = None
            self.get_logger().info("******* end *******")

    def grab(self,color,puppet):
        
        self.arm_hw.fold_arm(gripper="open",puppet=puppet)

        for i in range(10):       
            # 运动到压板预瞄点
            self.arm_hw.stop_flag = False
            self.arm_hw.aim_arm(gripper="open",puppet=puppet)
            self.arm_hw.run_flag = True
            self.ik_caculator.run(color=color,puppet=puppet)
            start_position = self.ik_caculator.get_target_tcp(color,puppet)
            
            while(self.arm_hw.run_flag == True and self.arm_hw.stop_flag == False):
                self.get_logger().info("Start listening TF...")
                map1.start_caculate(1)
                current_position = self.ik_caculator.get_target_tcp(color,puppet)
                if abs(current_position['x'] - start_position['x']) > 0.01 or\
                    abs(current_position['y'] - start_position['y']) > 0.01:
                    self.get_logger().error("TF changed,abord ")
                    self.arm_hw.stop_flag = True

            # aim瞄准绿色并向前探
            if not (self.call_aim(color,puppet)):
                return False
            x_off,y_off,z_off = self.aim(color=color,puppet=puppet)

            # 闭合夹爪
            self.arm_hw.gripper_control(gripper="close",puppet=puppet)

            # 机械臂后伸

            # 旋转压板
            self.arm_hw.motor_add_control(joint=5,angle=-1.57,puppet=puppet)

            # 打开夹爪
            self.arm_hw.gripper_control(gripper="open",puppet=puppet)
            
            # 回到预瞄点
            self.arm_hw.fold_arm(gripper="close",puppet=puppet)

    def put(self,color,puppet):

        self.arm_hw.fold_arm(gripper="open",puppet=puppet)

        for i in range(10):       
            # 运动到压板预瞄点
            self.arm_hw.stop_flag = False
            self.arm_hw.aim_arm(gripper="open",puppet=puppet)
            self.arm_hw.run_flag = True
            self.ik_caculator.run(color=color,puppet=puppet)
            start_position = self.ik_caculator.get_target_tcp(color,puppet)

            while(self.arm_hw.run_flag == True and self.arm_hw.stop_flag == False):
                self.get_logger().info("Start listening TF...")
                map1.start_caculate(1)
                current_position = self.ik_caculator.get_target_tcp(color,puppet)
                if abs(current_position['x'] - start_position['x']) > 0.01 or\
                    abs(current_position['y'] - start_position['y']) > 0.01:
                    self.get_logger().error("TF changed,abord ")
                    self.arm_hw.stop_flag = True

            if not (self.call_aim(color,puppet)):
                return False
            # 旋转压板
            self.arm_hw.motor_add_control(joint=5,angle=-1.57,puppet=puppet)

            # 机械臂前伸
            self.arm_hw.lock_rotation_flag=True
            # aim瞄准绿色并向前探
            x_off,y_off,z_off = self.aim(color=color,puppet=puppet)

            # 闭合夹爪
            self.arm_hw.gripper_control(gripper="close",puppet=puppet)

            # 机械臂后伸
            self.arm_hw.lock_rotation_flag=True

            # 旋转压板
            if puppet=='right':
                self.arm_hw.motor_add_control(joint=5,angle=1.6,puppet=puppet)
            if puppet=='left':
                self.arm_hw.motor_add_control(joint=5,angle=1.57,puppet=puppet)

            # 打开夹爪
            self.arm_hw.gripper_control(gripper="open",puppet=puppet)
            
            # 回到预瞄点
            self.arm_hw.run_flag = True
            self.arm_hw.fold_arm(gripper="close",puppet=puppet)
    
    def test(self):
        # 运动到压板预瞄点
        self.arm_hw.run(color="green",puppet="right")
        #self.arm_hw.run(color="green",puppet="right")
        time.sleep(3.2)

    def call_aim(self,color,puppet):
        # TODO:call服务获取
        self.aim_client.wait_for_service(timeout_sec=5)
        self.aim_dic['green'].clear()
        self.aim_dic['red'].clear()

        if puppet=='right':
            camera_topic = '/camera_r/color/image_raw'
        elif puppet=='left':
            camera_topic = '/camera_l/color/image_raw'
        
        res = self.aim_client(camera_topic,color)

        if res is not None:
            self.aim_dic[color].append(res.x)
            self.aim_dic[color].append(res.y)
            self.aim_dic[color].append(res.area)
            return True
        else:
            self.get_logger().error("aim failed")
            return False

    # 将aim色块中心坐标数值转换为机械臂坐标系下的三轴偏移
    def aim_to_off(self,aim_x,aim_y,aim_z):
        # 根据色块中心点得出三个轴的偏移量
        z_off = 0.001 * aim_y - 0.07
        x_off = 10/math.sqrt(aim_z)-0.03
        # 由于y_off的尺度和深度是线性关系，所以需要对x_off做修正
        # TODO: check math later
        y_off = 0.005 * aim_x * x_off
        return x_off,y_off,z_off
    
    # aim模块函数，该模块可以使用腕部的摄像头对颜色块进行瞄准
    def aim(self,color,puppet):

        self.get_logger().info("Start aim module")
        
        if color == 'green' and self.aim_dic['green']:
            aim_x = self.aim_dic['green'][0]
            aim_y = self.aim_dic['green'][1]
            aim_z = self.aim_dic['green'][2]
            # 获取三轴偏移
            x_off,y_off,z_off = self.aim_to_off(aim_x,aim_y,aim_z)
            self.arm_hw.run_flag = True
            self.ik_caculator.run(step_list=[0.085,y_off,0],color=color,puppet=puppet)
            while(self.arm_hw.run_flag == True):
                time.sleep(0.1)

            return x_off,y_off,z_off
        elif color == 'red' and self.aim_dic['red']:
            aim_x = self.aim_dic['red'][0]
            aim_y = self.aim_dic['red'][1]
            aim_z = self.aim_dic['red'][2]
            # 获取三轴偏移
            x_off,y_off,z_off = self.aim_to_off(aim_x,aim_y,aim_z)
            self.arm_hw.run_flag = True
            self.ik_caculator.run(step_list=[0.085,y_off,0],color=color,puppet=puppet)
            while(self.arm_hw.run_flag == True):
                time.sleep(0.1)
            return x_off,y_off,z_off
        else:
            self.get_logger().warn('aim green failed, aim list is empty')
            return None

    def task_cmd_callback(self,msg):
        self.get_logger().info("Got cmd ros msg")
        if msg.data == "right_grab_green" or msg.data == "right_put_green":
            self.cmd_type = "right_green"
        
        elif msg.data == "right_grab_red":
            self.cmd_type = "right_grab_red"
        
        elif msg.data == "right_put_red":
            self.cmd_type = "right_put_red"

        elif msg.data == "left_grab_green" or msg.data == "left_put_green":
            self.cmd_type = "left_green"
        
        elif msg.data == "left_grab_red":
            self.cmd_type = "left_grab_red"
        
        elif msg.data == "left_put_red":
            self.cmd_type = "left_put_red"
        
        elif msg.data == "init_left":
            self.cmd_type = "init_left"
        
        elif msg.data == "init_right":
            self.cmd_type = "init_right"

        elif msg.data == "right_open_gripper":
            self.cmd_type = "right_open_gripper"
        elif msg.data == "right_close_gripper":
            self.cmd_type = "right_close_gripper"
        elif msg.data == "left_open_gripper":
            self.cmd_type = "left_open_gripper"
        elif msg.data == "left_close_gripper":
            self.cmd_type = "left_close_gripper"

        elif msg.data == "test":
            self.cmd_type = "test" 
        elif msg.data == "stop":
            self.cmd_type = "stop"
    
    def aim_callback(self,msg):
        try:
            # 解析 JSON 数据
            data = json.loads(msg.data)
            # 清除字典
            self.aim_dic['green'].clear()
            self.aim_dic['red'].clear()
            color = data['color']
            position = data['position']

            # 访问 JSON 字段示例
            if  color=='green':
                self.aim_dic['green'].append(position['x'])
                self.aim_dic['green'].append(position['y'])
                self.aim_dic['green'].append(data['area'])

            if color=='red':
                self.aim_dic['red'].append(position['x'])
                self.aim_dic['red'].append(position['y'])
                self.aim_dic['red'].append(data['area'])

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e}")

        
def signal_handler(signal,frame):
    task_manager_node.update_thread_stop_event.set()
    print('You pressed Ctrl + C!')
    rclpy.shutdown()

if __name__=='__main__' :
    rclpy.init(args=None)

    parser = argparse.ArgumentParser(description="示例：解析命令行参数")

    # 可选参数
    parser.add_argument('--init', action='store_true', help='init arm before excute action')
    parser.add_argument('--skip_detect', action='store_true', help='skip rgb detect')

    args = parser.parse_args()
    task_manager_node = task_manager(args)
    task_manager_node.task_thread.start()
    task_manager_node.task_states_thread.start()
    rclpy.spin(task_manager_node)
    rclpy.shutdown()
