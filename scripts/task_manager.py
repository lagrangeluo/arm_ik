#!/bin/python3
import rospy
import time
from threading import Thread,Lock,Event
import signal

from ik import ik_caculator
from arm_hw import arm_hw

class task_manager:
    def __init__(self):
        # 初始化机械臂硬件控制器
        self.arm_hw = arm_hw()
        # 初始化逆解求解器
        self.ik_caculator = ik_caculator()
        
        self.task_thread = Thread(target=self.run_task)
        self.update_thread_stop_event = Event()
    
    def run_task(self):
        rospy.loginfo("wait 1 sec for ros callback")
        time.sleep(1)
        rospy.loginfo("-------start--------")
        
        # 机械臂归位
        self.ik_caculator.init_arm()
        
        # 运动到压板预瞄点
        self.ik_caculator.run()
        
        # 打开夹爪
        self.arm_hw.gripper_control(gripper="open")
        
        # 机械臂前伸
        self.ik_caculator.run(step_list=[0.05,0,0])
        
        # 闭合夹爪
        self.arm_hw.gripper_control(gripper="close")
        
        # 机械臂后伸
        self.ik_caculator.run(step_list=[0.02,0,0])
        
        # 旋转压板
        self.arm_hw.motor_add_control(joint=6,angle=0)
        
        # 打开夹爪
        self.arm_hw.gripper_control(gripper="open")
        
        # 回到预瞄点
        self.ik_caculator.run()
        
        # 机械臂归零
        self.ik_caculator.init_arm()
        
        rospy.loginfo("******* end *******") 
        if(self.update_thread_stop_event.is_set()):
            rospy.loginfo("SMACH: Timer update thread is stopped by event")
        
        
def signal_handler(signal,frame):
    task_manager_node.update_thread_stop_event.set()
    print('You pressed Ctrl + C!')
    rospy.signal_shutdown('Shutting down...')

if __name__=='__main__' :
    rospy.init_node("arm task manager")
    task_manager_node = task_manager()
    task_manager_node.task_thread.start()
    rospy.spin()
    