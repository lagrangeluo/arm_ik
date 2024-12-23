#!/bin/python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

#flag to avoid tow thread 

class traj_smoother:
  def __init__(self):
    # ros publisher and subscriber
    self.cmd_pub = rospy.Publisher('/master/joint_right', JointState, queue_size=10)
    self.arm_state = rospy.Subscriber("/puppet/joint_right", JointState, self.arm_state_callback)
    self.instruction_sub = rospy.Subscriber("/target_pose/right", JointState, self.target_pose_callback)
    
    # store the current joint state
    self.current_arm_state=[]
    
    # flag to indicate smoother state
    self.smooth_ing_flag = False
    
    # time interval for publish joint state,we wish the control rate will up to 200hz
    self.joint_interval  = 0.005
    self.joint_pub_rate = 200
    
    # response time for one execute time for arm, arm will move to target pose in this seconds
    self.response_time = 2


  def target_pose_callback(self,msg):
    if self.smooth_ing_flag == True:
      rospy.logwarn("The smooth flag is False,abord the callback")
    if not bool(self.current_arm_state):
      rospy.logwarn("current arm state is None")
    else:
      joint_interval_list = []
      for angle in msg.position:
        joint_interval_list.append(angle)
      joint_interval_list.append(0)
      
      arm_start_angle = []
      arm_start_angle = [x for x in self.current_arm_state]
      
      joint_interval_list = self.list1_sub_list2(joint_interval_list,arm_start_angle)
      print("Got the joint_interval_angle:", joint_interval_list)
      
      joint_cut_num = self.response_time*self.joint_pub_rate
      joint_step = [a/joint_cut_num for a in joint_interval_list]
      print("Got joint step: ",joint_step)
      
      for i in range(0,joint_cut_num):
        
        cmd = JointState()
        cmd.header.stamp = rospy.Time.now()
        cmd.name = ['joint0', 'joint1', 'joint2', 'joint3','joint4' ,'joint5','joint6']
        cmd.position = [i*a + b for a,b in zip(joint_step,arm_start_angle)]
        cmd.position = [round(x,3) for x in cmd.position]
        #cmd.position = [0,0,0,0,0,0,0]
        
        self.cmd_pub.publish(cmd)
        
        time.sleep(self.joint_interval)
        #time.sleep(0.05)
      
      
  def arm_state_callback(self,msg):
    self.current_arm_state.clear()
    for angle in msg.position:
      self.current_arm_state.append(angle)
    #print("Get current arm state: ",self.current_arm_state)
    
  def list1_sub_list2(self,list1,list2):
    if len(list1) != len(list2):
      raise ValueError("List1 sub List2 failed!")
    result = [a - b for a,b in zip(list1,list2)]
    return result
    
  

if __name__=='__main__' :
  rospy.init_node("traj_smooth_node")
  node = traj_smoother()
  rospy.loginfo("traj smooth node init success!")
  rospy.spin()
