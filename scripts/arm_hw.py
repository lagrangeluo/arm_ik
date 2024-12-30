#!/bin/python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

#flag to avoid tow thread 

class arm_hw:
  def __init__(self):
    # ros publisher and subscriber
    self.cmd_pub = rospy.Publisher('/master/joint_right', JointState, queue_size=10)
    self.arm_state = rospy.Subscriber("/puppet/joint_right", JointState, self.arm_state_callback)
    self.instruction_sub = rospy.Subscriber("/target_pose/right", JointState, self.target_pose_callback)
    
    # store the current joint state
    self.current_arm_state=[]
    self.arm_state_update_flag=False
    
    # 末端夹爪数值
    self.gripper_value = 0
    
    # time interval for publish joint state,we wish the control rate will up to 200hz
    self.joint_interval  = 0.005
    self.joint_pub_rate = 200
    
    # response time for one execute time for arm, arm will move to target pose in this seconds
    self.response_time = 2

  # get current arm joint angle
  def get_current_arm_state(self):
    if self.current_arm_state:
      return self.current_arm_state
    else:
      return None

  def gripper_control(self,gripper):
    if not bool(self.current_arm_state):
      rospy.logwarn("arm_hw: current arm state is None, abord gripper control")
    else:
      cmd = JointState()
      cmd.header.stamp = rospy.Time.now()
      cmd.name = ['joint0', 'joint1', 'joint2', 'joint3','joint4' ,'joint5','joint6']
      cmd.position = self.current_arm_state
      if gripper == "close":
        # 关闭夹爪
        self.gripper_value = 0
        cmd.position[6] = 0
      if gripper == "open":
        # 打开夹爪
        self.gripper_value = 4.3
        cmd.position[6] = 4.3
      
      cmd.position.pop()
      self.target_pose_callback(cmd)

      #self.cmd_pub.publish(cmd)
      
  def motor_add_control(self,joint,angle):
    if not bool(self.current_arm_state):
      rospy.logwarn("arm_hw: current arm state is None,abord motor add")
    else:
      cmd = JointState()
      cmd.header.stamp = rospy.Time.now()
      cmd.name = ['joint0', 'joint1', 'joint2', 'joint3','joint4' ,'joint5','joint6']
      cmd.position = self.current_arm_state
      cmd.position[joint] += angle
      cmd.position.pop()

      self.target_pose_callback(cmd)
      
      #self.cmd_pub.publish(cmd)


    return

  def target_pose_callback(self,msg):

    if not bool(self.current_arm_state):
      rospy.logerr("arm_hw: current arm state is None,abord execute cmd")
    else:
      joint_interval_list = []
      for angle in msg.position:
        joint_interval_list.append(angle)
      # 添加夹爪的数值
      joint_interval_list.append(self.gripper_value)
      
      arm_start_angle = []
      if len(self.current_arm_state)<7:
        while(len(self.current_arm_state)<7):
          rospy.logwarn("waiting for arm state update")
      arm_start_angle = [x for x in self.current_arm_state]
      rospy.loginfo(f"joint_interval:{joint_interval_list}")
      rospy.loginfo(f"arm_start_angle:{arm_start_angle}")

      joint_interval_list = self.list1_sub_list2(joint_interval_list,arm_start_angle)
      #print("Got the joint_interval_angle:", joint_interval_list)
      
      joint_cut_num = self.response_time*self.joint_pub_rate
      joint_step = [a/joint_cut_num for a in joint_interval_list]
      #print("Got joint step: ",joint_step)
      
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
    self.arm_state_update_flag=False
    self.current_arm_state.clear()
    for angle in msg.position:
      self.current_arm_state.append(angle)
    self.arm_state_update_flag=True
    #print("Get current arm state: ",self.current_arm_state)
    
  def list1_sub_list2(self,list1,list2):
    if len(list1) != len(list2):
      rospy.logerr(f"list1:{list1}")
      rospy.logerr(f"list2:{list2}")
      raise ValueError("List1 sub List2 failed!")
    result = [a - b for a,b in zip(list1,list2)]
    return result

if __name__=='__main__' :
  rospy.init_node("traj_smooth_node")
  node = arm_hw()
  rospy.loginfo("traj smooth node init success!")
  rospy.spin()
