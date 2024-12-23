#!/bin/python3
from ikpy import chain
import numpy as np
import rospy
import time
from pytransform3d import rotations
from sensor_msgs.msg import JointState
import tf2_ros
from std_msgs.msg import Header


class ik_caculator():
    def __init__(self):
        # rosnode
        rospy.init_node("pyik_node")
        self.pub = rospy.Publisher('/target_pose/right', JointState, queue_size=10)
        rospy.loginfo("ik caculator init")
        self.joint_state = JointState()
        self.joint_state.name = ['fl_joint1', 'fl_joint2', 'fl_joint3', 'fl_joint4','fl_joint5' ,'fl_joint6']

        # 创建 TF2 监听器和缓冲区
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # 定义坐标系
        self.parent_frame = 'base_link'
        self.child_frame = 'grab_link'

        ###### Left Arm ######
        left_arm_links = [
                            "fl_base_link",
                            "fl_link1",
                            "fl_link2",
                            "fl_link3",
                            "fl_link4",
                            "fl_link5",
                            ]

        left_arm_joints = [
                            "fl_joint1",
                            "fl_joint2",
                            "fl_joint3",
                            "fl_joint4",
                            "fl_joint5",
                            "fl_joint6",
                            ]

        left_arm_elements = [x for pair in zip(left_arm_links, left_arm_joints) for x in pair] + ["fl_link6"]
        rospy.loginfo(f'{left_arm_elements}')
        # Note: 'Links' in IKPY correspond to 'Joints' in URDF terminology. 'Links' in URDF are stripped by IKPY. 
        self.left_arm_chain = chain.Chain.from_urdf_file(
            "/home/abc/cobot_magic_point/src/arm_ik/urdf/arx5_description_ik.urdf",
            base_elements=left_arm_elements,
            #last_link_vector=[0.261, 0, -0.018],   # 如果这里没有腕关节, 该向量为t_elbow_wrist平移向量
            active_links_mask=[False]*0 + 7 * [True],  
            symbolic=False,
            name="left_arm")

        rospy.loginfo("init chain success")
        self.left_arm_chain.to_json_file(force=True)

        # 初始化机械臂
        self.init_arm()
        
    
    def init_arm(self):
        qpos = np.zeros(7)
        # 初始化旋转矩阵
        self.Transform_init = self.left_arm_chain.forward_kinematics(qpos)
        print("joint zero transform: ",self.Transform_init)

        qua = rotations.quaternion_from_matrix(self.Transform_init[:3,:3])
        #print("init q:", qua)
        #print("rotation: ",self.Transform_init[:3, :3])

        init_position = [0.05, 0, 0.02]
        init_orientation = self.Transform_init[:3,:3]
        print("start init arm in 3 seconds")
        q_init = self.left_arm_chain.inverse_kinematics(init_position,init_orientation,'all')
        q_init = [round(a,3) for a in q_init]

        joint_state_init = JointState()
        joint_state_init.position = [q_init[1],q_init[2],q_init[3],q_init[4],q_init[5],q_init[6]]
        joint_state_init.header = Header()
        joint_state_init.header.stamp = rospy.Time.now()
        time.sleep(0.2)
        self.pub.publish(joint_state_init)
        rospy.loginfo("arm init success!")
        time.sleep(3)

    
    def run(self):
        tf_warn_flag = False
        while not rospy.is_shutdown():
            try:
                if tf_warn_flag == True:
                    tf_warn_flag = False
                # 获取从 parent_frame 到 child_frame 的变换
                transform = self.tf_buffer.lookup_transform(self.parent_frame, self.child_frame, rospy.Time(0))
                target_x = transform.transform.translation.x
                target_y = transform.transform.translation.y
                target_z = transform.transform.translation.z
                
                # 打印平移和旋转信息
                rospy.loginfo("Translation: x=%f, y=%f, z=%f", transform.transform.translation.x, 
                                                            transform.transform.translation.y, 
                                                            transform.transform.translation.z)
                rospy.loginfo("Rotation: x=%f, y=%f, z=%f, w=%f", transform.transform.rotation.x,
                                                            transform.transform.rotation.y,
                                                            transform.transform.rotation.z,
                                                            transform.transform.rotation.w)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                if tf_warn_flag == False:
                    tf_warn_flag = True
                    rospy.logwarn("Transform not available yet.")
                time.sleep(1)
                continue

            target_position = [target_x, target_y, target_z]
            print("target position: ",target_position)
            target_orientation = self.Transform_init[:3,:3]
            #target_orientation = [[1,0,0],[0,1,0],[0,0,1]]
            #target_orientation = [0,0,0]
            q = self.left_arm_chain.inverse_kinematics(target_position,target_orientation,'all')
            q = [round(a,3) for a in q]
            print("ik joint angle: ", q)

            self.joint_state.position = [q[1],q[2],q[3],q[4],q[5],q[6]]
            self.joint_state.header = Header()
            self.joint_state.header.stamp = rospy.Time.now()
            time.sleep(0.2)
            self.joint_state.header.stamp = rospy.Time.now()
            self.pub.publish(self.joint_state)
            
if __name__=='__main__' :

    node = ik_caculator()
    node.run()
    #rospy.spin()
            