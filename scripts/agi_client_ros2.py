import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from geometry_msgs.msg import PoseStamped
import time
from sensor_msgs.msg import JointState
import threading

class AGIClient(Node):
    def __init__(self, node_name="agi_client"):
        super().__init__(node_name)
        # 1. 关节空间控制
        self.left_arm_joint_pub = self.create_publisher(Float64MultiArray, '/agi_hw/left_arm_joint_cmd', 10)
        self.right_arm_joint_pub = self.create_publisher(Float64MultiArray, '/agi_hw/right_arm_joint_cmd', 10)
        # 2. 笛卡尔空间控制
        self.left_arm_pose_pub = self.create_publisher(PoseStamped, '/agi_hw/left_arm_target_pose', 10)
        self.right_arm_pose_pub = self.create_publisher(PoseStamped, '/agi_hw/right_arm_target_pose', 10)
        # 3. 夹爪控制
        self.gripper_control_pub = self.create_publisher(Int32MultiArray, '/agi_hw/gripper_control', 10)
        # 4. 腰部升降头部控制
        self.waist_lift_head_pub = self.create_publisher(Float64MultiArray, '/agi_hw/waist_lift_head_cmd', 10)
        # 新增订阅者
        self.left_arm_pose_sub = self.create_subscription(
            PoseStamped,
            '/agi_hw/left_arm_current_pose',
            self.left_arm_pose_callback,
            10
        )
        self.right_arm_pose_sub = self.create_subscription(
            PoseStamped,
            '/agi_hw/right_arm_current_pose',
            self.right_arm_pose_callback,
            10
        )
        # 订阅joint_states
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        # 保存左臂和右臂7关节角度及时间戳
        self._left_arm_joint = None
        self._right_arm_joint = None
        self._left_arm_joint_time = None
        self._right_arm_joint_time = None
        # 关节名列表（可根据实际情况修改）
        self._left_joint_names = [
            "idx12_left_arm_joint1", "idx13_left_arm_joint2", "idx14_left_arm_joint3",
            "idx15_left_arm_joint4", "idx16_left_arm_joint5", "idx17_left_arm_joint6", "idx18_left_arm_joint7"
        ]
        self._right_joint_names = [
            "idx19_right_arm_joint1", "idx20_right_arm_joint2", "idx21_right_arm_joint3",
            "idx22_right_arm_joint4", "idx23_right_arm_joint5", "idx24_right_arm_joint6", "idx25_right_arm_joint7"
        ]

        self._left_arm_pose = None
        self._right_arm_pose = None
        self._left_arm_pose_time = None
        self._right_arm_pose_time = None

        # 启动spin线程
        # self._spin_thread = threading.Thread(target=self._spin_worker, daemon=True)
        # self._spin_thread.start()
        # self.get_logger().info("AGI_CLIENT 初始化完毕，spin线程已启动")
        time.sleep(0.2)


    #------------ 应用层 --------------#
    def moveL(self,cmd_x,cmd_y,cmd_z,rotate_state='0du'):
        if rotate_state=='0du':
            ori= [0.707, 0.0, 0.707, 0.0]
        elif rotate_state=='90du':
            ori= [ -0.5, 0.5, -0.5, 0.5]
        elif rotate_state=='45du_right':
            ori= [0.653, 0.271, 0.653, 0.271]
        elif rotate_state=='45du_left':
            ori= [-0.653, 0.271, -0.653, 0.271]
        else:
            self.get_logger().warn("Wrong rotate state!!! abord moveL")
            return
        
        self.send_right_arm_pose(pos = [cmd_x, cmd_y, cmd_z], ori=ori)
        # 延时，接口目前并不阻塞
        time.sleep(1)

    # def moveL(self,cmd_x,cmd_y,cmd_z,):
    #     if rotate_state=='0du':
    #         ori= [0.707, 0, 0.707, 0]
    #     elif rotate_state=='90du':
    #         ori= [ -0.5, 0.5, -0.5, 0.5]
    #     elif rotate_state=='45du_right':
    #         ori= [0.653, 0.271, 0.653, 0.271]
    #     elif rotate_state=='45du_left':
    #         ori= [-0.653, 0.271, -0.653, 0.271]
    #     else:
    #         self.get_logger().warn("Wrong rotate state!!! abord moveL")
    #         return
        
    #     self.send_right_arm_pose(pos = [cmd_x, cmd_y, cmd_z], ori)


    def moveL_delta(self, dx, dy, dz):
        """在当前右臂末端位姿基础上做相对位移"""
        current = self.get_latest_right_arm_pose()
        if current is None:
            self.get_logger().warn("moveL_delta: 当前右臂位姿无效，无法增量移动")
            return
        pos, ori = current
        new_pos = [pos[0] + dx, pos[1] + dy, pos[2] + dz]
        self.send_right_arm_pose(new_pos, ori)
        self.get_logger().info(f"moveL_delta: 基于当前点{pos}，增量({dx},{dy},{dz})，新点{new_pos}")
        time.sleep(1.5)


    def move_rotate(self, delta_angle):
        """在当前右臂关节角度基础上，旋转第7关节（末端关节）"""
        current_joints = self.get_latest_right_arm_joint()
        if current_joints is None:
            self.get_logger().warn("move_rotate: 当前右臂关节角度无效，无法旋转")
            return
        # 复制当前关节角度，修改第7关节（索引6）
        new_joints = current_joints.copy()
        new_joints[6] += delta_angle
        self.send_right_arm_joint(new_joints)
        self.get_logger().info(f"move_rotate: 第7关节从{current_joints[6]:.3f}旋转{delta_angle:.3f}到{new_joints[6]:.3f}")
        time.sleep(2)



    def open_gripper(self):
        self.send_gripper_control([2, 2])
    def close_gripper(self):
        self.send_gripper_control([1, 1])
    #---------------------------------#

    # 关节空间控制
    def send_left_arm_joint(self, angles):
        msg = Float64MultiArray()
        msg.data = list(angles)
        self.left_arm_joint_pub.publish(msg)
        self.get_logger().info(f"已发布左臂关节指令: {msg.data}")

    def send_right_arm_joint(self, angles):
        msg = Float64MultiArray()
        msg.data = list(angles)
        self.right_arm_joint_pub.publish(msg)
        self.get_logger().info(f"已发布右臂关节指令: {msg.data}")

    # 笛卡尔空间控制
    def send_left_arm_pose(self, pos, ori):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = ori[0]
        msg.pose.orientation.y = ori[1]
        msg.pose.orientation.z = ori[2]
        msg.pose.orientation.w = ori[3]
        self.left_arm_pose_pub.publish(msg)
        self.get_logger().info(f"已发布左臂笛卡尔指令: pos={pos}, ori={ori}")

    def send_right_arm_pose(self, pos, ori):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = ori[0]
        msg.pose.orientation.y = ori[1]
        msg.pose.orientation.z = ori[2]
        msg.pose.orientation.w = ori[3]
        self.right_arm_pose_pub.publish(msg)
        self.get_logger().info(f"已发布右臂笛卡尔指令: pos={pos}, ori={ori}")

    # 夹爪控制
    def send_gripper_control(self, data):
        msg = Int32MultiArray()
        msg.data = list(data)
        self.gripper_control_pub.publish(msg)
        self.get_logger().info(f"已发布夹爪控制指令: {msg.data}")

    # 腰部升降头部控制
    def send_waist_lift_head(self, data):
        msg = Float64MultiArray()
        msg.data = list(data)
        self.waist_lift_head_pub.publish(msg)
        self.get_logger().info(f"已发布腰部升降头部指令: {msg.data}")

    def left_arm_pose_callback(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ori = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self._left_arm_pose = (pos, ori)
        self._left_arm_pose_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def right_arm_pose_callback(self, msg):
        pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        ori = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self._right_arm_pose = (pos, ori)
        self._right_arm_pose_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def joint_states_callback(self, msg):
        name2idx = {n: i for i, n in enumerate(msg.name)}
        # 左臂
        try:
            left_angles = [msg.position[name2idx[n]] for n in self._left_joint_names]
            self._left_arm_joint = left_angles
            self._left_arm_joint_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().warn(f'左臂关节解析失败: {e}')
            self._left_arm_joint = None
            self._left_arm_joint_time = None
        # 右臂
        try:
            right_angles = [msg.position[name2idx[n]] for n in self._right_joint_names]
            self._right_arm_joint = right_angles
            self._right_arm_joint_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as e:
            self.get_logger().warn(f'右臂关节解析失败: {e}')
            self._right_arm_joint = None
            self._right_arm_joint_time = None


    def get_latest_left_arm_pose(self):
        """返回最新的左臂(pos, ori)元组，如果无或超时则返回None"""
        if self._left_arm_pose is None or self._left_arm_pose_time is None:
            self.get_logger().warn("左臂当前位姿尚未收到任何消息")
            return None
        now = self.get_clock().now().seconds_nanoseconds()
        now_sec = now[0] + now[1] * 1e-9
        if now_sec - self._left_arm_pose_time > 0.2:
            self.get_logger().warn("左臂当前位姿消息已过期")
            return None
        return self._left_arm_pose

    def get_latest_right_arm_pose(self):
        """返回最新的右臂(pos, ori)元组，如果无或超时则返回None"""
        if self._right_arm_pose is None or self._right_arm_pose_time is None:
            self.get_logger().warn("右臂当前位姿尚未收到任何消息")
            return None
        now = self.get_clock().now().seconds_nanoseconds()
        now_sec = now[0] + now[1] * 1e-9
        if now_sec - self._right_arm_pose_time > 0.2:
            self.get_logger().warn("右臂当前位姿消息已过期")
            return None
        return self._right_arm_pose

    def get_latest_left_arm_joint(self):
        """返回最新的左臂7关节角度list，如果无或超时则返回None"""
        if self._left_arm_joint is None or self._left_arm_joint_time is None:
            self.get_logger().warn("左臂当前关节角度尚未收到任何消息")
            return None
        now = self.get_clock().now().seconds_nanoseconds()
        now_sec = now[0] + now[1] * 1e-9
        if now_sec - self._left_arm_joint_time > 0.2:
            self.get_logger().warn("左臂当前关节角度消息已过期")
            return None
        return self._left_arm_joint

    def get_latest_right_arm_joint(self):
        """返回最新的右臂7关节角度list，如果无或超时则返回None"""
        if self._right_arm_joint is None or self._right_arm_joint_time is None:
            self.get_logger().warn("右臂当前关节角度尚未收到任何消息")
            return None
        now = self.get_clock().now().seconds_nanoseconds()
        now_sec = now[0] + now[1] * 1e-9
        if now_sec - self._right_arm_joint_time > 0.2:
            self.get_logger().warn("右臂当前关节角度消息已过期")
            return None
        return self._right_arm_joint

    def _spin_worker(self):
        """内部spin工作线程"""
        rclpy.spin(self)

    def shutdown(self):
        """关闭节点和线程"""
        self.destroy_node()
        rclpy.shutdown()
        if self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)

def main():
    rclpy.init()
    client = AGIClient()
    rclpy.spin_once(client)
    
    # 现在可以直接使用client，无需手动spin
    client.moveL(0.8, -0.2, 1.0, '0du')
    client.moveL(0.7, -0.2, 1.0, '0du')
    client.moveL(0.7, -0.1, 1.0, '0du')
    client.moveL(0.8, -0.1, 1.0, '0du')

    # client.open_gripper()
    # 示例：发布腰部升降头部指令
    # client.send_waist_lift_head([0.0, 0.2, 0.3])
    client.moveL_delta(-0.1,0,0)
    client.moveL_delta(0,-0.1,0)
    client.moveL_delta(0.1,0,0)
    client.moveL_delta(0,0.1,0)
    client.move_rotate(1.57)
    client.move_rotate(-1.57)

    # client.moveL_delta(0,0.1,0)
    # time.sleep(2)
    # client.close_gripper()
    
    # 退出时调用shutdown
    client.shutdown()

if __name__ == "__main__":
    main()
