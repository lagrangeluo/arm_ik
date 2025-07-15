#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

def transform_to_matrix(transform: TransformStamped):
    # 将TransformStamped转为4x4齐次矩阵
    t = transform.transform.translation
    q = transform.transform.rotation
    trans = np.array([t.x, t.y, t.z])
    quat = np.array([q.x, q.y, q.z, q.w])
    rot = R.from_quat(quat)
    mat = np.eye(4)
    mat[:3, :3] = rot.as_matrix()
    mat[:3, 3] = trans
    return mat

def matrix_to_transform(mat):
    # 4x4矩阵转为平移+四元数
    trans = mat[:3, 3]
    rot = R.from_matrix(mat[:3, :3])
    quat = rot.as_quat()  # [x, y, z, w]
    return trans, quat

class HandLinkRelativeTracker(Node):
    def __init__(self):
        super().__init__('hand_link_relative_tracker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.first_transform_mat = None
        self.first_received = False
        self.base_frame = 'libsurvive_world'
        self.hand_frame = 'LHR-617A2AAB'
        self.get_logger().info('启动 hand_link 相对第一帧变换跟踪...')

    def timer_callback(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame, self.hand_frame, rclpy.time.Time())
            current_mat = transform_to_matrix(trans)
            if not self.first_received:
                self.first_transform_mat = current_mat
                self.first_received = True
                self.get_logger().info('已记录第一帧 hand_link 变换')
                return
            # 计算相对第一帧的变换
            rel_mat = np.linalg.inv(self.first_transform_mat) @ current_mat
            rel_trans, rel_quat = matrix_to_transform(rel_mat)
            self.get_logger().info(
                f'hand_link相对第一帧: 平移=({rel_trans[0]:.3f}, {rel_trans[1]:.3f}, {rel_trans[2]:.3f}), '
                f'四元数=({rel_quat[0]:.3f}, {rel_quat[1]:.3f}, {rel_quat[2]:.3f}, {rel_quat[3]:.3f})'
            )
        except Exception as e:
            self.get_logger().warn(f'获取tf失败: {e}')

def main():
    rclpy.init()
    node = HandLinkRelativeTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()