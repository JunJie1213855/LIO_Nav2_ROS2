#!/usr/bin/env python3
"""
Airy Z 轴翻转修正节点 — 在 camera_init 世界帧直接施加固定旋转

原理:
  Airy extrinsic_R = [0,-1,0; -1,0,0; 0,0,-1]
  第三行 [0,0,-1] 导致 Z_imu = -Z_lidar（Z 轴朝下）。
  camera_init 是固定世界帧, 不随机器人运动。
  直接在 camera_init 帧施加 R_inv = R_ext^T, 对所有点一致生效, 不依赖里程计。
  (之前 world->body->R_inv->body->world 的 bug: R_b2w 随机器人旋转, 导致
   R_inv 的修正效果也被旋转, 看起来像点云跟着机器人转)

用法:
  /usr/bin/python3 scripts/airy_unflip.py

参数:
  input_cloud    输入点云 topic（默认 /cloud_registered）
  output_cloud   输出点云 topic（默认 /cloud_registered_unflipped）
  extrin_r       LiDAR->IMU 外参旋转矩阵, 9 个值逗号分隔
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def parse_matrix(s: str) -> np.ndarray:
    vals = [float(x.strip()) for x in s.split(",")]
    if len(vals) != 9:
        raise ValueError(f"extrin_r 需要 9 个浮点数, 收到 {len(vals)} 个")
    return np.array(vals).reshape(3, 3)


class AiryUnflip(Node):
    def __init__(self):
        super().__init__('airy_unflip')

        self.declare_parameter('input_cloud', '/cloud_registered')
        self.declare_parameter('output_cloud', '/cloud_registered_unflipped')
        self.declare_parameter(
            'extrin_r',
            '0.0,-1.0,0.0, -1.0,0.0,0.0, 0.0,0.0,-1.0')

        inp = self.get_parameter('input_cloud').value
        out = self.get_parameter('output_cloud').value
        extrin_r_str = self.get_parameter('extrin_r').value

        # 外参旋转矩阵 & 其逆（正交矩阵逆 = 转置）
        R_ext = parse_matrix(extrin_r_str)
        self.R_inv = R_ext.T

        self.sub_cloud = self.create_subscription(
            PointCloud2, inp, self.cloud_cb, 10)
        self.pub = self.create_publisher(PointCloud2, out, 10)

        self.get_logger().info(
            f'Airy unflip ready: {inp} -> {out}\n'
            f'  R_ext = {R_ext.flatten().tolist()}\n'
            f'  R_inv = {self.R_inv.flatten().tolist()}\n'
            f'  直接在 camera_init 世界帧施加固定旋转, 不依赖里程计')

    def cloud_cb(self, msg: PointCloud2):
        corrected = []
        total = 0

        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                           skip_nans=True):
            pt_w = np.array([p[0], p[1], p[2]])

            # camera_init 帧直接施加固定旋转（不经过 body 帧, 不依赖里程计）
            pt_corrected = self.R_inv @ pt_w

            corrected.append((float(pt_corrected[0]),
                              float(pt_corrected[1]),
                              float(pt_corrected[2])))
            total += 1

        out_msg = point_cloud2.create_cloud_xyz32(msg.header, corrected)
        self.pub.publish(out_msg)

        self.get_logger().info(
            f'{total} pts (Z 轴已恢复朝上)',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    rclpy.spin(AiryUnflip())


if __name__ == '__main__':
    main()
