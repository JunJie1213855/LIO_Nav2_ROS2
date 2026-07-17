#!/usr/bin/env python3
"""
Airy Z 轴翻转修正节点（方案二：extrinsic_R 逆旋转）

接收 FAST-LIO 的点云和里程计：world → body → R_inv × pt_body → world，
对外参旋转矩阵取逆后施加到 body 帧点云上，消除 Z 轴翻转。

原理:
  Airy extrinsic_R = [0,-1,0; -1,0,0; 0,0,-1]
  第三行 [0,0,-1] 导致 Z_imu = -Z_lidar（Z 轴朝下）。

  本节点在 body 帧施加 R_inv = R_ext^T（正交矩阵逆=转置），
  修正后 Z 轴恢复朝上，min_height/max_height 可用正常正值。

用法:
  # Airy 默认外参
  /usr/bin/python3 scripts/airy_unflip.py

  # 自定义外参
  /usr/bin/python3 scripts/airy_unflip.py \
    --ros-args -p extrin_r:="0.0,-1.0,0.0, -1.0,0.0,0.0, 0.0,0.0,-1.0"

参数:
  input_cloud    输入点云 topic（默认 /cloud_registered）
  input_odom     里程计 topic（默认 /Odometry）
  output_cloud   输出点云 topic（默认 /cloud_registered_unflipped）
  extrin_r       LiDAR→IMU 外参旋转矩阵，9 个值逗号分隔
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2


def parse_matrix(s: str) -> np.ndarray:
    """解析逗号分隔的 9 个浮点数为 3×3 矩阵"""
    vals = [float(x.strip()) for x in s.split(",")]
    if len(vals) != 9:
        raise ValueError(f"extrin_r 需要 9 个浮点数，收到 {len(vals)} 个")
    return np.array(vals).reshape(3, 3)


class AiryUnflip(Node):
    def __init__(self):
        super().__init__('airy_unflip')

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter('input_cloud', '/cloud_registered')
        self.declare_parameter('input_odom', '/Odometry')
        self.declare_parameter('output_cloud', '/cloud_registered_unflipped')
        self.declare_parameter(
            'extrin_r',
            '0.0,-1.0,0.0, -1.0,0.0,0.0, 0.0,0.0,-1.0')

        inp = self.get_parameter('input_cloud').value
        odom_t = self.get_parameter('input_odom').value
        out = self.get_parameter('output_cloud').value
        extrin_r_str = self.get_parameter('extrin_r').value

        # 外参 & 其逆（正交矩阵逆 = 转置）
        self.R_ext = parse_matrix(extrin_r_str)
        self.R_inv = self.R_ext.T       # LiDAR←IMU 方向

        self.latest_odom = None  # (tx, ty, tz, qx, qy, qz, qw)

        # ── 订阅 & 发布 ────────────────────────────────────────────
        self.sub_cloud = self.create_subscription(
            PointCloud2, inp, self.cloud_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, odom_t, self.odom_cb, 10)
        self.pub = self.create_publisher(PointCloud2, out, 10)

        self.get_logger().info(
            f'Airy unflip ready: {inp} → {out}\n'
            f'  R_ext = {self.R_ext.flatten().tolist()}\n'
            f'  R_inv = {self.R_inv.flatten().tolist()}\n'
            f'  Z 轴翻转已修正（Z 恢复朝上）')

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def cloud_cb(self, msg: PointCloud2):
        if self.latest_odom is None:
            return  # 等待首帧里程计

        tx, ty, tz, qx, qy, qz, qw = self.latest_odom

        # world → body 旋转矩阵 (R_w2b = R_b2w^T)
        R_b2w = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]])
        R_w2b = R_b2w.T
        T = np.array([tx, ty, tz])

        corrected = []
        total = 0
        inp = self.get_parameter('input_cloud').value
        out = self.get_parameter('output_cloud').value

        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                           skip_nans=True):
            pt_w = np.array([p[0], p[1], p[2]])

            # 1) world → body
            pt_b = R_w2b @ (pt_w - T)

            # 2) body 帧施加 R_inv = R_ext^T，消除 Z 翻转 & X/Y 交换
            pt_corrected_b = self.R_inv @ pt_b

            # 3) body → world
            pt_corrected_w = R_b2w @ pt_corrected_b + T

            corrected.append((float(pt_corrected_w[0]),
                              float(pt_corrected_w[1]),
                              float(pt_corrected_w[2])))
            total += 1

        out_msg = point_cloud2.create_cloud_xyz32(msg.header, corrected)
        self.pub.publish(out_msg)

        self.get_logger().info(
            f'{inp} → {out}: {total} pts (Z 轴已恢复朝上)',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    rclpy.spin(AiryUnflip())


if __name__ == '__main__':
    main()
