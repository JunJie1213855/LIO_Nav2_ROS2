#!/usr/bin/env python3
"""
地面+天花板联合滤波节点

订阅 FAST-LIO 的 /cloud_registered 和 /Odometry，将点云变换到 body 帧后，
同时滤除地面以下和天花板以上的点，发布 /cloud_registered_filtered。

用法:
  # 默认参数（适合室内环境）
  /usr/bin/python3 scripts/ground_ceiling_filter.py \
    --ros-args -p ground_z_threshold:=-0.4 -p ceiling_z_threshold:=2.5

  # 通过 launch 文件启动
  ros2 run <package> ground_ceiling_filter.py --ros-args -p ground_z_threshold:=-0.4

参数:
  input_cloud         输入点云 topic（默认 /cloud_registered）
  input_odom          里程计 topic（默认 /Odometry）
  output_cloud        输出点云 topic（默认 /cloud_registered_filtered）
  ground_z_threshold  body 帧 Z 下界，低于此值丢弃（默认 -0.4 m）
  ceiling_z_threshold body 帧 Z 上界，高于此值丢弃（默认 2.5 m）
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2


class GroundCeilingFilter(Node):
    def __init__(self):
        super().__init__('ground_ceiling_filter')

        # ── 参数声明 ──────────────────────────────────────────────
        self.declare_parameter('input_cloud', '/cloud_registered')
        self.declare_parameter('input_odom', '/Odometry')
        self.declare_parameter('output_cloud', '/cloud_registered_filtered')
        self.declare_parameter('ground_z_threshold', -0.4)
        self.declare_parameter('ceiling_z_threshold', 2.5)

        inp = self.get_parameter('input_cloud').value
        odom_t = self.get_parameter('input_odom').value
        out = self.get_parameter('output_cloud').value
        self.ground_thr = self.get_parameter('ground_z_threshold').value
        self.ceiling_thr = self.get_parameter('ceiling_z_threshold').value

        self.latest_odom = None  # (tx, ty, tz, qx, qy, qz, qw)

        # ── 订阅 & 发布 ────────────────────────────────────────────
        self.sub_cloud = self.create_subscription(
            PointCloud2, inp, self.cloud_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, odom_t, self.odom_cb, 10)
        self.pub = self.create_publisher(PointCloud2, out, 10)

        self.get_logger().info(
            f'Ground+Ceiling filter ready: {inp} → {out}  '
            f'keep Z_body ∈ ({self.ground_thr:.2f}, {self.ceiling_thr:.2f})')

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest_odom = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

    def cloud_cb(self, msg: PointCloud2):
        if self.latest_odom is None:
            return  # 等待首帧里程计

        tx, ty, tz, qx, qy, qz, qw = self.latest_odom

        # world → body 旋转矩阵 (R^T)
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]])
        T = np.array([tx, ty, tz])

        kept = []
        total = 0
        inp = self.get_parameter('input_cloud').value
        out = self.get_parameter('output_cloud').value
        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                           skip_nans=True):
            pt_w = np.array([p[0], p[1], p[2]])
            pt_b = R.T @ (pt_w - T)          # world → body
            total += 1
            if self.ground_thr < pt_b[2] < self.ceiling_thr:
                kept.append((p[0], p[1], p[2]))

        out_msg = point_cloud2.create_cloud_xyz32(msg.header, kept)
        self.pub.publish(out_msg)

        if total > 0:
            self.get_logger().info(
                f'{inp} → {out}: {total} → {len(kept)} pts '
                f'({100*len(kept)/total:.1f}% kept, '
                f'Z_body ∈ ({self.ground_thr:.2f}, {self.ceiling_thr:.2f}))',
                throttle_duration_sec=5.0)


def main():
    rclpy.init()
    rclpy.spin(GroundCeilingFilter())


if __name__ == '__main__':
    main()
