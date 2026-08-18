#!/usr/bin/env python3
"""
Airy Z 轴翻转修正节点 — 同时修正点云和位姿，保持坐标系一致性

原理:
  Airy 原生 Z 轴朝下。FAST-LIO 输出的点云和位姿都在翻转后的坐标空间中。
  施加绕 X 轴 180° 的固定旋转变换：
    R_correction = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
    X →  X  (前进方向不变)
    Y → -Y  (左右镜像，翻转 Z 轴时不可避免)
    Z → -Z  (Z 轴翻转向下→向上)

  之前使用 R_ext = [[0,-1,0],[-1,0,0],[0,0,-1]] 不仅翻转 Z，还把 X 和 Y 互换了，
  导致机器人沿 X 轴前进在 SLAM 地图上显示为 Y 轴移动。

位姿修正推导:
  世界帧施加旋转 M 后: v_world' = M @ v_world
  原始位姿矩阵 R (world→body): v_body = R @ v_world = R @ M^T @ v_world'
  因此新位姿矩阵: R' = R @ M^T   位置: t' = M @ t

用法:
  /usr/bin/python3 scripts/airy_unflip.py

参数:
  input_cloud      输入点云 topic（默认 /cloud_registered）
  output_cloud     输出点云 topic（默认 /cloud_registered_unflipped）
  input_odometry   输入里程计 topic（默认 /Odometry）
  output_odometry  输出里程计 topic（默认 /Odometry_unflipped）
  extrin_r         修正旋转矩阵, 9 个值逗号分隔（默认绕 X 轴 180°）
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2


def parse_matrix(s: str) -> np.ndarray:
    vals = [float(x.strip()) for x in s.split(",")]
    if len(vals) != 9:
        raise ValueError(f"extrin_r 需要 9 个浮点数, 收到 {len(vals)} 个")
    return np.array(vals).reshape(3, 3)


def rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 → 单位四元数 [w, x, y, z]"""
    trace = np.trace(R)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = s / 4.0
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = s / 4.0
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = s / 4.0
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = s / 4.0
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def quat_to_rotmat(q: np.ndarray) -> np.ndarray:
    """[w, x, y, z] → 3x3 旋转矩阵"""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


class AiryUnflip(Node):
    def __init__(self):
        super().__init__('airy_unflip')

        self.declare_parameter('input_cloud', '/cloud_registered')
        self.declare_parameter('output_cloud', '/cloud_registered_unflipped')
        self.declare_parameter('input_odometry', '/Odometry')
        self.declare_parameter('output_odometry', '/Odometry_unflipped')
        # self.declare_parameter(
        #     'extrin_r',
        #     '0.0,-1.0,0.0, -1.0,0.0,0.0, 0.0,0.0,-1.0')
        self.declare_parameter(
            'extrin_r',
            '1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,-1.0')
        inp_cloud = self.get_parameter('input_cloud').value
        out_cloud = self.get_parameter('output_cloud').value
        inp_odom = self.get_parameter('input_odometry').value
        out_odom = self.get_parameter('output_odometry').value
        extrin_r_str = self.get_parameter('extrin_r').value

        # 修正旋转矩阵 M（当前默认: 绕 X 轴 180°，翻转 Y 和 Z）
        M = parse_matrix(extrin_r_str)
        self.M_inv = M.T   # M 对称时 M.T = M，但显式写 .T 以兼容非对称场景

        # 点云
        self.sub_cloud = self.create_subscription(
            PointCloud2, inp_cloud, self.cloud_cb, 10)
        self.pub_cloud = self.create_publisher(PointCloud2, out_cloud, 10)

        # 里程计
        self.sub_odom = self.create_subscription(
            Odometry, inp_odom, self.odom_cb, 10)
        self.pub_odom = self.create_publisher(Odometry, out_odom, 10)

        self.get_logger().info(
            f'Airy unflip ready:\n'
            f'  cloud:    {inp_cloud} -> {out_cloud}\n'
            f'  odometry: {inp_odom} -> {out_odom}\n'
            f'  M = {M.flatten().tolist()}\n'
            f'  M_inv = {self.M_inv.flatten().tolist()}\n'
            f'  点云和位姿同时施加修正矩阵, 保持坐标系一致')

    # ── 点云修正 ────────────────────────────────────────────
    def cloud_cb(self, msg: PointCloud2):
        corrected = []
        total = 0

        for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                           skip_nans=True):
            pt_w = np.array([p[0], p[1], p[2]])
            pt_corrected = self.M_inv @ pt_w
            corrected.append((float(pt_corrected[0]),
                              float(pt_corrected[1]),
                              float(pt_corrected[2])))
            total += 1

        out_msg = point_cloud2.create_cloud_xyz32(msg.header, corrected)
        self.pub_cloud.publish(out_msg)

        self.get_logger().info(
            f'{total} pts (Z 轴已恢复朝上)',
            throttle_duration_sec=5.0)

    # ── 位姿修正 ────────────────────────────────────────────
    def odom_cb(self, msg: Odometry):
        # 位置修正: t' = R_inv @ t
        pos = np.array([msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z])
        pos_corrected = self.M_inv @ pos

        # 方向修正:
        # q 表示 世界→机体 的旋转。世界帧施加修正 M 后:
        #   v_body = R_old @ v_world
        #          = R_old @ M^T @ v_world'    (v_world = M^T @ v_world')
        #   因此 R_new = R_old @ M^T
        q_old = np.array([msg.pose.pose.orientation.w,
                          msg.pose.pose.orientation.x,
                          msg.pose.pose.orientation.y,
                          msg.pose.pose.orientation.z])
        R_old = quat_to_rotmat(q_old)
        R_new = R_old @ self.M_inv.T  # M_inv.T = M^T = 修正矩阵的转置
        q_new = rotmat_to_quat(R_new)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = float(pos_corrected[0])
        out.pose.pose.position.y = float(pos_corrected[1])
        out.pose.pose.position.z = float(pos_corrected[2])
        out.pose.pose.orientation.w = float(q_new[0])
        out.pose.pose.orientation.x = float(q_new[1])
        out.pose.pose.orientation.y = float(q_new[2])
        out.pose.pose.orientation.z = float(q_new[3])

        # 协方差修正
        # 6x6 协方差矩阵 (x, y, z, roll, pitch, yaw), 在世界帧中表达
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        cov_out = cov.copy()
        # 位置协方差: C_pp' = R_inv @ C_pp @ R_inv^T
        cov_out[0:3, 0:3] = self.M_inv @ cov[0:3, 0:3] @ self.M_inv.T
        # 位置-姿态交叉协方差 (小角度近似)
        cov_out[0:3, 3:6] = self.M_inv @ cov[0:3, 3:6]
        cov_out[3:6, 0:3] = cov[3:6, 0:3] @ self.M_inv.T
        # 姿态协方差: C_aa' ≈ R_inv @ C_aa @ R_inv^T (小角度近似)
        cov_out[3:6, 3:6] = self.M_inv @ cov[3:6, 3:6] @ self.M_inv.T
        out.pose.covariance = cov_out.flatten().tolist()

        # twist 在机体帧中表达, 不随世界帧旋转变化, 直接复制
        out.twist = msg.twist

        self.pub_odom.publish(out)

        self.get_logger().info(
            'odometry corrected',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    rclpy.spin(AiryUnflip())


if __name__ == '__main__':
    main()
