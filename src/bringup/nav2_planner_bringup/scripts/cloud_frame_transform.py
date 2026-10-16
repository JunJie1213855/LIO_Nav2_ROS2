#!/usr/bin/env python3
"""PointCloud2 坐标系变换节点（tf2）。

把 GroundSeg 输出的 /no_ground_cloud（livox_frame 系）转到 SCAN-Planner 期望的
odom 系。与 cloud_z_filter.py 一样做字节级原地变换，保留除 x/y/z 外的所有字段
（intensity 等），避免依赖 sensor_msgs_py 重建丢字段。

参数:
  source_frame 输入点云的 frame（默认 livox_frame）
  target_frame 输出点云的 frame（默认 odom）
话题:
  cloud_in  → 输入
  cloud_out → 输出
"""
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, TransformException


class CloudFrameTransform(Node):
    def __init__(self):
        super().__init__('cloud_frame_transform')
        self.declare_parameter('source_frame', 'livox_frame')
        self.declare_parameter('target_frame', 'odom')
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, 'cloud_in', self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, 'cloud_out',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(
            f'Transform cloud: {self.source_frame} -> {self.target_frame}')

    def callback(self, msg):
        field_offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in field_offsets for k in ('x', 'y', 'z')):
            self.get_logger().warn(
                'cloud missing x/y/z fields, drop frame', throttle_duration_sec=2.0)
            return
        ox = field_offsets['x']
        oy = field_offsets['y']
        oz = field_offsets['z']

        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup {self.target_frame}<-{self.source_frame} failed: {e}',
                throttle_duration_sec=2.0)
            return

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        data = bytearray(msg.data)
        n = msg.width * msg.height
        ps = msg.point_step
        for i in range(n):
            base = i * ps
            px = struct.unpack_from('f', data, base + ox)[0]
            py = struct.unpack_from('f', data, base + oy)[0]
            pz = struct.unpack_from('f', data, base + oz)[0]

            # v' = v + w*t + cross(q_vec, t),  t = 2*cross(q_vec, v)
            ux = qy * pz - qz * py
            uy = qz * px - qx * pz
            uz = qx * py - qy * px
            tx2 = 2.0 * ux
            ty2 = 2.0 * uy
            tz2 = 2.0 * uz
            cx = qy * tz2 - qz * ty2
            cy = qz * tx2 - qx * tz2
            cz = qx * ty2 - qy * tx2

            struct.pack_into('f', data, base + ox, px + qw * tx2 + cx + tx)
            struct.pack_into('f', data, base + oy, py + qw * ty2 + cy + ty)
            struct.pack_into('f', data, base + oz, pz + qw * tz2 + cz + tz)

        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = self.target_frame
        out.fields = msg.fields
        out.height = msg.height
        out.width = msg.width
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.is_bigendian = msg.is_bigendian
        out.is_dense = msg.is_dense
        out.data = bytes(data)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CloudFrameTransform()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
