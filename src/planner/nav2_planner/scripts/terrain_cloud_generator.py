#!/usr/bin/env python3
"""LIO 点云 → FAR Planner terrain cloud 转换节点。

FAR Planner 用点云的 intensity 字段当作"地形高度"来分类 free/obstacle
（utility.cpp ExtractFreeAndObsCloud: intensity < terrain_free_Z 视为 free）。
普通 LIO 点云(/registered_scan)的 intensity 是反射率，不是高度，
所以这里把每个点的 intensity 重写为 z(高度)。

订阅: /registered_scan (sensor_msgs/PointCloud2)
发布: /terrain_map   (x, y, z, intensity=z)
"""

import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class TerrainCloudGenerator(Node):
    def __init__(self):
        super().__init__("terrain_cloud_generator")
        self.sub = self.create_subscription(
            PointCloud2, "/registered_scan", self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, "/terrain_map", 10)
        self.get_logger().info("Terrain cloud generator ready")

    def callback(self, msg: PointCloud2):
        # 找 x/y/z 字段偏移
        offsets = {}
        for f in msg.fields:
            offsets[f.name] = (f.offset, f.datatype)

        if "x" not in offsets or "y" not in offsets or "z" not in offsets:
            self.get_logger().warn("cloud 缺少 x/y/z 字段，跳过")
            return

        ox, _ = offsets["x"]
        oy, _ = offsets["y"]
        oz, _ = offsets["z"]

        raw = bytearray()
        count = 0
        for i in range(msg.width * msg.height):
            base = i * msg.point_step
            x = struct.unpack_from("f", msg.data, base + ox)[0]
            y = struct.unpack_from("f", msg.data, base + oy)[0]
            z = struct.unpack_from("f", msg.data, base + oz)[0]

            # 输出 x, y, z, intensity(=z)
            raw += struct.pack("<4f", x, y, z, z)
            count += 1

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = count
        out.point_step = 16
        out.row_step = count * 16
        out.is_bigendian = False
        out.is_dense = True
        out.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out.data = bytes(raw)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TerrainCloudGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
