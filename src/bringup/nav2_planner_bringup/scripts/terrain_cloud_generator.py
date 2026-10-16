#!/usr/bin/env python3
"""LIO 点云 → FAR Planner terrain cloud 转换节点。

FAR Planner 用点云的 intensity 字段当作"相对地面高度"来分类 free/obstacle
(utility.cpp ExtractFreeAndObsCloud: intensity < terrain_free_Z 视为 free)。
普通 LIO 点云(/registered_scan)的 intensity 是反射率，不是高度，
所以这里把每个点的 intensity 重写为"相对地面高度" = z - 地面z。

地面高度自动估计：取每帧点云 z 的低分位数（默认 5%），并用 EMA 平滑，
避免单帧噪声导致地面估计抖动，也避免 LIO 原点不完全贴地时 free/obs 分类偏移。

参数:
  auto_ground (bool, 默认 true): true=自动估计地面高度, false=用固定 ground_z
  ground_z (float, 默认 0.0): 非自动模式下的固定地面高度
  ground_percentile (float, 默认 5.0): 自动估计用的 z 低分位数(%)
  ground_ema_alpha (float, 默认 0.1): 地面高度 EMA 平滑系数

订阅: /registered_scan (sensor_msgs/PointCloud2)
发布: /terrain_map   (x, y, z, intensity=z-地面z)
"""

import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class TerrainCloudGenerator(Node):
    def __init__(self):
        super().__init__("terrain_cloud_generator")

        self.declare_parameter("auto_ground", True)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("ground_percentile", 5.0)
        self.declare_parameter("ground_ema_alpha", 0.1)

        self._ground_z = None  # None = 尚未标定

        self.sub = self.create_subscription(
            PointCloud2, "/registered_scan", self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, "/terrain_map", 10)
        self.get_logger().info("Terrain cloud generator ready")

    def _estimate_ground_z(self, zs):
        """用 z 的低分位数估计地面高度(纯 Python, 不依赖 numpy)。"""
        percentile = float(self.get_parameter("ground_percentile").value)
        sorted_zs = sorted(zs)
        n = len(sorted_zs)
        if n == 0:
            return 0.0
        idx = int(n * percentile / 100.0)
        idx = max(0, min(idx, n - 1))
        return sorted_zs[idx]

    def callback(self, msg: PointCloud2):
        offsets = {}
        for f in msg.fields:
            offsets[f.name] = f.offset

        if "x" not in offsets or "y" not in offsets or "z" not in offsets:
            self.get_logger().warn("cloud 缺少 x/y/z 字段，跳过")
            return

        ox = offsets["x"]
        oy = offsets["y"]
        oz = offsets["z"]

        n = msg.width * msg.height
        if n == 0:
            return

        # 第一遍：只收集 z，用于估计地面高度
        zs = [0.0] * n
        for i in range(n):
            base = i * msg.point_step
            zs[i] = struct.unpack_from("<f", msg.data, base + oz)[0]

        # 更新地面高度
        if self.get_parameter("auto_ground").value:
            est = self._estimate_ground_z(zs)
            alpha = float(self.get_parameter("ground_ema_alpha").value)
            if self._ground_z is None:
                self._ground_z = est
                self.get_logger().info(f"地面高度标定完成: z={self._ground_z:.3f}")
            else:
                self._ground_z = self._ground_z * (1.0 - alpha) + est * alpha
        else:
            self._ground_z = float(self.get_parameter("ground_z").value)

        ground_z = self._ground_z

        # 第二遍：重读 x/y/z，写 intensity = z - 地面z (相对地面高度)
        raw = bytearray()
        for i in range(n):
            base = i * msg.point_step
            x = struct.unpack_from("<f", msg.data, base + ox)[0]
            y = struct.unpack_from("<f", msg.data, base + oy)[0]
            z = struct.unpack_from("<f", msg.data, base + oz)[0]
            raw += struct.pack("<4f", x, y, z, z - ground_z)

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = n
        out.point_step = 16
        out.row_step = n * 16
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
