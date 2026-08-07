#!/usr/bin/env python3
"""Z-axis PassThrough filter for point clouds."""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import struct

class CloudZFilter(Node):
    def __init__(self):
        super().__init__('cloud_z_filter')
        self.declare_parameter('z_min', -1.0)
        self.declare_parameter('z_max', 2.0)
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value

        self.sub = self.create_subscription(
            PointCloud2, 'cloud_in', self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, 'cloud_out', qos_profile_sensor_data)
        self.get_logger().info(f'Z filter active: [{self.z_min:.1f}, {self.z_max:.1f}] m')

    def callback(self, msg):
        fields = {f.name: (f.offset, f.datatype) for f in msg.fields}
        oz = fields.get('z', (8, 7))

        raw = bytearray()
        count = 0
        for i in range(msg.width * msg.height):
            base = i * msg.point_step
            z_bytes = msg.data[base + oz[0] : base + oz[0] + 4]
            z = struct.unpack('f', bytes(z_bytes))[0]
            if self.z_min <= z <= self.z_max:
                raw.extend(msg.data[base : base + msg.point_step])
                count += 1

        out = PointCloud2()
        out.header = msg.header
        out.fields = msg.fields
        out.height = 1
        out.point_step = msg.point_step
        out.is_bigendian = msg.is_bigendian
        out.is_dense = False
        out.data = bytes(raw)
        out.width = count
        out.row_step = count * msg.point_step
        self.pub.publish(out)

def main():
    rclpy.init()
    node = CloudZFilter()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
