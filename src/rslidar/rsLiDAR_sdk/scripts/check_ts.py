#!/usr/bin/env python3
"""Compare LiDAR and IMU timestamps in real time to verify clock sync."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu

class TimestampChecker(Node):
    def __init__(self):
        super().__init__('ts_checker')
        self.last_lidar = None
        self.last_imu = None
        self.lidar_count = 0
        self.imu_count = 0

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/rslidar_points', self.lidar_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/rslidar_imu_data', self.imu_cb, 100)

    def lidar_cb(self, msg):
        self.lidar_count += 1
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delta = ""
        if self.last_imu:
            diff_ms = (ts - self.last_imu) * 1000
            delta = f" | dIMU={diff_ms:+.1f}ms"
        self.last_lidar = ts
        print(f"[LiDAR #{self.lidar_count:4d}] {ts:.6f}{delta}")

    def imu_cb(self, msg):
        self.imu_count += 1
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delta = ""
        if self.last_lidar:
            diff_ms = (ts - self.last_lidar) * 1000
            delta = f" | dLDR={diff_ms:+.1f}ms"
        self.last_imu = ts
        if self.imu_count % 10 == 0 or self.imu_count <= 3:
            print(f"[IMU   #{self.imu_count:4d}] {ts:.6f}{delta}")

def main():
    rclpy.init()
    node = TimestampChecker()
    print("Timestamp Checker -- /rslidar_points vs /rslidar_imu_data")
    print(f"{'Source':>8} {'#':>6}  {'Timestamp':>16}  {'Delta':>15}")
    print("-" * 55)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
