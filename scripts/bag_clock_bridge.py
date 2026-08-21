#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bag_clock_bridge — 让 /clock 跟随 bag 消息时间戳, 对齐全链路模拟时间

问题背景:
  这个 bag (dataset/robosenseAiry/mapping) 录制于 use_lidar_clock=true 时代,
  消息时间戳 ~11865s(雷达坏时钟), 与录制时墙钟 (1785481507) 不一致。
  `ros2 bag play --clock` 发布的 /clock 是"录制起始墙钟", 不是消息时间戳,
  所以节点 use_sim_time=true 时 now() 仍与消息戳差 1.7e9 秒 → 依旧 TF_OLD_DATA。

本节点:
  订阅 /rslidar_points + /rslidar_imu_data (bag 内 topic),
  把最新收到消息的 header.stamp 作为当前"模拟时间"持续发布 /clock (RELIABLE)。
  于是全链路 use_sim_time=true 时 now() == bag 消息时间戳 → TF 不再被判为过去。

用法 (替代 ros2 bag play --clock):
  ros2 bag play <bag>                          # 不带 --clock
  python3 scripts/bag_clock_bridge.py          # 另开终端
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, Imu


def qos_best_effort():
    return QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )


class BagClockBridge(Node):
    def __init__(self):
        super().__init__('bag_clock_bridge')
        # /clock 用 RELIABLE 发布: RELIABLE 订阅者(echo) 和 BEST_EFFORT 订阅者
        # (rclcpp TimeSource) 都能收到。
        self.pub_ = self.create_publisher(
            Clock, '/clock',
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.latest_ = None  # (sec, nanosec)
        self.sub_pc_ = self.create_subscription(
            PointCloud2, '/rslidar_points', self.cb_pc, qos_best_effort())
        self.sub_imu_ = self.create_subscription(
            Imu, '/rslidar_imu_data', self.cb_imu, qos_best_effort())
        self.timer_ = self.create_timer(0.02, self.tick)  # 50Hz
        self.get_logger().info(
            'bag_clock_bridge 已启动: /clock 将跟随 bag 消息时间戳 (约 11865)')

    def _update(self, stamp):
        key = (stamp.sec, stamp.nanosec)
        if self.latest_ is None or key > self.latest_:
            self.latest_ = key

    def cb_pc(self, msg):
        self._update(msg.header.stamp)

    def cb_imu(self, msg):
        self._update(msg.header.stamp)

    def tick(self):
        if self.latest_ is None:
            return
        c = Clock()
        c.clock.sec = self.latest_[0]
        c.clock.nanosec = self.latest_[1]
        self.pub_.publish(c)


def main(args=None):
    rclpy.init(args=args)
    node = BagClockBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
