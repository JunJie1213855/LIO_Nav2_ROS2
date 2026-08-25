#!/usr/bin/env python3
"""
Z 偏移中继节点（DSV 地面机器人专用）

DSV Planner 的 RRT 根节点取 /state_estimation 的 z（地面≈0），但扩展节点的 z 被
getZvalue() 覆盖为 terrain_elev + kVehicleHeight(0.75)。这导致两个致命问题：

1. RRT 根在 z≈0 落进机器人正下方"未知地面体素"[-0.175,+0.175] 内
   —— 雷达看不到正下方地面，该体素永远无法变 free → 所有扩展被拒 → 树长不出来
   → gainFound()=false → 误判 "Exploration completed, returning home"。
2. 图顶点两种来源高度不一致：keypose(/state_estimation_at_scan) 在 z≈0.03，
   RRT 节点在 z≈0.78，|Δz|=0.75 > kMaxVertexDiffAlongZ(0.5) → 边建不起来
   → 图路径为空 → getGain()=0 → 同样触发 returning home。

本节点把 /odom 的 z 抬高 kVehicleHeight，使 DSV 所有内部坐标系统一在
"车辆高度" 上规划（与无人机版一致）。只影响 DSV 内部：octomap(/lidar_frame_pcd
+ TF，真实高度)、waypoint_follower(/way_point 二维驱动)、grid(x/y only) 均不受影响。

用法:
  python3 scripts/z_offset_relay.py --ros-args \
    -p z_offset:=0.75 -p output_estimation:=/state_estimation

参数:
  input_odom              输入里程计 topic（默认 /odom，来自 sensor_scan_generation）
  z_offset                Z 抬升量，= kVehicleHeight(0.75)（默认 0.75）
  output_estimation       /state_estimation 输出 topic（默认 /state_estimation）
  output_estimation_scan  /state_estimation_at_scan 输出 topic（默认 /state_estimation_at_scan）
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class ZOffsetRelay(Node):
    def __init__(self):
        super().__init__('z_offset_relay')

        self.declare_parameter('input_odom', '/odom')
        self.declare_parameter('z_offset', 0.75)
        self.declare_parameter('output_estimation', '/state_estimation')
        self.declare_parameter('output_estimation_scan', '/state_estimation_at_scan')

        inp = self.get_parameter('input_odom').value
        self.z_offset = self.get_parameter('z_offset').value
        out1 = self.get_parameter('output_estimation').value
        out2 = self.get_parameter('output_estimation_scan').value

        self.pub1 = self.create_publisher(Odometry, out1, 10)
        self.pub2 = self.create_publisher(Odometry, out2, 10)
        self.sub = self.create_subscription(Odometry, inp, self.odom_cb, 10) # 订阅 /odom 话题数据

        self.get_logger().info(
            f'Z-offset relay ready: {inp} → {out1} & {out2}  '
            f'(z += {self.z_offset:.2f} = kVehicleHeight)')

    def odom_cb(self, msg: Odometry): # 将 z 轴提高 0.75 m后，再做导航规划
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = msg.pose
        out.pose.pose.position.z += self.z_offset
        out.twist = msg.twist
        self.pub1.publish(out)
        self.pub2.publish(out)


def main():
    rclpy.init()
    rclpy.spin(ZOffsetRelay())


if __name__ == '__main__':
    main()
