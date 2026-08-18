#!/usr/bin/env python3
"""TF → Odometry 桥接节点

Cartographer 只发布 odom→base_footprint TF，不发布 /odom 话题。
但 Nav2 的 controller_server 和 velocity_smoother 需要 /odom (nav_msgs/Odometry)。

这个节点读取 TF 树中的 odom→base_footprint 变换，发布为标准 Odometry 消息。
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import math


class TfToOdometry(Node):
    def __init__(self):
        super().__init__('tf_to_odometry')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.last_pose = None
        self.last_time = None

        timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
        except Exception:
            return

        msg = Odometry()
        msg.header.stamp = t.header.stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = t.transform.rotation

        if self.last_pose is not None and self.last_time is not None:
            dt = (rclpy.time.Time.from_msg(t.header.stamp) -
                  self.last_time).nanoseconds * 1e-9
            if dt > 0.01:
                msg.twist.twist.linear.x = (
                    t.transform.translation.x - self.last_pose.position.x) / dt
                msg.twist.twist.linear.y = (
                    t.transform.translation.y - self.last_pose.position.y) / dt
                msg.twist.twist.angular.z = (
                    math.atan2(
                        2.0 * (t.transform.rotation.w * t.transform.rotation.z +
                               t.transform.rotation.x * t.transform.rotation.y),
                        1.0 - 2.0 * (t.transform.rotation.y ** 2 +
                                    t.transform.rotation.z ** 2))
                    - math.atan2(
                        2.0 * (self.last_pose.orientation.w * self.last_pose.orientation.z +
                               self.last_pose.orientation.x * self.last_pose.orientation.y),
                        1.0 - 2.0 * (self.last_pose.orientation.y ** 2 +
                                    self.last_pose.orientation.z ** 2))
                ) / dt

        self.last_pose = msg.pose.pose
        self.last_time = rclpy.time.Time.from_msg(t.header.stamp)
        self.odom_pub.publish(msg)


def main():
    rclpy.init()
    node = TfToOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
