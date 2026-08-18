#!/usr/bin/env python3
"""Waypoint follower with stuck detection and recovery."""
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 0.8)
        self.declare_parameter('goal_tolerance', 0.3)
        self.max_v = self.get_parameter('max_linear_vel').value
        self.max_w = self.get_parameter('max_angular_vel').value
        self.tolerance = self.get_parameter('goal_tolerance').value

        self.cx = self.cy = self.cyaw = 0.0
        self.prev_x = self.prev_y = 0.0
        self.stuck_time = 0.0
        self.target = None
        self.stuck = False

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.wp_sub = self.create_subscription(PointStamped, '/way_point', self.wp_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info('Waypoint follower v2 ready')

    def odom_cb(self, msg):
        self.cx = msg.pose.pose.position.x
        self.cy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.cyaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def wp_cb(self, msg):
        self.target = (msg.point.x, msg.point.y)
        self.get_logger().info(f'Waypoint: ({msg.point.x:.2f}, {msg.point.y:.2f})')

    def loop(self):
        if self.target is None:
            return
        dx = self.target[0] - self.cx
        dy = self.target[1] - self.cy
        dist = math.hypot(dx, dy)
        cmd = Twist()

        if dist < self.tolerance:
            self.cmd_pub.publish(cmd)
            return

        # 撞墙检测: 发了速度指令但位置几乎不动
        moved = math.hypot(self.cx - self.prev_x, self.cy - self.prev_y)
        self.prev_x, self.prev_y = self.cx, self.cy

        if moved < 0.02:  # 0.1s 内移动不到 2cm → 可能撞墙
            self.stuck_time += 0.1
        else:
            self.stuck_time = 0.0

        if self.stuck_time > 1.0:
            self.get_logger().warn('Stuck! Backing up and turning...', throttle_duration_sec=2.0)
            cmd.linear.x = -0.2
            cmd.angular.z = 0.8
            self.cmd_pub.publish(cmd)
            self.stuck_time = 0.0
            return

        # 正常导航
        target_yaw = math.atan2(dy, dx)
        dyaw = target_yaw - self.cyaw
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))
        cmd.linear.x = min(self.max_v, max(0.1, dist * 0.5))
        cmd.angular.z = max(-self.max_w, min(self.max_w, dyaw * 2.0))
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(WaypointFollower())

if __name__ == '__main__':
    main()
