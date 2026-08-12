#!/usr/bin/env python3
"""Simple waypoint follower: subscribes /way_point → publishes /cmd_vel

Uses a proportional controller in the robot body frame to drive toward
the waypoint. Stops when within arrival threshold and aligns yaw.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("max_angular_vel", 1.0)
        self.declare_parameter("arrival_dist", 0.3)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 2.0)

        self.max_v = self.get_parameter("max_linear_vel").value
        self.max_w = self.get_parameter("max_angular_vel").value
        self.arrival_dist = self.get_parameter("arrival_dist").value
        self.kp_v = self.get_parameter("kp_linear").value
        self.kp_w = self.get_parameter("kp_angular").value

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False

        self.waypoint = None  # (x, y, z) in odom frame

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.waypoint_sub = self.create_subscription(
            PointStamped, "/way_point", self.waypoint_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("WaypointFollower started")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.have_odom = True

    def waypoint_callback(self, msg: PointStamped):
        self.waypoint = (msg.point.x, msg.point.y, msg.point.z)
        self.get_logger().debug(f"New waypoint: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})")

    def control_loop(self):
        if not self.have_odom or self.waypoint is None:
            self.publish_zero()
            return

        dx = self.waypoint[0] - self.robot_x
        dy = self.waypoint[1] - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.arrival_dist:
            self.publish_zero()
            return

        # Target heading in odom frame
        target_yaw = math.atan2(dy, dx)

        # Yaw error in [-pi, pi]
        yaw_err = target_yaw - self.robot_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        # Turn in place if facing away from target
        if abs(yaw_err) > 1.0:
            v = 0.0
            w = self.kp_w * yaw_err
        else:
            v = min(self.kp_v * dist, self.max_v)
            w = self.kp_w * yaw_err

        w = max(-self.max_w, min(self.max_w, w))
        v = max(0.0, min(self.max_v, v))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def publish_zero(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
