#!/usr/bin/env python3
"""Waypoint follower with local obstacle avoidance.

订阅:
  /way_point  (geometry_msgs/PointStamped)  TARE 规划的下一个目标点
  /odom       (nav_msgs/Odometry)            机器人里程计
  /registered_scan (sensor_msgs/PointCloud2) 已配准点云(odom 帧)

发布:
  /cmd_vel    (geometry_msgs/Twist)          差速底盘速度指令

逻辑:
  1. 正常情况用 PID 朝 waypoint 前进(先转向后直行)
  2. 用前方点云做局部避障: 前方扇形区域出现障碍物时减速/停车/转向远离
"""

import math
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        # 运动参数
        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("max_angular_vel", 1.0)
        self.declare_parameter("arrival_dist", 0.3)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 2.0)
        # 避障参数
        self.declare_parameter("stop_dist", 0.45)      # 正前方障碍物距离阈值
        self.declare_parameter("slow_dist", 0.8)       # 开始减速的距离
        self.declare_parameter("robot_half_width", 0.35)  # 机器人半宽 + 余量
        self.declare_parameter("check_height_min", 0.05)  # 相对机器人高度的障碍物检查范围(排除地面)
        self.declare_parameter("check_height_max", 0.60)

        self.max_v = self.get_parameter("max_linear_vel").value
        self.max_w = self.get_parameter("max_angular_vel").value
        self.arrival_dist = self.get_parameter("arrival_dist").value
        self.kp_v = self.get_parameter("kp_linear").value
        self.kp_w = self.get_parameter("kp_angular").value
        self.stop_dist = self.get_parameter("stop_dist").value
        self.slow_dist = self.get_parameter("slow_dist").value
        self.half_width = self.get_parameter("robot_half_width").value
        self.z_min = self.get_parameter("check_height_min").value
        self.z_max = self.get_parameter("check_height_max").value

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False

        self.waypoint = None  # (x, y, z) in odom frame

        # 障碍物距离检测结果: 前方分左/中/右三个扇区的最小距离
        self.obs_left = float("inf")    # 左前方 (y < -half_width/2)
        self.obs_center = float("inf")  # 正前方
        self.obs_right = float("inf")   # 右前方 (y > half_width/2)

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10)
        self.waypoint_sub = self.create_subscription(
            PointStamped, "/way_point", self.waypoint_callback, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/registered_scan", self.cloud_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            "WaypointFollower started with avoidance "
            f"(stop={self.stop_dist}m, slow={self.slow_dist}m)")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.have_odom = True

    def waypoint_callback(self, msg: PointStamped):
        self.waypoint = (msg.point.x, msg.point.y, msg.point.z)

    def cloud_callback(self, msg: PointCloud2):
        if not self.have_odom:
            return
        # 解析 PointCloud2 的 xyz 字段
        x_off = y_off = z_off = None
        for f in msg.fields:
            if f.name == "x":
                x_off = f.offset
            elif f.name == "y":
                y_off = f.offset
            elif f.name == "z":
                z_off = f.offset
        if x_off is None or y_off is None:
            return

        step = msg.point_step
        data = msg.data
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)

        self.obs_left = float("inf")
        self.obs_center = float("inf")
        self.obs_right = float("inf")

        half = self.half_width
        for i in range(msg.width):
            base = i * step
            px = struct.unpack_from("f", data, base + x_off)[0]
            py = struct.unpack_from("f", data, base + y_off)[0]
            pz = struct.unpack_from("f", data, base + z_off)[0] if z_off is not None else self.robot_z

            # 相对机器人水平位置
            dx = px - self.robot_x
            dy = py - self.robot_y
            # 转 body 帧
            lx = dx * cos_yaw + dy * sin_yaw
            ly = -dx * sin_yaw + dy * cos_yaw

            # 只关心前方且高度范围内的点
            if lx < 0.05 or lx > self.slow_dist + 0.5:
                continue
            dz = pz - self.robot_z
            if dz < self.z_min or dz > self.z_max:
                continue
            if abs(ly) > half:
                continue

            # 分扇区记录最小距离
            if ly < -half / 3.0:
                self.obs_left = min(self.obs_left, lx)
            elif ly > half / 3.0:
                self.obs_right = min(self.obs_right, lx)
            else:
                self.obs_center = min(self.obs_center, lx)

    def control_loop(self):
        if not self.have_odom or self.waypoint is None:
            self.publish_zero()
            return

        # 最近的障碍物距离
        nearest_obs = min(self.obs_left, self.obs_center, self.obs_right)

        # 紧急: 正前方障碍物太近
        if self.obs_center < self.stop_dist:
            # 原地转向, 朝更开阔的一侧避开
            turn_dir = 1.0 if self.obs_left > self.obs_right else -1.0
            cmd = Twist()
            cmd.angular.z = turn_dir * 0.6
            self.cmd_pub.publish(cmd)
            return

        dx = self.waypoint[0] - self.robot_x
        dy = self.waypoint[1] - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.arrival_dist:
            self.publish_zero()
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = target_yaw - self.robot_yaw
        yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))

        # 转向远离障碍物: 前方有障碍时修正目标方向
        if nearest_obs < self.slow_dist and abs(yaw_err) < 1.2:
            # 前方有障碍且大致朝它走, 偏向开阔侧
            if self.obs_left > self.obs_right:
                yaw_err += 0.6  # 往左偏
            else:
                yaw_err -= 0.6  # 往右偏

        if abs(yaw_err) > 1.0:
            v = 0.0
            w = self.kp_w * yaw_err
        else:
            v = min(self.kp_v * dist, self.max_v)
            w = self.kp_w * yaw_err

        # 前方障碍物减速
        if nearest_obs < self.slow_dist:
            ratio = max(0.0, (nearest_obs - self.stop_dist) /
                        (self.slow_dist - self.stop_dist))
            v = min(v, self.max_v * ratio * 0.8)

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
