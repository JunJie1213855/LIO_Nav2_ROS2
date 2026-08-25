#!/usr/bin/env python3
"""FAR Planner 专用 waypoint follower（带滞回的局部避障）。

在 TARE waypoint_follower 基础上修复"左右摇头"震荡问题：
障碍物正对时 obs_left 和 obs_right 很接近，open_left 会来回翻转，
导致转向方向不停切换。这里加入 last_turn_dir 记忆 + 滞回阈值，
只有另一侧明显更开阔(> hysteresis_margin)时才翻转方向。

订阅: /way_point + /odom + /registered_scan
发布: /cmd_vel
"""

import math
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class WaypointFollowerFar(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("max_angular_vel", 1.0)
        self.declare_parameter("arrival_dist", 0.3)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 2.0)
        self.declare_parameter("stop_dist", 0.45)
        self.declare_parameter("slow_dist", 0.8)
        self.declare_parameter("robot_half_width", 0.35)
        self.declare_parameter("check_height_min", 0.05)
        self.declare_parameter("check_height_max", 0.60)
        # 滞回：另一侧要比当前侧开阔这么多(m)才翻转方向
        self.declare_parameter("hysteresis_margin", 0.30)
        # 每多少秒重置一次转向记忆（避免一直朝一个方向死转）
        self.declare_parameter("turn_dir_timeout", 3.0)

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
        self.hyst_margin = self.get_parameter("hysteresis_margin").value
        self.turn_dir_timeout = self.get_parameter("turn_dir_timeout").value

        self.robot_x = self.robot_y = self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False
        self.waypoint = None

        self.obs_left = float("inf")
        self.obs_center = float("inf")
        self.obs_right = float("inf")

        # 滞回状态
        self.last_turn_dir = 0        # 1=left, -1=right, 0=none
        self.last_turn_time = self.get_clock().now()

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.waypoint_sub = self.create_subscription(PointStamped, "/way_point", self.waypoint_callback, 10)
        self.cloud_sub = self.create_subscription(PointCloud2, "/registered_scan", self.cloud_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"WaypointFollower-FAR started (hysteresis={self.hyst_margin}m, "
            f"turn_timeout={self.turn_dir_timeout}s)")

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

        self.obs_left = self.obs_center = self.obs_right = float("inf")
        half = self.half_width
        for i in range(msg.width):
            base = i * step
            px = struct.unpack_from("f", data, base + x_off)[0]
            py = struct.unpack_from("f", data, base + y_off)[0]
            pz = struct.unpack_from("f", data, base + z_off)[0] if z_off is not None else self.robot_z

            dx = px - self.robot_x
            dy = py - self.robot_y
            lx = dx * cos_yaw + dy * sin_yaw
            ly = -dx * sin_yaw + dy * cos_yaw

            if lx < 0.05 or lx > self.slow_dist + 0.5:
                continue
            dz = pz - self.robot_z
            if dz < self.z_min or dz > self.z_max:
                continue
            if abs(ly) > half:
                continue

            # ly > 0 为机器人左侧
            if ly < -half / 3.0:
                self.obs_right = min(self.obs_right, lx)
            elif ly > half / 3.0:
                self.obs_left = min(self.obs_left, lx)
            else:
                self.obs_center = min(self.obs_center, lx)

    def control_loop(self):
        if not self.have_odom or self.waypoint is None:
            self.publish_zero()
            return

        nearest_obs = min(self.obs_left, self.obs_center, self.obs_right)

        dx = self.waypoint[0] - self.robot_x
        dy = self.waypoint[1] - self.robot_y
        dist = math.hypot(dx, dy)

        if dist < self.arrival_dist:
            self.publish_zero()
            self.last_turn_dir = 0
            return

        goal_yaw = math.atan2(dy, dx)
        goal_rel = goal_yaw - self.robot_yaw
        goal_rel = math.atan2(math.sin(goal_rel), math.cos(goal_rel))

        open_left = self.obs_left > self.obs_right

        if self.obs_center < self.stop_dist:
            # 正前方被挡：用滞回决定转向方向
            self.last_turn_dir = self._decide_turn_dir(open_left)
            desired_rel = self.last_turn_dir * 1.2
        elif nearest_obs < self.slow_dist:
            goal_on_open = (open_left and goal_rel > 0.0) or (
                not open_left and goal_rel < 0.0)
            if goal_on_open:
                desired_rel = goal_rel
                self.last_turn_dir = 0
            else:
                self.last_turn_dir = self._decide_turn_dir(open_left)
                desired_rel = self.last_turn_dir * 0.8
        else:
            desired_rel = goal_rel
            self.last_turn_dir = 0

        yaw_err = desired_rel

        if abs(yaw_err) > 0.5:
            v = 0.0
        else:
            v = min(self.kp_v * dist, self.max_v)

        w = self.kp_w * yaw_err

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

    def _decide_turn_dir(self, open_left):
        """带滞回的转向方向决策，避免左右来回翻转。"""
        now = self.get_clock().now()
        if (now - self.last_turn_time).nanoseconds * 1e-9 > self.turn_dir_timeout:
            self.last_turn_dir = 0  # 超时重置，重新选方向

        if self.last_turn_dir == 0:
            # 首次选择
            self.last_turn_time = now
            return 1 if open_left else -1

        if self.last_turn_dir == 1:
            # 之前向左，只有右侧明显更开阔(> hysteresis_margin)才翻转向右
            if self.obs_right > self.obs_left + self.hyst_margin:
                self.last_turn_time = now
                return -1
            return 1
        else:
            # 之前向右，只有左侧明显更开阔才翻转向左
            if self.obs_left > self.obs_right + self.hyst_margin:
                self.last_turn_time = now
                return 1
            return -1

    def publish_zero(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = WaypointFollowerFar()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
