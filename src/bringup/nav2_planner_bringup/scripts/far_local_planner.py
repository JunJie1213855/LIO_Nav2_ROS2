#!/usr/bin/env python3
"""FAR Planner 局部规划器：VFH 间隙跟随 + 启动原地旋转建图。

解决两个问题:
1. 启动阶段 FAR 可见图为空 -> 全局路径退化成 odom->goal 直线。
   此节点在收到第一个目标前原地缓慢旋转，让 FAR 持续观察到"新障碍点"，
   从而快速积累可见图。
2. 原 waypoint_follower_far.py 只是 0.7m 宽的"保险杠"，无法绕障。
   这里改为 VFH（极坐标障碍直方图）：在 /registered_scan 与
   /navigation_boundary 上做 360° 障碍分布，选择离目标方向最近、
   且宽度足够容纳机器人的自由扇区。

订阅:
  /way_point           geometry_msgs/PointStamped   (map)
  /odom                nav_msgs/Odometry            (odom)
  /registered_scan     sensor_msgs/PointCloud2      (odom)
  /navigation_boundary geometry_msgs/PolygonStamped (map)
发布:
  /cmd_vel             geometry_msgs/Twist

注意: 本 launch 里 map->odom 是 identity 静态 TF，故 map 与 odom 坐标数值一致。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PolygonStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class FarLocalPlanner(Node):
    def __init__(self):
        super().__init__("far_local_planner")

        # ---- 参数 ----
        self.declare_parameter("max_linear_vel", 0.5)
        self.declare_parameter("max_angular_vel", 1.0)
        self.declare_parameter("arrival_dist", 0.3)
        self.declare_parameter("kp_linear", 0.8)
        self.declare_parameter("kp_angular", 2.0)
        self.declare_parameter("obstacle_range", 2.5)
        self.declare_parameter("robot_radius", 0.35)
        self.declare_parameter("safe_margin", 0.25)
        self.declare_parameter("stop_dist", 0.50)
        self.declare_parameter("slow_dist", 1.20)
        self.declare_parameter("z_min", 0.05)
        self.declare_parameter("z_max", 2.0)
        self.declare_parameter("n_bins", 72)
        self.declare_parameter("boundary_sample_step", 0.15)
        self.declare_parameter("boundary_range", 3.0)
        # 启动原地旋转
        self.declare_parameter("init_rotate_enable", True)
        self.declare_parameter("init_rotate_vel", 0.4)
        self.declare_parameter("init_rotate_duration", 18.0)
        self.declare_parameter("startup_delay", 6.0)
        self.declare_parameter("control_rate", 10.0)

        p = lambda n: self.get_parameter(n).value
        self.max_v = p("max_linear_vel")
        self.max_w = p("max_angular_vel")
        self.arrival_dist = p("arrival_dist")
        self.kp_v = p("kp_linear")
        self.kp_w = p("kp_angular")
        self.obs_range = p("obstacle_range")
        self.robot_r = p("robot_radius")
        self.safe_dist = self.robot_r + p("safe_margin")
        self.stop_dist = p("stop_dist")
        self.slow_dist = p("slow_dist")
        self.z_min = p("z_min")
        self.z_max = p("z_max")
        self.n_bins = int(p("n_bins"))
        self.bound_step = p("boundary_sample_step")
        self.bound_range = p("boundary_range")
        self.init_rot_enable = p("init_rotate_enable")
        self.init_rot_vel = p("init_rotate_vel")
        self.init_rot_dur = p("init_rotate_duration")
        self.startup_delay = p("startup_delay")
        self.control_rate = p("control_rate")

        self.bin_rad = 2.0 * math.pi / self.n_bins

        # ---- 状态 ----
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_yaw = 0.0
        self.have_odom = False
        self.first_odom_time = None
        self.last_ctrl_t = None
        self.waypoint = None  # (x, y)

        self.cloud = None    # numpy (N,2) odom 帧 xy
        self.cloud_z = None  # numpy (N,)  odom 帧 z

        self.boundary_edges = []  # [( (x1,y1), (x2,y2) ), ...]

        self.rotated = 0.0  # 已累计初始旋转角度 (rad)

        # ---- pub/sub ----
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.wp_sub = self.create_subscription(PointStamped, "/way_point", self.wp_cb, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2, "/registered_scan", self.cloud_cb, 5)
        self.bound_sub = self.create_subscription(
            PolygonStamped, "/navigation_boundary", self.bound_cb, 5)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(1.0 / self.control_rate, self.control)

        self.get_logger().info(
            f"FarLocalPlanner started (VFH, init_rotate={self.init_rot_enable}, "
            f"rate={self.control_rate}Hz, robot_r={self.robot_r}m)")

    # ---------- 回调 ----------
    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        sy = 2.0 * (q.w * q.z + q.x * q.y)
        cy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(sy, cy)
        if not self.have_odom:
            self.have_odom = True
            self.first_odom_time = self.get_clock().now()

    def wp_cb(self, msg: PointStamped):
        self.waypoint = (msg.point.x, msg.point.y)

    def cloud_cb(self, msg: PointCloud2):
        xo = yo = zo = None
        for f in msg.fields:
            if f.name == "x":
                xo = f.offset
            elif f.name == "y":
                yo = f.offset
            elif f.name == "z":
                zo = f.offset
        if xo is None or yo is None:
            return
        n = msg.width * msg.height
        if n <= 0:
            return
        # 注意: np.frombuffer 的 offset 是"连续读"(stride=itemsize)，而点云字段是
        # 交错的(x/y/z/intensity 按 point_step 字节交错)。直接用 offset 会把
        # y/z/intensity 的值混进 x，产生大量 NaN 和错误坐标。
        # 必须按 point_step 做 stride 逐字段取：先读整段 float，再每隔 npt 取一个。
        raw = np.frombuffer(msg.data, dtype=np.float32)
        npt = msg.point_step // 4  # 每个点的 float32 数量
        xs = raw[xo // 4::npt][:n]
        ys = raw[yo // 4::npt][:n]
        if zo is not None:
            zs = raw[zo // 4::npt][:n]
        else:
            zs = np.full(n, self.robot_z, dtype=np.float32)
        self.cloud = np.column_stack((xs, ys))
        self.cloud_z = zs

    def bound_cb(self, msg: PolygonStamped):
        pts = [(p.x, p.y) for p in msg.polygon.points]
        edges = []
        # FAR 把每条边界边作为相邻两个点发布（z 每边 +0.001）
        for i in range(0, len(pts) - 1, 2):
            edges.append((pts[i], pts[i + 1]))
        self.boundary_edges = edges

    # ---------- 控制 ----------
    def control(self):
        if not self.have_odom:
            self.pub(0.0, 0.0)
            return

        now = self.get_clock().now()
        dt = 0.0
        if self.last_ctrl_t is not None:
            dt = (now - self.last_ctrl_t).nanoseconds * 1e-9
        self.last_ctrl_t = now

        # 启动阶段：无目标时原地旋转，让 FAR 积累可见图
        if self.waypoint is None:
            elapsed = (now - self.first_odom_time).nanoseconds * 1e-9
            if (self.init_rot_enable and elapsed >= self.startup_delay
                    and self.rotated < self.init_rot_dur):
                self.rotated += self.init_rot_vel * dt
                self.pub(0.0, self.init_rot_vel)
            else:
                self.pub(0.0, 0.0)
            return

        # 导航阶段
        dx = self.waypoint[0] - self.robot_x
        dy = self.waypoint[1] - self.robot_y
        dist = math.hypot(dx, dy)
        if dist < self.arrival_dist:
            self.pub(0.0, 0.0)
            return

        goal_rel = wrap_pi(math.atan2(dy, dx) - self.robot_yaw)
        obs_dist = self._build_histogram()
        heading = self._pick_heading(obs_dist, goal_rel)
        clearance = self._clearance_around(obs_dist, heading)

        heading_err = heading  # 已为机器人系相对角

        v = 0.0
        if abs(heading_err) < 0.6:
            v = min(self.kp_v * dist, self.max_v)
        ratio = (clearance - self.stop_dist) / max(self.slow_dist - self.stop_dist, 1e-3)
        ratio = max(0.0, min(1.0, ratio))
        v = min(v, self.max_v * ratio * 0.8)
        if clearance < self.stop_dist:
            v = 0.0

        w = self.kp_w * heading_err
        w = max(-self.max_w, min(self.max_w, w))

        self.pub(v, w)

    def _build_histogram(self):
        """每个角度 bin 到最近障碍的距离 (n_bins,)，inf 表示该方向无障碍。"""
        obs = np.full(self.n_bins, np.inf)
        cos_y = math.cos(self.robot_yaw)
        sin_y = math.sin(self.robot_yaw)

        def add_xy(px, py):
            dx = px - self.robot_x
            dy = py - self.robot_y
            r = math.hypot(dx, dy)
            if r < 0.15 or r > self.obs_range:
                return
            lx = dx * cos_y + dy * sin_y
            ly = -dx * sin_y + dy * cos_y
            ang = math.atan2(ly, lx)
            half = math.asin(min(1.0, self.robot_r / r))
            a0 = ang - half
            a1 = ang + half
            i0 = int(math.floor((a0 + math.pi) / (2.0 * math.pi) * self.n_bins))
            i1 = int(math.floor((a1 + math.pi) / (2.0 * math.pi) * self.n_bins))
            for b in range(i0, i1 + 1):
                bi = b % self.n_bins
                if r < obs[bi]:
                    obs[bi] = r

        # 1) 点云障碍（过滤掉地面点，只保留离地一定高度的障碍）
        if self.cloud is not None and len(self.cloud) > 0:
            dz = self.cloud_z - self.robot_z
            ddx = self.cloud[:, 0] - self.robot_x
            ddy = self.cloud[:, 1] - self.robot_y
            rr = np.hypot(ddx, ddy)
            mask = (dz >= self.z_min) & (dz <= self.z_max)
            mask &= (rr >= 0.15) & (rr <= self.obs_range + 0.5)
            idx = np.where(mask)[0]
            step = max(1, len(idx) // 400)
            for i in idx[::step]:
                add_xy(float(self.cloud[i, 0]), float(self.cloud[i, 1]))

        # 2) 导航边界作为虚拟墙
        for (p1, p2) in self.boundary_edges:
            x1, y1 = p1
            x2, y2 = p2
            d1 = math.hypot(x1 - self.robot_x, y1 - self.robot_y)
            d2 = math.hypot(x2 - self.robot_x, y2 - self.robot_y)
            if d1 > self.bound_range and d2 > self.bound_range:
                continue
            seg_len = math.hypot(x2 - x1, y2 - y1)
            if seg_len < 1e-3:
                add_xy(x1, y1)
                continue
            n = max(1, int(seg_len / self.bound_step))
            for k in range(n + 1):
                t = k / n
                add_xy(x1 + (x2 - x1) * t, y1 + (y2 - y1) * t)

        return obs

    def _pick_heading(self, obs_dist, goal_rel):
        """选择目标方向附近、宽度足够的自由扇区。返回机器人系相对航向角。"""
        free = obs_dist >= self.safe_dist
        n = self.n_bins

        if free.all():
            return goal_rel

        # 目标方向本身自由 -> 直接朝目标
        goal_bin = int(((goal_rel + math.pi) / (2.0 * math.pi) * n)) % n
        if free[goal_bin]:
            return goal_rel

        # 找所有极大自由扇区，选择"离目标最近的可达方向"
        starts = [i for i in range(n) if free[i] and not free[(i - 1) % n]]
        best_heading = None
        best_err = float("inf")
        for s in starts:
            j = s
            length = 0
            while free[j % n] and length < n:
                j += 1
                length += 1
            start_ang = self._bin_to_ang(s)
            arc_len = length * self.bin_rad
            cand = self._clamp_angle_to_sector(goal_rel, start_ang, arc_len)
            err = abs(wrap_pi(cand - goal_rel))
            if err < best_err:
                best_err = err
                best_heading = cand

        if best_heading is None:
            b = int(np.argmax(obs_dist))
            return b * self.bin_rad - math.pi + self.bin_rad / 2.0

        return best_heading

    def _clearance_around(self, obs_dist, heading, half_bins=3):
        n = self.n_bins
        b = int(((heading + math.pi) / (2.0 * math.pi) * n)) % n
        vals = [obs_dist[(b + k) % n] for k in range(-half_bins, half_bins + 1)]
        return float(min(vals))

    def _bin_to_ang(self, b):
        return b * self.bin_rad - math.pi

    def _clamp_angle_to_sector(self, ang, start_ang, arc_len):
        """把 ang 拉进 [start_ang, start_ang+arc_len] 的圆弧内（取圆弧上最近点）。"""
        two_pi = 2.0 * math.pi
        rel = (ang - start_ang) % two_pi  # [0, 2pi)
        rel = min(max(rel, 0.0), arc_len)
        return wrap_pi(start_ang + rel)

    def pub(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = FarLocalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()