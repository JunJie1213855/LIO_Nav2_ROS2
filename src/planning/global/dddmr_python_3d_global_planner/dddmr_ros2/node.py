"""PointCloud2 -> DDDMR -> nav_msgs/Path, with serialized background jobs."""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import fields

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from dddmr_py import GlobalPlanner3D, PlannerConfig
from .conversions import cloud_xyz, path_message, transform_xyz


class PlannerNode(Node):
    def __init__(self, **kwargs):
        super().__init__('dddmr_planner', **kwargs)
        defaults = {
            'map_topic': '/global_points', 'path_topic': '/init_path',
            'goal_topic': '/goal_pose', 'initial_pose_topic': '/initialpose',
            'odom_topic': '', 'start': [0.0, 0.0, 0.0],
            'update_map': False, 'map_transient_local': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        cfg = PlannerConfig()
        for field in fields(cfg):
            self.declare_parameter('planner.' + field.name, getattr(cfg, field.name))
        self.config = PlannerConfig.from_dict({
            f.name: self.get_parameter('planner.' + f.name).value for f in fields(cfg)})
        self._start = np.asarray(self.get_parameter('start').value, dtype=float)
        if self._start.shape != (3,) or not np.isfinite(self._start).all():
            raise ValueError('start must be three finite coordinates [x, y, z]')
        self._start_msg = None
        self._goal = None
        self._planner = None
        self._frame = ''
        self._map_accepted = False
        self._pending_map = None
        self._revision = 0
        self._needs_plan = False
        self._future = None
        self._job = None
        self._worker = ThreadPoolExecutor(max_workers=1)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        path_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._path_pub = self.create_publisher(Path, self.param('path_topic'), path_qos)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            durability=(DurabilityPolicy.TRANSIENT_LOCAL
                                        if self.param('map_transient_local')
                                        else DurabilityPolicy.VOLATILE))
        self._map_sub = self.create_subscription(
            PointCloud2, self.param('map_topic'), self._on_map, map_qos)
        self._goal_sub = self.create_subscription(
            PoseStamped, self.param('goal_topic'), self._on_goal, 10)
        self._start_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.param('initial_pose_topic'), self._on_start, 10)
        self._odom_sub = None
        if self.param('odom_topic'):
            self._odom_sub = self.create_subscription(
                Odometry, self.param('odom_topic'), self._on_odom,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"Waiting for PointCloud2 on {self.param('map_topic')}; "
            f"goals: {self.param('goal_topic')}; path: {self.param('path_topic')}")

    def param(self, name):
        return self.get_parameter(name).value

    def _clear_path(self):
        self._path_pub.publish(path_message(None, self._frame, self.get_clock().now().to_msg()))

    def _on_map(self, msg):
        if self._map_accepted and not self.param('update_map'):
            return
        if not msg.header.frame_id:
            self.get_logger().error('Map frame_id is empty; ignoring map')
            return
        self._pending_map = msg  # Keep only the newest map while a worker is busy.
        self._map_accepted = True
        self._frame = msg.header.frame_id
        self._planner = None
        self._revision += 1
        self._needs_plan = self._goal is not None
        self._clear_path()

    def _on_goal(self, msg):
        self._goal = msg
        self._revision += 1
        self._needs_plan = True
        self._clear_path()
        if self._planner is None:
            self.get_logger().info('Goal queued; waiting for map preprocessing')

    def _on_start(self, msg):
        pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
        self._start_msg = pose
        self._revision += 1
        self._needs_plan = self._goal is not None
        self._clear_path()

    def _on_odom(self, msg):
        # Use the newest odometry at the next goal; do not replan at odometry rate.
        self._start_msg = PoseStamped(header=msg.header, pose=msg.pose.pose)

    def _position(self, msg):
        if not msg.header.frame_id:
            raise ValueError('Start/goal frame_id is empty')
        p = msg.pose.position
        if msg.header.frame_id == self._frame:
            point = np.array([p.x, p.y, p.z], dtype=float)
        else:
            # Latest TF avoids requiring historical transforms for an old RViz click.
            tf = self._tf.lookup_transform(self._frame, msg.header.frame_id, Time())
            point = transform_xyz(p, tf.transform)
        if not np.isfinite(point).all():
            raise ValueError('Start/goal contains non-finite coordinates')
        return point

    def _build(self, msg):
        return GlobalPlanner3D(cloud_xyz(msg), self.config, verbose=True)

    def _tick(self):
        if self._future is not None:
            if not self._future.done():
                return
            future, (kind, revision) = self._future, self._job
            self._future = None
            try:
                result = future.result()
                if kind == 'map':
                    # Goals may change during building; only a newer map supersedes it.
                    if self._pending_map is None:
                        self._planner = result
                        self.get_logger().info(
                            f'Map ready ({len(result.ground.nodes)} nodes), frame={self._frame}')
                elif revision == self._revision:
                    self._path_pub.publish(path_message(
                        result, self._frame, self.get_clock().now().to_msg()))
                    if result.success:
                        self.get_logger().info(
                            f'Published {len(result.points)} poses, length={result.length:.2f} m')
                    else:
                        self.get_logger().warn(f'Planning failed: {result.message}')
            except Exception as exc:
                self.get_logger().error(f'{kind} failed: {exc}')
                if kind == 'map' and self._pending_map is None:
                    self._map_accepted = False  # Allow a corrected map to be received.
                if revision == self._revision:
                    self._clear_path()
        if self._pending_map is not None:
            msg, self._pending_map = self._pending_map, None
            self._job = ('map', self._revision)
            self._future = self._worker.submit(self._build, msg)
            self.get_logger().info(f'Building map ({msg.width * msg.height} points)…')
        elif self._needs_plan and self._planner is not None:
            self._needs_plan = False
            try:
                start = self._position(self._start_msg) if self._start_msg else self._start.copy()
                goal = self._position(self._goal)
                self._job = ('plan', self._revision)
                self._future = self._worker.submit(self._planner.make_plan, start, goal)
            except Exception as exc:
                self._clear_path()
                self.get_logger().error(f'Cannot resolve start/goal; resend goal after TF is ready: {exc}')

    def destroy_node(self):
        self._worker.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlannerNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
