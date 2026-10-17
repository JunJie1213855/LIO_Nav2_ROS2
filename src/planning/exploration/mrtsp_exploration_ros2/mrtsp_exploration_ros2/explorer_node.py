"""
Copyright 2026 Mert Güler

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

#!/usr/bin/env python3

import math
import time
import traceback
from enum import Enum
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray

from .frontier_costs import CostWeights, RobotState, build_cost_matrix
from .frontier_model import Frontier
from .map_optimizer import optimize_map
from .mrtsp_solver import greedy_mrtsp_order
from .occupancy_grid import PAPER_FREE, OccupancyGridAdapter, paper_image_to_occupancy_grid
from .visualization import build_frontier_markers, build_selected_frontier_pose
from .wfd_frontier import extract_frontiers


class GoalLifecycleState(Enum):
    IDLE = "idle"
    SENDING = "sending"
    ACTIVE = "active"
    SUPERSEDED = "superseded"
    CANCELING = "canceling"


class MRTSPExplorerNode(Node):
    def __init__(self) -> None:
        super().__init__("mrtsp_explorer")

        self.map_topic = self.declare_parameter("map_topic", "/map").value
        self.costmap_topic = self.declare_parameter(
            "costmap_topic", "/global_costmap/costmap"
        ).value
        self.local_costmap_topic = self.declare_parameter(
            "local_costmap_topic", "/local_costmap/costmap"
        ).value
        self.navigate_to_pose_action_name = self.declare_parameter(
            "navigate_to_pose_action_name", "navigate_to_pose"
        ).value
        self.global_frame = self.declare_parameter("global_frame", "map").value
        self.robot_base_frame = self.declare_parameter(
            "robot_base_frame", "base_footprint"
        ).value
        self.sigma_s = float(self.declare_parameter("sigma_s", 2.0).value)
        self.sigma_r = float(self.declare_parameter("sigma_r", 30.0).value)
        self.dilation_kernel_radius_cells = int(
            self.declare_parameter("dilation_kernel_radius_cells", 1).value
        )
        self.sensor_effective_range_m = float(
            self.declare_parameter("sensor_effective_range_m", 1.5).value
        )
        self.weight_distance_wd = float(
            self.declare_parameter("weight_distance_wd", 1.0).value
        )
        self.weight_gain_ws = float(
            self.declare_parameter("weight_gain_ws", 1.0).value
        )
        self.max_linear_speed_vmax = float(
            self.declare_parameter("max_linear_speed_vmax", 0.5).value
        )
        self.max_angular_speed_wmax = float(
            self.declare_parameter("max_angular_speed_wmax", 1.0).value
        )
        self.occ_threshold = int(self.declare_parameter("occ_threshold", 50).value)
        self.min_frontier_size_cells = int(
            self.declare_parameter("min_frontier_size_cells", 5).value
        )
        self.publish_debug_topics = bool(
            self.declare_parameter("publish_debug_topics", True).value
        )
        self.planning_rate_hz = float(
            self.declare_parameter("planning_rate_hz", 1.0).value
        )
        self.frontier_marker_topic = self.declare_parameter(
            "frontier_marker_topic", "/explore/frontiers"
        ).value
        self.selected_frontier_topic = self.declare_parameter(
            "selected_frontier_topic", "/explore/selected_frontier"
        ).value
        self.optimized_map_topic = self.declare_parameter(
            "optimized_map_topic", "/explore/optimized_map"
        ).value
        self.marker_scale = float(self.declare_parameter("marker_scale", 0.15).value)
        self.frontier_min_goal_distance_m = float(
            self.declare_parameter("frontier_min_goal_distance_m", 0.0).value
        )
        self.goal_preemption_on_frontier_opened = bool(
            self.declare_parameter(
                "goal_preemption_on_frontier_opened", False
            ).value
        )
        self.goal_preemption_on_blocked_goal = bool(
            self.declare_parameter(
                "goal_preemption_on_blocked_goal", False
            ).value
        )
        self.goal_preemption_min_interval_s = float(
            self.declare_parameter("goal_preemption_min_interval_s", 2.0).value
        )
        self.goal_preemption_skip_if_within_m = float(
            self.declare_parameter("goal_preemption_skip_if_within_m", 0.75).value
        )
        self.return_to_start_on_complete = bool(
            self.declare_parameter("return_to_start_on_complete", False).value
        )

        self.map_msg: Optional[OccupancyGrid] = None
        self.costmap_msg: Optional[OccupancyGrid] = None
        self.local_costmap_msg: Optional[OccupancyGrid] = None
        self.goal_handle = None
        self.goal_in_progress = False
        self.goal_state = GoalLifecycleState.IDLE
        self.current_dispatch_id = 0
        self.dispatch_states = {}
        self.active_goal_kind: Optional[str] = None
        self.active_target = None
        self.active_frontier: Optional[Frontier] = None
        self.active_goal_sent_time = None
        self.cancel_request_in_progress = False
        self.pending_cancel_reason: Optional[str] = None
        self.expected_goal_cancel = False
        self.replacement_candidate_frontier: Optional[Frontier] = None
        self.replacement_candidate_hits = 0
        self.replacement_required_hits = 2
        self.start_pose: Optional[PoseStamped] = None
        self.exploration_complete = False
        self.return_to_start_started = False
        self.return_to_start_completed = False
        self.goal_preemption_enabled = (
            self.goal_preemption_on_frontier_opened
            or self.goal_preemption_on_blocked_goal
        )
        self._last_log_times = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_to_pose_action_name,
        )

        self.frontier_pub = self.create_publisher(
            MarkerArray, self.frontier_marker_topic, 10
        )
        self.selected_frontier_pub = self.create_publisher(
            PoseStamped, self.selected_frontier_topic, 10
        )
        self.optimized_map_pub = self.create_publisher(
            OccupancyGrid, self.optimized_map_topic, 10
        )

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, transient_qos
        )
        self.create_subscription(
            OccupancyGrid, self.map_topic, self._map_callback, volatile_qos
        )
        self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._costmap_callback, volatile_qos
        )
        if self.goal_preemption_on_blocked_goal and self.local_costmap_topic:
            self.create_subscription(
                OccupancyGrid,
                self.local_costmap_topic,
                self._local_costmap_callback,
                volatile_qos,
            )

        self.timer = self.create_timer(
            1.0 / max(self.planning_rate_hz, 1.0),
            self._on_timer,
        )

        self.get_logger().info(
            "Starting MRTSP frontier explorer: map='%s', costmap='%s', local_costmap='%s', "
            "action='%s', sigma_s=%.2f, sigma_r=%.2f, sensor_range=%.2f, wd=%.2f, ws=%.2f, "
            "min_goal_distance=%.2f, frontier_open_preemption=%s, blocked_goal_preemption=%s, "
            "return_to_start_on_complete=%s"
            % (
                self.map_topic,
                self.costmap_topic,
                self.local_costmap_topic,
                self.navigate_to_pose_action_name,
                self.sigma_s,
                self.sigma_r,
                self.sensor_effective_range_m,
                self.weight_distance_wd,
                self.weight_gain_ws,
                self.frontier_min_goal_distance_m,
                self.goal_preemption_on_frontier_opened,
                self.goal_preemption_on_blocked_goal,
                self.return_to_start_on_complete,
            )
        )

    def _log_throttled(self, key: str, level: str, message: str, period: float = 3.0) -> None:
        now = time.monotonic()
        last = self._last_log_times.get(key, 0.0)
        if now - last < period:
            return
        if level == "warn":
            self.get_logger().warn(message)
        elif level == "error":
            self.get_logger().error(message)
        elif level == "debug":
            self.get_logger().debug(message)
        else:
            self.get_logger().info(message)
        self._last_log_times[key] = now

    def _map_callback(self, msg: OccupancyGrid) -> None:
        first_map = self.map_msg is None
        self.map_msg = msg
        if first_map:
            self.get_logger().info(
                "Received first map: frame='%s', size=%dx%d, resolution=%.3f"
                % (
                    msg.header.frame_id,
                    msg.info.width,
                    msg.info.height,
                    msg.info.resolution,
                )
            )

    def _costmap_callback(self, msg: OccupancyGrid) -> None:
        first_costmap = self.costmap_msg is None
        self.costmap_msg = msg
        if first_costmap:
            self.get_logger().info(
                "Received first costmap: frame='%s', size=%dx%d, resolution=%.3f"
                % (
                    msg.header.frame_id,
                    msg.info.width,
                    msg.info.height,
                    msg.info.resolution,
                )
            )

    def _local_costmap_callback(self, msg: OccupancyGrid) -> None:
        first_local_costmap = self.local_costmap_msg is None
        self.local_costmap_msg = msg
        if first_local_costmap:
            self.get_logger().info(
                "Received first local costmap: frame='%s', size=%dx%d, resolution=%.3f"
                % (
                    msg.header.frame_id,
                    msg.info.width,
                    msg.info.height,
                    msg.info.resolution,
                )
            )

    def _lookup_robot_state(self) -> Optional[RobotState]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self._log_throttled(
                "wait_tf",
                "warn",
                "Waiting for TF %s -> %s: %s"
                % (self.global_frame, self.robot_base_frame, exc),
            )
            return None

        q = transform.transform.rotation
        yaw = self._yaw_from_quaternion(q)
        return RobotState(
            position=np.array(
                [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                ],
                dtype=float,
            ),
            yaw=yaw,
        )

    @staticmethod
    def _yaw_from_quaternion(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )

    def _transform_xy_point(
        self,
        point: Sequence[float],
        source_frame: str,
        target_frame: str,
    ) -> Optional[np.ndarray]:
        if not source_frame or not target_frame or source_frame == target_frame:
            return np.asarray(point, dtype=float)

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException as exc:
            self._log_throttled(
                f"wait_tf_{source_frame}_to_{target_frame}",
                "debug",
                "Waiting for TF %s -> %s while validating active frontier: %s"
                % (source_frame, target_frame, exc),
                period=2.0,
            )
            return None

        yaw = self._yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x = float(point[0])
        y = float(point[1])
        return np.array(
            [
                cos_yaw * x - sin_yaw * y + transform.transform.translation.x,
                sin_yaw * x + cos_yaw * y + transform.transform.translation.y,
            ],
            dtype=float,
        )

    def _cost_at_world_point(
        self,
        costmap_msg: Optional[OccupancyGrid],
        world_point: Sequence[float],
    ) -> Optional[int]:
        if costmap_msg is None:
            return None

        source_frame = (
            self.map_msg.header.frame_id if self.map_msg is not None else ""
        ) or self.global_frame
        target_frame = costmap_msg.header.frame_id or source_frame
        costmap_point = self._transform_xy_point(
            world_point,
            source_frame,
            target_frame,
        )
        if costmap_point is None:
            return None

        adapter = OccupancyGridAdapter(
            costmap_msg,
            occupied_threshold=self.occ_threshold,
        )
        cell = adapter.world_to_map(costmap_point[0], costmap_point[1])
        if cell is None:
            return None

        return adapter.get_cost(cell.x, cell.y)

    def _frontier_match_tolerance_m(self) -> float:
        map_resolution = (
            self.map_msg.info.resolution if self.map_msg is not None else 0.05
        )
        return max(0.25, 5.0 * map_resolution)

    def _frontiers_equivalent(
        self,
        first_frontier: Optional[Frontier],
        second_frontier: Optional[Frontier],
    ) -> bool:
        if first_frontier is None or second_frontier is None:
            return False

        tolerance = self._frontier_match_tolerance_m()
        centroid_distance = float(
            np.linalg.norm(first_frontier.centroid_array - second_frontier.centroid_array)
        )
        goal_distance = float(
            np.linalg.norm(first_frontier.center_array - second_frontier.center_array)
        )
        return centroid_distance <= tolerance and goal_distance <= tolerance

    def _frontier_exists_in_set(
        self,
        frontier: Optional[Frontier],
        frontiers: Sequence[Frontier],
    ) -> bool:
        if frontier is None:
            return False

        return any(
            self._frontiers_equivalent(frontier, candidate) for candidate in frontiers
        )

    def _active_goal_cost_status(self) -> Optional[str]:
        if self.active_frontier is None:
            return None

        goal_point = self.active_frontier.center_array
        local_cost = self._cost_at_world_point(self.local_costmap_msg, goal_point)
        if local_cost is not None and local_cost > self.occ_threshold:
            return (
                "Active frontier goal moved into a blocked local costmap cell "
                f"(cost={local_cost})"
            )

        global_cost = self._cost_at_world_point(self.costmap_msg, goal_point)
        if global_cost is not None and global_cost > self.occ_threshold:
            return (
                "Active frontier goal moved into a blocked global costmap cell "
                f"(cost={global_cost})"
            )

        return None

    def _request_active_goal_cancel(self, reason: str) -> None:
        if self.pending_cancel_reason is None:
            self.pending_cancel_reason = reason

        if self.goal_handle is None or self.cancel_request_in_progress:
            return

        cancel_reason = self.pending_cancel_reason or "Canceling active frontier goal"
        self.pending_cancel_reason = None
        self.cancel_request_in_progress = True
        self._set_goal_state(GoalLifecycleState.CANCELING)
        self.get_logger().info(cancel_reason)
        dispatch_id = self.current_dispatch_id
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda future, dispatch_id=dispatch_id: self._cancel_response_callback(
                future, dispatch_id
            )
        )

    def _cancel_response_callback(self, future, dispatch_id: int) -> None:
        if dispatch_id != self.current_dispatch_id:
            return

        try:
            cancel_response = future.result()
        except Exception:
            self.cancel_request_in_progress = False
            self._set_goal_state(GoalLifecycleState.ACTIVE)
            self.get_logger().warn(
                "Failed to cancel active frontier goal:\n%s"
                % traceback.format_exc()
            )
            return

        if not cancel_response.goals_canceling:
            self.cancel_request_in_progress = False
            self._set_goal_state(GoalLifecycleState.ACTIVE)
            self.get_logger().warn(
                "Active frontier goal cancel request was not accepted"
            )
            return

        self.expected_goal_cancel = True

    def _reachable_goal_point(
        self,
        frontier: Frontier,
        optimized_image: np.ndarray,
        map_adapter: OccupancyGridAdapter,
        robot_state: RobotState,
    ) -> np.ndarray:
        """Pick a free cell adjacent to the frontier as the navigation goal.

        The frontier center usually lies inside unknown space. Dispatching it directly
        makes Nav2 "reach" it within its goal tolerance at the frontier edge - the robot
        never enters sensor range of the frontier, so it never opens, and the same goal
        gets re-dispatched (instant SUCCEEDED) forever. Dispatching a free cell next to
        the frontier gives Nav2 a genuinely reachable goal from which the sensor covers
        the frontier cells. The free cell nearest the frontier center is chosen so the
        robot approaches the middle of the frontier; the caller gates this point by its
        distance to the robot.
        """
        best_world: Optional[np.ndarray] = None
        best_dist = float("inf")
        center = np.asarray(frontier.center_array, dtype=float)
        for cell in frontier.cells:
            x, y = cell
            for nx in range(x - 1, x + 2):
                for ny in range(y - 1, y + 2):
                    if not map_adapter.in_bounds(nx, ny):
                        continue
                    if optimized_image[ny, nx] != PAPER_FREE:
                        continue
                    world = np.asarray(
                        map_adapter.map_to_world(nx, ny),
                        dtype=float,
                    )
                    dist = float(np.linalg.norm(world - center))
                    if dist < best_dist:
                        best_dist = dist
                        best_world = world
        return best_world if best_world is not None else frontier.center_array

    def _build_goal_pose(
        self,
        robot_state: RobotState,
        target_point: np.ndarray,
    ) -> PoseStamped:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(target_point[0])
        goal_pose.pose.position.y = float(target_point[1])
        goal_pose.pose.position.z = 0.0

        goal_yaw = math.atan2(
            target_point[1] - robot_state.position[1],
            target_point[0] - robot_state.position[0],
        )
        goal_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
        goal_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
        return goal_pose

    def _record_start_pose(self, robot_state: RobotState) -> None:
        if self.start_pose is not None:
            return

        start_pose = PoseStamped()
        start_pose.header.frame_id = self.global_frame
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = float(robot_state.position[0])
        start_pose.pose.position.y = float(robot_state.position[1])
        start_pose.pose.position.z = 0.0
        start_pose.pose.orientation.w = math.cos(robot_state.yaw / 2.0)
        start_pose.pose.orientation.z = math.sin(robot_state.yaw / 2.0)
        self.start_pose = start_pose
        self.get_logger().info(
            "Recorded exploration start pose: (%.2f, %.2f)"
            % (
                start_pose.pose.position.x,
                start_pose.pose.position.y,
            )
        )

    @staticmethod
    def _is_within_xy_tolerance(
        first_point: Sequence[float],
        second_point: Sequence[float],
        tolerance: float = 0.25,
    ) -> bool:
        return float(
            np.linalg.norm(
                np.asarray(first_point, dtype=float) - np.asarray(second_point, dtype=float)
            )
        ) <= tolerance

    def _build_return_to_start_pose(self) -> Optional[PoseStamped]:
        if self.start_pose is None:
            return None

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = self.start_pose.pose
        return goal_pose

    def _send_goal(
        self,
        pose: PoseStamped,
        frontier: Optional[Frontier],
        goal_kind: str,
        dispatch_context: str = "default",
    ) -> bool:
        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self._log_throttled(
                "wait_action",
                "warn",
                "NavigateToPose action '%s' is not available yet"
                % self.navigate_to_pose_action_name,
            )
            return False

        goal = NavigateToPose.Goal()
        goal.pose = pose

        if self.goal_in_progress and self.current_dispatch_id in self.dispatch_states:
            self._mark_dispatch_state(
                self.current_dispatch_id, GoalLifecycleState.SUPERSEDED
            )
        self.current_dispatch_id += 1
        dispatch_id = self.current_dispatch_id
        send_future = self.navigate_client.send_goal_async(goal)
        self._mark_dispatch_state(dispatch_id, GoalLifecycleState.SENDING)
        self._set_goal_state(GoalLifecycleState.SENDING)
        self.active_goal_kind = goal_kind
        self.active_target = np.array(
            [pose.pose.position.x, pose.pose.position.y],
            dtype=float,
        )
        self.active_frontier = frontier
        self.active_goal_sent_time = self.get_clock().now()
        send_future.add_done_callback(
            lambda future, dispatch_id=dispatch_id, goal_kind=goal_kind: self._goal_response_callback(
                future,
                dispatch_id,
                goal_kind,
            )
        )
        self.get_logger().debug(
            "Dispatched %s goal (%s) with dispatch_id=%d"
            % (goal_kind, dispatch_context, dispatch_id)
        )
        return True

    def _goal_response_callback(self, future, dispatch_id: int, goal_kind: str) -> None:
        try:
            goal_handle = future.result()
        except Exception:
            if dispatch_id != self.current_dispatch_id:
                return
            self._clear_active_goal_state()
            if goal_kind == "return_to_start":
                self.return_to_start_started = False
            self.get_logger().error(
                "Failed to send NavigateToPose goal:\n%s" % traceback.format_exc()
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            if dispatch_id != self.current_dispatch_id:
                return
            self._clear_active_goal_state()
            if goal_kind == "return_to_start":
                self.return_to_start_started = False
            self.get_logger().warn("NavigateToPose goal rejected")
            return

        if dispatch_id != self.current_dispatch_id:
            goal_handle.cancel_goal_async()
            return

        self.goal_handle = goal_handle
        self._mark_dispatch_state(dispatch_id, GoalLifecycleState.ACTIVE)
        self._set_goal_state(GoalLifecycleState.ACTIVE)
        self.get_logger().info(
            "%s goal accepted for (%.2f, %.2f)"
            % (
                "Return-to-start" if goal_kind == "return_to_start" else "NavigateToPose",
                self.active_target[0],
                self.active_target[1],
            )
        )
        self.goal_handle.get_result_async().add_done_callback(
            lambda future, dispatch_id=dispatch_id, goal_kind=goal_kind: self._goal_result_callback(
                future,
                dispatch_id,
                goal_kind,
            )
        )

    def _goal_result_callback(self, future, dispatch_id: int, goal_kind: str) -> None:
        expected_cancel = self.expected_goal_cancel if dispatch_id == self.current_dispatch_id else False

        try:
            result_packet = future.result()
        except Exception:
            if dispatch_id != self.current_dispatch_id:
                return
            self.get_logger().error(
                "Failed while waiting for NavigateToPose result:\n%s"
                % traceback.format_exc()
            )
            if goal_kind == "return_to_start":
                self.return_to_start_started = False
            self._clear_active_goal_state()
            return

        if dispatch_id != self.current_dispatch_id:
            return

        self.cancel_request_in_progress = False
        self.pending_cancel_reason = None
        self.expected_goal_cancel = False
        if result_packet is None:
            self.get_logger().warn("NavigateToPose returned no result payload")
            if goal_kind == "return_to_start":
                self.return_to_start_started = False
            self._clear_active_goal_state()
            return

        status_name = {
            GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "EXECUTING",
            GoalStatus.STATUS_CANCELING: "CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
        }.get(result_packet.status, str(result_packet.status))
        if result_packet.status == GoalStatus.STATUS_CANCELED and expected_cancel:
            self.get_logger().debug("Active frontier goal canceled for replanning")
        elif (
            result_packet.status == GoalStatus.STATUS_SUCCEEDED
            and goal_kind == "return_to_start"
        ):
            self.return_to_start_started = False
            self.return_to_start_completed = True
            self.get_logger().info("Returned to start pose. Exploration complete.")
        else:
            if goal_kind == "return_to_start":
                self.return_to_start_started = False
            # Humble's NavigateToPose_Result has no error_code/error_msg
            # (they were added in Iron/Jazzy); use getattr so both work.
            result = result_packet.result
            self.get_logger().info(
                "NavigateToPose finished with status=%s, error_code=%s, error_msg='%s'"
                % (
                    status_name,
                    getattr(result, "error_code", "n/a"),
                    getattr(result, "error_msg", "n/a"),
                )
            )
        self._clear_active_goal_state()

    def _set_goal_state(self, state: GoalLifecycleState) -> None:
        self.goal_state = state
        self.goal_in_progress = state != GoalLifecycleState.IDLE

    def _mark_dispatch_state(self, dispatch_id: int, state: GoalLifecycleState) -> None:
        self.dispatch_states[dispatch_id] = state

    def _clear_active_goal_state(self) -> None:
        self._set_goal_state(GoalLifecycleState.IDLE)
        self.goal_handle = None
        self.active_goal_kind = None
        self.active_target = None
        self.active_frontier = None
        self.active_goal_sent_time = None
        self.dispatch_states = {}
        self._reset_replacement_candidate_tracking()

    def _reset_replacement_candidate_tracking(self) -> None:
        self.replacement_candidate_frontier = None
        self.replacement_candidate_hits = 0

    def _has_stable_replacement_candidate(self, candidate: Optional[Frontier]) -> bool:
        if candidate is None:
            self._reset_replacement_candidate_tracking()
            return False

        if (
            self.replacement_candidate_frontier is not None
            and self._frontiers_equivalent(
                candidate, self.replacement_candidate_frontier
            )
        ):
            self.replacement_candidate_hits += 1
        else:
            self.replacement_candidate_frontier = candidate
            self.replacement_candidate_hits = 1

        return self.replacement_candidate_hits >= self.replacement_required_hits

    def _handle_exploration_complete(
        self,
        robot_state: RobotState,
        optimized_map_msg: OccupancyGrid,
    ) -> None:
        self._publish_debug_outputs([], optimized_map_msg, None)

        if not self.exploration_complete:
            self.exploration_complete = True
            self.get_logger().info(
                "Exploration complete: no frontiers remain on the optimized map"
            )

        if not self.return_to_start_on_complete or self.return_to_start_completed:
            return

        if self.start_pose is None:
            self.return_to_start_completed = True
            self.get_logger().warn(
                "Exploration completed but no start pose was recorded for return-to-start"
            )
            return

        if self.goal_in_progress or self.return_to_start_started:
            return

        start_point = np.array(
            [
                self.start_pose.pose.position.x,
                self.start_pose.pose.position.y,
            ],
            dtype=float,
        )
        if self._is_within_xy_tolerance(robot_state.position, start_point):
            self.return_to_start_completed = True
            self.get_logger().info("Exploration finished at the start pose")
            return

        goal_pose = self._build_return_to_start_pose()
        if goal_pose is None:
            return

        if self._send_goal(goal_pose, frontier=None, goal_kind="return_to_start"):
            self.return_to_start_started = True
            self.get_logger().info(
                "Exploration complete, returning to start pose: (%.2f, %.2f)"
                % (
                    goal_pose.pose.position.x,
                    goal_pose.pose.position.y,
                )
            )

    def _compute_frontier_context(
        self,
        robot_state: RobotState,
    ) -> Tuple[OccupancyGrid, List[Frontier], OccupancyGridAdapter]:
        map_adapter = OccupancyGridAdapter(
            self.map_msg,
            occupied_threshold=self.occ_threshold,
        )
        costmap_adapter = OccupancyGridAdapter(
            self.costmap_msg,
            occupied_threshold=self.occ_threshold,
        )

        optimization_result = optimize_map(
            map_adapter,
            sigma_s=self.sigma_s,
            sigma_r=self.sigma_r,
            dilation_kernel_radius_cells=self.dilation_kernel_radius_cells,
        )
        optimized_map_msg = paper_image_to_occupancy_grid(
            optimization_result.optimized_image,
            self.map_msg,
        )
        optimized_map_msg.header.stamp = self.get_clock().now().to_msg()

        frontiers = extract_frontiers(
            optimization_result.optimized_image,
            map_adapter,
            costmap_adapter,
            robot_world_position=robot_state.position,
            min_frontier_size_cells=self.min_frontier_size_cells,
            occ_threshold=self.occ_threshold,
        )
        return (
            optimized_map_msg,
            frontiers,
            map_adapter,
            optimization_result.optimized_image,
        )

    def _filter_frontiers_by_min_goal_distance(
        self,
        frontiers: Sequence[Frontier],
        robot_state: RobotState,
    ) -> List[Frontier]:
        if self.frontier_min_goal_distance_m <= 0.0:
            return list(frontiers)

        filtered_frontiers = [
            frontier
            for frontier in frontiers
            if np.linalg.norm(frontier.center_array - robot_state.position)
            >= self.frontier_min_goal_distance_m
        ]
        skipped_count = len(frontiers) - len(filtered_frontiers)
        if skipped_count > 0:
            self._log_throttled(
                "close_frontiers",
                "info",
                "Skipping %d frontiers closer than %.2f m to the robot"
                % (
                    skipped_count,
                    self.frontier_min_goal_distance_m,
                ),
                period=2.0,
            )
        if not filtered_frontiers and frontiers:
            # All frontiers are within the min distance (robot surrounded). Fall back
            # to the nearest one so exploration never freezes; it will open as the
            # robot drives to it.
            nearest = min(
                frontiers,
                key=lambda f: np.linalg.norm(
                    f.center_array - robot_state.position
                ),
            )
            self._log_throttled(
                "all_frontiers_too_close",
                "warn",
                "All %d frontiers within %.2f m; falling back to nearest"
                % (len(frontiers), self.frontier_min_goal_distance_m),
                period=2.0,
            )
            return [nearest]
        return filtered_frontiers

    def _compute_frontier_order(
        self,
        frontiers: Sequence[Frontier],
        map_adapter: OccupancyGridAdapter,
        robot_state: RobotState,
    ) -> List[int]:
        frontier_start_world_points: List[np.ndarray] = [
            np.array(
                map_adapter.map_to_world(
                    frontier.start_cell[0],
                    frontier.start_cell[1],
                ),
                dtype=float,
            )
            for frontier in frontiers
        ]
        cost_matrix = build_cost_matrix(
            frontiers=frontiers,
            frontier_start_world_points=frontier_start_world_points,
            robot_state=robot_state,
            weights=CostWeights(
                distance_wd=self.weight_distance_wd,
                gain_ws=self.weight_gain_ws,
            ),
            sensor_effective_range_m=self.sensor_effective_range_m,
            max_linear_speed_vmax=self.max_linear_speed_vmax,
            max_angular_speed_wmax=self.max_angular_speed_wmax,
        )
        return greedy_mrtsp_order(cost_matrix)

    def _consider_preempt_active_goal(self, robot_state: RobotState) -> None:
        if (
            not self.goal_preemption_enabled
            or not self.goal_in_progress
            or self.goal_handle is None
            or self.active_frontier is None
            or self.cancel_request_in_progress
        ):
            return

        active_goal_cost_status = (
            self._active_goal_cost_status()
            if self.goal_preemption_on_blocked_goal
            else None
        )
        if active_goal_cost_status is None and not self.goal_preemption_on_frontier_opened:
            return

        if self.active_goal_sent_time is None and active_goal_cost_status is None:
            return

        elapsed = (
            (self.get_clock().now() - self.active_goal_sent_time).nanoseconds / 1e9
            if self.active_goal_sent_time is not None
            else 0.0
        )
        if (
            active_goal_cost_status is None
            and self.goal_preemption_on_frontier_opened
            and elapsed < self.goal_preemption_min_interval_s
        ):
            return

        active_goal_distance = float(
            np.linalg.norm(self.active_frontier.center_array - robot_state.position)
        )
        if (
            active_goal_cost_status is None
            and self.goal_preemption_on_frontier_opened
            and active_goal_distance <= self.goal_preemption_skip_if_within_m
        ):
            return

        (
            optimized_map_msg,
            raw_frontiers,
            map_adapter,
            optimized_image,
        ) = self._compute_frontier_context(robot_state)
        active_frontier_exists = (
            self._frontier_exists_in_set(
                self.active_frontier,
                raw_frontiers,
            )
            if self.goal_preemption_on_frontier_opened
            else True
        )
        frontiers = self._filter_frontiers_by_min_goal_distance(
            raw_frontiers,
            robot_state,
        )

        replacement_frontier = None
        if frontiers:
            order = self._compute_frontier_order(frontiers, map_adapter, robot_state)
            if order:
                replacement_frontier = frontiers[order[0]]

        if active_goal_cost_status is None and active_frontier_exists:
            self._reset_replacement_candidate_tracking()
            selected_pose = build_selected_frontier_pose(
                self.active_frontier,
                frame_id=self.global_frame,
                stamp=self.get_clock().now().to_msg(),
            )
            self._publish_debug_outputs(
                raw_frontiers,
                optimized_map_msg,
                selected_pose,
            )
            return

        selected_pose = None
        if replacement_frontier is not None:
            selected_pose = build_selected_frontier_pose(
                replacement_frontier,
                frame_id=self.global_frame,
                stamp=self.get_clock().now().to_msg(),
            )
        self._publish_debug_outputs(raw_frontiers, optimized_map_msg, selected_pose)

        reselection_reason = active_goal_cost_status
        if reselection_reason is None:
            reselection_reason = (
                "The active frontier opened while navigating, "
                "so replanning to the next available frontier"
            )
        if replacement_frontier is not None:
            if not self._has_stable_replacement_candidate(replacement_frontier):
                return
            replacement_pose = self._build_goal_pose(
                robot_state, replacement_frontier.center_array
            )
            if self._send_goal(
                replacement_pose,
                replacement_frontier,
                goal_kind="frontier",
                dispatch_context="replacement",
            ):
                self.get_logger().info(
                    "Replacing active frontier goal without explicit cancel: %s"
                    % reselection_reason
                )
            return

        self._reset_replacement_candidate_tracking()
        reselection_reason += "; no replacement frontier is available"
        self._request_active_goal_cancel(reselection_reason)

    def _publish_debug_outputs(
        self,
        frontiers,
        optimized_map_msg: OccupancyGrid,
        selected_pose: Optional[PoseStamped],
    ) -> None:
        if not self.publish_debug_topics:
            return

        stamp = self.get_clock().now().to_msg()
        self.frontier_pub.publish(
            build_frontier_markers(
                frontiers,
                frame_id=self.global_frame,
                stamp=stamp,
                marker_scale=self.marker_scale,
            )
        )
        self.optimized_map_pub.publish(optimized_map_msg)
        if selected_pose is not None:
            self.selected_frontier_pub.publish(selected_pose)

    def _on_timer(self) -> None:
        if self.map_msg is None or self.costmap_msg is None:
            if self.map_msg is None:
                self._log_throttled(
                    "wait_map",
                    "warn",
                    "Waiting for map messages on '%s'" % self.map_topic,
                )
            if self.costmap_msg is None:
                self._log_throttled(
                    "wait_costmap",
                    "warn",
                    "Waiting for costmap messages on '%s'" % self.costmap_topic,
                )
            return

        robot_state = self._lookup_robot_state()
        if robot_state is None:
            return
        self._record_start_pose(robot_state)

        try:
            if self.exploration_complete:
                if self.return_to_start_on_complete and not self.return_to_start_completed:
                    self._handle_exploration_complete(
                        robot_state,
                        self.map_msg,
                    )
                return

            if self.goal_in_progress:
                self._consider_preempt_active_goal(robot_state)
                return

            (
                optimized_map_msg,
                frontiers,
                map_adapter,
                optimized_image,
            ) = self._compute_frontier_context(robot_state)
            if not frontiers:
                self._handle_exploration_complete(robot_state, optimized_map_msg)
                return

            frontiers = self._filter_frontiers_by_min_goal_distance(
                frontiers, robot_state
            )
            if self.frontier_min_goal_distance_m > 0.0:
                if not frontiers:
                    self._publish_debug_outputs(frontiers, optimized_map_msg, None)
                    self._log_throttled(
                        "all_frontiers_too_close",
                        "warn",
                        "All detected frontiers are within %.2f m; waiting for map growth"
                        % self.frontier_min_goal_distance_m,
                        period=2.0,
                    )
                    return

            order = self._compute_frontier_order(frontiers, map_adapter, robot_state)
            if not order:
                self._publish_debug_outputs(frontiers, optimized_map_msg, None)
                self._log_throttled(
                    "no_valid_order",
                    "warn",
                    "Frontier set produced no valid MRTSP ordering",
                    period=2.0,
                )
                return

            # Pick the first frontier (in MRTSP order) whose *reachable goal point* is
            # at least `frontier_min_goal_distance_m` from the robot. The center may pass
            # the min-distance filter while the reachable goal (a free neighbor of the
            # frontier) sits within Nav2's xy goal tolerance of the robot - dispatching it
            # yields an instant SUCCEEDED with no movement, the frontier never opens, and
            # the same goal is re-picked forever. Gating on the actual goal point breaks
            # that loop.
            selected_frontier = None
            selected_goal = None
            for idx in order:
                candidate = frontiers[idx]
                candidate_goal = self._reachable_goal_point(
                    candidate,
                    optimized_image,
                    map_adapter,
                    robot_state,
                )
                if self.frontier_min_goal_distance_m <= 0.0 or np.linalg.norm(
                    candidate_goal - robot_state.position
                ) >= self.frontier_min_goal_distance_m:
                    selected_frontier = candidate
                    selected_goal = candidate_goal
                    break
            if selected_frontier is None:
                # Every reachable goal is within min distance (robot surrounded). Fall
                # back to the best candidate so exploration never freezes.
                selected_frontier = frontiers[order[0]]
                selected_goal = self._reachable_goal_point(
                    selected_frontier,
                    optimized_image,
                    map_adapter,
                    robot_state,
                )
                self._log_throttled(
                    "all_goals_too_close",
                    "warn",
                    "All frontier goals within %.2f m; falling back to nearest"
                    % self.frontier_min_goal_distance_m,
                    period=2.0,
                )

            selected_pose = build_selected_frontier_pose(
                selected_frontier,
                frame_id=self.global_frame,
                stamp=self.get_clock().now().to_msg(),
            )
            self._publish_debug_outputs(frontiers, optimized_map_msg, selected_pose)

            goal_pose = self._build_goal_pose(robot_state, selected_goal)
            if self._send_goal(goal_pose, selected_frontier, goal_kind="frontier"):
                self.get_logger().info(
                    "Dispatching frontier goal at (%.2f, %.2f) from MRTSP order of %d frontiers"
                    % (
                        selected_goal[0],
                        selected_goal[1],
                        len(order),
                    )
                )
        except Exception:
            self.get_logger().error(
                "Explorer timer failed:\n%s" % traceback.format_exc()
            )


def main() -> None:
    rclpy.init()
    node = MRTSPExplorerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
