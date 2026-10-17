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
import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .frontier_model import Frontier


@dataclass(frozen=True)
class RobotState:
    position: np.ndarray
    yaw: float


@dataclass(frozen=True)
class CostWeights:
    distance_wd: float
    gain_ws: float


def euclidean(lhs: Sequence[float], rhs: Sequence[float]) -> float:
    return float(np.linalg.norm(np.asarray(lhs, dtype=float) - np.asarray(rhs, dtype=float)))


def frontier_information_gain(frontier: Frontier) -> float:
    return float(frontier.size)


def frontier_path_cost(
    source_frontier: Frontier,
    target_frontier: Frontier,
    sensor_effective_range_m: float,
) -> float:
    source = np.asarray(source_frontier.center_point, dtype=float)
    target_centroid = np.asarray(target_frontier.centroid, dtype=float)
    target_center = np.asarray(target_frontier.center_point, dtype=float)
    target_start = np.asarray(target_frontier.start_cell, dtype=float)

    # start_cell is stored in grid coordinates, so convert using world points from center/centroid geometry.
    # The traversal terms only need the target frontier geometry consistency.
    start_world = target_center + (target_start - np.asarray(target_frontier.center_cell, dtype=float)) * 0.0
    # Since the paper defines geometric relationships on frontier geometry while this implementation exposes
    # only world points, the target start point is approximated by the target center point in world space unless
    # the caller replaces it with a converted world coordinate before calling this function.
    start_world = np.asarray(target_frontier.center_point, dtype=float)

    dm = euclidean(source, target_center)
    dn = euclidean(source, target_centroid)
    du = euclidean(target_center, start_world)
    dv = euclidean(target_centroid, start_world)
    return max(dm + du, dn + dv) - float(sensor_effective_range_m)


def frontier_path_cost_with_start_world(
    source_frontier: Frontier,
    target_frontier: Frontier,
    start_world: Sequence[float],
    sensor_effective_range_m: float,
) -> float:
    source = np.asarray(source_frontier.center_point, dtype=float)
    target_centroid = np.asarray(target_frontier.centroid, dtype=float)
    target_center = np.asarray(target_frontier.center_point, dtype=float)
    start = np.asarray(start_world, dtype=float)

    dm = euclidean(source, target_center)
    dn = euclidean(source, target_centroid)
    du = euclidean(target_center, start)
    dv = euclidean(target_centroid, start)
    return max(dm + du, dn + dv) - float(sensor_effective_range_m)


def initial_frontier_path_cost(
    robot_position: Sequence[float],
    target_frontier: Frontier,
    start_world: Sequence[float],
    sensor_effective_range_m: float,
) -> float:
    source = np.asarray(robot_position, dtype=float)
    target_centroid = np.asarray(target_frontier.centroid, dtype=float)
    target_center = np.asarray(target_frontier.center_point, dtype=float)
    start = np.asarray(start_world, dtype=float)

    dm = euclidean(source, target_center)
    dn = euclidean(source, target_centroid)
    du = euclidean(target_center, start)
    dv = euclidean(target_centroid, start)
    return max(dm + du, dn + dv) - float(sensor_effective_range_m)


def angle_wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def lower_bound_time_cost(
    robot_state: RobotState,
    target_point: Sequence[float],
    max_linear_speed_vmax: float,
    max_angular_speed_wmax: float,
) -> float:
    vmax = max(float(max_linear_speed_vmax), 1e-6)
    wmax = max(float(max_angular_speed_wmax), 1e-6)
    target = np.asarray(target_point, dtype=float)
    distance_term = euclidean(robot_state.position, target) / vmax
    target_yaw = math.atan2(
        target[1] - robot_state.position[1],
        target[0] - robot_state.position[0],
    )
    yaw_delta = abs(angle_wrap(target_yaw - robot_state.yaw))
    heading_term = min(yaw_delta, 2.0 * math.pi - yaw_delta) / wmax
    return min(distance_term, heading_term)


def build_cost_matrix(
    frontiers: Sequence[Frontier],
    frontier_start_world_points: Sequence[Sequence[float]],
    robot_state: RobotState,
    weights: CostWeights,
    sensor_effective_range_m: float,
    max_linear_speed_vmax: float,
    max_angular_speed_wmax: float,
) -> np.ndarray:
    frontier_count = len(frontiers)
    matrix = np.full((frontier_count + 1, frontier_count + 1), np.inf, dtype=float)

    for row in range(1, frontier_count + 1):
        matrix[row, 0] = 0.0

    for row in range(frontier_count + 1):
        for column in range(1, frontier_count + 1):
            if row == column:
                continue

            gain = weights.gain_ws * frontier_information_gain(frontiers[column - 1])
            if gain <= 0.0:
                continue

            if row == 0:
                path_cost = initial_frontier_path_cost(
                    robot_state.position,
                    frontiers[column - 1],
                    frontier_start_world_points[column - 1],
                    sensor_effective_range_m,
                )
                ratio = (weights.distance_wd * path_cost) / gain
                matrix[row, column] = ratio + lower_bound_time_cost(
                    robot_state,
                    frontiers[column - 1].center_point,
                    max_linear_speed_vmax,
                    max_angular_speed_wmax,
                )
            else:
                path_cost = frontier_path_cost_with_start_world(
                    frontiers[row - 1],
                    frontiers[column - 1],
                    frontier_start_world_points[column - 1],
                    sensor_effective_range_m,
                )
                matrix[row, column] = (weights.distance_wd * path_cost) / gain

    return matrix
