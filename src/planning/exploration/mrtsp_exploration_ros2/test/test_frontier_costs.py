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
import numpy as np

from mrtsp_exploration_ros2.frontier_costs import (
    CostWeights,
    RobotState,
    build_cost_matrix,
    frontier_path_cost_with_start_world,
)
from mrtsp_exploration_ros2.frontier_model import Frontier


def make_frontier(start_cell, centroid, center_point, center_cell, size):
    return Frontier(
        cells=(start_cell, center_cell),
        start_cell=start_cell,
        centroid=centroid,
        center_point=center_point,
        center_cell=center_cell,
        size=size,
    )


def test_frontier_path_cost_matches_paper_formula():
    source = make_frontier((0, 0), (0.0, 0.0), (0.0, 0.0), (0, 0), 8)
    target = make_frontier((3, 0), (4.0, 0.0), (5.0, 0.0), (5, 0), 10)

    cost = frontier_path_cost_with_start_world(
        source,
        target,
        start_world=(3.0, 0.0),
        sensor_effective_range_m=2.0,
    )

    assert cost == 5.0


def test_cost_matrix_builds_robot_row_and_frontier_rows():
    frontiers = [
        make_frontier((3, 0), (4.0, 0.0), (5.0, 0.0), (5, 0), 10),
        make_frontier((0, 3), (0.0, 4.0), (0.0, 5.0), (0, 5), 5),
    ]
    robot_state = RobotState(position=np.array([0.0, 0.0]), yaw=0.0)

    matrix = build_cost_matrix(
        frontiers=frontiers,
        frontier_start_world_points=[(3.0, 0.0), (0.0, 3.0)],
        robot_state=robot_state,
        weights=CostWeights(distance_wd=1.0, gain_ws=1.0),
        sensor_effective_range_m=2.0,
        max_linear_speed_vmax=1.0,
        max_angular_speed_wmax=1.0,
    )

    assert matrix.shape == (3, 3)
    assert matrix[1, 0] == 0.0
    assert np.isfinite(matrix[0, 1])
