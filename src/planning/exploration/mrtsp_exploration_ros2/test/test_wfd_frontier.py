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

from mrtsp_exploration_ros2.occupancy_grid import OccupancyGridAdapter
from mrtsp_exploration_ros2.wfd_frontier import extract_frontiers

from test_helpers import make_grid


def test_wfd_extracts_deterministic_frontier_cluster():
    map_grid = make_grid(
        [
            [-1, -1, -1, -1, -1],
            [-1, 0, 0, 0, -1],
            [-1, 0, 0, 0, -1],
            [-1, 0, 0, 0, -1],
            [-1, -1, -1, -1, -1],
        ]
    )
    costmap_grid = make_grid(
        [
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ]
    )

    map_adapter = OccupancyGridAdapter(map_grid, occupied_threshold=50)
    costmap_adapter = OccupancyGridAdapter(costmap_grid, occupied_threshold=50)
    optimized_image = map_adapter.to_paper_image()

    frontiers = extract_frontiers(
        optimized_image=optimized_image,
        map_adapter=map_adapter,
        costmap_adapter=costmap_adapter,
        robot_world_position=(2.5, 2.5),
        min_frontier_size_cells=4,
        occ_threshold=50,
    )

    assert len(frontiers) == 1
    assert frontiers[0].size >= 8
    assert frontiers[0].start_cell in frontiers[0].cells


def test_wfd_handles_smaller_costmap_without_index_error():
    map_grid = make_grid(
        [
            [-1, -1, -1, -1, -1],
            [-1, 0, 0, 0, -1],
            [-1, 0, 0, 0, -1],
            [-1, 0, 0, 0, -1],
            [-1, -1, -1, -1, -1],
        ]
    )
    costmap_grid = make_grid(
        [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]
    )

    map_adapter = OccupancyGridAdapter(map_grid, occupied_threshold=50)
    costmap_adapter = OccupancyGridAdapter(costmap_grid, occupied_threshold=50)
    optimized_image = map_adapter.to_paper_image()

    frontiers = extract_frontiers(
        optimized_image=optimized_image,
        map_adapter=map_adapter,
        costmap_adapter=costmap_adapter,
        robot_world_position=(2.5, 2.5),
        min_frontier_size_cells=1,
        occ_threshold=50,
    )

    assert isinstance(frontiers, list)
