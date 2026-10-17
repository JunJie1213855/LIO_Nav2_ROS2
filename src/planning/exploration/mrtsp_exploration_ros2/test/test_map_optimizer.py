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

from mrtsp_exploration_ros2.map_optimizer import optimize_map
from mrtsp_exploration_ros2.occupancy_grid import (
    PAPER_FREE,
    PAPER_OCCUPIED,
    OccupancyGridAdapter,
)

from test_helpers import make_grid


def test_map_optimizer_preserves_obstacles_and_expands_known_space():
    grid = make_grid(
        [
            [100, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, -1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0],
        ]
    )
    adapter = OccupancyGridAdapter(grid, occupied_threshold=50)

    result = optimize_map(
        adapter,
        sigma_s=1.0,
        sigma_r=30.0,
        dilation_kernel_radius_cells=1,
    )

    assert result.optimized_image[0, 0] == PAPER_OCCUPIED
    assert result.optimized_image[2, 2] == PAPER_FREE
