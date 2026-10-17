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

from collections import deque
from typing import Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

from .frontier_model import Frontier, GridCell
from .occupancy_grid import PAPER_FREE, PAPER_OCCUPIED, PAPER_UNKNOWN, OccupancyGridAdapter


def _find_nearest_free_cell(
    robot_cell: GridCell,
    optimized_image: np.ndarray,
    map_adapter: OccupancyGridAdapter,
) -> Optional[GridCell]:
    queue = deque([robot_cell])
    visited: Set[GridCell] = {robot_cell}

    while queue:
        cell = queue.popleft()
        if optimized_image[cell[1], cell[0]] == PAPER_FREE:
            return cell
        for neighbor in map_adapter.neighbors8(cell[0], cell[1]):
            item = (neighbor.x, neighbor.y)
            if item in visited:
                continue
            visited.add(item)
            queue.append(item)
    return None


def _is_frontier_cell(
    cell: GridCell,
    optimized_image: np.ndarray,
    map_adapter: OccupancyGridAdapter,
    costmap_adapter: OccupancyGridAdapter,
    occ_threshold: int,
) -> bool:
    x, y = cell
    if optimized_image[y, x] != PAPER_UNKNOWN:
        return False

    has_free_neighbor = False
    for neighbor in map_adapter.neighbors8(x, y):
        nx, ny = neighbor.x, neighbor.y
        neighbor_world = map_adapter.map_to_world(nx, ny)
        costmap_cell = costmap_adapter.world_to_map(
            neighbor_world[0],
            neighbor_world[1],
        )
        # Global costmap coverage can temporarily lag behind the map bounds.
        # Treat cells outside the costmap as blocked instead of crashing.
        if costmap_cell is None:
            return False
        if costmap_adapter.get_cost(costmap_cell.x, costmap_cell.y) >= occ_threshold:
            return False
        if optimized_image[ny, nx] == PAPER_FREE:
            has_free_neighbor = True
    return has_free_neighbor


def _build_frontier(
    frontier_cells: Sequence[GridCell],
    start_cell: GridCell,
    map_adapter: OccupancyGridAdapter,
) -> Frontier:
    cell_world_points = np.array(
        [map_adapter.map_to_world(cell[0], cell[1]) for cell in frontier_cells],
        dtype=float,
    )
    centroid = tuple(cell_world_points.mean(axis=0).tolist())
    center_index = int(np.argmin(np.linalg.norm(cell_world_points - cell_world_points.mean(axis=0), axis=1)))
    center_cell = frontier_cells[center_index]
    center_point = tuple(cell_world_points[center_index].tolist())
    return Frontier(
        cells=tuple(frontier_cells),
        start_cell=start_cell,
        centroid=centroid,
        center_point=center_point,
        center_cell=center_cell,
        size=len(frontier_cells),
    )


def extract_frontiers(
    optimized_image: np.ndarray,
    map_adapter: OccupancyGridAdapter,
    costmap_adapter: OccupancyGridAdapter,
    robot_world_position: Sequence[float],
    min_frontier_size_cells: int,
    occ_threshold: int,
) -> List[Frontier]:
    robot_cell = map_adapter.world_to_map(robot_world_position[0], robot_world_position[1])
    if robot_cell is None:
        return []

    start_cell = _find_nearest_free_cell((robot_cell.x, robot_cell.y), optimized_image, map_adapter)
    if start_cell is None:
        return []

    map_open = deque([start_cell])
    map_open_set: Set[GridCell] = {start_cell}
    map_closed: Set[GridCell] = set()
    frontiers: List[Frontier] = []

    while map_open:
        point = map_open.popleft()
        map_open_set.discard(point)
        if point in map_closed:
            continue

        if _is_frontier_cell(point, optimized_image, map_adapter, costmap_adapter, occ_threshold):
            frontier_queue = deque([point])
            frontier_open: Set[GridCell] = {point}
            frontier_closed: Set[GridCell] = set()
            frontier_cells: List[GridCell] = []
            frontier_start = point

            while frontier_queue:
                candidate = frontier_queue.popleft()
                frontier_open.discard(candidate)
                if candidate in map_closed or candidate in frontier_closed:
                    continue
                if _is_frontier_cell(candidate, optimized_image, map_adapter, costmap_adapter, occ_threshold):
                    frontier_cells.append(candidate)
                    for neighbor in map_adapter.neighbors8(candidate[0], candidate[1]):
                        item = (neighbor.x, neighbor.y)
                        if (
                            item not in frontier_open
                            and item not in frontier_closed
                            and item not in map_closed
                        ):
                            frontier_open.add(item)
                            frontier_queue.append(item)
                frontier_closed.add(candidate)

            if len(frontier_cells) >= min_frontier_size_cells:
                ordered_cells = tuple(sorted(frontier_cells, key=lambda cell: (cell[1], cell[0])))
                frontiers.append(_build_frontier(ordered_cells, frontier_start, map_adapter))
                map_closed.update(frontier_cells)

        for neighbor in map_adapter.neighbors8(point[0], point[1]):
            item = (neighbor.x, neighbor.y)
            if item in map_open_set or item in map_closed:
                continue
            if any(
                optimized_image[next_neighbor.y, next_neighbor.x] == PAPER_FREE
                for next_neighbor in map_adapter.neighbors8(neighbor.x, neighbor.y)
            ):
                map_open.append(item)
                map_open_set.add(item)

        map_closed.add(point)

    return frontiers
