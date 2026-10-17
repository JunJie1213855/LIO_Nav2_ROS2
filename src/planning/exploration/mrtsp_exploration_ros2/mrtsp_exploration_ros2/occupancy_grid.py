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
from typing import Iterator, Optional, Tuple

import numpy as np
from nav_msgs.msg import MapMetaData, OccupancyGrid


PAPER_FREE = 255
PAPER_UNKNOWN = 205
PAPER_OCCUPIED = 0


@dataclass(frozen=True)
class GridSize:
    width: int
    height: int


@dataclass(frozen=True)
class GridCell:
    x: int
    y: int


class OccupancyGridAdapter:
    """Helpers for OccupancyGrid indexing and paper-specific image conversion."""

    def __init__(self, msg: OccupancyGrid, occupied_threshold: int = 50) -> None:
        self.msg = msg
        self.size = GridSize(width=msg.info.width, height=msg.info.height)
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.occupied_threshold = occupied_threshold
        self._data = np.asarray(msg.data, dtype=np.int16).reshape(
            self.size.height,
            self.size.width,
        )

    @property
    def data(self) -> np.ndarray:
        return self._data

    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.size.width and 0 <= y < self.size.height

    def get_cost(self, x: int, y: int) -> int:
        return int(self._data[y, x])

    def classify_cell(self, x: int, y: int) -> int:
        value = self.get_cost(x, y)
        if value < 0:
            return PAPER_UNKNOWN
        if value >= self.occupied_threshold:
            return PAPER_OCCUPIED
        return PAPER_FREE

    def to_paper_image(self) -> np.ndarray:
        image = np.full(
            (self.size.height, self.size.width),
            PAPER_UNKNOWN,
            dtype=np.uint8,
        )
        image[self._data >= self.occupied_threshold] = PAPER_OCCUPIED
        image[(self._data >= 0) & (self._data < self.occupied_threshold)] = PAPER_FREE
        return image

    def map_to_world(self, x: int, y: int) -> Tuple[float, float]:
        world_x = self.origin_x + (x + 0.5) * self.resolution
        world_y = self.origin_y + (y + 0.5) * self.resolution
        return (world_x, world_y)

    def world_to_map(self, world_x: float, world_y: float) -> Optional[GridCell]:
        if world_x < self.origin_x or world_y < self.origin_y:
            return None

        map_x = int(math.floor((world_x - self.origin_x) / self.resolution))
        map_y = int(math.floor((world_y - self.origin_y) / self.resolution))

        if not self.in_bounds(map_x, map_y):
            return None

        return GridCell(map_x, map_y)

    def neighbors8(self, x: int, y: int) -> Iterator[GridCell]:
        for nx in range(x - 1, x + 2):
            for ny in range(y - 1, y + 2):
                if nx == x and ny == y:
                    continue
                if self.in_bounds(nx, ny):
                    yield GridCell(nx, ny)


def paper_image_to_occupancy_grid(
    image: np.ndarray,
    reference: OccupancyGrid,
) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header = reference.header
    msg.info = MapMetaData()
    msg.info.map_load_time = reference.info.map_load_time
    msg.info.resolution = reference.info.resolution
    msg.info.width = reference.info.width
    msg.info.height = reference.info.height
    msg.info.origin = reference.info.origin

    occupancy = np.full_like(image, -1, dtype=np.int16)
    occupancy[image == PAPER_FREE] = 0
    occupancy[image == PAPER_OCCUPIED] = 100
    msg.data = occupancy.reshape(-1).tolist()
    return msg
