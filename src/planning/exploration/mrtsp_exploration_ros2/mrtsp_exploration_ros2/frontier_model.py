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
from dataclasses import dataclass
from typing import Tuple

import numpy as np


GridCell = Tuple[int, int]
WorldPoint = Tuple[float, float]


@dataclass(frozen=True)
class Frontier:
    """Immutable frontier description used across the package."""

    cells: Tuple[GridCell, ...]
    start_cell: GridCell
    centroid: WorldPoint
    center_point: WorldPoint
    center_cell: GridCell
    size: int

    @property
    def centroid_array(self) -> np.ndarray:
        return np.array(self.centroid, dtype=float)

    @property
    def center_array(self) -> np.ndarray:
        return np.array(self.center_point, dtype=float)
