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

from typing import List

import numpy as np


def greedy_mrtsp_order(cost_matrix: np.ndarray) -> List[int]:
    if cost_matrix.size == 0 or cost_matrix.shape[0] <= 1:
        return []

    current_index = 0
    unvisited = set(range(1, cost_matrix.shape[1]))
    ordered_frontiers: List[int] = []

    while unvisited:
        next_index = min(unvisited, key=lambda candidate: cost_matrix[current_index, candidate])
        if not np.isfinite(cost_matrix[current_index, next_index]):
            break
        ordered_frontiers.append(next_index - 1)
        unvisited.remove(next_index)
        current_index = next_index

    return ordered_frontiers
