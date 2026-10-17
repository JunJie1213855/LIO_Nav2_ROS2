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

import numpy as np
from scipy import ndimage

from .occupancy_grid import PAPER_FREE, PAPER_OCCUPIED, PAPER_UNKNOWN, OccupancyGridAdapter


@dataclass(frozen=True)
class MapOptimizationResult:
    raw_image: np.ndarray
    filtered_image: np.ndarray
    optimized_image: np.ndarray


def _disk_kernel(radius: int) -> np.ndarray:
    radius = max(0, int(radius))
    if radius == 0:
        return np.ones((1, 1), dtype=bool)

    yy, xx = np.ogrid[-radius : radius + 1, -radius : radius + 1]
    return (xx * xx + yy * yy) <= (radius * radius)


def bilateral_filter(
    image: np.ndarray,
    sigma_s: float,
    sigma_r: float,
) -> np.ndarray:
    sigma_s = max(float(sigma_s), 1e-6)
    sigma_r = max(float(sigma_r), 1e-6)
    radius = max(1, int(math.ceil(2.0 * sigma_s)))

    image_float = image.astype(np.float32)
    padded = np.pad(image_float, radius, mode="edge")
    center = padded[radius:-radius, radius:-radius]
    weighted_sum = np.zeros_like(center, dtype=np.float32)
    normalization = np.zeros_like(center, dtype=np.float32)

    yy, xx = np.mgrid[-radius : radius + 1, -radius : radius + 1]
    spatial_kernel = np.exp(-((xx * xx + yy * yy) / (2.0 * sigma_s * sigma_s)))

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            shifted = padded[
                radius + dy : radius + dy + image.shape[0],
                radius + dx : radius + dx + image.shape[1],
            ]
            range_kernel = np.exp(
                -(((center - shifted) ** 2) / (2.0 * sigma_r * sigma_r))
            )
            weights = spatial_kernel[dy + radius, dx + radius] * range_kernel
            weighted_sum += weights * shifted
            normalization += weights

    normalization = np.maximum(normalization, 1e-6)
    return weighted_sum / normalization


def optimize_map(
    adapter: OccupancyGridAdapter,
    sigma_s: float,
    sigma_r: float,
    dilation_kernel_radius_cells: int,
) -> MapOptimizationResult:
    raw_image = adapter.to_paper_image()
    filtered = bilateral_filter(raw_image, sigma_s=sigma_s, sigma_r=sigma_r)

    optimized = np.full(raw_image.shape, PAPER_UNKNOWN, dtype=np.uint8)
    optimized[filtered >= ((PAPER_FREE + PAPER_UNKNOWN) / 2.0)] = PAPER_FREE
    optimized[raw_image == PAPER_OCCUPIED] = PAPER_OCCUPIED

    free_mask = optimized == PAPER_FREE
    expanded_free = ndimage.binary_dilation(
        free_mask,
        structure=_disk_kernel(dilation_kernel_radius_cells),
    )
    optimized[(expanded_free) & (optimized != PAPER_OCCUPIED)] = PAPER_FREE
    optimized[raw_image == PAPER_OCCUPIED] = PAPER_OCCUPIED

    return MapOptimizationResult(
        raw_image=raw_image,
        filtered_image=filtered.astype(np.float32),
        optimized_image=optimized,
    )
