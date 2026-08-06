#!/usr/bin/env python3
"""PGM → PNG / JPG 转换脚本"""

import argparse
from pathlib import Path
from PIL import Image
import numpy as np


def pgm_to_image(pgm_path: str, output_path: str = None, fmt: str = "png",
                 invert: bool = False, threshold: int = None):
    pgm_path = Path(pgm_path)
    if not pgm_path.exists():
        raise FileNotFoundError(f"找不到文件: {pgm_path}")

    img = Image.open(pgm_path)
    arr = np.array(img)

    print(f"原始: {pgm_path}  |  尺寸: {img.size}  |  mode: {img.mode}")
    print(f"像素值范围: [{arr.min()}, {arr.max()}]")

    if threshold is not None:
        arr = np.where(arr > threshold, 255, 0).astype(np.uint8)
        img = Image.fromarray(arr)
        print(f"已二值化 (阈值={threshold})")

    if invert:
        arr = 255 - np.array(img)
        img = Image.fromarray(arr)
        print("已反转")

    if output_path is None:
        output_path = pgm_path.with_suffix(f".{fmt}")
    else:
        output_path = Path(output_path)

    save_kwargs = {}
    if fmt == "jpg":
        img = img.convert("RGB")
        save_kwargs["quality"] = 95

    img.save(output_path, **save_kwargs)
    print(f"输出: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PGM → PNG/JPG 转换")
    parser.add_argument("input", help="PGM 文件路径")
    parser.add_argument("-o", "--output", default=None, help="输出路径 (默认同目录替换后缀)")
    parser.add_argument("-f", "--format", default="png", choices=["png", "jpg"],
                        help="输出格式 (默认: png)")
    parser.add_argument("-i", "--invert", action="store_true", help="反转颜色")
    parser.add_argument("-t", "--threshold", type=int, default=None,
                        help="二值化阈值 0-255 (灰色未知区域归并)")
    args = parser.parse_args()

    pgm_to_image(args.input, args.output, args.format, args.invert, args.threshold)
