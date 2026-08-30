#!/usr/bin/env python3
"""PGM 地图转 PNG/JPG

用法:
  python3 pgm_to_image.py <pgm文件> [--output <输出路径>] [--format png|jpg]

示例:
  # 转成 png（默认）
  python3 pgm_to_image.py src/planner/nav2_planner_bringup/map/test_map__2.pgm

  # 指定输出路径和格式
  python3 pgm_to_image.py src/planner/nav2_planner_bringup/map/test_map__2.pgm -o /tmp/map.jpg --format jpg
"""

import argparse
import sys
from pathlib import Path

try:
    import numpy as np
    from PIL import Image
except ImportError:
    print("需要安装 Pillow 和 numpy:")
    print("  pip install Pillow numpy")
    sys.exit(1)


def pgm_to_image(pgm_path: str, output_path: str, fmt: str):
    """读取 PGM，转为 PNG/JPG。ROS map_server 的 PGM 是 mode=P，0=空闲, 254=占据。"""
    pgm = Image.open(pgm_path)

    # PGM 是 palette 模式，转为灰度方便查看
    if pgm.mode == "P":
        img = pgm.convert("L")
    else:
        img = pgm

    arr = np.array(img)

    # ROS 约定: 0=空闲(黑), 254=占据(白), 205=未知。翻转灰度，让障碍物为深色
    # 即: 空闲 0→255(白), 占据 254→1(黑), 未知 205→50(灰)
    out = np.full_like(arr, 128, dtype=np.uint8)  # 默认灰色
    out[arr == 0] = 255      # 空闲 → 白
    out[arr == 254] = 0      # 占据 → 黑
    out[arr == 205] = 128    # 未知 → 灰

    result = Image.fromarray(out, mode="L")
    result.save(output_path, format=fmt.upper())
    print(f"已保存: {output_path} ({img.size[0]}×{img.size[1]})")
    return output_path


def main():
    parser = argparse.ArgumentParser(description="PGM 地图 → PNG/JPG")
    parser.add_argument("pgm_file", help="PGM 文件路径")
    parser.add_argument("-o", "--output", help="输出文件路径（默认: 同目录同名改后缀）")
    parser.add_argument("--format", choices=["png", "jpg"], default="png", help="输出格式 (默认: png)")

    args = parser.parse_args()

    pgm_path = Path(args.pgm_file)
    if not pgm_path.exists():
        print(f"错误: 文件不存在: {pgm_path}")
        sys.exit(1)

    if args.output:
        output_path = args.output
    else:
        ext = "jpg" if args.format == "jpg" else "png"
        output_path = str(pgm_path.with_suffix(f".{ext}"))

    pgm_to_image(str(pgm_path), output_path, args.format)


if __name__ == "__main__":
    main()
