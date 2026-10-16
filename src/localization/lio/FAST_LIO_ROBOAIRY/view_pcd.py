#!/usr/bin/env python3
"""PCD 查看器 — 支持 Z 轴过滤、体素降采样、尺寸统计。"""

import sys
import argparse
import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description='PCD 点云查看器')
    parser.add_argument('file', help='PCD 文件路径')
    parser.add_argument('--zmin', type=float, default=None, help='Z 下界(m)，去掉地面')
    parser.add_argument('--zmax', type=float, default=None, help='Z 上界(m)，去掉天花板')
    parser.add_argument('--voxel', type=float, default=0, help='体素降采样(m)，0=不降')
    parser.add_argument('--no-view', action='store_true', help='只看统计，不显示窗口')
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.file)
    pts = np.asarray(pcd.points)

    print(f"文件: {args.file}")
    print(f"点数: {len(pts)}")
    print(f"X: [{pts[:,0].min():.2f}, {pts[:,0].max():.2f}] m")
    print(f"Y: [{pts[:,1].min():.2f}, {pts[:,1].max():.2f}] m")
    print(f"Z: [{pts[:,2].min():.2f}, {pts[:,2].max():.2f}] m")

    if args.zmin is not None or args.zmax is not None:
        zl = args.zmin if args.zmin else -np.inf
        zh = args.zmax if args.zmax else np.inf
        idx = np.where((pts[:, 2] > zl) & (pts[:, 2] < zh))[0]
        pcd = pcd.select_by_index(idx)
        print(f"Z 过滤 [{zl}, {zh}]: {len(pts)} → {len(pcd.points)}")

    if args.voxel > 0:
        pcd = pcd.voxel_down_sample(args.voxel)
        print(f"体素 {args.voxel}m: → {len(pcd.points)}")

    if args.no_view:
        return

    o3d.visualization.draw_geometries(
        [pcd, o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)],
        window_name=args.file, width=1200, height=800)


if __name__ == '__main__':
    main()
