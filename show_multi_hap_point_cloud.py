#!/usr/bin/env python3
"""
show_multi_hap_point_cloud.py

複数台の HAP 点群を TS 座標系に変換して 3D 可視化する。

使い方:
  python3 show_multi_hap_point_cloud.py [-n N ...] [--data-folder PATH]

オプション:
  --hap-num, -n  対象 HAP 番号（複数可、デフォルト: 101 102）
  --data-folder  データフォルダのパス

入力（data-folder 以下）:
  input_data/hap<N>.csv                   : HAP 点群
  output_data/hap<N>_coorsys_py.yaml       : キャリブ結果

Original: showMultiHapPointCloud.m
"""

import os
import argparse
from pathlib import Path

import numpy as np
import yaml
import open3d as o3d
from scipy.spatial.transform import Rotation

from hap_csv_io import load_hap_csv


SCRIPT_DIR          = Path(__file__).resolve().parent
DEFAULT_HAP_NUMS    = [101, 102]
DEFAULT_DATA_FOLDER = str(SCRIPT_DIR / 'data')

# 台数に応じて割り当てる表示色（3台目以降も区別可能）
HAP_COLORS = [
    [1.0, 0.3, 0.3],   # 赤
    [0.3, 0.6, 1.0],   # 青
    [0.3, 0.9, 0.4],   # 緑
    [1.0, 0.85, 0.2],  # 黄
    [0.8, 0.4, 1.0],   # 紫
    [1.0, 0.6, 0.2],   # 橙
    [0.2, 0.9, 0.9],   # シアン
    [0.9, 0.4, 0.6],   # ピンク
]
HAP_COLOR_NAMES = ['赤', '青', '緑', '黄', '紫', '橙', 'シアン', 'ピンク']


# ============================================================
# ヘルパー関数
# ============================================================

def load_transform_from_yaml(yaml_path):
    """
    位置・姿勢 YAML から 4×4 同次変換行列を復元する。

    YAML フォーマット:
      Position: {x, y, z}  [mm]
      Rotation: {roll, pitch, yaw}  [deg]  ZYX オイラー角
    """
    with open(yaml_path) as f:
        d = yaml.safe_load(f)

    t = np.array([
        d['Position']['x'] / 1000.0,   # mm → m
        d['Position']['y'] / 1000.0,
        d['Position']['z'] / 1000.0,
    ])
    R = Rotation.from_euler(
        'ZYX',
        [d['Rotation']['yaw'], d['Rotation']['pitch'], d['Rotation']['roll']],
        degrees=True
    ).as_matrix()

    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t
    return T


def load_pointcloud_from_csv(csv_path):
    """CSV 点群を読み込み xyz と intensity を返す。"""
    print(f"点群読み込み中: {csv_path}")
    xyz, intensity, _tag = load_hap_csv(csv_path)
    print(f"  点数: {len(xyz)}")
    return xyz, intensity


def transform_points(xyz, T):
    """xyz (N×3) を 4×4 同次変換行列 T で変換する。"""
    n     = len(xyz)
    pts_h = np.hstack([xyz, np.ones((n, 1))])
    return (T @ pts_h.T).T[:, :3]


def make_pcd(xyz, color):
    """numpy 配列から Open3D PointCloud オブジェクトを生成する。"""
    pcd        = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.paint_uniform_color(color)
    return pcd


def hap_color(index):
    """表示色と色名を返す（パレットを循環）。"""
    i = index % len(HAP_COLORS)
    return HAP_COLORS[i], HAP_COLOR_NAMES[i]


# ============================================================
# メイン処理
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description='複数台の HAP 点群を TS 座標系に変換して 3D 可視化する'
    )
    parser.add_argument(
        '--hap-num',
        '-n',
        type=int,
        nargs='+',
        default=list(DEFAULT_HAP_NUMS),
        metavar='N',
        help=f"対象 HAP 番号（複数可、デフォルト: {' '.join(map(str, DEFAULT_HAP_NUMS))}）",
    )
    parser.add_argument(
        '--data-folder',
        '-d',
        type=str,
        default=DEFAULT_DATA_FOLDER,
        metavar='PATH',
        help=f'データフォルダ（デフォルト: {DEFAULT_DATA_FOLDER}）',
    )
    return parser.parse_args()


def main():
    args      = parse_args()
    folder    = os.path.expanduser(args.data_folder)
    hap_nums  = args.hap_num

    print(f"HAP番号     : {' '.join(map(str, hap_nums))}")
    print(f"データフォルダ: {folder}\n")

    pcds         = []
    legend_parts = []

    for i, hap_num in enumerate(hap_nums):
        color, color_name = hap_color(i)

        csv_path = os.path.join(folder, 'input_data', f'hap{hap_num}.csv')
        yaml_path = os.path.join(folder, 'output_data', f'hap{hap_num}_coorsys_py.yaml')

        xyz, _ = load_pointcloud_from_csv(csv_path)
        T = load_transform_from_yaml(yaml_path)
        print(f"\nT (hap{hap_num}):\n{np.round(T, 4)}")

        print(f"\nTS座標系へ変換中... (HAP{hap_num})")
        xyz_ts = transform_points(xyz, T)
        pcds.append(make_pcd(xyz_ts, color))
        legend_parts.append(f'HAP{hap_num} ({color_name})')

    print("\n可視化ウィンドウを開きます（ウィンドウを閉じると終了）")
    o3d.visualization.draw_geometries(
        pcds,
        window_name=' + '.join(legend_parts) + '  ─  TS座標系',
        width=1280,
        height=720,
    )


if __name__ == '__main__':
    main()
