#!/usr/bin/env python3
"""
showMultiHapPointCloud.py

2台のHAP点群をTS座標系に変換して3D可視化する。

使い方:
  python3 showMultiHapPointCloud.py [--hap-num1 N1] [--hap-num2 N2] [--data-folder PATH]

オプション:
  --hap-num1    N1    1台目の HAP 番号（デフォルト: 101）
  --hap-num2    N2    2台目の HAP 番号（デフォルト: 102）
  --data-folder PATH  データフォルダのパス

入力（data-folder 以下）:
  inputData/hap<N>.csv                   : HAP 点群
  outputData/hap<N>Coorsys_py.yaml       : キャリブ結果

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
DEFAULT_HAP_NUM1    = 101
DEFAULT_HAP_NUM2    = 102
DEFAULT_DATA_FOLDER = str(SCRIPT_DIR / 'data')


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


# ============================================================
# メイン処理
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description='2台のHAP点群をTS座標系に変換して3D可視化する'
    )
    parser.add_argument('--hap-num1',    '-n1', type=int, default=DEFAULT_HAP_NUM1,
                        metavar='N1',   help=f'1台目の HAP 番号（デフォルト: {DEFAULT_HAP_NUM1}）')
    parser.add_argument('--hap-num2',    '-n2', type=int, default=DEFAULT_HAP_NUM2,
                        metavar='N2',   help=f'2台目の HAP 番号（デフォルト: {DEFAULT_HAP_NUM2}）')
    parser.add_argument('--data-folder', '-d',  type=str, default=DEFAULT_DATA_FOLDER,
                        metavar='PATH', help=f'データフォルダ（デフォルト: {DEFAULT_DATA_FOLDER}）')
    return parser.parse_args()


def main():
    args   = parse_args()
    folder = os.path.expanduser(args.data_folder)
    n1     = args.hap_num1
    n2     = args.hap_num2

    print(f"HAP番号1    : {n1}")
    print(f"HAP番号2    : {n2}")
    print(f"データフォルダ: {folder}\n")

    # ----------------------------------------------------------------
    # 点群読み込み
    # ----------------------------------------------------------------
    xyz1, _ = load_pointcloud_from_csv(
        os.path.join(folder, 'inputData', f'hap{n1}.csv'))
    xyz2, _ = load_pointcloud_from_csv(
        os.path.join(folder, 'inputData', f'hap{n2}.csv'))

    # ----------------------------------------------------------------
    # 変換行列読み込み（YAML → 4×4 同次変換行列）
    # ----------------------------------------------------------------
    yaml1 = os.path.join(folder, 'outputData', f'hap{n1}Coorsys_py.yaml')
    yaml2 = os.path.join(folder, 'outputData', f'hap{n2}Coorsys_py.yaml')
    T1    = load_transform_from_yaml(yaml1)
    T2    = load_transform_from_yaml(yaml2)
    print(f"\nT1 (hap{n1}):\n{np.round(T1, 4)}")
    print(f"\nT2 (hap{n2}):\n{np.round(T2, 4)}")

    # ----------------------------------------------------------------
    # TS座標系へ変換
    # ----------------------------------------------------------------
    print("\nTS座標系へ変換中...")
    xyz1_ts = transform_points(xyz1, T1)
    xyz2_ts = transform_points(xyz2, T2)

    # ----------------------------------------------------------------
    # Open3D で可視化
    # ----------------------------------------------------------------
    print("可視化ウィンドウを開きます（ウィンドウを閉じると終了）")
    pcd1 = make_pcd(xyz1_ts, [1.0, 0.3, 0.3])  # 赤系（hap1）
    pcd2 = make_pcd(xyz2_ts, [0.3, 0.6, 1.0])  # 青系（hap2）

    o3d.visualization.draw_geometries(
        [pcd1, pcd2],
        window_name=f'HAP{n1} (赤) + HAP{n2} (青)  ─  TS座標系',
        width=1280,
        height=720,
    )


if __name__ == '__main__':
    main()
