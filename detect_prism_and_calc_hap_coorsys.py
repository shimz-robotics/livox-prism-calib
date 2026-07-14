#!/usr/bin/env python3
"""
detect_prism_and_calc_hap_coorsys.py

LiDAR（HAP）点群からプリズムを検出し、TS計測値との対応づけにより
LiDAR座標系の位置・姿勢を求める。

使い方:
  python3 detect_prism_and_calc_hap_coorsys.py [--hap-num N] [--data-folder PATH] [--config PATH]

オプション:
  --hap-num     N     処理する HAP 番号（デフォルト: 101）
  --data-folder PATH  データフォルダのパス（デフォルト: ./data）
  --config      PATH  検出パラメータ YAML（デフォルト: data/input_data/detect_prism_params.yaml）

入力（data-folder 以下）:
  input_data/prism_pos_<N>.csv       : TS で取得した3つのプリズム位置 [m]（HAP番号 N に対応）
  input_data/hap<N>.csv            : HAP 点群（lidar_to_csv.py 出力: x,y,z,intensity,tag）
                                 旧形式（index 付き 8列）も読み込み可
  input_data/detect_prism_params.yaml: 検出パラメータ（diff_distance, cluster_radius, tolerance, arm_max）

出力（data-folder 以下）:
  output_data/hapXXX_coorsys_py.yaml       : LiDAR 位置・姿勢

Original: detectPrismAndCalcHapCoorsys.m
"""

import os
import argparse
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.distance import cdist
from scipy.ndimage import binary_dilation
from scipy.spatial.transform import Rotation

from hap_csv_io import load_hap_csv


# ============================================================
# デフォルト値
# ============================================================
SCRIPT_DIR          = Path(__file__).resolve().parent
DEFAULT_HAP_NUM     = 101
DEFAULT_DATA_FOLDER = './data'
DEFAULT_CONFIG      = SCRIPT_DIR / 'data' / 'input_data' / 'detect_prism_params.yaml'

DEFAULT_PARAMS = {
    'diff_distance': 0.05,
    'cluster_radius': 0.075,
    'tolerance':     0.075,
    'arm_max':       1024,
}


def load_params(config_path=None):
    """検出パラメータ YAML を読み込む。ファイルが無い場合はデフォルト値を返す。"""
    path = Path(config_path).expanduser() if config_path else DEFAULT_CONFIG
    params = DEFAULT_PARAMS.copy()
    if path.is_file():
        with open(path) as f:
            loaded = yaml.safe_load(f) or {}
        params.update(loaded)
    else:
        print(f'WARNING: 設定ファイルが見つかりません。デフォルト値を使用: {path}')
    return params


# ============================================================
# ヘルパー関数
# ============================================================

def is_isosceles_triangle(points, diff_distance):
    """
    3点が二等辺三角形かどうかを判定する。
    最小辺長差が diff_distance 未満の場合 True を返す。
    （対応づけには3辺の長さが異なる必要があるため、True の場合は処理中断）
    """
    p1, p2, p3 = points[0], points[1], points[2]
    r1 = np.linalg.norm(p2 - p1)
    r2 = np.linalg.norm(p3 - p2)
    r3 = np.linalg.norm(p1 - p3)
    d1, d2, d3 = abs(r2 - r1), abs(r3 - r2), abs(r1 - r3)
    return min(d1, d2, d3) < diff_distance


def crop_neighbor_points(center_point, xyz, r):
    """
    center_point の周囲 r[m] 以内の点を抽出し、残りの点を返す。

    Returns
    -------
    cropped  : ndarray  抽出された点群
    rest     : ndarray  残りの点群
    has_rest : bool     残りの点があるか
    """
    cx, cy, cz = center_point
    mask = (
        (xyz[:, 0] > cx - r) & (xyz[:, 0] < cx + r) &
        (xyz[:, 1] > cy - r) & (xyz[:, 1] < cy + r) &
        (xyz[:, 2] > cz - r) & (xyz[:, 2] < cz + r)
    )
    cropped  = xyz[mask]
    rest     = xyz[~mask]
    has_rest = len(rest) > 0
    return cropped, rest, has_rest


def points2arm_ls_mtx(points):
    """点群の点間距離行列を計算する"""
    return cdist(points, points)


def arm_ls_mtx2arm_l01(arm_ls_mtx, m2arm, arm_max):
    """
    点間距離行列からアーム長インデックス01ベクトルを作成する。
    各行: その点が持つアーム長インデックスに 1 を立てたバイナリベクトル
    """
    N = arm_ls_mtx.shape[0]
    arm_ls_idx_mtx = np.ceil(arm_ls_mtx * m2arm).astype(int)

    # MATLABのトリック: 行列を横に2倍にして巡回的に扱う
    arm_ls_idx_mtx2 = np.hstack([arm_ls_idx_mtx, arm_ls_idx_mtx])

    arm_l01 = np.zeros((N, arm_max), dtype=np.int32)
    for i in range(N):
        tmp_indexes = arm_ls_idx_mtx2[i, i + 1:i + N]
        tmp_indexes = tmp_indexes[(tmp_indexes > 0) & (tmp_indexes < arm_max)]
        arm_l01[i, tmp_indexes] = 1
    return arm_l01


def select_mch_candidate(match_arm_cnt_list):
    """
    照合アーム数リストから照合候補を抽出する。
    各基準点に対して一致アーム数最大の対象点インデックスの組み合わせを求める。
    """
    N = match_arm_cnt_list.shape[1]
    index1, index2 = [], []
    n_can = 0
    match_arm_cnt_max = match_arm_cnt_list.max(axis=0)

    for i in range(N):
        if match_arm_cnt_max[i] != 0:
            n_can += 1
            match_idx2 = np.where(match_arm_cnt_list[:, i] == match_arm_cnt_max[i])[0]
            for idx in match_idx2:
                index2.append(idx)
                index1.append(i)

    return np.array(index1, dtype=int), np.array(index2, dtype=int), n_can


def get_mch_constellation(mch_arm_ls_mtx, index1, index2):
    """
    2点間照合行列から長さ分布がマッチする組み合わせを求める。
    アーム長一致数の少ない候補を順次除去して最良マッチを探す。

    Returns
    -------
    index_out1, index_out2 : 合致した点群インデックス
    flag : 1=成功, -1=失敗
    """
    flag       = -1
    index_out1 = index1.copy()
    index_out2 = index2.copy()
    n_mch      = mch_arm_ls_mtx.shape[1]

    while True:
        mch_cnt_ary = mch_arm_ls_mtx.sum(axis=1)
        sort_index  = np.argsort(mch_cnt_ary, kind='stable')
        sort_value  = mch_cnt_ary[sort_index]

        min_value   = sort_value[0]
        nxt_min_idx = 0
        for n in range(1, n_mch):
            if sort_value[n] != min_value:
                nxt_min_idx = n
                break

        if nxt_min_idx == 0:
            if min_value == n_mch - 1:
                flag = 1
            break
        elif nxt_min_idx > n_mch - 2:
            break

        fine_index     = sort_index[nxt_min_idx:]
        index_out1     = index_out1[fine_index]
        index_out2     = index_out2[fine_index]
        mch_arm_ls_mtx = mch_arm_ls_mtx[np.ix_(fine_index, fine_index)]
        n_mch          = n_mch - nxt_min_idx

    return index_out1, index_out2, flag


def matching2constellation(p1, p2, tolerance, arm_max=1024):
    """
    2点群を点群配置（形状）により照合して対応づける。

    Parameters
    ----------
    p1        : ndarray (N, 3)  基準点群（TS計測値）
    p2        : ndarray (M, 3)  対応づける点群（LiDAR検出値）
    tolerance : float           長さ一致許容誤差 [m]
    arm_max   : int             アーム長インデックス行列の最大次元

    Returns
    -------
    p1_out, p2_out : 対応づけられた点群
    flag : 1=成功, 0=失敗
    """
    flag   = 0
    p1_out = p1.copy()
    p2_out = p2.copy()

    arm2m = tolerance / 3.0
    m2arm = 1.0 / arm2m

    arm_ls_mtx1 = points2arm_ls_mtx(p1)
    arm_ls_mtx2 = points2arm_ls_mtx(p2)
    arm_l01_p1  = arm_ls_mtx2arm_l01(arm_ls_mtx1, m2arm, arm_max)
    arm_l01_p2  = arm_ls_mtx2arm_l01(arm_ls_mtx2, m2arm, arm_max)

    # 1×3 の構造要素でアーム長インデックスを膨張（許容誤差の適用）
    # MATLAB: imdilate(armL01_p2, strel("rectangle",[1,3]))
    # → 2D行列全体に対して 1×3 のstrelで膨張（列方向のみ拡張）
    se_err             = np.ones((1, 3), dtype=bool)
    arm_l01_p2_dilated = binary_dilation(
        arm_l01_p2.astype(bool), structure=se_err
    ).astype(np.int32)

    match_arm_cnt_list = arm_l01_p2_dilated @ arm_l01_p1.T

    if match_arm_cnt_list.sum() == 0:
        return p1_out, p2_out, flag

    index1, index2, n_can = select_mch_candidate(match_arm_cnt_list)

    if n_can > 2 and len(index1) > 0:
        arm_ls_mtx1_sub = arm_ls_mtx1[np.ix_(index1, index1)]
        arm_ls_mtx2_sub = arm_ls_mtx2[np.ix_(index2, index2)]
        zero_mask       = (arm_ls_mtx1_sub != 0)

        mch_arm_ls_mtx  = (
            (3 * arm2m > np.abs(arm_ls_mtx2_sub - arm_ls_mtx1_sub)) & zero_mask
        ).astype(float)

        idx1_out, idx2_out, flag = get_mch_constellation(
            mch_arm_ls_mtx, index1, index2
        )
        if flag == 1:
            p1_out = p1[idx1_out]
            p2_out = p2[idx2_out]

    return p1_out, p2_out, flag


def rigid_body_transform_mtx(p, q):
    """SVD法で q→p の剛体変換行列を求める"""
    p, q     = np.array(p, dtype=float), np.array(q, dtype=float)
    q_mean   = q.mean(axis=0)
    p_mean   = p.mean(axis=0)
    C        = (q - q_mean).T @ (p - p_mean)
    U, _, Vt = np.linalg.svd(C)
    diag     = np.ones(3)
    diag[-1] = np.sign(np.linalg.det(U @ Vt))
    R        = Vt.T @ np.diag(diag) @ U.T
    t        = p_mean - R @ q_mean
    H        = np.eye(4)
    H[:3, :3] = R
    H[:3, 3]  = t
    return H


# ============================================================
# メイン処理
# ============================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description='LiDAR（HAP）点群からプリズムを検出し、TS座標系でのLiDAR位置・姿勢を求める'
    )
    parser.add_argument(
        '--hap-num', '-n',
        type=int,
        default=DEFAULT_HAP_NUM,
        metavar='N',
        help=f'処理する HAP 番号（デフォルト: {DEFAULT_HAP_NUM}）'
    )
    parser.add_argument(
        '--data-folder', '-d',
        type=str,
        default=DEFAULT_DATA_FOLDER,
        metavar='PATH',
        help=f'データフォルダのパス（デフォルト: {DEFAULT_DATA_FOLDER}）'
    )
    parser.add_argument(
        '--config', '-c',
        type=str,
        default=None,
        metavar='PATH',
        help=f'検出パラメータ YAML（デフォルト: {DEFAULT_CONFIG}）'
    )
    return parser.parse_args()


def main():
    args        = parse_args()
    hap_num     = args.hap_num
    data_folder = args.data_folder
    params      = load_params(args.config)

    print(f"HAP番号     : {hap_num}")
    print(f"データフォルダ: {data_folder}")
    print(f"検出パラメータ: diff_distance={params['diff_distance']}, "
          f"cluster_radius={params['cluster_radius']}, "
          f"tolerance={params['tolerance']}, arm_max={params['arm_max']}")

    # ----------------------------------------------------------------
    # TSで測量したプリズム位置を読み込む
    # ----------------------------------------------------------------
    prism_file = os.path.join(data_folder, 'input_data', f'prism_pos_{hap_num}.csv')
    if not os.path.isfile(prism_file):
        print(f'ERROR: TSプリズム位置ファイルが見つかりません: {prism_file}')
        return
    ts_points = np.loadtxt(prism_file, delimiter=',')[:3]
    print(f"TSプリズム位置 [m] ({prism_file}):\n{ts_points}")

    if is_isosceles_triangle(ts_points, params['diff_distance']):
        print('ERROR: 2辺の長さが近似しています（二等辺三角形）。処理を中止します。')
        return

    # ----------------------------------------------------------------
    # HAP点群データを読み込む
    # ----------------------------------------------------------------
    hap_file = os.path.join(data_folder, 'input_data', f'hap{hap_num}.csv')
    print(f"\n点群ファイル読み込み中: {hap_file}")
    xyz, intensity, tag = load_hap_csv(hap_file)
    print(f"点群総数: {len(xyz)}")

    # ----------------------------------------------------------------
    # 高輝度点の抽出 → クラスタリング → TS点群との対応づけ
    # ----------------------------------------------------------------
    flag1              = True
    i                  = -1
    ts_points_matched  = None
    hap_points_matched = None

    while flag1:
        i                   += 1
        intensity_threshold  = 255 - i

        if intensity_threshold < 100:
            print('適切なプリズムが見つかりませんでした。')
            return

        mask      = (intensity >= intensity_threshold) & (tag == 64)
        xyz_prism = xyz[mask]
        print(f"IntensityThreshold = {intensity_threshold}, 高輝度点数 = {len(xyz_prism)}")

        if len(xyz_prism) == 0:
            continue

        # cluster_radius 内の近傍点をクラスタリングして各プリズム点群を抽出
        cluster_radius = params['cluster_radius']
        prism_clusters = []
        xyz_rest       = xyz_prism.copy()

        while len(xyz_rest) > 0:
            center                      = xyz_rest[0]
            cropped, xyz_rest, has_rest = crop_neighbor_points(
                center, xyz_rest, cluster_radius
            )
            if len(cropped) > 0:
                prism_clusters.append(cropped)
            if not has_rest:
                break

        cnt = len(prism_clusters)
        if cnt < 3:
            continue

        # 各クラスタの重心を算出（LiDAR座標系でのプリズム位置）
        center_prism = np.array([c.mean(axis=0) for c in prism_clusters])
        hap_points   = center_prism

        # TS点群とLiDAR点群を形状マッチングで対応づけ
        ts_m, hap_m, flag = matching2constellation(
            ts_points, hap_points,
            params['tolerance'], arm_max=params['arm_max']
        )

        if flag == 1:
            ts_points_matched  = ts_m
            hap_points_matched = hap_m
            flag1              = False
            print(f"対応づけ成功（threshold={intensity_threshold}）")

    if ts_points_matched is None:
        print('対応づけに失敗しました。')
        return

    # ----------------------------------------------------------------
    # 剛体変換行列の計算（TS座標系からみたLiDAR座標系の位置・姿勢）
    # ----------------------------------------------------------------
    ts_H_hap         = rigid_body_transform_mtx(ts_points_matched, hap_points_matched)
    t                = ts_H_hap[:3, 3]
    R                = ts_H_hap[:3, :3]
    yaw, pitch, roll = Rotation.from_matrix(R).as_euler('ZYX', degrees=True)

    print(f"\n--- 結果 ---")
    print(f"roll : {roll:7.4f} [deg]")
    print(f"pitch: {pitch:7.4f} [deg]")
    print(f"yaw  : {yaw:7.4f} [deg]")
    print(f"x    : {int(round(t[0]*1000))} [mm]")
    print(f"y    : {int(round(t[1]*1000))} [mm]")
    print(f"z    : {int(round(t[2]*1000))} [mm]")

    # ----------------------------------------------------------------
    # 結果確認（変換誤差チェック）
    # ----------------------------------------------------------------
    hap_ex  = np.hstack([hap_points_matched, np.ones((3, 1))])
    ts_calc = (ts_H_hap @ hap_ex.T).T[:, :3]
    err     = np.linalg.norm(ts_calc - ts_points_matched, axis=1)
    print(f"変換誤差（各プリズム）: {np.round(err * 1000, 2)} [mm]")

    # ----------------------------------------------------------------
    # YAML出力
    # ----------------------------------------------------------------
    os.makedirs(os.path.join(data_folder, 'output_data'), exist_ok=True)
    yaml_out = os.path.join(data_folder, 'output_data', f'hap{hap_num}_coorsys_py.yaml')
    data_out = {
        'Position': {
            'x': int(round(t[0] * 1000)),
            'y': int(round(t[1] * 1000)),
            'z': int(round(t[2] * 1000)),
        },
        'Rotation': {
            'roll':  float(roll),
            'pitch': float(pitch),
            'yaw':   float(yaw),
        }
    }
    with open(yaml_out, 'w') as f:
        yaml.dump(data_out, f, default_flow_style=False, allow_unicode=True)
    print(f"\nYAML出力完了: {yaml_out}")

    # ----------------------------------------------------------------
    # 期待値との比較
    # ----------------------------------------------------------------
    ref_yaml = os.path.join(data_folder, 'output_data', f'hap{hap_num}_coorsys.yaml')
    if os.path.exists(ref_yaml):
        with open(ref_yaml) as f:
            ref = yaml.safe_load(f)
        print(f"\n--- MATLABとの比較 ---")
        print(f"{'':8s} {'Python':>10s} {'MATLAB':>10s}")
        print(f"{'x[mm]':8s} {data_out['Position']['x']:>10d} {ref['Position']['x']:>10d}")
        print(f"{'y[mm]':8s} {data_out['Position']['y']:>10d} {ref['Position']['y']:>10d}")
        print(f"{'z[mm]':8s} {data_out['Position']['z']:>10d} {ref['Position']['z']:>10d}")
        print(f"{'roll':8s} {data_out['Rotation']['roll']:>10.4f} {ref['Rotation']['roll']:>10.4f}")
        print(f"{'pitch':8s} {data_out['Rotation']['pitch']:>10.4f} {ref['Rotation']['pitch']:>10.4f}")
        print(f"{'yaw':8s} {data_out['Rotation']['yaw']:>10.4f} {ref['Rotation']['yaw']:>10.4f}")


if __name__ == '__main__':
    main()
