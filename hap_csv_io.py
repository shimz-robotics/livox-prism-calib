"""
hap_csv_io.py

HAP 点群 CSV の読み書きフォーマットを統一する。

新形式 (lidar_to_csv.py 出力):
  x, y, z, intensity, tag  (5列、ヘッダなし)

旧形式 (従来):
  index, x, y, z, intensity, tag, line, timestamp  (8列)
"""

from pathlib import Path

import numpy as np


def load_hap_csv(csv_path):
    """
    HAP 点群 CSV を読み込む。

    Parameters
    ----------
    csv_path : str | Path

    Returns
    -------
    xyz : ndarray, shape (N, 3)
    intensity : ndarray, shape (N,)
    tag : ndarray, shape (N,)
    """
    raw = np.loadtxt(csv_path, delimiter=',')
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)

    ncols = raw.shape[1]
    if ncols >= 8:
        # 旧形式: index, x, y, z, intensity, tag, line, timestamp
        xyz = raw[:, 1:4]
        intensity = raw[:, 4]
        tag = raw[:, 5]
    elif ncols >= 5:
        # 新形式: x, y, z, intensity, tag
        xyz = raw[:, 0:3]
        intensity = raw[:, 3]
        tag = raw[:, 4]
    else:
        raise ValueError(
            f"Unsupported CSV format ({ncols} columns): {csv_path}. "
            "Expected 5 columns (x,y,z,intensity,tag) or 8+ columns (legacy)."
        )

    return xyz, intensity, tag
