#!/usr/bin/env python3
"""
hap_ip_map.py

HAP 番号 ↔ LiDAR IP のマッピングを YAML から読み込む共通ヘルパー。
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Union

import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_IP_MAP_PATH = SCRIPT_DIR / "data" / "input_data" / "hap_ip_map.yaml"


def ip_to_topic(ip: str) -> str:
    """LiDAR IP から multi_topic=1 時の PointCloud2 トピック名を生成する。"""
    return f"/livox/lidar_{ip.replace('.', '_')}"


def load_hap_num_to_ip(
    path: Optional[Union[str, Path]] = None,
) -> Dict[int, str]:
    """
    hap_ip_map.yaml を読み込み、{HAP番号: IP} 辞書を返す。

    Parameters
    ----------
    path :
        YAML パス。None なら DEFAULT_IP_MAP_PATH。

    Raises
    ------
    FileNotFoundError
        YAML が存在しないとき
    ValueError
        形式が不正なとき
    """
    map_path = Path(path).expanduser() if path else DEFAULT_IP_MAP_PATH
    if not map_path.is_file():
        raise FileNotFoundError(f"HAP IP マップが見つかりません: {map_path}")

    with open(map_path) as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or "hap_num_to_ip" not in data:
        raise ValueError(
            f"YAML に hap_num_to_ip がありません: {map_path}"
        )

    raw = data["hap_num_to_ip"]
    if not isinstance(raw, dict) or not raw:
        raise ValueError(
            f"hap_num_to_ip が空または不正です: {map_path}"
        )

    result: Dict[int, str] = {}
    for key, value in raw.items():
        try:
            hap_num = int(key)
        except (TypeError, ValueError) as e:
            raise ValueError(
                f"HAP 番号が整数ではありません: {key!r} ({map_path})"
            ) from e
        if not isinstance(value, str) or not value.strip():
            raise ValueError(
                f"HAP{hap_num} の IP が不正です: {value!r} ({map_path})"
            )
        result[hap_num] = value.strip()

    return result


def resolve_ip(
    hap_num: int,
    hap_num_to_ip: Optional[Dict[int, str]] = None,
    path: Optional[Union[str, Path]] = None,
) -> str:
    """HAP 番号に対応する IP を返す。未登録なら KeyError。"""
    if hap_num_to_ip is None:
        hap_num_to_ip = load_hap_num_to_ip(path)
    if hap_num not in hap_num_to_ip:
        raise KeyError(
            f"HAP番号 {hap_num} の IP マッピングがありません。"
            " hap_ip_map.yaml の hap_num_to_ip に追加してください。"
        )
    return hap_num_to_ip[hap_num]


def resolve_topic(
    hap_num: int,
    hap_num_to_ip: Optional[Dict[int, str]] = None,
    path: Optional[Union[str, Path]] = None,
) -> str:
    """HAP 番号に対応する PointCloud2 トピックを返す。"""
    return ip_to_topic(resolve_ip(hap_num, hap_num_to_ip, path))
