#!/usr/bin/env python3
"""
update_hap_config_from_coorsys.py

hap<N>_coorsys_py.yaml の位置・姿勢を HAP_config.json の extrinsic_parameter に反映する。

使い方:
  python3 update_hap_config_from_coorsys.py [--hap-num N ...] [--data-folder PATH]

オプション:
  --hap-num N ...     対象 HAP 番号（複数指定可、デフォルト: 101 102）
  --data-folder PATH  キャリブ YAML の親フォルダ（output_data を含む）
  --hap-config PATH   更新先 HAP_config.json
  --ip-map PATH       HAP番号→IP マップ YAML（デフォルト: data/input_data/hap_ip_map.yaml）
  --yes               確認プロンプトをスキップして更新
  --no-backup         更新前の .bak を作成しない
  --dry-run           プレビューのみ（ファイルは更新しない）
  --reset             指定 HAP の extrinsic_parameter をゼロにリセット（YAML 不要）
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Dict, Iterable, Union

import yaml

from hap_ip_map import DEFAULT_IP_MAP_PATH, load_hap_num_to_ip

SCRIPT_DIR = Path(__file__).resolve().parent


def _default_ws_dir() -> Path:
    """ROS 2 ワークスペースの場所を解決する。

    優先順: 環境変数 LIVOX_WS → リポジトリ内 ros2_livox_ws → ~/ros2_livox_ws
    """
    env = os.environ.get("LIVOX_WS")
    if env:
        return Path(env).expanduser()
    local_ws = SCRIPT_DIR / "ros2_livox_ws"
    if local_ws.is_dir():
        return local_ws
    return Path.home() / "ros2_livox_ws"


DEFAULT_HAP_CONFIG_PATH = _default_ws_dir() / "src/livox_ros_driver2/config/HAP_config.json"
DEFAULT_DATA_FOLDER = SCRIPT_DIR / "data"
DEFAULT_HAP_NUMS = (101, 102)

# Livox ドライバのデフォルト extrinsic（README 準拠）
ZERO_EXTRINSIC = {
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "x": 0,
    "y": 0,
    "z": 0,
}


def coorsys_yaml_path(data_folder: Union[str, Path], hap_num: int) -> Path:
    """data-folder/output_data/hap<N>_coorsys_py.yaml のパスを返す。"""
    return Path(data_folder).expanduser() / "output_data" / f"hap{hap_num}_coorsys_py.yaml"


def build_hap_num_to_yaml(
    data_folder: Union[str, Path], hap_nums: Iterable[int]
) -> Dict[int, Path]:
    """HAP 番号ごとのキャリブ YAML パス辞書を組み立てる。"""
    folder = Path(data_folder).expanduser()
    result = {}
    for hap_num in hap_nums:
        path = coorsys_yaml_path(folder, hap_num)
        if not path.is_file():
            raise FileNotFoundError(
                f"キャリブ結果が見つかりません: {path}\n"
                f"先に detect_prism_and_calc_hap_coorsys.py -n {hap_num} を実行してください。"
            )
        result[hap_num] = path
    return result


def coorsys_yaml_to_extrinsic(yaml_path: Union[str, Path]) -> dict:
    """
    hap<N>_coorsys_py.yaml を Livox extrinsic_parameter 形式に変換する。

    Returns
    -------
    dict
        roll, pitch, yaw [deg] (float), x, y, z [mm] (int)
    """
    with open(yaml_path) as f:
        d = yaml.safe_load(f)

    return {
        "roll": float(d["Rotation"]["roll"]),
        "pitch": float(d["Rotation"]["pitch"]),
        "yaw": float(d["Rotation"]["yaw"]),
        "x": int(d["Position"]["x"]),
        "y": int(d["Position"]["y"]),
        "z": int(d["Position"]["z"]),
    }


def hap_nums_to_extrinsic_by_ip(
    hap_nums: Iterable[int],
    extrinsic: dict,
    hap_num_to_ip: Dict[int, str],
) -> Dict[str, dict]:
    """HAP 番号リストから {IP: extrinsic} 辞書を組み立てる。"""
    extrinsic_by_ip = {}
    for hap_num in hap_nums:
        if hap_num not in hap_num_to_ip:
            raise KeyError(
                f"HAP番号 {hap_num} の IP マッピングがありません。"
                " hap_ip_map.yaml の hap_num_to_ip に追加してください。"
            )
        extrinsic_by_ip[hap_num_to_ip[hap_num]] = dict(extrinsic)
    return extrinsic_by_ip


def apply_extrinsics_to_config(
    config_path: Union[str, Path],
    extrinsic_by_ip: Dict[str, dict],
    backup: bool = True,
) -> list[str]:
    """
    HAP_config.json の lidar_configs[].extrinsic_parameter を更新する。

    Parameters
    ----------
    config_path : path to HAP_config.json
    extrinsic_by_ip : {lidar IP: extrinsic_parameter dict}
    backup : True なら .bak を作成

    Returns
    -------
    list[str]
        更新した IP のリスト
    """
    config_path = Path(config_path).expanduser().resolve()

    if backup:
        shutil.copy2(config_path, str(config_path) + ".bak")

    with open(config_path) as f:
        cfg = json.load(f)

    updated_ips = []
    for entry in cfg.get("lidar_configs", []):
        ip = entry.get("ip")
        if ip in extrinsic_by_ip:
            entry["extrinsic_parameter"] = extrinsic_by_ip[ip]
            updated_ips.append(ip)

    missing_ips = set(extrinsic_by_ip.keys()) - set(updated_ips)
    if missing_ips:
        raise ValueError(
            f"HAP_config.json に次の IP のエントリがありません: {sorted(missing_ips)}"
        )

    with open(config_path, "w") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")

    return updated_ips


def update_hap_config_extrinsics(
    config_path: Union[str, Path],
    hap_num_to_yaml: Dict[int, Union[str, Path]],
    hap_num_to_ip: Dict[int, str],
    backup: bool = True,
) -> list[str]:
    """キャリブ YAML に基づき extrinsic_parameter を更新する。"""
    extrinsic_by_ip = {}
    for hap_num, yaml_path in hap_num_to_yaml.items():
        if hap_num not in hap_num_to_ip:
            raise KeyError(
                f"HAP番号 {hap_num} の IP マッピングがありません。"
                " hap_ip_map.yaml の hap_num_to_ip に追加してください。"
            )
        extrinsic_by_ip[hap_num_to_ip[hap_num]] = coorsys_yaml_to_extrinsic(yaml_path)

    return apply_extrinsics_to_config(config_path, extrinsic_by_ip, backup=backup)


def reset_hap_config_extrinsics(
    config_path: Union[str, Path],
    hap_nums: Iterable[int],
    hap_num_to_ip: Dict[int, str],
    backup: bool = True,
) -> list[str]:
    """指定 HAP の extrinsic_parameter をゼロにリセットする。"""
    extrinsic_by_ip = hap_nums_to_extrinsic_by_ip(
        hap_nums, ZERO_EXTRINSIC, hap_num_to_ip
    )
    return apply_extrinsics_to_config(config_path, extrinsic_by_ip, backup=backup)


def format_extrinsic_summary(hap_num: int, yaml_path: Union[str, Path]) -> str:
    """更新内容の1行サマリ。"""
    e = coorsys_yaml_to_extrinsic(yaml_path)
    return (
        f"  HAP{hap_num}: roll={e['roll']:.4f}, pitch={e['pitch']:.4f}, "
        f"yaw={e['yaw']:.4f}, x={e['x']}, y={e['y']}, z={e['z']}"
    )


def format_zero_extrinsic_summary(hap_num: int) -> str:
    """リセット内容の1行サマリ。"""
    e = ZERO_EXTRINSIC
    return (
        f"  HAP{hap_num}: roll={e['roll']}, pitch={e['pitch']}, "
        f"yaw={e['yaw']}, x={e['x']}, y={e['y']}, z={e['z']}"
    )


def print_update_preview(
    config_path: Union[str, Path],
    hap_nums: Iterable[int],
    hap_num_to_yaml: Dict[int, Union[str, Path]],
) -> None:
    """更新内容のプレビューを表示する。"""
    config_path = Path(config_path).expanduser().resolve()
    print("\n--- HAP_config.json 更新プレビュー ---")
    print(f"対象ファイル: {config_path}")
    for hap_num in hap_nums:
        print(format_extrinsic_summary(hap_num, hap_num_to_yaml[hap_num]))


def print_reset_preview(
    config_path: Union[str, Path],
    hap_nums: Iterable[int],
) -> None:
    """リセット内容のプレビューを表示する。"""
    config_path = Path(config_path).expanduser().resolve()
    print("\n--- HAP_config.json リセットプレビュー ---")
    print(f"対象ファイル: {config_path}")
    for hap_num in hap_nums:
        print(format_zero_extrinsic_summary(hap_num))


def prompt_and_update_hap_config(
    config_path: Union[str, Path],
    hap_nums: Iterable[int],
    hap_num_to_yaml: Dict[int, Union[str, Path]],
    hap_num_to_ip: Dict[int, str],
    auto_yes: bool = False,
    backup: bool = True,
) -> bool:
    """
    確認プロンプトのあと HAP_config.json を更新する。

    Returns
    -------
    bool
        更新した場合 True
    """
    config_path = Path(config_path).expanduser().resolve()
    print_update_preview(config_path, hap_nums, hap_num_to_yaml)

    if not auto_yes:
        answer = input(
            "\n上記の extrinsic_parameter で HAP_config.json を更新しますか? [y/N]: "
        ).strip().lower()
        if answer not in ("y", "yes"):
            print("更新をキャンセルしました。")
            return False

    updated = update_hap_config_extrinsics(
        config_path, hap_num_to_yaml, hap_num_to_ip, backup=backup
    )
    print(f"\nHAP_config.json を更新しました（対象 IP: {', '.join(updated)}）")
    if backup:
        print(f"バックアップ: {config_path}.bak")
    print("反映には livox_ros_driver2 の再起動が必要です。")
    return True


def prompt_and_reset_hap_config(
    config_path: Union[str, Path],
    hap_nums: Iterable[int],
    hap_num_to_ip: Dict[int, str],
    auto_yes: bool = False,
    backup: bool = True,
) -> bool:
    """
    確認プロンプトのあと、指定 HAP の extrinsic_parameter をゼロにリセットする。

    Returns
    -------
    bool
        更新した場合 True
    """
    config_path = Path(config_path).expanduser().resolve()
    print_reset_preview(config_path, hap_nums)

    if not auto_yes:
        answer = input(
            "\n上記の extrinsic_parameter で HAP_config.json をリセットしますか? [y/N]: "
        ).strip().lower()
        if answer not in ("y", "yes"):
            print("リセットをキャンセルしました。")
            return False

    updated = reset_hap_config_extrinsics(
        config_path, hap_nums, hap_num_to_ip, backup=backup
    )
    print(f"\nHAP_config.json をリセットしました（対象 IP: {', '.join(updated)}）")
    if backup:
        print(f"バックアップ: {config_path}.bak")
    print("反映には livox_ros_driver2 の再起動が必要です。")
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="キャリブ YAML を HAP_config.json の extrinsic_parameter に反映する"
    )
    parser.add_argument(
        "--hap-num",
        "-n",
        type=int,
        nargs="+",
        default=list(DEFAULT_HAP_NUMS),
        metavar="N",
        help=f"対象 HAP 番号（複数可、デフォルト: {' '.join(map(str, DEFAULT_HAP_NUMS))}）",
    )
    parser.add_argument(
        "--data-folder",
        "-d",
        type=str,
        default=str(DEFAULT_DATA_FOLDER),
        metavar="PATH",
        help=f"データフォルダ（デフォルト: {DEFAULT_DATA_FOLDER}）",
    )
    parser.add_argument(
        "--hap-config",
        type=str,
        default=str(DEFAULT_HAP_CONFIG_PATH),
        metavar="PATH",
        help="更新先 HAP_config.json のパス",
    )
    parser.add_argument(
        "--ip-map",
        type=str,
        default=str(DEFAULT_IP_MAP_PATH),
        metavar="PATH",
        help=f"HAP番号→IP マップ YAML（デフォルト: {DEFAULT_IP_MAP_PATH}）",
    )
    parser.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="確認プロンプトをスキップして更新",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="更新前の .bak を作成しない",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="プレビューのみ表示（ファイルは更新しない）",
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="指定 HAP の extrinsic_parameter をゼロにリセット（YAML 不要）",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    hap_nums = args.hap_num
    config_path = Path(args.hap_config).expanduser()

    if not config_path.is_file():
        print(f"HAP_config.json が見つかりません: {config_path}", file=sys.stderr)
        return 1

    try:
        hap_num_to_ip = load_hap_num_to_ip(args.ip_map)
    except (FileNotFoundError, ValueError) as e:
        print(e, file=sys.stderr)
        return 1

    if args.reset:
        if args.dry_run:
            print_reset_preview(config_path, hap_nums)
            print("\n（--dry-run のためファイルは更新しませんでした）")
            return 0

        try:
            prompt_and_reset_hap_config(
                config_path=config_path,
                hap_nums=hap_nums,
                hap_num_to_ip=hap_num_to_ip,
                auto_yes=args.yes,
                backup=not args.no_backup,
            )
        except (KeyError, ValueError) as e:
            print(e, file=sys.stderr)
            return 1
        return 0

    try:
        hap_num_to_yaml = build_hap_num_to_yaml(args.data_folder, hap_nums)
    except FileNotFoundError as e:
        print(e, file=sys.stderr)
        return 1

    if args.dry_run:
        print_update_preview(config_path, hap_nums, hap_num_to_yaml)
        print("\n（--dry-run のためファイルは更新しませんでした）")
        return 0

    try:
        prompt_and_update_hap_config(
            config_path=config_path,
            hap_nums=hap_nums,
            hap_num_to_yaml=hap_num_to_yaml,
            hap_num_to_ip=hap_num_to_ip,
            auto_yes=args.yes,
            backup=not args.no_backup,
        )
    except (KeyError, ValueError) as e:
        print(e, file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
