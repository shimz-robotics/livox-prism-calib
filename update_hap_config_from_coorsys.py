#!/usr/bin/env python3
"""
update_hap_config_from_coorsys.py

hap<N>_coorsys_py.yaml の位置・姿勢を、本リポジトリ管理のマスター
data/input_data/HAP_config.json の extrinsic_parameter に反映し、
ドライバのワークスペース config（src 側・install 側）へ配備コピーする。

使い方:
  python3 update_hap_config_from_coorsys.py [--hap-num N ...] [--data-folder PATH]

オプション:
  --hap-num N ...     対象 HAP 番号（複数指定可、デフォルト: 101 102）
  --data-folder PATH  キャリブ YAML の親フォルダ（output_data を含む）
  --master PATH       マスター HAP_config.json（デフォルト: data/input_data/HAP_config.json）
  --hap-config PATH   配備先ドライバ config（src 側。install 側は自動導出）
  --no-deploy         マスターのみ更新し、ドライバ config へ配備しない
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
from typing import Dict, Iterable, Optional, Sequence, Union

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
DEFAULT_MASTER_CONFIG_PATH = SCRIPT_DIR / "data/input_data/HAP_config.json"


def derive_install_config_path(config_path: Union[str, Path]) -> Optional[Path]:
    """src 側 config パスから、ドライバが実行時に読む install 側のパスを導出する。

    通常の colcon build では install 側は実体コピーのため、src 側だけ更新しても
    実行時に反映されない。install 側が存在しない場合は None を返す。
    --symlink-install の場合は src 側と同一実体の symlink パスが返る。
    """
    config_path = Path(config_path).expanduser().resolve()
    parts = config_path.parts
    if len(parts) < 4 or parts[-4:-1] != ("src", "livox_ros_driver2", "config"):
        return None
    ws_dir = Path(*parts[:-4])
    install_path = (
        ws_dir / "install/livox_ros_driver2/share/livox_ros_driver2/config" / parts[-1]
    )
    if not install_path.is_file():
        return None
    return install_path


def resolve_deploy_paths(hap_config_path: Union[str, Path]) -> list[Path]:
    """配備先（ワークスペースの src 側・install 側 config）を解決する。

    src 側が存在しなければ空リスト（ワークスペース未構築）。
    install 側が src 側と同一実体の場合（--symlink-install）は src 側のみ。
    """
    src = Path(hap_config_path).expanduser()
    if not src.is_file():
        return []
    src = src.resolve()
    paths = [src]
    install = derive_install_config_path(src)
    if install is not None and install.resolve() != src:
        paths.append(install)
    return paths


def check_ip_map_consistency(
    config_path: Union[str, Path], hap_num_to_ip: Dict[int, str]
) -> list[str]:
    """マスター config と hap_ip_map.yaml の IP の不一致を警告文リストで返す。"""
    with open(Path(config_path).expanduser()) as f:
        cfg = json.load(f)
    config_ips = {e.get("ip") for e in cfg.get("lidar_configs", [])}
    map_ips = set(hap_num_to_ip.values())
    warnings = []
    only_in_map = sorted(map_ips - config_ips)
    only_in_config = sorted(config_ips - map_ips)
    if only_in_map:
        warnings.append(
            f"hap_ip_map.yaml にあるが HAP_config.json にない IP: {only_in_map}"
        )
    if only_in_config:
        warnings.append(
            f"HAP_config.json にあるが hap_ip_map.yaml にない IP: {only_in_config}"
        )
    return warnings


def deploy_master_config(
    master_path: Union[str, Path],
    deploy_paths: Sequence[Union[str, Path]],
    backup: bool = True,
) -> None:
    """マスター config をドライバの config へ配備（丸ごとコピー）する。"""
    master_path = Path(master_path).expanduser().resolve()
    for path in deploy_paths:
        path = Path(path).expanduser().resolve()
        if backup and path.is_file():
            shutil.copy2(path, str(path) + ".bak")
        shutil.copy2(master_path, path)
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


def _print_target_summary(
    master_path: Union[str, Path],
    deploy_paths: Sequence[Union[str, Path]],
) -> None:
    """マスター・配備先の一覧を表示する。"""
    print(f"マスター: {Path(master_path).expanduser().resolve()}")
    if deploy_paths:
        for path in deploy_paths:
            print(f"配備先: {Path(path).expanduser().resolve()}")
    else:
        print("配備先: なし（ワークスペース未検出、または --no-deploy）")


def print_update_preview(
    master_path: Union[str, Path],
    deploy_paths: Sequence[Union[str, Path]],
    hap_nums: Iterable[int],
    hap_num_to_yaml: Dict[int, Union[str, Path]],
) -> None:
    """更新内容のプレビューを表示する。"""
    print("\n--- HAP_config.json 更新プレビュー ---")
    _print_target_summary(master_path, deploy_paths)
    for hap_num in hap_nums:
        print(format_extrinsic_summary(hap_num, hap_num_to_yaml[hap_num]))


def print_reset_preview(
    master_path: Union[str, Path],
    deploy_paths: Sequence[Union[str, Path]],
    hap_nums: Iterable[int],
) -> None:
    """リセット内容のプレビューを表示する。"""
    print("\n--- HAP_config.json リセットプレビュー ---")
    _print_target_summary(master_path, deploy_paths)
    for hap_num in hap_nums:
        print(format_zero_extrinsic_summary(hap_num))


def print_reflect_hint(deploy_paths: Sequence[Union[str, Path]]) -> None:
    """更新結果を実行時に反映するために必要な操作を案内する。"""
    if not deploy_paths:
        print(
            "警告: ドライバの config へ配備していないため、実行時には反映されません。\n"
            "ワークスペース構築後（scripts/setup_ros2_ws.sh 参照）に再実行してください。"
        )
        return
    install_covered = any(
        "install" in Path(p).resolve().parts
        or derive_install_config_path(p) is not None
        for p in deploy_paths
    )
    if install_covered:
        print("反映には livox_ros_driver2 の再起動が必要です。")
    else:
        print(
            "警告: ドライバが実行時に読む install 側の config が見つからないため、"
            "src 側のみに配備しました。\n"
            "点群への反映にはワークスペースのリビルドが必要です。"
        )


def prompt_and_update_hap_config(
    master_path: Union[str, Path],
    deploy_paths: Sequence[Union[str, Path]],
    hap_nums: Iterable[int],
    hap_num_to_yaml: Dict[int, Union[str, Path]],
    hap_num_to_ip: Dict[int, str],
    auto_yes: bool = False,
    backup: bool = True,
) -> bool:
    """
    確認プロンプトのあとマスターを更新し、ドライバの config へ配備する。

    Returns
    -------
    bool
        更新した場合 True
    """
    master_path = Path(master_path).expanduser().resolve()
    print_update_preview(master_path, deploy_paths, hap_nums, hap_num_to_yaml)

    if not auto_yes:
        answer = input(
            "\n上記の extrinsic_parameter で HAP_config.json を更新しますか? [y/N]: "
        ).strip().lower()
        if answer not in ("y", "yes"):
            print("更新をキャンセルしました。")
            return False

    print()
    updated = update_hap_config_extrinsics(
        master_path, hap_num_to_yaml, hap_num_to_ip, backup=backup
    )
    print(f"{master_path} を更新しました（対象 IP: {', '.join(updated)}）")
    if backup:
        print(f"バックアップ: {master_path}.bak")
    deploy_master_config(master_path, deploy_paths, backup=backup)
    for path in deploy_paths:
        print(f"配備しました: {path}")
    print_reflect_hint(deploy_paths)
    return True


def prompt_and_reset_hap_config(
    master_path: Union[str, Path],
    deploy_paths: Sequence[Union[str, Path]],
    hap_nums: Iterable[int],
    hap_num_to_ip: Dict[int, str],
    auto_yes: bool = False,
    backup: bool = True,
) -> bool:
    """
    確認プロンプトのあと、マスターの指定 HAP をゼロにリセットして配備する。

    Returns
    -------
    bool
        更新した場合 True
    """
    master_path = Path(master_path).expanduser().resolve()
    print_reset_preview(master_path, deploy_paths, hap_nums)

    if not auto_yes:
        answer = input(
            "\n上記の extrinsic_parameter で HAP_config.json をリセットしますか? [y/N]: "
        ).strip().lower()
        if answer not in ("y", "yes"):
            print("リセットをキャンセルしました。")
            return False

    print()
    updated = reset_hap_config_extrinsics(
        master_path, hap_nums, hap_num_to_ip, backup=backup
    )
    print(f"{master_path} をリセットしました（対象 IP: {', '.join(updated)}）")
    if backup:
        print(f"バックアップ: {master_path}.bak")
    deploy_master_config(master_path, deploy_paths, backup=backup)
    for path in deploy_paths:
        print(f"配備しました: {path}")
    print_reflect_hint(deploy_paths)
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
        "--master",
        type=str,
        default=str(DEFAULT_MASTER_CONFIG_PATH),
        metavar="PATH",
        help=f"マスター HAP_config.json（デフォルト: {DEFAULT_MASTER_CONFIG_PATH}）",
    )
    parser.add_argument(
        "--hap-config",
        type=str,
        default=str(DEFAULT_HAP_CONFIG_PATH),
        metavar="PATH",
        help="配備先ドライバ config（src 側。install 側は自動導出）",
    )
    parser.add_argument(
        "--no-deploy",
        action="store_true",
        help="マスターのみ更新し、ドライバ config へ配備しない",
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
    master_path = Path(args.master).expanduser()

    if not master_path.is_file():
        print(f"マスター HAP_config.json が見つかりません: {master_path}", file=sys.stderr)
        return 1

    try:
        hap_num_to_ip = load_hap_num_to_ip(args.ip_map)
    except (FileNotFoundError, ValueError) as e:
        print(e, file=sys.stderr)
        return 1

    for warning in check_ip_map_consistency(master_path, hap_num_to_ip):
        print(f"警告: {warning}", file=sys.stderr)

    deploy_paths = [] if args.no_deploy else resolve_deploy_paths(args.hap_config)

    if args.reset:
        if args.dry_run:
            print_reset_preview(master_path, deploy_paths, hap_nums)
            print("\n（--dry-run のためファイルは更新しませんでした）")
            return 0

        try:
            prompt_and_reset_hap_config(
                master_path=master_path,
                deploy_paths=deploy_paths,
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
        print_update_preview(master_path, deploy_paths, hap_nums, hap_num_to_yaml)
        print("\n（--dry-run のためファイルは更新しませんでした）")
        return 0

    try:
        prompt_and_update_hap_config(
            master_path=master_path,
            deploy_paths=deploy_paths,
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
