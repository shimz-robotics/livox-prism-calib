# livox-prism-calib

Livox HAP をプリズム（TS）計測値でキャリブレーションする Python ツール群です。

## 構成

| ファイル | 役割 |
|---------|------|
| `lidar_to_csv.py` | ROS 2 点群 → CSV 記録 |
| `detect_prism_and_calc_hap_coorsys.py` | プリズム検出・LiDARの位置姿勢算出 |
| `show_multi_hap_point_cloud.py` | 複数 HAP の TS 座標系可視化 |
| `update_hap_config_from_coorsys.py` | 結果を `HAP_config.json` に反映 |
| `hap_ip_map.py` / `hap_csv_io.py` | 共通ユーティリティ |
| `data/HAP_config.json` | 現場設定マスター（キャリブ結果の反映先、ドライバへ配備） |
| `data/input_data/` | 入力（パラメータ・プリズム位置・点群 CSV） |
| `data/output_data/` | キャリブ結果 YAML |
| `docs/livox_calib_manual.md` | 手順マニュアル |
| `docs/calib_quickstart.md` | 現場向けクイック手順 |

## セットアップ

```bash
pip install -r requirements.txt
```

点群記録・設定反映には ROS 2 と livox_ros_driver2 のワークスペースが必要です。  
未構築の場合は `./scripts/setup_ros2_ws.sh` で構築できます（sudo 不要、デフォルトはリポジトリ内 `./ros2_livox_ws/`・Git 管理外。`~/ros2_livox_ws` など任意の場所も引数で指定可）。

詳細は [docs/livox_calib_manual.md](docs/livox_calib_manual.md) を参照してください。  
現場向けの短縮手順は [docs/calib_quickstart.md](docs/calib_quickstart.md) です。

## クイックスタート

```bash
cd /path/to/livox-prism-calib

# 1. 点群記録（ROS 2 環境を sourced した状態で）
python3 lidar_to_csv.py -n 123

# 2. キャリブ
python3 detect_prism_and_calc_hap_coorsys.py -n 123 -d ./data

# 3. HAP_config.json へ反映
python3 update_hap_config_from_coorsys.py -n 123
```

デフォルトのデータフォルダはリポジトリ内の `./data` です。  
`update_hap_config_from_coorsys.py` は本リポジトリ管理のマスター `data/HAP_config.json` を更新し、ワークスペース内のドライバ config（src 側と、実行時に読まれる install 側の実体コピー）へ配備します。  
ワークスペースは環境変数 `LIVOX_WS` → リポジトリ内 `./ros2_livox_ws` → `~/ros2_livox_ws` の順で自動解決されます（`--hap-config` で配備先の直接指定、`--no-deploy` でマスターのみ更新も可）。

## 注意

`data/input_data/hap*.csv`（点群）はサイズが大きいため Git 管理対象外です。現場データはローカルに置いてください。
