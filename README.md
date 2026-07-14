# livox_calib

Livox HAP をプリズム（TS）計測値でキャリブレーションする Python ツール群です。

## 構成

| ファイル | 役割 |
|---------|------|
| `lidar_to_csv.py` | ROS 2 点群 → CSV 記録 |
| `detectPrismAndCalcHapCoorsys.py` | プリズム検出・LiDARの位置姿勢算出 |
| `showMultiHapPointCloud.py` | 複数 HAP の TS 座標系可視化 |
| `update_hap_config_from_coorsys.py` | 結果を `HAP_config.json` に反映 |
| `hap_ip_map.py` / `hap_csv_io.py` | 共通ユーティリティ |
| `data/inputData/` | 入力（パラメータ・プリズム位置・点群 CSV） |
| `data/outputData/` | キャリブ結果 YAML |
| `docs/livox_calib_manual.md` | 手順マニュアル |

## セットアップ

```bash
pip install -r requirements.txt
```

点群記録・設定反映には ROS 2 と livox_ros_driver2（例: `~/ros2_livox_ws`）が必要です。

詳細は [docs/livox_calib_manual.md](docs/livox_calib_manual.md) を参照してください。

## クイックスタート

```bash
cd /path/to/livox_calib

# 1. 点群記録（ROS 2 環境を sourced した状態で）
python3 lidar_to_csv.py --hap-num 123

# 2. キャリブ
python3 detectPrismAndCalcHapCoorsys.py -n 123 -d ./data

# 3. HAP_config.json へ反映
python3 update_hap_config_from_coorsys.py -n 123
```

デフォルトのデータフォルダはリポジトリ内の `./data` です。  
`update_hap_config_from_coorsys.py` のデフォルト更新先は `~/ros2_livox_ws/src/livox_ros_driver2/config/HAP_config.json` です（`--hap-config` で変更可）。

## 注意

`data/inputData/hap*.csv`（点群）はサイズが大きいため Git 管理対象外です。現場データはローカルに置いてください。
