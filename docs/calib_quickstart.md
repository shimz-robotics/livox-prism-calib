# LiDAR キャリブレーション クイック手順

現場向けの短縮手順です。詳細は [`livox_calib_manual.md`](livox_calib_manual.md) を参照してください。

## キャリブレーション用ターゲットプリズムデータの準備

1. 点群中の少なくとも３点にターゲットプリズムを設置する
2. トータルステーションで設置したターゲットプリズムの位置を測定する
3. 測定した値を X,Y,Z の順で `data/input_data/prism_pos_<num>.csv` に記述する
4. LiDARのIPと番号の対応を`data/input_data/hap_ip_map.yaml`に記述する

以下は、このプロジェクトのパスに移動して実行する。

```bash
cd /path/to/livox-prism-calib
```

## JSONファイルのリセット

101と102を初期化したい場合

```bash
python3 update_hap_config_from_coorsys.py --reset -n 101 102
```

## LiDARドライバ起動

```bash
source /opt/ros/humble/setup.bash
source ./ros2_livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 点群データ取得

`<num>` に対して `<sec>` の点群データを取得する

```bash
python3 lidar_to_csv.py -n 101 --duration 10
```

```bash
python3 lidar_to_csv.py -n 102 --duration 10
```

## キャリブレーションの実行

```bash
python3 detect_prism_and_calc_hap_coorsys.py -n 101
```

```bash
python3 detect_prism_and_calc_hap_coorsys.py -n 102
```

## HAP_config.json への反映

```bash
python3 update_hap_config_from_coorsys.py -n 101 102
```

## 確認

反映にはドライバの再起動が必要です。起動中のドライバを Ctrl+C で停止してから再度起動します。

```bash
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```
