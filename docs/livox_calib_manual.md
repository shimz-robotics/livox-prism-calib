# LiDAR（HAP）プリズムキャリブレーション マニュアル

スクリプトの保存先: このリポジトリのルート（`livox-prism-calib/`）

---

## 概要

LiDAR（HAP）で取得した点群からプリズムを自動検出し、  
トータルステーション（TS）による計測値との対応づけを行うことで、  
**LiDAR座標系の位置・姿勢（6-DoF）** を求めるキャリブレーションツールです。

元の実装: `matlab_ws/LivoxCalibByPrisms/source/detectPrismAndCalcHapCoorsys.m`

---

## 必要ライブラリ

```bash
pip install numpy scipy pyyaml
```

---

## 入力ファイル構成

```
<data-folder>/
  inputData/
    PrismPos<NUM>.csv # TSで計測した3つのプリズム位置 [m]（HAP番号 NUM に対応）
    hap<NUM>.csv      # HAPで取得した点群ファイル（X Y Z Intensity Tag の列を含む）

data/inputData/
  detectPrismParams.yaml  # プリズム検出パラメータ（デフォルト、`--data-folder` とは独立）
  hapIpMap.yaml           # HAP番号 → LiDAR IP（点群記録・HAP_config.json 反映で共用）
```

### PrismPosNUM.csv の形式

例: HAP101 用 → `PrismPos101.csv`、HAP102 用 → `PrismPos102.csv`


| X [m]    | Y [m]    | Z [m]   |
| -------- | -------- | ------- |
| -34.02   | -10.0411 | -1.3492 |
| -35.288  | -12.5448 | -1.3364 |
| -33.1169 | -13.5946 | -1.4024 |


### hapNUM.csv の形式

`lidar_to_csv.py` で出力する形式（ヘッダなし、5列）:


| 列   | 内容        | 単位・備考                  |
| --- | --------- | ---------------------- |
| 1   | x         | [m]                    |
| 2   | y         | [m]                    |
| 3   | z         | [m]                    |
| 4   | intensity | 反射強度（高輝度点をプリズム候補として使用） |
| 5   | tag       | 点の分類タグ（`64` のみ対象）      |


旧形式（先頭に index 列など 8列以上）の CSV も読み込み可能です。

---

## 出力ファイル

```
<data-folder>/
  outputData/
    hap<NUM>Coorsys_py.yaml        # LiDAR位置・姿勢（YAML形式）
```

### YAML出力の形式

```yaml
Position:
  x: -37762   # [mm]
  y: -16506   # [mm]
  z:   4176   # [mm]
Rotation:
  roll:  -4.0200   # [deg]
  pitch: 54.7936   # [deg]
  yaw:   -2.9133   # [deg]
```

---

## 実行方法

```bash
cd /path/to/livox-prism-calib
```

### キャリブレーション用点群データの取得

ROS 2 上の Livox HAP 点群を CSV に記録します。  
スクリプト: `lidar_to_csv.py`

#### 前提

- `ros2_livox_ws` で `livox_ros_driver2` をビルド済みであること
- `HAP_config.json` の IP 設定が実機と一致していること
- `rviz_HAP_launch.py` で `multi_topic=1`（LiDAR ごとにトピックが分かれる設定）

#### HAP 番号とトピックの対応

HAP 番号 → IP の対応は `data/inputData/hapIpMap.yaml` で設定します。  
点群トピックは IP から自動生成されます（例: `192.168.0.101` → `/livox/lidar_192_168_0_101`）。

```yaml
hap_num_to_ip:
  101: "192.168.0.101"
  102: "192.168.0.102"
  123: "192.168.1.123"
  124: "192.168.1.124"
```

`HAP_config.json` の `lidar_configs[].ip` と一致させてください。新しい HAP を使う場合は、この YAML に追記します。


| HAP 番号 | LiDAR IP（デフォルト例） | 点群トピック（自動）                 | 出力ファイル（デフォルト）               |
| ------ | --------------- | --------------------------- | --------------------------- |
| 101    | 192.168.0.101   | `/livox/lidar_192_168_0_101` | `data/inputData/hap101.csv` |
| 102    | 192.168.0.102   | `/livox/lidar_192_168_0_102` | `data/inputData/hap102.csv` |


#### 手順

**ターミナル 1** — LiDAR ドライバと RViz を起動:

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/ros2_livox_ws/install/setup.bash
```

```bash
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

**ターミナル 2** — 点群を CSV に記録（例: HAP101 を 10 秒）:

```bash
cd /path/to/livox-prism-calib
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/ros2_livox_ws/install/setup.bash
```

HAP101 → `data/inputData/hap101.csv`:

```bash
python3 lidar_to_csv.py --hap-num 101 --duration 10
```

HAP102 も取得する場合:

```bash
python3 lidar_to_csv.py --hap-num 102 --duration 10
```

トピックを直接指定する場合:

```bash
python3 lidar_to_csv.py --topic /livox/lidar_192_168_0_101 --duration 30 --output hap101.csv
```

#### lidar_to_csv.py オプション一覧


| オプション             | デフォルト                              | 説明                                       |
| ----------------- | ---------------------------------- | ---------------------------------------- |
| `--hap-num N`     | `101`                              | 出力ファイル名 `hap<N>.csv` とデフォルトトピックの選択       |
| `--duration SEC`  | `10.0`                             | 記録時間 [秒]                                 |
| `--topic NAME`    | （hap-num + hapIpMap.yaml から自動）    | 購読する PointCloud2 トピック（指定時は hap-num より優先） |
| `--output FILE`   | `hap<N>.csv`                       | 出力 CSV ファイル名                             |
| `--data-dir PATH` | `./data/inputData`                 | 出力先ディレクトリ                                |
| `--ip-map PATH`   | `data/inputData/hapIpMap.yaml`     | HAP番号→IP マップ YAML                        |


#### 注意

- 記録時間が長いと点数が非常に多くなり、CSV が巨大になることがあります。まずは **5〜10 秒** 程度で試してください。
- `ros-humble-sensor-msgs-py` が未インストールの場合:

```bash
sudo apt install ros-humble-sensor-msgs-py
```

- 取得した CSV は `detectPrismAndCalcHapCoorsys.py` の入力（`inputData/hap<NUM>.csv`）としてそのまま使えます。

---

### キャリブレーション用ターゲットプリズムデータの準備

- 点群中の少なくとも３点にターゲットプリズムを設置する
- トータルステーションで設置したターゲットプリズムの位置を測定する
- 測定した値を X,Y,Z の順で HAPxxx.csv に記述する

### キャリブレーションの実行

```bash
cd /path/to/livox-prism-calib
```

デフォルト値で実行（HAP番号=101、データフォルダ=./data）:

```bash
python3 detectPrismAndCalcHapCoorsys.py
```

HAP番号とデータフォルダを指定して実行:

```bash
python3 detectPrismAndCalcHapCoorsys.py --hap-num 102 --data-folder ./data
```

短縮オプション:

```bash
python3 detectPrismAndCalcHapCoorsys.py -n 102 -d ./data
```

### オプション一覧


| オプション                | 短縮形       | デフォルト                                   | 説明                                          |
| -------------------- | --------- | --------------------------------------- | ------------------------------------------- |
| `--hap-num N`        | `-n N`    | `101`                                   | 処理する HAP 番号                                 |
| `--data-folder PATH` | `-d PATH` | `./data`                                | データフォルダのパス（`inputData` / `outputData` を含む親） |
| `--config PATH`      | `-c PATH` | `data/inputData/detectPrismParams.yaml` | 検出パラメータ YAML のパス                            |


#### 注意

##### 検出パラメータの調整

プリズム検出や対応づけが失敗する場合、  
`data/inputData/detectPrismParams.yaml` の値を変更して再実行できます。

```yaml
diff_distance: 0.05      # TSプリズム3点の二等辺三角形判定 [m]
cluster_radius: 0.075    # 高輝度点のクラスタリング半径 [m]
tolerance: 0.075         # TS↔LiDAR 形状マッチングの許容誤差 [m]
arm_max: 1024            # マッチング内部の行列サイズ（通常は変更不要）
```


| パラメータ            | 役割                  | 調整の目安                              |
| ---------------- | ------------------- | ---------------------------------- |
| `cluster_radius` | プリズム候補点をまとめる半径      | プリズムが分離しすぎる→大きく、複数プリズムが1つにまとまる→小さく |
| `tolerance`      | 3点間距離の一致判定の許容幅      | 対応づけ失敗時は段階的に大きくする                  |
| `diff_distance`  | TS計測3点が二等辺三角形とみなす閾値 | 誤検出時のみ小さく（厳しく）する。根本的には3点配置の見直しが必要  |
| `arm_max`        | マッチングアルゴリズムの内部上限    | 通常はデフォルト（1024）のまま                  |


パラメータは一度に1つずつ小さく変更し、毎回再実行して確認してください。  
起動時に `検出パラメータ: ...` として読み込まれた値が表示されます。

##### 実行例

デフォルト設定で実行:

```bash
python3 detectPrismAndCalcHapCoorsys.py -n 101 -d ./data
```

カスタム設定を指定:

```bash
python3 detectPrismAndCalcHapCoorsys.py -n 101 -d ./data --config ./data/inputData/detectPrismParams.yaml
```

> **補足**: `--config` を省略した場合、デフォルトは `data/inputData/detectPrismParams.yaml` です（`--data-folder` とは独立）。

##### その他の確認事項

- プリズムは少なくとも3点、互いに十分離れた配置にする（二等辺三角形に近い配置は不可）
- 点群 CSV に Tag=64 の高輝度点が含まれているか確認する
- 記録時間が短すぎてプリズム点が少ない場合は、`lidar_to_csv.py` の `--duration` を延ばす

---

## 処理フロー

```
1. TSプリズム位置の読み込み（PrismPos\<NUM\>.csv）
        ↓
2. HAP点群の読み込み（hap<NUM>.csv）
        ↓
3. 高輝度点の抽出（Intensity閾値を255から下げながら反復）
        ↓
4. cluster_radius（デフォルト 75 mm）以内の近傍点クラスタリング → 各プリズム重心の算出
        ↓
5. TS点群 ↔ LiDAR点群のコンステレーションマッチング
        ↓
6. SVDによる剛体変換行列（tsHhap）の計算
        ↓
7. YAML ファイルへの出力
```

---

## アルゴリズムの詳細

### プリズム検出

- Intensityが高い点（255から順に閾値を下げて探索）かつ Tag=64 の点を候補とする。
- `cluster_radius`（デフォルト 75 mm）の近傍クラスタリングで個別プリズムを分離し、各クラスタの重心をプリズム位置とする。

### コンステレーションマッチング

- TS点群・LiDAR点群それぞれの全ペア間距離を計算し、形状（辺長の組み合わせ）が一致する対応を探す。
- 非二等辺三角形の検出に基づき２点の対応を決定する。

### 剛体変換

- SVD分解（特異値分解）により、TS座標系からみたLiDAR座標系の回転行列・並進ベクトルを求める。
- 回転はオイラー角（ZYX順: Yaw-Pitch-Roll）で表現する。

---

## 点群可視化

キャリブレーション後、複数 HAP の点群を TS 座標系で重ね表示して結果を確認できます。

### スクリプト

`showMultiHapPointCloud.py`  
元の実装: `matlab_ws/LivoxCalibByPrisms/source/showMultiHapPointCloud.m`

### 必要ライブラリ

```bash
pip install open3d
```

### 入力ファイル

```
<data-folder>/
  inputData/
    hap<N>.csv                       # HAP 点群（CSV）
  outputData/
    hap<N>Coorsys_py.yaml            # キャリブ結果（detectPrismAndCalcHapCoorsys.py の出力）
```

> **注意**: 可視化の前に、表示したい全 HAP 番号に対して  
> `detectPrismAndCalcHapCoorsys.py` を実行して YAML を生成しておく必要があります。

### 実行方法

デフォルト（HAP101 赤 + HAP102 青）:

```bash
cd /path/to/livox-prism-calib
```

```bash
python3 showMultiHapPointCloud.py
```

HAP番号とデータフォルダを指定:

```bash
python3 showMultiHapPointCloud.py --hap-num1 101 --hap-num2 102 --data-folder ./data
```

### オプション一覧


| オプション                | 短縮形       | デフォルト                                             | 説明             |
| -------------------- | --------- | ------------------------------------------------- | -------------- |
| `--hap-num1 N`       | `-n1 N`   | `101`                                             | 1台目の HAP 番号（赤） |
| `--hap-num2 N`       | `-n2 N`   | `102`                                             | 2台目の HAP 番号（青） |
| `--data-folder PATH` | `-d PATH` | `./data`（リポジトリ内） | データフォルダ        |


### 処理フロー

```
1. hap<N>.csv から点群読み込み（2台分）
       ↓
2. hap<N>Coorsys_py.yaml から変換行列を復元（ZYX オイラー角 → 4×4 同次変換行列）
       ↓
3. 各点群を TS 座標系へ変換
       ↓
4. Open3D ウィンドウで2台分を色分けして表示（HAP1: 赤、HAP2: 青）
```

---

## HAP_config.json への反映

キャリブ結果 YAML の内容を  
`ros2_livox_ws/src/livox_ros_driver2/config/HAP_config.json` の  
`lidar_configs[].extrinsic_parameter` に書き込みます。

可視化で結果を確認したあと、別スクリプトで反映してください。

### スクリプト

`update_hap_config_from_coorsys.py`

### 必要ライブラリ

```bash
pip install pyyaml
```

（キャリブ・可視化と同じ）

### 入力ファイル

```
<data-folder>/
  outputData/
    hap<N>Coorsys_py.yaml    # detectPrismAndCalcHapCoorsys.py の出力
```


HAP 番号 → JSON 上の IP は `data/inputData/hapIpMap.yaml` で解決します（`lidar_to_csv.py` と同じファイル）。


| HAP | YAML                    | JSON 上の IP（hapIpMap.yaml） |
| --- | ----------------------- | ------------------------- |
| 101 | `hap101Coorsys_py.yaml` | `192.168.0.101`           |
| 102 | `hap102Coorsys_py.yaml` | `192.168.0.102`           |


反映される値: `roll/pitch/yaw` [deg]、`x/y/z` [mm]（YAML と同じ）

### 実行方法

```bash
cd /path/to/livox-prism-calib
```

デフォルト（HAP101 + HAP102、確認プロンプトあり）:

```bash
python3 update_hap_config_from_coorsys.py
```

対象 HAP とデータフォルダを指定:

```bash
python3 update_hap_config_from_coorsys.py -n 101 102 -d ./data
```

HAP101 のみ:

```bash
python3 update_hap_config_from_coorsys.py -n 101
```

確認なしで更新:

```bash
python3 update_hap_config_from_coorsys.py --yes
```

プレビューのみ（ファイルは更新しない）:

```bash
python3 update_hap_config_from_coorsys.py --dry-run
```

### extrinsic_parameter のリセット

指定した HAP 番号に対応する `extrinsic_parameter` のみを、Livox ドライバのデフォルト値（すべて 0）に戻します。キャリブ YAML は不要です。指定していない HAP の設定は変更されません。

複数 HAP をまとめてリセット（例: HAP101 + HAP102）:

```bash
python3 update_hap_config_from_coorsys.py --reset -n 101 102
```

HAP101 のみリセット:

```bash
python3 update_hap_config_from_coorsys.py --reset -n 101
```

確認なしでリセット:

```bash
python3 update_hap_config_from_coorsys.py --reset -n 101 102 --yes
```

リセット内容のプレビューのみ:

```bash
python3 update_hap_config_from_coorsys.py --reset -n 101 102 --dry-run
```

リセット後の値:


| キー                     | 値     | 単位    |
| ---------------------- | ----- | ----- |
| `roll`, `pitch`, `yaw` | `0.0` | [deg] |
| `x`, `y`, `z`          | `0`   | [mm]  |


### オプション一覧


| オプション                | 短縮形        | デフォルト                                             | 説明                                      |
| -------------------- | ---------- | ------------------------------------------------- | --------------------------------------- |
| `--hap-num N ...`    | `-n N ...` | `101 102`                                         | 対象 HAP 番号（複数指定可）                        |
| `--data-folder PATH` | `-d PATH`  | `./data`（リポジトリ内） | キャリブ YAML の親フォルダ（`--reset` 時は未使用）       |
| `--hap-config PATH`  |            | `~/ros2_livox_ws/.../HAP_config.json`             | 更新先 Livox 設定 JSON                       |
| `--ip-map PATH`      |            | `data/inputData/hapIpMap.yaml`                    | HAP番号→IP マップ YAML                        |
| `--reset`            |            | （オフ）                                              | 指定 HAP の `extrinsic_parameter` をゼロにリセット |
| `--yes`              | `-y`       | （オフ）                                              | 確認プロンプトをスキップして更新                        |
| `--no-backup`        |            | （オフ）                                              | 更新前の `.bak` を作成しない                      |
| `--dry-run`          |            | （オフ）                                              | プレビューのみ（ファイルは更新しない）                     |


### 注意

- 更新前に `HAP_config.json.bak` が作成されます（`--no-backup` 指定時を除く）
- **livox_ros_driver2 の再起動**後に点群へ反映されます

### 処理フロー（キャリブ結果の反映）

```
1. outputData/hap<N>Coorsys_py.yaml を読み込み
       ↓
2. extrinsic_parameter 形式（roll/pitch/yaw [deg], x/y/z [mm]）に変換
       ↓
3. hapIpMap.yaml で HAP 番号 → LiDAR IP を解決
       ↓
4. プレビュー表示 → 確認後、HAP_config.json を更新
```

### 処理フロー（リセット）

```
1. hapIpMap.yaml で HAP 番号 → LiDAR IP に変換
       ↓
2. 該当 IP の extrinsic_parameter のみをすべて 0 に設定
       ↓
3. プレビュー表示 → 確認後、HAP_config.json を更新
```

---

## 関連ファイル


| ファイル                                                                 | 説明                                           |
| -------------------------------------------------------------------- | -------------------------------------------- |
| `lidar_to_csv.py`                          | ROS2 点群 → CSV 記録                             |
| `hap_ip_map.py`                            | HAP番号→IP マップ YAML 読み込み（共通）                   |
| `hap_csv_io.py`                            | HAP 点群 CSV 読み込み（新/旧形式対応）                     |
| `detectPrismAndCalcHapCoorsys.py`          | プリズム検出・キャリブスクリプト                             |
| `data/inputData/detectPrismParams.yaml`    | プリズム検出パラメータ（`cluster_radius`, `tolerance` 等） |
| `data/inputData/hapIpMap.yaml`             | HAP番号 → LiDAR IP（点群記録・config 反映で共用）           |
| `showMultiHapPointCloud.py`                | HAP 点群の TS 座標系可視化                            |
| `update_hap_config_from_coorsys.py`        | キャリブ YAML → HAP_config.json 反映               |
| `data/`                                    | 点群 CSV（`inputData`）・キャリブ結果（`outputData`）     |
| `matlab_ws/LivoxCalibByPrisms/`（別リポジトリ） | 元の MATLAB 実装（参考）                             |
| `~/ros2_livox_ws/.../HAP_config.json`      | 反映先 Livox ドライバ設定（別ワークスペース）                  |


# 大まかな手順

## キャリブレーション用ターゲットプリズムデータの準備

- 点群中の少なくとも３点にターゲットプリズムを設置する
- トータルステーションで設置したターゲットプリズムの位置を測定する
- 測定した値を X,Y,Z の順で HAPxxx.csv に記述する

## JSONファイルのリセット

```bash
cd /path/to/livox-prism-calib
```

```bash
python3 update_hap_config_from_coorsys.py --reset -n 123
```

```bash
python3 update_hap_config_from_coorsys.py --reset -n 124
```

## LiDARドライバ起動

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/ros2_livox_ws/install/setup.bash
```

```bash
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

## 点群データ取得

```bash
cd /path/to/livox-prism-calib
```

```bash
python3 lidar_to_csv.py --hap-num 123 --duration 10
```

```bash
python3 lidar_to_csv.py --hap-num 124 --duration 10
```

## キャリブレーションの実行

```bash
python3 detectPrismAndCalcHapCoorsys.py -n 123 -d ./data
```

```bash
python3 detectPrismAndCalcHapCoorsys.py -n 124 -d ./data
```

## 点群可視化(確認)

```bash
python3 showMultiHapPointCloud.py --hap-num1 123 --hap-num2 124 --data-folder ./data
```

## HAP_config.json への反映

```bash
python3 update_hap_config_from_coorsys.py -n 123 124 -d ./data
```

## 確認

```bash
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

