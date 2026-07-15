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

## 環境構築（ROS 2 ワークスペース）

点群記録・設定反映には ROS 2 Humble と `livox_ros_driver2` のワークスペースが必要です。  
未構築の場合は、付属のセットアップスクリプトで sudo 不要・自己完結のワークスペースを構築できます。

```bash
cd /path/to/livox-prism-calib
./scripts/setup_ros2_ws.sh
```

構築先はデフォルトでリポジトリ内 `./ros2_livox_ws/`（Git 管理外）です。  
場所を変えたい場合は引数で指定します（例: `./scripts/setup_ros2_ws.sh ~/ros2_livox_ws`）。

構築されるワークスペースの構成:

```
ros2_livox_ws/
  sdk/Livox-SDK2/       # Livox SDK ソース
  sdk_install/          # SDK のインストール先（/usr/local の代わり）
  src/livox_ros_driver2/
  rebuild.sh            # 再ビルド用スクリプト
```

- 再ビルド（SDK・ドライバのソース更新後など）は `./ros2_livox_ws/rebuild.sh` を実行します。公式の `build.sh` は SDK が `/usr/local` にある前提のため、このワークスペースでは使えません。
- ワークスペースを別の場所へ移動した場合はクリーン再ビルドが必要です（`rm -rf build install log sdk_install sdk/Livox-SDK2/build` → `./rebuild.sh`。RPATH と colcon 生成物に絶対パスが埋まるため）。
- ワークスペースのディレクトリを丸ごと削除すれば環境も消えます（`/usr/local` を汚しません）。
- sudo が使える環境で公式手順（`/usr/local` へのインストール）を使う場合は [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) / [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) の README を参照してください。

---

## 入力ファイル構成

```
<data-folder>/
  input_data/
    prism_pos_<NUM>.csv # TSで計測した3つのプリズム位置 [m]（HAP番号 NUM に対応）
    hap<NUM>.csv      # HAPで取得した点群ファイル（X Y Z Intensity Tag の列を含む）

data/input_data/
  detect_prism_params.yaml  # プリズム検出パラメータ（デフォルト、`--data-folder` とは独立）
  hap_ip_map.yaml           # HAP番号 → LiDAR IP（点群記録・HAP_config.json 反映で共用）
```

### prism_pos_<NUM>.csv の形式

例: HAP101 用 → `prism_pos_101.csv`、HAP102 用 → `prism_pos_102.csv`


| X [m]    | Y [m]    | Z [m]   |
| -------- | -------- | ------- |
| -34.02   | -10.0411 | -1.3492 |
| -35.288  | -12.5448 | -1.3364 |
| -33.1169 | -13.5946 | -1.4024 |


### hap<NUM>.csv の形式

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
  output_data/
    hap<NUM>_coorsys_py.yaml        # LiDAR位置・姿勢（YAML形式）
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

- `livox_ros_driver2` のワークスペースをビルド済みであること（未構築の場合は「環境構築（ROS 2 ワークスペース）」を参照。以下の例はリポジトリ内 `./ros2_livox_ws` の場合。`~/ros2_livox_ws` などに置いた場合はパスを読み替え）
- `HAP_config.json` の IP 設定が実機と一致していること
- `rviz_HAP_launch.py` で `multi_topic=1`（LiDAR ごとにトピックが分かれる設定）

#### HAP 番号とトピックの対応

HAP 番号 → IP の対応は `data/input_data/hap_ip_map.yaml` で設定します。  
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
| 101    | 192.168.0.101   | `/livox/lidar_192_168_0_101` | `data/input_data/hap101.csv` |
| 102    | 192.168.0.102   | `/livox/lidar_192_168_0_102` | `data/input_data/hap102.csv` |


#### 手順

**ターミナル 1** — LiDAR ドライバと RViz を起動:

```bash
cd /path/to/livox-prism-calib
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ./ros2_livox_ws/install/setup.bash
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
source ./ros2_livox_ws/install/setup.bash
```

HAP101 → `data/input_data/hap101.csv`:

```bash
python3 lidar_to_csv.py -n 101 --duration 10
```

HAP102 も取得する場合:

```bash
python3 lidar_to_csv.py -n 102 --duration 10
```

トピックを直接指定する場合:

```bash
python3 lidar_to_csv.py --topic /livox/lidar_192_168_0_101 --duration 30 --output hap101.csv
```

#### lidar_to_csv.py オプション一覧


| オプション             | 短縮形    | デフォルト                              | 説明                                       |
| ----------------- | ------ | ---------------------------------- | ---------------------------------------- |
| `--hap-num N`     | `-n N` | `101`                              | 出力ファイル名 `hap<N>.csv` とデフォルトトピックの選択       |
| `--duration SEC`  |        | `10.0`                             | 記録時間 [秒]                                 |
| `--wait-timeout SEC` |     | `10.0`                             | 最初のメッセージ待ちのタイムアウト [秒]（0 以下で無効）        |
| `--topic NAME`    |        | （hap-num + hap_ip_map.yaml から自動）    | 購読する PointCloud2 トピック（指定時は hap-num より優先） |
| `--output FILE`   |        | `hap<N>.csv`                       | 出力 CSV ファイル名                             |
| `--data-dir PATH` |        | `./data/input_data`                 | 出力先ディレクトリ                                |
| `--ip-map PATH`   |        | `data/input_data/hap_ip_map.yaml`     | HAP番号→IP マップ YAML                        |


#### 注意

- 記録時間が長いと点数が非常に多くなり、CSV が巨大になることがあります。まずは **5〜10 秒** 程度で試してください。
- `ros-humble-sensor-msgs-py` が未インストールの場合:

```bash
sudo apt install ros-humble-sensor-msgs-py
```

- 取得した CSV は `detect_prism_and_calc_hap_coorsys.py` の入力（`input_data/hap<NUM>.csv`）としてそのまま使えます。

#### トラブルシューティング: 記録が始まらない・タイムアウトする

「No message received on ...」で終了する場合、購読トピックに点群が流れていません。以下を確認してください。

1. ドライバーが起動しているか（ターミナル1の `ros2 launch` が動いているか）
2. トピックが存在するか:

```bash
ros2 topic list
```

3. `/livox/lidar` しかない場合、ドライバーが `multi_topic=0` で起動しています。`ros2_livox_ws/src/livox_ros_driver2/launch_ROS2/rviz_HAP_launch.py` の `multi_topic = 0` を `1` に変更し、`ros2_livox_ws/rebuild.sh` で再ビルドしてからドライバーを再起動してください（`scripts/setup_ros2_ws.sh` で構築したワークスペースは自動で `multi_topic=1` になります）。

---

### キャリブレーション用ターゲットプリズムデータの準備

- 点群中の少なくとも３点にターゲットプリズムを設置する
- トータルステーションで設置したターゲットプリズムの位置を測定する
- 測定した値を X,Y,Z の順で prism_pos_xxx.csv に記述する

### キャリブレーションの実行

```bash
cd /path/to/livox-prism-calib
```

デフォルト値で実行（HAP番号=101、データフォルダ=./data）:

```bash
python3 detect_prism_and_calc_hap_coorsys.py
```

HAP番号とデータフォルダを指定して実行:

```bash
python3 detect_prism_and_calc_hap_coorsys.py --hap-num 102 --data-folder ./data
```

短縮オプション:

```bash
python3 detect_prism_and_calc_hap_coorsys.py -n 102 -d ./data
```

### オプション一覧


| オプション                | 短縮形       | デフォルト                                   | 説明                                          |
| -------------------- | --------- | --------------------------------------- | ------------------------------------------- |
| `--hap-num N`        | `-n N`    | `101`                                   | 処理する HAP 番号                                 |
| `--data-folder PATH` | `-d PATH` | `./data`                                | データフォルダのパス（`input_data` / `output_data` を含む親） |
| `--config PATH`      | `-c PATH` | `data/input_data/detect_prism_params.yaml` | 検出パラメータ YAML のパス                            |


#### 注意

##### 検出パラメータの調整

プリズム検出や対応づけが失敗する場合、  
`data/input_data/detect_prism_params.yaml` の値を変更して再実行できます。

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
python3 detect_prism_and_calc_hap_coorsys.py -n 101 -d ./data
```

カスタム設定を指定:

```bash
python3 detect_prism_and_calc_hap_coorsys.py -n 101 -d ./data --config ./data/input_data/detect_prism_params.yaml
```

> **補足**: `--config` を省略した場合、デフォルトは `data/input_data/detect_prism_params.yaml` です（`--data-folder` とは独立）。

##### その他の確認事項

- プリズムは少なくとも3点、互いに十分離れた配置にする（二等辺三角形に近い配置は不可）
- 点群 CSV に Tag=64 の高輝度点が含まれているか確認する
- 記録時間が短すぎてプリズム点が少ない場合は、`lidar_to_csv.py` の `--duration` を延ばす

---

## 処理フロー

```
1. TSプリズム位置の読み込み（prism_pos_<NUM>.csv）
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

`show_multi_hap_point_cloud.py`  
元の実装: `matlab_ws/LivoxCalibByPrisms/source/showMultiHapPointCloud.m`

### 必要ライブラリ

```bash
pip install open3d
```

### 入力ファイル

```
<data-folder>/
  input_data/
    hap<N>.csv                       # HAP 点群（CSV）
  output_data/
    hap<N>_coorsys_py.yaml            # キャリブ結果（detect_prism_and_calc_hap_coorsys.py の出力）
```

> **注意**: 可視化の前に、表示したい全 HAP 番号に対して  
> `detect_prism_and_calc_hap_coorsys.py` を実行して YAML を生成しておく必要があります。

### 実行方法

デフォルト（HAP101 赤 + HAP102 青）:

```bash
cd /path/to/livox-prism-calib
```

```bash
python3 show_multi_hap_point_cloud.py
```

HAP番号とデータフォルダを指定:

```bash
python3 show_multi_hap_point_cloud.py -n 101 102 --data-folder ./data
```

3台以上を重ね表示:

```bash
python3 show_multi_hap_point_cloud.py -n 101 102 123 124
```

### オプション一覧


| オプション                | 短縮形       | デフォルト                                             | 説明             |
| -------------------- | --------- | ------------------------------------------------- | -------------- |
| `--hap-num N ...`    | `-n N ...` | `101 102`                                         | 対象 HAP 番号（複数可。色は赤→青→緑→…の順） |
| `--data-folder PATH` | `-d PATH` | `./data`（リポジトリ内） | データフォルダ        |


### 処理フロー

```
1. hap<N>.csv から点群読み込み（2台分）
       ↓
2. hap<N>_coorsys_py.yaml から変換行列を復元（ZYX オイラー角 → 4×4 同次変換行列）
       ↓
3. 各点群を TS 座標系へ変換
       ↓
4. Open3D ウィンドウで2台分を色分けして表示（HAP1: 赤、HAP2: 青）
```

---

## HAP_config.json への反映

キャリブ結果 YAML の内容を、本リポジトリ管理のマスター  
`data/HAP_config.json` の `lidar_configs[].extrinsic_parameter` に書き込み、  
ワークスペースのドライバ config（src 側・install 側）へ配備（コピー）します。

あわせて `host_net_info` の IP（LiDAR が点群を送り返す先＝この PC の IP）を、  
対象 LiDAR への経路から自動検出した自機 IP に書き換えます（通常更新・`--reset` 共通のデフォルト動作）。  
PC やネットワーク構成が変わっても、本スクリプトの実行だけでホスト IP が追従します。

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
  output_data/
    hap<N>_coorsys_py.yaml    # detect_prism_and_calc_hap_coorsys.py の出力
```


HAP 番号 → JSON 上の IP は `data/input_data/hap_ip_map.yaml` で解決します（`lidar_to_csv.py` と同じファイル）。


| HAP | YAML                    | JSON 上の IP（hap_ip_map.yaml） |
| --- | ----------------------- | ------------------------- |
| 101 | `hap101_coorsys_py.yaml` | `192.168.0.101`           |
| 102 | `hap102_coorsys_py.yaml` | `192.168.0.102`           |


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
| `--master PATH`      |            | `data/HAP_config.json`（リポジトリ内） | マスター HAP_config.json（キャリブ結果の反映先）        |
| `--hap-config PATH`  |            | `<WS>/src/livox_ros_driver2/config/HAP_config.json`（`<WS>` は `LIVOX_WS` → `./ros2_livox_ws` → `~/ros2_livox_ws` の順で解決） | 配備先ドライバ config（src 側。install 側は自動導出） |
| `--no-deploy`        |            | （オフ）                                              | マスターのみ更新し、ドライバ config へ配備しない          |
| `--ip-map PATH`      |            | `data/input_data/hap_ip_map.yaml`                    | HAP番号→IP マップ YAML                        |
| `--reset`            |            | （オフ）                                              | 指定 HAP の `extrinsic_parameter` をゼロにリセット |
| `--yes`              | `-y`       | （オフ）                                              | 確認プロンプトをスキップして更新                        |
| `--no-backup`        |            | （オフ）                                              | 更新前の `.bak` を作成しない                      |
| `--dry-run`          |            | （オフ）                                              | プレビューのみ（ファイルは更新しない）                     |


### 注意

- 現場設定の真実（マスター）は本リポジトリの `data/HAP_config.json` です。IP 構成の変更やキャリブ結果はまずマスターに反映し、ドライバへは本スクリプトで配備します。ドライバ側 config を直接編集しても、次回の配備で上書きされます
- `host_net_info` の IP が実 IP と異なるままだと、ドライバは「bind failed」で起動に失敗します。自機 IP を検出できない場合（LiDAR への経路がない、LiDAR 用 NIC 未接続など）は警告を表示し、`host_net_info` は既存値のまま配備されるので、警告が出たら LAN ケーブルと IP 設定を確認してください
- 更新・配備前に各ファイルの `.bak` が作成されます（`--no-backup` 指定時を除く）
- ドライバが実行時に読むのは install 側（`<WS>/install/livox_ros_driver2/share/livox_ros_driver2/config/HAP_config.json`）の実体コピーです。スクリプトは src 側・install 側の両方へ配備します
- install 側まで配備された場合、**livox_ros_driver2 の再起動**後に点群へ反映されます
- install 側が見つからない場合（警告が表示されます）は src 側のみの配備となり、反映にはワークスペースのリビルドが必要です
- 起動時に `hap_ip_map.yaml` とマスターの `lidar_configs[].ip` の不一致をチェックし、警告を表示します（二重管理の不一致検出）

### 処理フロー（キャリブ結果の反映）

```
1. output_data/hap<N>_coorsys_py.yaml を読み込み
       ↓
2. extrinsic_parameter 形式（roll/pitch/yaw [deg], x/y/z [mm]）に変換
       ↓
3. hap_ip_map.yaml で HAP 番号 → LiDAR IP を解決
       ↓
4. プレビュー表示 → 確認後、マスター（data/HAP_config.json）を更新
       ↓
5. マスターをドライバ config（src 側・install 側）へ配備
```

### 処理フロー（リセット）

```
1. hap_ip_map.yaml で HAP 番号 → LiDAR IP に変換
       ↓
2. 該当 IP の extrinsic_parameter のみをすべて 0 に設定
       ↓
3. プレビュー表示 → 確認後、マスター（data/HAP_config.json）を更新
       ↓
4. マスターをドライバ config（src 側・install 側）へ配備
```

---

## 関連ファイル


| ファイル                                                                 | 説明                                           |
| -------------------------------------------------------------------- | -------------------------------------------- |
| `lidar_to_csv.py`                          | ROS2 点群 → CSV 記録                             |
| `hap_ip_map.py`                            | HAP番号→IP マップ YAML 読み込み（共通）                   |
| `hap_csv_io.py`                            | HAP 点群 CSV 読み込み（新/旧形式対応）                     |
| `detect_prism_and_calc_hap_coorsys.py`          | プリズム検出・キャリブスクリプト                             |
| `data/input_data/detect_prism_params.yaml`    | プリズム検出パラメータ（`cluster_radius`, `tolerance` 等） |
| `data/input_data/hap_ip_map.yaml`             | HAP番号 → LiDAR IP（点群記録・config 反映で共用）           |
| `data/HAP_config.json`             | 現場設定マスター（Git 管理、キャリブ結果の反映先）             |
| `show_multi_hap_point_cloud.py`                | HAP 点群の TS 座標系可視化                            |
| `update_hap_config_from_coorsys.py`        | キャリブ YAML → HAP_config.json 反映               |
| `data/`                                    | 点群 CSV（`input_data`）・キャリブ結果（`output_data`）     |
| `matlab_ws/LivoxCalibByPrisms/`（別リポジトリ） | 元の MATLAB 実装（参考）                             |
| `<WS>/src/livox_ros_driver2/config/HAP_config.json` | 配備先 Livox ドライバ設定（`<WS>` はワークスペースの場所。install 側にも配備）          |


# 大まかな手順

現場向けの短縮手順は [`calib_quickstart.md`](calib_quickstart.md) を参照してください。
