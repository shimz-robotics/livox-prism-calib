#!/bin/bash
# setup_ros2_ws.sh — livox_ros_driver2 のワークスペースを sudo 不要で構築する
#
# 使い方:
#   ./scripts/setup_ros2_ws.sh [WS_DIR]
#     WS_DIR: 構築先（デフォルト: リポジトリ内 ./ros2_livox_ws）
#
# 公式手順（Livox-SDK2 を /usr/local へ sudo make install → build.sh humble）の
# 代替。SDK をワークスペース内 sdk_install/ にインストールし、CMAKE_PREFIX_PATH と
# RPATH を指定してビルドするため、sudo 不要で /usr/local を汚さない。
# ワークスペースのディレクトリを丸ごと削除すれば環境も消える。
#
# 構築後の構成:
#   <WS_DIR>/
#     sdk/Livox-SDK2/       SDK ソース（COLCON_IGNORE 設置済み）
#     sdk_install/          SDK のインストール先（/usr/local の代わり）
#     src/livox_ros_driver2/
#     rebuild.sh            再ビルド用スクリプト（公式 build.sh の代替）
set -eu

REPO_DIR=$(cd "$(dirname "$0")/.." && pwd)
WS_DIR=${1:-"$REPO_DIR/ros2_livox_ws"}
ROS_SETUP=/opt/ros/humble/setup.bash

SDK_REPO=https://github.com/Livox-SDK/Livox-SDK2.git
DRIVER_REPO=https://github.com/Livox-SDK/livox_ros_driver2.git

# 前提チェック
if [ ! -f "$ROS_SETUP" ]; then
    echo "エラー: $ROS_SETUP がありません。ROS 2 Humble をインストールしてください。" >&2
    exit 1
fi
for cmd in git cmake make colcon; do
    if ! command -v "$cmd" > /dev/null; then
        echo "エラー: $cmd が見つかりません。インストールしてください。" >&2
        exit 1
    fi
done

mkdir -p "$WS_DIR"
WS_DIR=$(cd "$WS_DIR" && pwd)
echo "ワークスペース構築先: $WS_DIR"

# SDK・ドライバの取得（既存ならスキップ、再実行しても安全）
if [ ! -d "$WS_DIR/sdk/Livox-SDK2" ]; then
    git clone "$SDK_REPO" "$WS_DIR/sdk/Livox-SDK2"
fi
# colcon が SDK を ROS パッケージとして拾わないようにする
# （置かないとビルド順序が保証されないまま SDK もビルド対象になる）
touch "$WS_DIR/sdk/Livox-SDK2/COLCON_IGNORE"

if [ ! -d "$WS_DIR/src/livox_ros_driver2" ]; then
    git clone "$DRIVER_REPO" "$WS_DIR/src/livox_ros_driver2"
fi

# 再ビルドスクリプトを生成（初回ビルドもこれを使う）
cat > "$WS_DIR/rebuild.sh" << 'EOS'
#!/bin/bash
# livox_ros_driver2 の再ビルドスクリプト（公式 build.sh の代替）
#
# このワークスペースは sudo 不要の自己完結構成:
#   sdk/Livox-SDK2/   SDK ソース
#   sdk_install/      SDK のインストール先（/usr/local の代わり）
#   src/livox_ros_driver2/
# 公式 build.sh は SDK が /usr/local にある前提なので使えない。
# CMAKE_PREFIX_PATH と RPATH の指定込みでビルドするのがこのスクリプト。
set -e
cd "$(dirname "$0")"
WS_DIR=$(pwd)
SDK_PREFIX="$WS_DIR/sdk_install"

source /opt/ros/humble/setup.bash

# SDK のビルド・インストール（ソース更新時も同じ手順で反映される）
cmake -S "$WS_DIR/sdk/Livox-SDK2" -B "$WS_DIR/sdk/Livox-SDK2/build" -DCMAKE_BUILD_TYPE=Release
make -C "$WS_DIR/sdk/Livox-SDK2/build" -j"$(nproc)"
cmake --install "$WS_DIR/sdk/Livox-SDK2/build" --prefix "$SDK_PREFIX"

# build.sh 相当のファイル差し替え（ROS2 用）
cd src/livox_ros_driver2
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/
cd ../..

CMAKE_PREFIX_PATH="$SDK_PREFIX:$CMAKE_PREFIX_PATH" colcon build --cmake-args \
    -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON

rm -rf src/livox_ros_driver2/launch/

echo "ビルド完了。使う前に: source $WS_DIR/install/setup.bash"
EOS
chmod +x "$WS_DIR/rebuild.sh"

"$WS_DIR/rebuild.sh"

echo ""
echo "セットアップ完了: $WS_DIR"
echo "使う前に:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $WS_DIR/install/setup.bash"
echo "再ビルド（SDK・ドライバのソース更新後など）:"
echo "  $WS_DIR/rebuild.sh"
