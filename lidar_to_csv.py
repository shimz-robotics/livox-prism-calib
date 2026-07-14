#!/usr/bin/env python3
"""
lidar_to_csv.py

指定した Livox HAP の PointCloud2 トピックを購読し、
x, y, z, intensity, tag の順で CSV に保存する。

使い方:
  # ドライバ起動後、別ターミナルで実行
  python3 lidar_to_csv.py --hap-num 101 --duration 10

  python3 lidar_to_csv.py \\
    --topic /livox/lidar_192_168_0_101 \\
    --duration 30 \\
    --output hap101.csv

オプション:
  --topic       購読トピック（--hap-num より優先）
  --hap-num     出力ファイル名 hap<N>.csv 用（デフォルト: 101）
  --duration    記録時間 [秒]（デフォルト: 10.0）
  --output      出力 CSV ファイル名（デフォルト: hap<hap-num>.csv）
  --data-dir    出力先ディレクトリ（デフォルト: ./data/input_data）
  --ip-map      HAP番号→IP マップ YAML（デフォルト: data/input_data/hap_ip_map.yaml）

前提:
  - livox_ros_driver2 が xfer_format=0 (PointCloud2) で起動していること
  - multi_topic=1 のときトピックは hap_ip_map.yaml の IP から自動生成:
      192.168.0.101 -> /livox/lidar_192_168_0_101
"""

import argparse
import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from hap_ip_map import DEFAULT_IP_MAP_PATH, load_hap_num_to_ip, resolve_topic

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATA_DIR = SCRIPT_DIR / "data" / "input_data"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Livox PointCloud2 -> CSV (x,y,z,intensity,tag)"
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="PointCloud2 topic (overrides --hap-num topic mapping)",
    )
    parser.add_argument(
        "--hap-num",
        type=int,
        default=101,
        help="HAP number for default topic and output filename hap<N>.csv",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Recording duration in seconds",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output CSV filename (default: hap<hap-num>.csv)",
    )
    parser.add_argument(
        "--data-dir",
        default=str(DEFAULT_DATA_DIR),
        help="Output directory for CSV files",
    )
    parser.add_argument(
        "--ip-map",
        default=str(DEFAULT_IP_MAP_PATH),
        metavar="PATH",
        help=f"HAP番号→IP マップ YAML（デフォルト: {DEFAULT_IP_MAP_PATH}）",
    )
    return parser.parse_args()


def resolve_topic_and_output(args):
    if args.topic:
        topic = args.topic
    else:
        try:
            hap_num_to_ip = load_hap_num_to_ip(args.ip_map)
            topic = resolve_topic(args.hap_num, hap_num_to_ip)
        except (FileNotFoundError, ValueError, KeyError) as e:
            raise SystemExit(str(e)) from e

    data_dir = Path(args.data_dir).expanduser().resolve()
    filename = args.output if args.output else f"hap{args.hap_num}.csv"
    output_path = data_dir / filename
    return topic, output_path


class LidarCsvRecorder(Node):
    def __init__(self, topic: str, duration_sec: float, output_path: Path):
        super().__init__("lidar_csv_recorder")
        self.duration_sec = duration_sec
        self.output_path = output_path
        self.t0 = None
        self.done = False
        self.fp = None
        self.writer = None
        self.point_count = 0
        self.msg_count = 0

        self.sub = self.create_subscription(
            PointCloud2, topic, self.callback, 10
        )
        self.get_logger().info(
            f"Recording '{topic}' for {duration_sec}s -> {output_path}"
        )

    def callback(self, msg: PointCloud2):
        if self.done:
            return

        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self.fp = open(self.output_path, "w", newline="")
            self.writer = csv.writer(self.fp)

        if now - self.t0 >= self.duration_sec:
            self._finish()
            return

        self.msg_count += 1
        for p in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z", "intensity", "tag"),
            skip_nans=False,
        ):
            self.writer.writerow([p[0], p[1], p[2], p[3], int(p[4])])
            self.point_count += 1

    def _finish(self):
        if self.done:
            return
        self.done = True
        if self.fp:
            self.fp.close()
            self.fp = None
        elapsed = time.monotonic() - self.t0 if self.t0 else 0.0
        self.get_logger().info(
            f"Done: {self.msg_count} messages, {self.point_count} points "
            f"in {elapsed:.2f}s -> {self.output_path}"
        )


def main():
    args = parse_args()
    try:
        topic, output_path = resolve_topic_and_output(args)
    except SystemExit as e:
        print(e)
        return 1

    if args.duration <= 0:
        print("ERROR: --duration must be positive.")
        return 1

    rclpy.init()
    node = LidarCsvRecorder(topic, args.duration, output_path)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
        if not node.done:
            node._finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
