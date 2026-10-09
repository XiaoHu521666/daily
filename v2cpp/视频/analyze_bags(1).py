#!/usr/bin/env python3
"""读取指定目录下所有 rosbag (rosbag2) 的 metadata.yaml，提取录制开始和结束时间。"""

import csv
import os
from datetime import datetime, timedelta, timezone
from typing import Optional
import yaml


BEIJING_TZ = timezone(timedelta(hours=8))

CHAN_MAP = {
    "650": "10.141.50.50", "651": "10.141.51.50", "652": "10.141.52.50",
    "653": "10.141.53.50", "654": "10.141.54.50", "655": "10.141.55.50"
}

def parse_metadata(metadata_path: str) -> Optional[dict]:
    with open(metadata_path, 'r') as f:
        data = yaml.safe_load(f)

    info = data.get("rosbag2_bagfile_information", {})
    if not info:
        return None

    start_ns = info.get("starting_time", {}).get("nanoseconds_since_epoch")
    duration_ns = info.get("duration", {}).get("nanoseconds")

    if start_ns is None:
        return None

    start_ts = start_ns / 1e9
    start_dt = datetime.fromtimestamp(start_ts, tz=timezone.utc)
    start_ms = int(start_ns / 1e6)

    end_dt = None
    end_ms = None
    duration_sec = None
    if duration_ns is not None:
        duration_sec = duration_ns / 1e9
        end_ns = start_ns + duration_ns
        end_ms = int(end_ns / 1e6)
        end_dt = datetime.fromtimestamp(start_ts + duration_sec, tz=timezone.utc)

    msg_count = info.get("message_count", 0)

    return {
        "start_ns": start_ns,
        "end_ns": end_ns if duration_ns is not None else None,
        "duration_sec": duration_sec,
        "message_count": msg_count,
    }


def fmt_ns(ns: int) -> str:
    ms = ns // 1_000_000
    ts = ns / 1e9
    dt = datetime.fromtimestamp(ts, tz=BEIJING_TZ)
    return dt.strftime("%Y-%m-%d %H:%M:%S") + f".{ms % 1000:03d}"


def analyze_root(root_dir: str, csv_path: str) -> None:
    bag_dirs = []
    for dirpath, dirnames, filenames in os.walk(root_dir):
        if "metadata.yaml" in filenames:
            bag_dirs.append(dirpath)

    if not bag_dirs:
        print(f"未在 {root_dir} 下找到任何 rosbag 目录")
        return

    bag_dirs.sort()

    def build_rows():
        for bag_dir in bag_dirs:
            meta_path = os.path.join(bag_dir, "metadata.yaml")
            try:
                info = parse_metadata(meta_path)
            except Exception:
                continue
            if info is None:
                continue
            basename = os.path.basename(bag_dir)

            chan_no = ""
            parts = bag_dir.split(os.path.sep)  # 从根到叶
            for path in reversed(parts):  # 从最深开始
                if not path:  # 跳过空（如根目录）
                    continue
                for key in CHAN_MAP:
                    if key and path.startswith(key):
                        chan_no = key
                        break
                if chan_no:
                    break

            start_str = fmt_ns(info["start_ns"])
            end_str = fmt_ns(info["end_ns"]) if info["end_ns"] else "N/A"
            dur_str = f"{info['duration_sec']:.1f}" if info["duration_sec"] is not None else "N/A"
            yield [basename, chan_no, start_str, end_str, dur_str, str(info["message_count"]), bag_dir]

    # 写入 CSV
    with open(csv_path, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(["目录名", "设备", "开始时间（北京时间）", "结束时间（北京时间）", "持续(秒)", "消息数", "路径"])
        rows = list(build_rows())
        writer.writerows(rows)

    # 打印到终端
    # print(f"{'目录名':<60} {'开始时间（北京时间）':<26}  {'结束时间（北京时间）':<26}  {'持续(秒)':>10}  {'消息数':>8}")
    # print("-" * 150)

    for row in rows:
        basename, chan_no, start_str, end_str, dur_str, msg_count, bag_dir = row
        # print(f"{basename:<60} {start_str:<26}  {end_str:<26}  {dur_str:>10}  {msg_count:>8}")
        print(f"  路径: {bag_dir}")

    print(f"\nCSV 已保存至: {csv_path}")


def main():
    root_dirs = [
        "/mnt/data_storage/研发/HIT/新版堆扫",
        # "/mnt/data_storage/研发/HIT/堆场",
    ]

    for root_dir in root_dirs:
        name = os.path.basename(root_dir)
        csv_path = os.path.join(root_dir, f"{name}_bag_metadata.csv")
        analyze_root(root_dir, csv_path)


if __name__ == "__main__":
    main()
