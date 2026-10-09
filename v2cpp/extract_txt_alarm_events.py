#!/usr/bin/env python3
"""从离线点云分析文本中按数据包汇总雷达报警并导出 CSV。"""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path


PACKAGE_RE = re.compile(r"^处理:\s+(.+?)\s*$")
LIDAR_RE = re.compile(
    r"^\[(L\d+)\]\s+ts=(\d+)_(\d+)\s+"
    r"(\d{4}_\d{2}_\d{2}\s+\d{2}:\d{2}:\d{2}\.\d{3})"
)
DISTANCE_RE = re.compile(
    r"^(\d+)_(\d+)\s+nearest_distance=([-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*$"
)
LIDAR_TO_AREA = {"L21": 1, "L20": 2, "L18": 3, "L19": 4}
BEIJING_TZ = timezone(timedelta(hours=8))
OUTPUT_HEADER = [
    "第一层数据集文件夹", "报警雷达", "区域", "报警时间（北京时间）",
    "结束时间（北京时间）", "持续时长（秒）", "报警最小距离（米）",
    "报警级别", "报警消息数", "DB3文件",
]


@dataclass(frozen=True)
class AlarmSample:
    timestamp: float
    time_text: str
    lidar: str
    distance: float
    package: str


def timestamp_value(seconds: str, fraction: str) -> float:
    # 日志格式固定为“秒_毫秒”，毫秒字段可能省略前导零，例如 _98 表示 098 ms。
    return int(seconds) + int(fraction) / 1000


def parse_samples(
    input_path: Path, alarm_distance: float
) -> tuple[list[str], list[AlarmSample], int]:
    packages: list[str] = []
    samples: list[AlarmSample] = []
    package = ""
    last_lidar = ""
    last_lidar_timestamp: float | None = None
    failed_package_count = 0

    with input_path.open("r", encoding="utf-8", errors="replace") as source:
        for line in source:
            line = line.rstrip("\r\n")

            if match := PACKAGE_RE.match(line):
                package = Path(match.group(1)).name
                packages.append(package)
                last_lidar = ""
                last_lidar_timestamp = None
                continue

            if line.startswith("[PLC] 打开 rosbag 失败:"):
                failed_package_count += 1
                continue

            if match := LIDAR_RE.match(line):
                last_lidar = match.group(1)
                last_lidar_timestamp = timestamp_value(match.group(2), match.group(3))
                continue

            match = DISTANCE_RE.match(line)
            if not match or not package or not last_lidar:
                continue

            timestamp = timestamp_value(match.group(1), match.group(2))
            distance = float(match.group(3))
            if distance >= alarm_distance:
                continue

            # 距离结果紧跟触发计算的雷达帧；时间不一致时不冒然归属雷达。
            if last_lidar_timestamp is None or abs(timestamp - last_lidar_timestamp) > 0.002:
                continue

            time_text = datetime.fromtimestamp(timestamp, tz=BEIJING_TZ).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            samples.append(
                AlarmSample(timestamp, time_text, last_lidar, distance, package)
            )

    return packages, samples, failed_package_count


def alarm_level(samples: list[AlarmSample], stop_distance: float) -> str:
    has_stop = any(sample.distance < stop_distance for sample in samples)
    has_slowdown = any(sample.distance >= stop_distance for sample in samples)
    if has_stop and has_slowdown:
        return "减速报警→停止报警"
    return "停止报警" if has_stop else "减速报警"


def read_reference_rows(reference_path: Path) -> tuple[list[str], list[str]]:
    with reference_path.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.reader(source)
        next(reader, None)
        package_rows = list(dict.fromkeys(
            part.strip() for row in reader if row for part in row[0].split(";") if part.strip()
        ))
    return OUTPUT_HEADER.copy(), package_rows


def write_csv(
    header: list[str],
    package_rows: list[str],
    samples: list[AlarmSample],
    output_path: Path,
    stop_distance: float,
) -> int:
    samples_by_package: dict[str, list[AlarmSample]] = {}
    for sample in samples:
        samples_by_package.setdefault(sample.package, []).append(sample)

    with output_path.open("w", encoding="utf-8-sig", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(header)
        package_with_alarm_count = 0
        for package_cell in package_rows:
            package_samples = samples_by_package.get(package_cell, [])
            if not package_samples:
                writer.writerow([package_cell, "", "", "", "", "", "", "", "", ""])
                continue

            package_with_alarm_count += 1
            package_samples.sort(key=lambda sample: sample.timestamp)
            samples_by_lidar: dict[str, list[AlarmSample]] = {}
            for sample in package_samples:
                samples_by_lidar.setdefault(sample.lidar, []).append(sample)
            rows = []
            for lidar, lidar_samples in sorted(
                samples_by_lidar.items(), key=lambda item: (item[1][0].timestamp, item[0])
            ):
                start, end = lidar_samples[0], lidar_samples[-1]
                rows.append([
                    lidar,
                    str(LIDAR_TO_AREA.get(lidar, "")),
                    start.time_text,
                    end.time_text,
                    f"{end.timestamp - start.timestamp:.3f}",
                    f"{min(sample.distance for sample in lidar_samples):.3f}",
                    alarm_level(lidar_samples, stop_distance),
                    str(len(lidar_samples)),
                    "",
                ])
            writer.writerow([package_cell, *[";".join(values) for values in zip(*rows)]])
    return package_with_alarm_count


def main() -> None:
    parser = argparse.ArgumentParser(description="按数据包汇总文本中的雷达报警并输出 CSV")
    parser.add_argument("input", type=Path, help="输入文本路径，例如 /home/wxh/code/v2cpp/3.txt")
    parser.add_argument("-o", "--output", type=Path, help="输出 CSV 路径")
    parser.add_argument(
        "--reference",
        type=Path,
        default=Path(__file__).with_name("1_大车防撞雷达报警_按数据集汇总.csv"),
        help="仅使用参考 CSV 第一列确定数据包及顺序，每个数据包输出一行，不修改原表",
    )
    parser.add_argument("--alarm-distance", type=float, default=16.0, help="报警距离阈值，默认 16 米")
    parser.add_argument("--stop-distance", type=float, default=5.5, help="停止报警距离阈值，默认 5.5 米")
    args = parser.parse_args()

    input_path = args.input.resolve()
    output_path = (
        args.output or input_path.with_name(f"{input_path.stem}_大车防撞雷达报警.csv")
    ).resolve()

    packages, samples, failed_package_count = parse_samples(input_path, args.alarm_distance)
    header, package_rows = read_reference_rows(args.reference.resolve())
    package_with_alarm_count = write_csv(
        header, package_rows, samples, output_path, args.stop_distance
    )

    print(f"已读取数据包：{len(packages)} 个（其中打开失败 {failed_package_count} 个）")
    print(f"按参考 CSV 第一列输出：{len(package_rows)} 行")
    print(f"报警距离记录：{len(samples)} 条")
    print(f"有报警信息的行：{package_with_alarm_count} 行")
    print(f"无报警信息的行：{len(package_rows) - package_with_alarm_count} 行")
    print(f"CSV：{output_path}")


if __name__ == "__main__":
    main()
