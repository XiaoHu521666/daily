#!/usr/bin/env python3
"""从大车防撞节点日志中提取连续报警事件并导出 CSV。"""

import argparse
import ast
import csv
import re
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


TIMESTAMP_FORMAT = "%Y-%m-%d %H:%M:%S.%f"
BLOCK_START_RE = re.compile(
    r"^(?P<time>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})"
    r".*\| _send_fused_det_msg:\d+ -\s*$"
)
LOG_LINE_RE = re.compile(r"^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}")


@dataclass
class AlarmSample:
    time: datetime
    area: str
    side: str
    level: str
    devices: set[str]
    sources: set[str]
    min_distance: float


@dataclass
class AlarmEvent:
    start: datetime
    end: datetime
    side: str
    levels: list[str]
    devices: set[str]
    sources: set[str]
    min_distance: float


def parse_devices(value: str) -> list[str]:
    try:
        devices = ast.literal_eval(value)
    except (SyntaxError, ValueError):
        return []
    return [str(device) for device in devices] if isinstance(devices, list) else []


def make_sample(
    fields: dict[str, str],
    timestamp: datetime,
    area_devices: dict[str, dict[str, set[str]]],
) -> AlarmSample | None:
    zone1 = fields.get("zone1") == "True"
    zone2 = fields.get("zone2") == "True"
    if not zone1 and not zone2:
        return None

    try:
        camera_distance = float(fields["distance_camera"])
        lidar_distance = float(fields["distance_lidar"])
    except (KeyError, ValueError):
        return None

    area = fields.get("area", "")
    level = "停止报警" if zone2 else "减速报警"
    sources: set[str] = set()
    devices: set[str] = set()
    distances: list[float] = []

    if lidar_distance < 9999:
        sources.add("雷达")
        devices.update(area_devices.get(area, {}).get("雷达", set()))
        distances.append(lidar_distance)
    if camera_distance < 9999:
        sources.add("相机")
        devices.update(area_devices.get(area, {}).get("相机", set()))
        distances.append(camera_distance)
    if not distances:
        return None

    if not devices:
        devices.update(parse_devices(fields.get("device", "[]")))
    side = "左侧" if area in {"1", "3"} else "右侧" if area in {"2", "4"} else f"区域{area}"
    return AlarmSample(timestamp, area, side, level, devices, sources, min(distances))


def parse_log(log_path: Path) -> list[AlarmSample]:
    blocks: list[tuple[datetime, dict[str, str]]] = []
    block_time: datetime | None = None
    fields: dict[str, str] = {}

    def finish_block() -> None:
        if block_time is not None:
            blocks.append((block_time, fields.copy()))

    with log_path.open("r", encoding="utf-8", errors="replace") as log_file:
        for line in log_file:
            start_match = BLOCK_START_RE.match(line)
            if start_match:
                finish_block()
                block_time = datetime.strptime(start_match.group("time"), TIMESTAMP_FORMAT)
                fields = {}
                continue

            if block_time is None:
                continue
            if LOG_LINE_RE.match(line):
                finish_block()
                block_time = None
                fields = {}
                continue

            key, separator, value = line.strip().partition(":")
            if separator:
                fields[key.strip()] = value.strip()

    finish_block()
    area_devices: dict[str, dict[str, set[str]]] = {}
    for _, block_fields in blocks:
        area = block_fields.get("area", "")
        for device in parse_devices(block_fields.get("device", "[]")):
            source = "雷达" if device.startswith("L") else "相机" if device.startswith("C") else ""
            if source:
                area_devices.setdefault(area, {}).setdefault(source, set()).add(device)

    samples = [make_sample(block_fields, timestamp, area_devices) for timestamp, block_fields in blocks]
    return [sample for sample in samples if sample is not None]


def merge_events(samples: list[AlarmSample], max_gap_seconds: float) -> list[AlarmEvent]:
    events: list[AlarmEvent] = []
    active: dict[str, AlarmEvent] = {}

    for sample in sorted(samples, key=lambda item: item.time):
        key = sample.side
        event = active.get(key)
        if event is None or (sample.time - event.end).total_seconds() > max_gap_seconds:
            if event is not None:
                events.append(event)
            active[key] = AlarmEvent(
                sample.time,
                sample.time,
                sample.side,
                [sample.level],
                sample.devices.copy(),
                sample.sources.copy(),
                sample.min_distance,
            )
            continue

        event.end = sample.time
        if sample.level not in event.levels:
            event.levels.append(sample.level)
        event.devices.update(sample.devices)
        event.sources.update(sample.sources)
        event.min_distance = min(event.min_distance, sample.min_distance)

    events.extend(active.values())
    return sorted(events, key=lambda event: event.start)


def write_csv(events: list[AlarmEvent], output_path: Path) -> None:
    with output_path.open("w", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["报警时间", "持续时间(秒)", "报警来源", "设备", "报警级别", "最小距离(m)"])
        for event in events:
            writer.writerow(
                [
                    event.start.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    f"{(event.end - event.start).total_seconds():.3f}",
                    "+".join(source for source in ("雷达", "相机") if source in event.sources),
                    "+".join(sorted(event.devices, key=lambda device: (device[0], int(device[1:])))),
                    "→".join(event.levels),
                    f"{event.min_distance:.3f}",
                ]
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="统计大车防撞日志中的连续报警事件")
    parser.add_argument("log", type=Path, help="输入日志路径")
    parser.add_argument("-o", "--output", type=Path, help="输出 CSV 路径")
    parser.add_argument(
        "--max-gap",
        type=float,
        default=1.0,
        help="同设备同级别连续报警允许的最大间隔秒数（默认：1.0）",
    )
    args = parser.parse_args()

    output_path = args.output or args.log.with_name(f"{args.log.stem}_报警统计.csv")
    samples = parse_log(args.log)
    events = merge_events(samples, args.max_gap)
    write_csv(events, output_path)
    print(f"已读取 {len(samples)} 条报警记录，合并为 {len(events)} 个报警事件")
    print(f"CSV 已保存到：{output_path.resolve()}")


if __name__ == "__main__":
    main()
#运行命令：python3 /home/wxh/code/v2cpp/export_collision_alarms.py "/home/wxh/code/v2cpp/algl_gantry_collision_detect_node_v2.2026-09-08_00-01-06_603933.log"