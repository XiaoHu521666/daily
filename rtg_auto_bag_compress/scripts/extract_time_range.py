#!/usr/bin/env python3

import argparse
import calendar
import json
import re
import shutil
import sqlite3
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError


def parse_local_time(value: str, time_zone: ZoneInfo) -> int:
    normalized = value.strip().replace("T", " ")
    date_time, dot, fraction = normalized.partition(".")
    try:
        parsed = datetime.strptime(date_time, "%Y-%m-%d %H:%M:%S")
    except ValueError as error:
        raise argparse.ArgumentTypeError(
            f"时间格式错误: {value}，应为 YYYY-MM-DD HH:MM:SS[.纳秒]"
        ) from error

    if dot:
        if not fraction.isdigit() or len(fraction) > 9:
            raise argparse.ArgumentTypeError("小数秒必须是 1 到 9 位数字")
        nanoseconds = int(fraction.ljust(9, "0"))
    else:
        nanoseconds = 0

    utc_time = parsed.replace(tzinfo=time_zone).astimezone(timezone.utc)
    epoch_seconds = calendar.timegm(utc_time.utctimetuple())
    return epoch_seconds * 1_000_000_000 + nanoseconds


def format_local_time(timestamp_ns: int, time_zone: ZoneInfo) -> str:
    seconds, nanoseconds = divmod(timestamp_ns, 1_000_000_000)
    local_time = datetime.fromtimestamp(seconds, timezone.utc).astimezone(time_zone)
    return f"{local_time:%Y-%m-%d %H:%M:%S}.{nanoseconds:09d}"


def read_bag_range(metadata_path: Path) -> tuple[int, int, int]:
    content = metadata_path.read_text(encoding="utf-8")
    top_level = content.split("\n  topics_with_message_count:", maxsplit=1)[0]

    start_match = re.search(
        r"^  starting_time:\s*\n    nanoseconds_since_epoch:\s*(\d+)\s*$",
        top_level,
        re.MULTILINE,
    )
    duration_match = re.search(
        r"^  duration:\s*\n    nanoseconds:\s*(\d+)\s*$", top_level, re.MULTILINE
    )
    count_match = re.search(r"^  message_count:\s*(\d+)\s*$", top_level, re.MULTILINE)
    if not start_match or not duration_match or not count_match:
        raise ValueError(f"无法读取 rosbag2 时间元数据: {metadata_path}")

    start_ns = int(start_match.group(1))
    duration_ns = int(duration_match.group(1))
    message_count = int(count_match.group(1))
    return start_ns, start_ns + duration_ns, message_count


def find_matching_bags(source: Path, start_ns: int, end_ns: int):
    matches = []
    for metadata_path in sorted(source.rglob("metadata.yaml")):
        bag_start_ns, bag_end_ns, message_count = read_bag_range(metadata_path)
        if bag_start_ns < end_ns and bag_end_ns >= start_ns:
            matches.append((metadata_path.parent, bag_start_ns, bag_end_ns, message_count))
    return matches


def run_command(command: list[str]) -> None:
    subprocess.run(command, check=True)


def convert_to_uncompressed(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="rtg_bag_convert_") as temporary_directory:
        temporary_path = Path(temporary_directory)
        temporary_source = temporary_path / source.name
        ignore = shutil.ignore_patterns("*.db3") if any(source.glob("*.zstd")) else None
        shutil.copytree(source, temporary_source, ignore=ignore)

        options_path = temporary_path / "output_options.yaml"
        options_path.write_text(
            "output_bags:\n"
            f"  - uri: {json.dumps(str(destination))}\n"
            "    storage_id: sqlite3\n"
            "    all_topics: true\n",
            encoding="utf-8",
        )
        run_command(
            [
                "ros2",
                "bag",
                "convert",
                "-i",
                str(temporary_source),
                "-o",
                str(options_path),
            ]
        )


def trim_database(database_path: Path, start_ns: int, end_ns: int) -> tuple[int, int]:
    with sqlite3.connect(database_path) as connection:
        original_count = connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        connection.execute(
            "DELETE FROM messages WHERE timestamp < ? OR timestamp >= ?",
            (start_ns, end_ns),
        )
        kept_count = connection.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        connection.commit()
        connection.execute("VACUUM")
    return original_count, kept_count


def destination_for(output: Path, source: Path, bag_path: Path) -> Path:
    relative_path = bag_path.relative_to(source)
    return output / relative_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="按服务器接收时间筛选、解压并精确裁剪变化事件 rosbag2 文件"
    )
    parser.add_argument("--source", required=True, type=Path, help="压缩事件包根目录")
    parser.add_argument("--start", required=True, help="开始时间，包含该时刻")
    parser.add_argument("--end", required=True, help="结束时间，不包含该时刻")
    parser.add_argument(
        "--output", type=Path, help="导出根目录；非 --list-only 模式必须提供且不能已存在"
    )
    parser.add_argument("--timezone", default="Asia/Shanghai", help="输入时间所在时区")
    parser.add_argument(
        "--list-only", action="store_true", help="只列出命中的压缩事件包，不复制或解压"
    )
    return parser


def main() -> int:
    parser = build_parser()
    arguments = parser.parse_args()

    try:
        time_zone = ZoneInfo(arguments.timezone)
    except ZoneInfoNotFoundError:
        parser.error(f"系统中不存在时区: {arguments.timezone}")

    source = arguments.source.expanduser().resolve()
    if not source.is_dir():
        parser.error(f"源目录不存在: {source}")
    if not arguments.list_only and arguments.output is None:
        parser.error("执行解压导出时必须提供 --output")

    try:
        start_ns = parse_local_time(arguments.start, time_zone)
        end_ns = parse_local_time(arguments.end, time_zone)
    except argparse.ArgumentTypeError as error:
        parser.error(str(error))
    if start_ns >= end_ns:
        parser.error("--end 必须晚于 --start")

    matches = find_matching_bags(source, start_ns, end_ns)
    print(
        f"查询区间 [{format_local_time(start_ns, time_zone)}, "
        f"{format_local_time(end_ns, time_zone)})，命中 {len(matches)} 个事件包"
    )
    for bag_path, bag_start_ns, bag_end_ns, message_count in matches:
        print(
            f"  {bag_path}\n"
            f"    原始范围 {format_local_time(bag_start_ns, time_zone)} ~ "
            f"{format_local_time(bag_end_ns, time_zone)}，{message_count} 条"
        )

    if arguments.list_only or not matches:
        return 0

    output = arguments.output.expanduser().resolve()
    if output.exists():
        parser.error(f"输出目录已存在，为避免覆盖请换一个新目录: {output}")
    if output == source or source in output.parents:
        parser.error("输出目录不能位于源归档目录内部")
    output.mkdir(parents=True)

    total_original = 0
    total_kept = 0
    for bag_path, _, _, _ in matches:
        destination = destination_for(output, source, bag_path)
        print(f"转换为未压缩 bag: {bag_path} -> {destination}")
        convert_to_uncompressed(bag_path, destination)

        databases = sorted(destination.glob("*.db3"))
        if not databases:
            raise RuntimeError(f"解压后未找到 db3 文件: {destination}")
        bag_kept = 0
        for database in databases:
            original_count, kept_count = trim_database(database, start_ns, end_ns)
            total_original += original_count
            total_kept += kept_count
            bag_kept += kept_count
        if bag_kept == 0:
            shutil.rmtree(destination)
            print("  精确时间范围内没有消息，跳过该候选事件包")
            continue
        (destination / "metadata.yaml").unlink(missing_ok=True)
        run_command(["ros2", "bag", "reindex", "-s", "sqlite3", str(destination)])
        print(f"  精确保留 {bag_kept} 条消息")

    print(f"导出完成：候选事件包共 {total_original} 条，时间范围内保留 {total_kept} 条")
    print(f"结果目录: {output}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, ValueError, sqlite3.Error, subprocess.CalledProcessError) as error:
        print(f"错误: {error}", file=sys.stderr)
        sys.exit(1)
