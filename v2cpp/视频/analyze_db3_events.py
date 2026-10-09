#!/usr/bin/env python3
"""扫描 rosbag2 SQLite DB3，提取设备字段并汇总为一个事件 CSV。

程序不依赖 yaml、rosbag2_py 或 sqlite3 命令行工具。它直接读取 DB3 的
topics/messages 表，并对本项目常用的 PlcMsg、GantryPosMsg、ExecStatusMsg
做 CDR 解码。若执行环境已 source ROS2，则优先使用 ROS2 类型支持解码；
单独使用本脚本时会自动回退到内置 CDR 解码器。

默认输入是用户给出的 Windows 共享路径。WSL 下会自动把
\\\\10.0.0.110\\Data_Storage\\... 映射为 /mnt/data_storage/...
。
"""

from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
import re
import shutil
import sqlite3
import struct
import sys
import time
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable, Iterable, Optional


BEIJING_TZ = timezone(timedelta(hours=8))
DEFAULT_INPUT = "/mnt/data_storage/研发/HIT/堆场/652_2026_07_21_堆场逻辑控制/652_2026_07_21_1"
DEFAULT_OUTPUT_NAME = "新版堆扫_DB3事件汇总.csv"

PLC_TYPE = "sensor_device_interfaces/msg/PlcMsg"
GANTRY_TYPE = "rtg_algorithm_interfaces/msg/GantryPosMsg"
EXEC_TYPE = "rtg_algorithm_interfaces/msg/ExecStatusMsg"

ACTION_NAMES = {1: "停车/待机", 2: "抓箱", 3: "放箱"}
MOVE_STEP_NAMES = {
    "gantry_moving": "大车移动",
    "gantry_arrive": "大车到位",
    "gantry_adjusting": "大车调整",
    "trolley_moving": "小车移动",
    "trolley_arrive": "小车到位",
    "trolley_adjusting": "小车调整",
    "hoist_arrive": "起升到位",
    "hoist_lowering": "起升向下",
    "hoist_lifting": "起升向上",
    "picking": "抓箱中",
    "landing": "放箱中",
    "repeat_landing": "二次放箱中",
    "pick_complete": "抓箱完成",
    "drop_complete": "放箱完成",
    "waiting_cps": "等待集卡到位",
    "waiting_confirm": "等待远控确认",
}

# 653 当前使用的排位参考小车位置；索引 0 是非业务 0 排，1~6 为业务排。
ROW_POSITION_REFERENCE = (0.35, 3.58, 6.42, 9.26, 12.10, 14.94, 17.78)

CSV_FIELDS = [
    "包目录",
    "DB3文件",
    "记录类型",
    "开始时间（北京时间）",
    "结束时间（北京时间）",
    "开始时间戳（毫秒）",
    "结束时间戳（毫秒）",
    "经历时间（秒）",
    "消息数",
    "Topic/字段来源",
    "事件消息数",
    "大车贝位_起",
    "大车贝位_止",
    "Block",
    "大车坐标_起",
    "大车坐标_止",
    "小车位置_起",
    "小车位置_止",
    "小车速度平均值",
    "小车速度最大绝对值",
    "小车方向",
    "起始排",
    "终止排",
    "层",
    "任务ID",
    "动作类型",
    "状态/步骤",
    "载箱状态",
    "抓放信号",
    "说明",
    "解码方式",
    "异常或限制",
]


class DecodeError(ValueError):
    """CDR 消息无法按已知接口结构读取。"""


class CdrReader:
    """读取 ROS 2 CDR little/big endian 数据。

    CDR 的四字节封装头不属于后续字段的对齐基准，因此对齐基准从
    payload 起点（offset=4）计算，而不是从整个 blob 的 offset=0 计算。
    """

    def __init__(self, data: bytes):
        if len(data) < 4:
            raise DecodeError("CDR 数据少于 4 字节封装头")
        self.data = data
        self.pos = 4
        self.fmt = "<" if data[1] == 1 else ">"

    def align(self, size: int) -> None:
        self.pos += -((self.pos - 4) % size) % size

    def read(self, fmt: str) -> Any:
        size = struct.calcsize(self.fmt + fmt)
        self.align(size)
        end = self.pos + size
        if end > len(self.data):
            raise DecodeError(f"CDR 字段越界: pos={self.pos}, size={size}, len={len(self.data)}")
        value = struct.unpack_from(self.fmt + fmt, self.data, self.pos)[0]
        self.pos = end
        return value

    def i32(self) -> int:
        return int(self.read("i"))

    def u32(self) -> int:
        return int(self.read("I"))

    def i64(self) -> int:
        return int(self.read("q"))

    def f32(self) -> float:
        return float(self.read("f"))

    def f64(self) -> float:
        return float(self.read("d"))

    def boolean(self) -> bool:
        return bool(self.read("B"))

    def string(self) -> str:
        self.align(4)
        length = self.u32()
        end = self.pos + length
        if end > len(self.data):
            raise DecodeError(f"CDR 字符串越界: pos={self.pos}, length={length}, len={len(self.data)}")
        raw = self.data[self.pos:end]
        self.pos = end
        if raw.endswith(b"\0"):
            raw = raw[:-1]
        return raw.decode("utf-8", errors="replace")

    def header(self) -> dict[str, Any]:
        return {"sec": self.i32(), "nanosec": self.u32(), "frame_id": self.string()}

    def sequence_f64(self) -> list[float]:
        return [self.f64() for _ in range(self.u32())]


def _plc_schema() -> list[tuple[str, str]]:
    """PlcMsg.msg 的字段顺序；顺序改变会影响 CDR 后续字段偏移。"""

    fields: list[tuple[str, str]] = [
        ("plc_id", "i32"),
        ("source_timestamp", "i64"),
        ("plc_connect", "bool"),
        ("no_estop", "bool"),
        ("spreader_lock", "bool"),
        ("spreader_unlock", "bool"),
        ("spreader_20f", "bool"),
        ("spreader_40f", "bool"),
        ("spreader_45f", "bool"),
        ("spreader_landed", "bool"),
        ("spreader_pump_on_state", "i32"),
        ("spreader_home", "i32"),
        ("spreader_weight", "f64"),
        ("ctrl_on_state", "i32"),
        ("ctrl_mode_local", "bool"),
        ("ctrl_mode_remote", "bool"),
        ("ctrl_mode_manual", "bool"),
        ("ctrl_mode_auto", "bool"),
        ("ros_connect_state", "bool"),
        ("gantry_velocity_1", "f64"),
        ("gantry_velocity_2", "f64"),
        ("gantry_velocity_3", "f64"),
        ("gantry_velocity_4", "f64"),
        ("gantry_pos", "f64"),
        ("joystick_gantry_fb", "i32"),
        ("joystick_trolley_fb", "i32"),
        ("joystick_hoist_fb", "i32"),
        ("trolley_pos", "f64"),
        ("trolley_velocity", "f64"),
        ("hoist_height", "f64"),
        ("hoist_velocity", "f64"),
        ("left_crane_state", "i32"),
        ("left_crane_gantry_pos", "f64"),
        ("left_crane_gantry_velocity", "f64"),
        ("right_crane_state", "i32"),
        ("right_crane_gantry_pos", "f64"),
        ("right_crane_gantry_velocity", "f64"),
        ("trolley_anchor_state", "i32"),
        ("spreader_skew_right", "bool"),
        ("spreader_skew_left", "bool"),
        ("spreader20ft_cmd", "bool"),
        ("spreader40ft_cmd", "bool"),
        ("spreader45ft_cmd", "bool"),
        ("spreader_rope_loose", "bool"),
        ("flipper_left_down", "bool"),
        ("flipper_right_down", "bool"),
        ("over_load_fb", "bool"),
        ("wheel_at0", "bool"),
        ("wheel_at90", "bool"),
        ("wheel_at16", "bool"),
        ("wheel_at_park", "bool"),
        ("wheel_locked", "bool"),
        ("wheel_unlocked", "bool"),
        ("wheel_pump1_on", "bool"),
        ("wheel_pump2_on", "bool"),
        ("wheel_pump_off", "bool"),
        ("wheel_turn_auto_fb", "bool"),
        ("wheel_turn_manual_fb", "bool"),
    ]
    fields.extend((f"laser{i}_fault", "bool") for i in range(1, 9))
    fields.extend((f"laser{i}_height", "i32") for i in range(1, 9))
    fields.extend(
        [
            ("auto_anti_sway", "bool"),
            ("gantry_move_assist", "bool"),
            ("agss1", "i32"),
            ("agss2", "i32"),
            ("aptt_gsd", "bool"),
            ("general_bypass_fb", "bool"),
            ("human_bypass_fb", "bool"),
            ("agss_left_gsd", "bool"),
            ("agss_right_gsd", "bool"),
            ("speed_limit", "bool"),
            ("spreader_not_top_limit", "bool"),
            ("bypass_atl_fb", "bool"),
            ("bypass_jack_set_fb", "bool"),
            ("bypass_landing_fb", "bool"),
            ("bypass_ls_fb", "bool"),
            ("bypass_over_height_fb", "bool"),
            ("bypass_over_load_fb", "bool"),
            ("bypass_spreader_fb", "bool"),
            ("gantry_run_permit_left", "bool"),
            ("gantry_run_permit_right", "bool"),
            ("trolley_run_permit_forward", "bool"),
            ("trolley_run_permit_backward", "bool"),
            ("hoist_run_permit_up", "bool"),
            ("hoist_run_permit_down", "bool"),
            ("skew_pos", "i32"),
        ]
    )
    return fields


PLC_SCHEMA = _plc_schema()


def manual_decode(type_name: str, data: bytes) -> dict[str, Any]:
    reader = CdrReader(data)
    values: dict[str, Any] = {"header": reader.header()}

    if type_name == PLC_TYPE:
        for name, kind in PLC_SCHEMA:
            values[name] = getattr(reader, {"i32": "i32", "i64": "i64", "f64": "f64", "bool": "boolean"}[kind])()
        return values

    if type_name == GANTRY_TYPE:
        values.update(
            {
                "block": reader.string(),
                "bay_no": reader.string(),
                "bay_size": reader.i32(),
                "gantry_pos": reader.f32(),
                "gps_position": reader.sequence_f64(),
                "gantry_deviation": reader.i32(),
                "gantry_angle": reader.f32(),
                "gps_state": reader.i32(),
                "bay_on_position": reader.boolean(),
                "vision_pos_state": reader.i32(),
                "vision_offset": reader.f32(),
                "vision_deviation1": reader.f32(),
                "vision_deviation2": reader.f32(),
                "in_auto_alignment_range": reader.boolean(),
                "on_position_distance_1": reader.i32(),
                "on_position_distance_2": reader.i32(),
                "gantry_left_deviation": reader.i32(),
                "gantry_right_deviation": reader.i32(),
                "trust_state": reader.i32(),
                "auto_steering_permit": reader.boolean(),
                "offset_from_bay_center": reader.f32(),
            }
        )
        return values

    if type_name == EXEC_TYPE:
        for name, kind in [
            ("task_id", "string"),
            ("action_index", "i32"),
            ("state", "string"),
            ("move_step", "string"),
            ("action_type", "i32"),
            ("action_mode", "string"),
            ("position_type", "i32"),
            ("row_no", "string"),
            ("block", "string"),
            ("tier_no", "string"),
            ("bay_no", "string"),
            ("bay_size", "i32"),
            ("truck_pos", "string"),
            ("truck_type", "string"),
            ("hoist_height", "f32"),
            ("trolley_pos", "f32"),
            ("gantry_pos", "f32"),
            ("object_height", "f32"),
        ]:
            values[name] = getattr(reader, {"string": "string", "i32": "i32", "f32": "f32"}[kind])()
        return values

    raise DecodeError(f"没有内置解码器: {type_name}")


class Ros2Decoder:
    """可选 ROS2 解码器；没有 source ROS2 时构造失败，由调用方回退。"""

    def __init__(self) -> None:
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        self.deserialize_message = deserialize_message
        self.get_message = get_message
        self.classes: dict[str, Any] = {}

    def decode(self, type_name: str, data: bytes) -> dict[str, Any]:
        if type_name not in self.classes:
            self.classes[type_name] = self.get_message(type_name)
        message = self.deserialize_message(data, self.classes[type_name])
        result: dict[str, Any] = {}
        for name in self._fields_for(type_name):
            result[name] = getattr(message, name)
        return result

    @staticmethod
    def _fields_for(type_name: str) -> list[str]:
        if type_name == PLC_TYPE:
            return ["header"] + [name for name, _ in PLC_SCHEMA]
        if type_name == GANTRY_TYPE:
            return [
                "header", "block", "bay_no", "bay_size", "gantry_pos", "gps_position",
                "gantry_deviation", "gantry_angle", "gps_state", "bay_on_position",
                "vision_pos_state", "vision_offset", "vision_deviation1", "vision_deviation2",
                "in_auto_alignment_range", "on_position_distance_1", "on_position_distance_2",
                "gantry_left_deviation", "gantry_right_deviation", "trust_state",
                "auto_steering_permit", "offset_from_bay_center",
            ]
        if type_name == EXEC_TYPE:
            return [
                "header", "task_id", "action_index", "state", "move_step", "action_type",
                "action_mode", "position_type", "row_no", "block", "tier_no", "bay_no",
                "bay_size", "truck_pos", "truck_type", "hoist_height", "trolley_pos",
                "gantry_pos", "object_height",
            ]
        return []


@dataclass
class Sample:
    timestamp: int
    values: dict[str, Any]


@dataclass
class TopicInfo:
    topic_id: int
    name: str
    type_name: str
    serialization: str
    count: int = 0
    first_ns: Optional[int] = None
    last_ns: Optional[int] = None


@dataclass
class BagData:
    path: Path
    total_messages: int = 0
    first_ns: Optional[int] = None
    last_ns: Optional[int] = None
    topics: list[TopicInfo] = field(default_factory=list)
    plc: list[Sample] = field(default_factory=list)
    gantry: list[Sample] = field(default_factory=list)
    execution: list[Sample] = field(default_factory=list)
    decoder: str = "未解码"
    decode_failures: Counter[str] = field(default_factory=Counter)
    first_decode_error: str = ""
    warnings: list[str] = field(default_factory=list)


@dataclass
class MetadataSummary:
    start_ns: Optional[int]
    duration_ns: Optional[int]
    message_count: Optional[int]
    topic_counts: dict[str, tuple[str, int]]


@dataclass
class Segment:
    samples: list[Sample]

    @property
    def start(self) -> int:
        return self.samples[0].timestamp

    @property
    def end(self) -> int:
        return self.samples[-1].timestamp


def normalize_path(raw: str) -> Path:
    """兼容 WSL 挂载路径、Windows UNC 路径和当前系统原生路径。"""

    value = raw.strip().strip('"')
    direct = Path(value)
    if direct.exists():
        return direct.resolve()

    if os.name != "nt" and (value.startswith("\\\\") or value.startswith("//")):
        normalized = value.replace("/", "\\")
        marker = "\\Data_Storage\\"
        lower = normalized.lower()
        marker_index = lower.find(marker.lower())
        if marker_index >= 0:
            suffix = normalized[marker_index + len(marker):].replace("\\", "/")
            mounted = Path("/mnt/data_storage") / suffix
            if mounted.exists() or mounted.parent.exists():
                return mounted.resolve()
    return direct


def fmt_ns(timestamp_ns: Optional[int]) -> str:
    if timestamp_ns is None:
        return ""
    dt = datetime.fromtimestamp(timestamp_ns / 1_000_000_000, tz=BEIJING_TZ)
    millis = (timestamp_ns // 1_000_000) % 1000
    return dt.strftime("%Y-%m-%d %H:%M:%S") + f".{millis:03d}"


def timestamp_ms(timestamp_ns: Optional[int]) -> str:
    if timestamp_ns is None:
        return ""
    return str(timestamp_ns // 1_000_000)


def fmt_number(value: Any, digits: int = 3) -> str:
    if value is None or isinstance(value, bool):
        return "" if value is None else ("是" if value else "否")
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return str(number)
    return f"{number:.{digits}f}"


def duration_seconds(start: Optional[int], end: Optional[int]) -> str:
    if start is None or end is None:
        return ""
    return f"{max(0, end - start) / 1_000_000_000:.3f}"


def topic_is(type_name: str, name: str, expected: str) -> bool:
    if type_name == expected:
        return True
    if expected == PLC_TYPE:
        return "plc" in name.lower() and type_name.endswith("/PlcMsg")
    if expected == GANTRY_TYPE:
        return "gantry" in name.lower() and type_name.endswith("/GantryPosMsg")
    if expected == EXEC_TYPE:
        return "task_exec_status" in name.lower() and type_name.endswith("/ExecStatusMsg")
    return False


def parse_yaml_scalar(text: str) -> str:
    value = text.strip()
    if value.startswith('"'):
        try:
            return str(json.loads(value))
        except json.JSONDecodeError:
            return value.strip('"')
    if value.startswith("'") and value.endswith("'"):
        return value[1:-1].replace("''", "'")
    return value


def read_metadata_summary(bag_dir: Path) -> Optional[MetadataSummary]:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        return None
    start_ns: Optional[int] = None
    duration_ns: Optional[int] = None
    message_count: Optional[int] = None
    topic_counts: dict[str, tuple[str, int]] = {}
    current_topic: Optional[dict[str, Any]] = None
    in_topics = False
    section = ""
    try:
        for raw_line in metadata_path.read_text(encoding="utf-8").splitlines():
            stripped = raw_line.strip()
            indent = len(raw_line) - len(raw_line.lstrip())
            if stripped == "topics_with_message_count:":
                in_topics = True
                section = ""
                continue
            if in_topics and stripped.startswith("- topic_metadata:"):
                current_topic = {}
                continue
            if in_topics and current_topic is not None:
                if stripped.startswith("name:"):
                    current_topic["name"] = parse_yaml_scalar(stripped.split(":", 1)[1])
                elif stripped.startswith("type:"):
                    current_topic["type"] = parse_yaml_scalar(stripped.split(":", 1)[1])
                elif stripped.startswith("message_count:"):
                    name = str(current_topic.get("name", ""))
                    type_name = str(current_topic.get("type", ""))
                    if name:
                        topic_counts[name] = (type_name, int(stripped.split(":", 1)[1].strip()))
                    current_topic = None
                continue
            if indent == 2 and stripped == "duration:":
                section = "duration"
            elif indent == 2 and stripped == "starting_time:":
                section = "starting_time"
            elif indent == 2 and stripped.startswith("message_count:"):
                message_count = int(stripped.split(":", 1)[1].strip())
            elif indent == 4 and stripped.startswith("nanoseconds:"):
                value = int(stripped.split(":", 1)[1].strip())
                if section == "duration":
                    duration_ns = value
            elif indent == 4 and stripped.startswith("nanoseconds_since_epoch:"):
                if section == "starting_time":
                    start_ns = int(stripped.split(":", 1)[1].strip())
    except (OSError, UnicodeError, ValueError) as exc:
        print(f"  metadata.yaml 读取失败，回退 DB3 扫描: {exc}", flush=True)
        return None
    if start_ns is None or duration_ns is None or message_count is None or not topic_counts:
        return None
    return MetadataSummary(start_ns, duration_ns, message_count, topic_counts)


def find_extracted_dir(db3: Path) -> Optional[Path]:
    parent = db3.parent
    candidate = parent.parent / f"{parent.name}_extracted"
    if candidate.is_dir():
        return candidate
    try:
        siblings = sorted(
            path for path in parent.parent.iterdir()
            if path.is_dir() and path.name.startswith(parent.name) and path.name.endswith("_extracted")
        )
    except OSError:
        return None
    return siblings[0] if siblings else None


def find_child_dir(root: Path, names: set[str]) -> Optional[Path]:
    try:
        for child in root.iterdir():
            if child.is_dir() and child.name.lower() in names:
                return child
    except OSError:
        return None
    return None


def read_sidecar_sample(json_path: Path) -> Optional[Sample]:
    try:
        values = json.loads(json_path.read_text(encoding="utf-8"))
        stamp = values.get("header", {}).get("stamp", {})
        sec = int(stamp["sec"])
        nanosec = int(stamp.get("nanosec", 0))
        return Sample(sec * 1_000_000_000 + nanosec, values)
    except (OSError, UnicodeError, json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None


def sidecar_samples(root: Path, folder_names: set[str]) -> list[Sample]:
    folder = find_child_dir(root, folder_names)
    if folder is None:
        return []
    json_files = sorted(folder.glob("*.json"))
    total = len(json_files)
    if not total:
        return []
    samples: list[Sample] = []
    started = time.monotonic()
    last_report = started
    worker_count = min(16, max(4, total))
    if total >= 100:
        print(
            f"  [侧车读取] {folder.name}: 0/{total} 个 JSON；并发 {worker_count} 个读取线程。",
            flush=True,
        )
    with ThreadPoolExecutor(max_workers=worker_count) as executor:
        futures = [executor.submit(read_sidecar_sample, json_path) for json_path in json_files]
        for index, future in enumerate(as_completed(futures), start=1):
            sample = future.result()
            if sample is not None:
                samples.append(sample)
            now = time.monotonic()
            if total >= 100 and (now - last_report >= 5 or index == total):
                elapsed = max(now - started, 0.001)
                print(
                    f"  [侧车读取] {folder.name}: {index}/{total} 个 JSON；"
                    f"有效 {len(samples)} 个；耗时 {elapsed:.0f}s；速率 {index / elapsed:.1f} 个/s。",
                    flush=True,
                )
                last_report = now
    samples.sort(key=lambda item: item.timestamp)
    return samples


def load_from_extracted(db3: Path) -> Optional[BagData]:
    """当 *_extracted 与 metadata.yaml 的目标消息数完全对应时走快速路径。"""

    metadata = read_metadata_summary(db3.parent)
    extracted = find_extracted_dir(db3)
    if metadata is None or extracted is None:
        return None
    bag = BagData(path=db3, decoder="json_extracted_sidecar")
    bag.first_ns = metadata.start_ns
    bag.last_ns = metadata.start_ns + metadata.duration_ns
    bag.total_messages = metadata.message_count

    # 快速路径不打开 DB3；Topic 名称、类型和数量全部来自 metadata.yaml。
    for topic_id, (name, (type_name, count)) in enumerate(metadata.topic_counts.items(), start=1):
        bag.topics.append(TopicInfo(topic_id, name, type_name, "cdr", count))

    sidecars = {
        PLC_TYPE: sidecar_samples(extracted, {"plc"}),
        GANTRY_TYPE: sidecar_samples(extracted, {"gantryposmsg", "gantry_pos_msg"}),
        EXEC_TYPE: sidecar_samples(extracted, {"task_exec_status", "execstatusmsg"}),
    }
    expected_counts = {
        expected: sum(
            count for name, (type_name, count) in metadata.topic_counts.items()
            if topic_is(type_name, name, expected)
        )
        for expected in (PLC_TYPE, GANTRY_TYPE, EXEC_TYPE)
    }
    if any(len(sidecars[expected]) != expected_counts[expected] for expected in expected_counts):
        return None
    bag.plc = sidecars[PLC_TYPE]
    bag.gantry = sidecars[GANTRY_TYPE]
    bag.execution = sidecars[EXEC_TYPE]
    bag.warnings.append("事件字段来自对应 *_extracted JSON；未读取 DB3 messages，消息总数/Topic 来自 metadata.yaml")
    return bag


def open_readonly(db3: Path) -> sqlite3.Connection:
    uri = f"file:{db3}?mode=ro&immutable=1"
    return sqlite3.connect(uri, uri=True, timeout=30)


def _load_bag_db3(db3: Path, ros2_decoder: Optional[Ros2Decoder]) -> BagData:
    bag = BagData(path=db3)
    try:
        with open_readonly(db3) as db:
            topic_rows = db.execute(
                "SELECT id, name, type, serialization_format FROM topics ORDER BY id"
            ).fetchall()
            topics_by_id: dict[int, TopicInfo] = {}
            for topic_id, name, type_name, serialization in topic_rows:
                topic = TopicInfo(topic_id, name, type_name, serialization)
                bag.topics.append(topic)
                topics_by_id[topic_id] = topic

            # timestamp_idx 只读 timestamp/topic_id/id，不把雷达 data BLOB 拉过来。
            # 先一次索引扫描收集统计信息和目标消息的 id，再按主键批量取三类小消息，
            # 避免对 1GB 级 DB3 重复做 GROUP BY 和 Topic 全表扫描。
            wanted = [
                (PLC_TYPE, bag.plc),
                (GANTRY_TYPE, bag.gantry),
                (EXEC_TYPE, bag.execution),
            ]
            wanted_topic_types: dict[int, str] = {}
            for topic in bag.topics:
                for expected_type, _ in wanted:
                    if topic_is(topic.type_name, topic.name, expected_type):
                        wanted_topic_types[topic.topic_id] = expected_type
                        break

            wanted_rows: list[tuple[int, int, str]] = []
            scan_started = time.monotonic()
            last_report = scan_started
            scanned_rows = 0
            print("  [DB3扫描] 正在扫描 messages 索引；大文件期间会每 5 秒报告一次进度。", flush=True)
            try:
                indexed_rows = db.execute(
                    "SELECT id, timestamp, topic_id FROM messages INDEXED BY timestamp_idx ORDER BY timestamp"
                )
            except sqlite3.OperationalError:
                indexed_rows = db.execute(
                    "SELECT id, timestamp, topic_id FROM messages ORDER BY timestamp"
                )
            for message_id, timestamp, topic_id in indexed_rows:
                scanned_rows += 1
                topic = topics_by_id.get(topic_id)
                if topic is None:
                    continue
                topic.count += 1
                topic.first_ns = timestamp if topic.first_ns is None else min(topic.first_ns, timestamp)
                topic.last_ns = timestamp if topic.last_ns is None else max(topic.last_ns, timestamp)
                bag.total_messages += 1
                bag.first_ns = timestamp if bag.first_ns is None else min(bag.first_ns, timestamp)
                bag.last_ns = timestamp if bag.last_ns is None else max(bag.last_ns, timestamp)
                expected_type = wanted_topic_types.get(topic_id)
                if expected_type:
                    wanted_rows.append((message_id, timestamp, expected_type))
                now = time.monotonic()
                if now - last_report >= 5:
                    elapsed = max(now - scan_started, 0.001)
                    print(
                        f"  [DB3扫描] 已检查 {scanned_rows} 条；目标消息 {len(wanted_rows)} 条；"
                        f"耗时 {elapsed:.0f}s；速率 {scanned_rows / elapsed:.1f} 条/s",
                        flush=True,
                    )
                    last_report = now
            elapsed = max(time.monotonic() - scan_started, 0.001)
            print(
                f"  [DB3扫描] 索引扫描完成：{scanned_rows} 条；目标消息 {len(wanted_rows)} 条；"
                f"耗时 {elapsed:.1f}s。",
                flush=True,
            )

            # SQLite 默认变量上限通常为 999；批量按主键取数据，只触碰目标消息所在页。
            decode_started = time.monotonic()
            last_report = decode_started
            decoded_rows = 0
            print(f"  [字段解码] 开始解码目标消息：{len(wanted_rows)} 条。", flush=True)
            for offset in range(0, len(wanted_rows), 400):
                chunk = wanted_rows[offset:offset + 400]
                placeholders = ",".join("?" for _ in chunk)
                data_by_id = {
                    message_id: blob
                    for message_id, blob in db.execute(
                        f"SELECT id, data FROM messages WHERE id IN ({placeholders})",
                        [message_id for message_id, _, _ in chunk],
                    )
                }
                for message_id, timestamp, expected_type in chunk:
                    try:
                        blob = data_by_id[message_id]
                        if ros2_decoder is not None:
                            try:
                                values = ros2_decoder.decode(expected_type, bytes(blob))
                                bag.decoder = "ros2_deserialize"
                            except Exception:
                                values = manual_decode(expected_type, bytes(blob))
                                bag.decoder = "mixed_ros2/manual_cdr"
                        else:
                            values = manual_decode(expected_type, bytes(blob))
                            if bag.decoder == "未解码":
                                bag.decoder = "manual_cdr"
                        target = next(samples for expected, samples in wanted if expected == expected_type)
                        target.append(Sample(int(timestamp), values))
                    except Exception as exc:  # 单条坏消息不阻断其他 DB3
                        bag.decode_failures[expected_type] += 1
                        if not bag.first_decode_error:
                            bag.first_decode_error = f"{expected_type}: {type(exc).__name__}: {exc}"
                decoded_rows += len(chunk)
                now = time.monotonic()
                if now - last_report >= 5:
                    elapsed = max(now - decode_started, 0.001)
                    print(
                        f"  [字段解码] 已处理 {decoded_rows}/{len(wanted_rows)} 条；"
                        f"耗时 {elapsed:.0f}s；速率 {decoded_rows / elapsed:.1f} 条/s",
                        flush=True,
                    )
                    last_report = now
            elapsed = max(time.monotonic() - decode_started, 0.001)
            print(
                f"  [字段解码] 完成：{decoded_rows} 条；耗时 {elapsed:.1f}s。",
                flush=True,
            )
            for samples in (bag.plc, bag.gantry, bag.execution):
                samples.sort(key=lambda item: item.timestamp)
    except Exception as exc:
        bag.warnings.append(f"SQLite 读取失败: {type(exc).__name__}: {exc}")
        return bag

    if not bag.topics:
        bag.warnings.append("未找到 topics 表或 topics 为空")
    if not bag.plc:
        bag.warnings.append("没有成功解码 PlcMsg，无法可靠判断小车速度、静止和带箱移动")
    if not bag.gantry:
        bag.warnings.append("没有成功解码 GantryPosMsg，贝位只能留空")
    if not bag.execution:
        bag.warnings.append("没有成功解码 ExecStatusMsg，抓放箱任务和排号只能留空")
    if bag.decode_failures:
        bag.warnings.append(
            "字段解码失败 "
            + ", ".join(f"{key} {count} 条" for key, count in bag.decode_failures.items())
        )
        if bag.first_decode_error:
            bag.warnings.append(f"首个解码错误: {bag.first_decode_error}")
    return bag


def load_bag(
    db3: Path,
    ros2_decoder: Optional[Ros2Decoder],
    data_source: str = "auto",
    sidecar_db3: Optional[Path] = None,
) -> BagData:
    if data_source in ("auto", "extracted"):
        sidecar_source = sidecar_db3 or db3
        extracted_bag = load_from_extracted(sidecar_source)
        if extracted_bag is not None:
            print(
                f"  [快速路径] 使用 {find_extracted_dir(sidecar_source)} 中的目标 JSON；"
                f"PLC={len(extracted_bag.plc)}，Gantry={len(extracted_bag.gantry)}，"
                f"ExecStatus={len(extracted_bag.execution)}。",
                flush=True,
            )
            return extracted_bag
        if data_source == "extracted":
            raise RuntimeError(
                "--data-source extracted 要求 metadata.yaml 与目标 *_extracted JSON 数量完整对应"
            )
        print("  [快速路径] 未找到完整 *_extracted 侧车数据，回退 DB3 扫描。", flush=True)
    return _load_bag_db3(db3, ros2_decoder)


def make_topic_summary(bag: BagData) -> str:
    return "; ".join(
        f"{topic.name}<{topic.type_name}>={topic.count}"
        for topic in bag.topics
    )


def safe_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def row_from_trolley_position(value: Any) -> str:
    """按 653 排位标定，把 PLC 小车位置映射为两位排号。"""

    position = safe_float(value)
    if position is None:
        return ""
    row_index = min(
        range(len(ROW_POSITION_REFERENCE)),
        key=lambda index: abs(ROW_POSITION_REFERENCE[index] - position),
    )
    return f"{row_index:02d}"


def hoist_is_moving(sample: Sample, threshold: float) -> bool:
    velocity = safe_float(sample.values.get("hoist_velocity"))
    return velocity is not None and abs(velocity) >= threshold


def add_motion_features(
    samples: list[Sample],
    trolley_threshold: float,
    gantry_threshold: float,
    box_weight_threshold: float,
) -> None:
    previous: Optional[Sample] = None
    for sample in samples:
        values = sample.values
        trolley_velocity = safe_float(values.get("trolley_velocity"))
        trolley_pos = safe_float(values.get("trolley_pos"))
        gantry_pos = safe_float(values.get("gantry_pos"))
        trolley_derived = 0.0
        gantry_derived = 0.0
        if previous is not None:
            delta_sec = (sample.timestamp - previous.timestamp) / 1_000_000_000
            if 0 < delta_sec <= 2:
                old_trolley = safe_float(previous.values.get("trolley_pos"))
                old_gantry = safe_float(previous.values.get("gantry_pos"))
                if trolley_pos is not None and old_trolley is not None:
                    trolley_derived = (trolley_pos - old_trolley) / delta_sec
                if gantry_pos is not None and old_gantry is not None:
                    gantry_derived = (gantry_pos - old_gantry) / delta_sec
        effective_trolley = trolley_velocity
        # 有控制器速度反馈时以反馈为准；位置差受量化和录包时间戳抖动影响，
        # 在低速/停止阶段不应再把它放大成“运动”。
        if effective_trolley is None:
            if abs(trolley_derived) >= trolley_threshold:
                effective_trolley = trolley_derived
        gantry_velocities = [
            safe_float(values.get(f"gantry_velocity_{index}"))
            for index in range(1, 5)
        ]
        gantry_velocities = [value for value in gantry_velocities if value is not None]
        max_gantry_velocity = max((abs(value) for value in gantry_velocities), default=0.0)
        effective_gantry = safe_float(values.get("gantry_velocity_1"))
        if not gantry_velocities and abs(gantry_derived) >= gantry_threshold:
            max_gantry_velocity = abs(gantry_derived)
            effective_gantry = gantry_derived
        values["_effective_trolley_velocity"] = effective_trolley if effective_trolley is not None else 0.0
        values["_effective_gantry_velocity"] = effective_gantry if effective_gantry is not None else 0.0
        values["_trolley_moving"] = abs(values["_effective_trolley_velocity"]) >= trolley_threshold
        values["_gantry_moving"] = max_gantry_velocity >= gantry_threshold
        values["_motion_axis"] = "+".join(
            axis for axis, moving in (
                ("小车", values["_trolley_moving"]),
                ("大车", values["_gantry_moving"]),
            ) if moving
        )
        weight = safe_float(values.get("spreader_weight"))
        locked = bool(values.get("spreader_lock"))
        landed = bool(values.get("spreader_landed"))
        if weight is not None and weight >= box_weight_threshold:
            values["_box_state"] = "带箱（称重确认）"
            values["_loaded_strong"] = True
        elif locked and not landed:
            values["_box_state"] = "疑似带箱（闭锁且未着箱）"
            values["_loaded_strong"] = False
        elif landed:
            values["_box_state"] = "着箱/接触箱"
            values["_loaded_strong"] = False
        else:
            values["_box_state"] = "未见带箱证据"
            values["_loaded_strong"] = False
        previous = sample


def contiguous_segments(
    samples: list[Sample],
    predicate: Callable[[Sample], bool],
    max_gap_ns: int,
    min_duration_ns: int,
    merge_gap_ns: int = 0,
) -> list[Segment]:
    raw: list[Segment] = []
    active: list[Sample] = []
    previous_timestamp: Optional[int] = None

    def flush() -> None:
        nonlocal active
        if active and active[-1].timestamp - active[0].timestamp >= min_duration_ns:
            raw.append(Segment(active))
        active = []

    for sample in samples:
        if not predicate(sample):
            flush()
            previous_timestamp = sample.timestamp
            continue
        if active and previous_timestamp is not None and sample.timestamp - previous_timestamp > max_gap_ns:
            flush()
        active.append(sample)
        previous_timestamp = sample.timestamp
    flush()

    if merge_gap_ns <= 0 or len(raw) < 2:
        return raw
    merged: list[Segment] = [raw[0]]
    for segment in raw[1:]:
        last = merged[-1]
        if segment.start - last.end <= merge_gap_ns:
            merged[-1] = Segment(last.samples + segment.samples)
        else:
            merged.append(segment)
    return merged


def nearest_sample(samples: list[Sample], timestamp: int, max_delta_ns: int = 5_000_000_000) -> Optional[Sample]:
    if not samples:
        return None
    timestamps = [sample.timestamp for sample in samples]
    index = bisect.bisect_left(timestamps, timestamp)
    candidates = []
    if index < len(samples):
        candidates.append(samples[index])
    if index > 0:
        candidates.append(samples[index - 1])
    sample = min(candidates, key=lambda item: abs(item.timestamp - timestamp))
    return sample if abs(sample.timestamp - timestamp) <= max_delta_ns else None


def samples_between(samples: list[Sample], start: int, end: int) -> list[Sample]:
    if not samples:
        return []
    timestamps = [sample.timestamp for sample in samples]
    left = max(0, bisect.bisect_left(timestamps, start) - 1)
    right = min(len(samples), bisect.bisect_right(timestamps, end) + 1)
    return samples[left:right]


def ordered_unique(values: Iterable[Any]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for value in values:
        text = str(value).strip()
        if not text or text in seen or text.lower() in {"none", "null"}:
            continue
        seen.add(text)
        result.append(text)
    return result


def bay_text(sample: Optional[Sample]) -> str:
    if sample is None:
        return ""
    values = sample.values
    block = str(values.get("block", "")).strip()
    bay = str(values.get("bay_no", "")).strip()
    size = values.get("bay_size")
    if not block and not bay:
        return ""
    suffix = f"({int(size)}尺)" if safe_float(size) is not None and int(float(size)) else ""
    return f"{block}-{bay}{suffix}" if block else f"{bay}{suffix}"


def bay_sequence(samples: list[Sample]) -> list[str]:
    result: list[str] = []
    for sample in samples:
        text = bay_text(sample)
        if text and (not result or result[-1] != text):
            result.append(text)
    return result


def position_text(value: Any) -> str:
    return fmt_number(value, 3)


def direction_text(values: list[float]) -> str:
    if not values:
        return "不明"
    positive = sum(value > 0 for value in values)
    negative = sum(value < 0 for value in values)
    if positive and negative:
        return "正反向变化"
    if positive:
        return "正向"
    if negative:
        return "反向"
    return "速度为零/由位置差判断"


def segment_speed_stats(segment: Segment) -> tuple[str, str, str, str]:
    velocities = [
        safe_float(sample.values.get("_effective_trolley_velocity"))
        for sample in segment.samples
    ]
    velocities = [value for value in velocities if value is not None]
    if not velocities:
        return "", "", "", ""
    mean = sum(velocities) / len(velocities)
    max_abs = max(abs(value) for value in velocities)
    return fmt_number(mean), fmt_number(max_abs), direction_text(velocities), str(len(velocities))


def box_state_for_segment(segment: Segment) -> str:
    states = ordered_unique(sample.values.get("_box_state", "") for sample in segment.samples)
    return "；".join(states)


def row_values_for_segment(
    segment: Segment,
    execution: list[Sample],
    gantry: list[Sample],
) -> dict[str, str]:
    in_range = samples_between(execution, segment.start, segment.end)
    start_exec = nearest_sample(execution, segment.start)
    end_exec = nearest_sample(execution, segment.end)
    start_gantry = nearest_sample(gantry, segment.start)
    end_gantry = nearest_sample(gantry, segment.end)
    rows = ordered_unique(sample.values.get("row_no", "") for sample in in_range)
    tiers = ordered_unique(sample.values.get("tier_no", "") for sample in in_range)
    context_exec = in_range or [sample for sample in (start_exec, end_exec) if sample is not None]
    task_ids = ordered_unique(sample.values.get("task_id", "") for sample in context_exec)
    actions = ordered_unique(
        f"{int(float(sample.values.get('action_type')))}="
        f"{ACTION_NAMES.get(int(float(sample.values.get('action_type'))), str(sample.values.get('action_type')))}"
        for sample in context_exec
        if safe_float(sample.values.get("action_type")) is not None
        and int(float(sample.values.get("action_type"))) in ACTION_NAMES
    )
    steps = ordered_unique(
        MOVE_STEP_NAMES.get(
            str(sample.values.get("move_step", "")),
            str(sample.values.get("move_step", "")),
        )
        for sample in context_exec
    )
    mean_speed, max_speed, direction, speed_count = segment_speed_stats(segment)
    positions = [safe_float(sample.values.get("trolley_pos")) for sample in segment.samples]
    positions = [value for value in positions if value is not None]
    position_start_row = row_from_trolley_position(positions[0]) if positions else ""
    position_end_row = row_from_trolley_position(positions[-1]) if positions else ""
    start_row = position_start_row or (rows[0] if rows else "")
    end_row = position_end_row or (rows[-1] if rows else "")
    row_note = ""
    if positions and start_row and end_row:
        row_note = (
            f"PLC 小车位置 {position_text(positions[0])}→{position_text(positions[-1])} m"
            f" 映射排 {start_row}→{end_row}"
        )
        if rows:
            row_note += f"；ExecStatus 记录排号={ '、'.join(rows) }（任务目标字段）"
    elif rows:
        row_note = f"ExecStatus 记录排号={ '、'.join(rows) }（未取得可映射的 PLC 小车位置）"
    return {
        "大车贝位_起": bay_text(start_gantry),
        "大车贝位_止": bay_text(end_gantry),
        "Block": str((start_gantry or end_gantry).values.get("block", "")) if (start_gantry or end_gantry) else "",
        "大车坐标_起": position_text((start_gantry or start_exec).values.get("gantry_pos")) if (start_gantry or start_exec) else "",
        "大车坐标_止": position_text((end_gantry or end_exec).values.get("gantry_pos")) if (end_gantry or end_exec) else "",
        "小车位置_起": position_text(positions[0]) if positions else position_text(start_exec.values.get("trolley_pos")) if start_exec else "",
        "小车位置_止": position_text(positions[-1]) if positions else position_text(end_exec.values.get("trolley_pos")) if end_exec else "",
        "小车速度平均值": mean_speed,
        "小车速度最大绝对值": max_speed,
        "小车方向": direction,
        "起始排": start_row,
        "终止排": end_row,
        "层": tiers[0] if tiers else "",
        "任务ID": "、".join(task_ids),
        "动作类型": "、".join(actions),
        "状态/步骤": "、".join(steps),
        "载箱状态": box_state_for_segment(segment),
        "抓放信号": "、".join(actions),
        "事件消息数": speed_count,
        "_row_mapping_note": row_note,
    }


def row_motion_note(fields: dict[str, str]) -> str:
    mapping_note = fields.get("_row_mapping_note", "")
    if mapping_note:
        return mapping_note
    start_row = fields.get("起始排", "")
    end_row = fields.get("终止排", "")
    if start_row and end_row and start_row != end_row:
        return f"ExecStatus 记录排号由{start_row}变化到{end_row}"
    if start_row:
        return f"ExecStatus 仅提供当前/目标排{start_row}，未直接提供运动前的起始排"
    return "ExecStatus 未提供可匹配的排号"


def with_exec_steps(label: str, fields: dict[str, str]) -> str:
    steps = fields.get("状态/步骤", "")
    return f"{label}；ExecStatus步骤={steps}" if steps else label


def empty_row(bag: BagData, record_type: str) -> dict[str, str]:
    row = {field_name: "" for field_name in CSV_FIELDS}
    row["包目录"] = str(bag.path.parent)
    row["DB3文件"] = bag.path.name
    row["记录类型"] = record_type
    row["解码方式"] = bag.decoder
    return row


def event_row(
    bag: BagData,
    record_type: str,
    start: Optional[int],
    end: Optional[int],
    source: str,
    description: str,
    extra: Optional[dict[str, Any]] = None,
) -> dict[str, str]:
    row = empty_row(bag, record_type)
    row.update(
        {
            "开始时间（北京时间）": fmt_ns(start),
            "结束时间（北京时间）": fmt_ns(end),
            "开始时间戳（毫秒）": timestamp_ms(start),
            "结束时间戳（毫秒）": timestamp_ms(end),
            "经历时间（秒）": duration_seconds(start, end),
            "Topic/字段来源": source,
            "说明": description,
            "异常或限制": "；".join(bag.warnings),
        }
    )
    if extra:
        for key, value in extra.items():
            if key in row:
                row[key] = "" if value is None else str(value)
    return row


def add_overview_row(bag: BagData, box_weight_threshold: float) -> dict[str, str]:
    row = event_row(
        bag,
        "包概览",
        bag.first_ns,
        bag.last_ns,
        make_topic_summary(bag),
        "；".join(bag.warnings),
    )
    bays = bay_sequence(bag.gantry)
    rows = ordered_unique(sample.values.get("row_no", "") for sample in bag.execution)
    task_keys: set[tuple[str, int, int]] = set()
    for sample in bag.execution:
        action_type = safe_float(sample.values.get("action_type"))
        action_index = safe_float(sample.values.get("action_index"))
        if action_type is not None and action_index is not None:
            task_keys.add(
                (
                    str(sample.values.get("task_id", "")),
                    int(action_index),
                    int(action_type),
                )
            )
    actions = Counter(action_type for _, _, action_type in task_keys)
    strong_loaded = sum(bool(sample.values.get("_loaded_strong")) for sample in bag.plc)
    overview_fields = (
        row_values_for_segment(Segment(bag.plc), bag.execution, bag.gantry)
        if bag.plc
        else {}
    )
    row_evidence = overview_fields.get("_row_mapping_note", "") or (
        f"ExecStatus 排号={ '、'.join(rows) if rows else '无' }"
    )
    row.update(
        {
            "消息数": str(bag.total_messages),
            "事件消息数": str(len(bag.execution)),
            "大车贝位_起": bays[0] if bays else "",
            "大车贝位_止": bays[-1] if bays else "",
            "Block": overview_fields.get("Block", ""),
            "大车坐标_起": overview_fields.get("大车坐标_起", ""),
            "大车坐标_止": overview_fields.get("大车坐标_止", ""),
            "小车位置_起": overview_fields.get("小车位置_起", ""),
            "小车位置_止": overview_fields.get("小车位置_止", ""),
            "小车速度平均值": overview_fields.get("小车速度平均值", ""),
            "小车速度最大绝对值": overview_fields.get("小车速度最大绝对值", ""),
            "小车方向": overview_fields.get("小车方向", ""),
            "起始排": overview_fields.get("起始排", "") or (rows[0] if rows else ""),
            "终止排": overview_fields.get("终止排", "") or (rows[-1] if rows else ""),
            "层": overview_fields.get("层", ""),
            "任务ID": overview_fields.get("任务ID", ""),
            "状态/步骤": overview_fields.get("状态/步骤", ""),
            "动作类型": "; ".join(f"{ACTION_NAMES.get(key, str(key))}={value}" for key, value in sorted(actions.items())),
            "载箱状态": (
                "有称重带箱样本"
                if strong_loaded
                else f"未见称重超过 {box_weight_threshold:g} 吨样本"
            ),
            "抓放信号": "；".join(
                [
                    f"抓箱任务 {actions.get(2, 0)} 个",
                    f"放箱任务 {actions.get(3, 0)} 个",
                    f"ExecStatus消息 {len(bag.execution)} 条",
                ]
            ),
            "说明": (
                f"消息时间范围由 SQLite messages.timestamp 计算；"
                f"贝位序列={ '→'.join(bays) if bays else '无 GantryPosMsg' }；"
                f"{row_evidence}。"
                + ("；" + "；".join(bag.warnings) if bag.warnings else "")
            ),
        }
    )
    return row


def build_event_rows(bag: BagData, args: argparse.Namespace) -> list[dict[str, str]]:
    add_motion_features(
        bag.plc,
        args.trolley_speed_threshold,
        args.gantry_speed_threshold,
        args.box_weight_threshold,
    )
    rows = [add_overview_row(bag, args.box_weight_threshold)]

    gap_ns = int(args.max_sample_gap_seconds * 1_000_000_000)
    merge_ns = int(args.merge_gap_seconds * 1_000_000_000)
    min_event_ns = int(args.min_event_seconds * 1_000_000_000)
    min_stationary_ns = int(args.min_stationary_seconds * 1_000_000_000)

    if bag.plc:
        trolley_segments = contiguous_segments(
            bag.plc,
            lambda sample: bool(sample.values.get("_trolley_moving")),
            gap_ns,
            min_event_ns,
            merge_ns,
        )
        for segment in trolley_segments:
            fields = row_values_for_segment(segment, bag.execution, bag.gantry)
            row_note = row_motion_note(fields)
            loaded = str(fields.get("载箱状态", "")).startswith(("带箱", "疑似带箱"))
            record_type = "小车带箱移动" if loaded else "小车移动"
            fields["状态/步骤"] = with_exec_steps(
                "PLC 带箱小车移动" if loaded else "PLC 速度/位置反馈",
                fields,
            )
            description = (
                f"小车带箱移动；{row_note}；速度为控制器 trolley_velocity 的统计值；"
                f"{fields['载箱状态']}。"
                if loaded
                else f"{row_note}；速度为控制器 trolley_velocity 的统计值；{fields['载箱状态']}。"
            )
            rows.append(
                event_row(
                    bag,
                    record_type,
                    segment.start,
                    segment.end,
                    (
                        "/rtg/plc_topic_35 PlcMsg[trolley_pos,trolley_velocity,spreader_weight,"
                        "spreader_lock,spreader_landed]；按 trolley_pos 映射排号；关联 ExecStatusMsg[row_no]"
                        if loaded
                        else "/rtg/plc_topic_35 PlcMsg[trolley_pos,trolley_velocity]；按 trolley_pos 映射排号；关联 ExecStatusMsg[row_no]"
                    ),
                    description,
                    fields,
                )
            )

        gantry_segments = contiguous_segments(
            bag.plc,
            lambda sample: bool(sample.values.get("_gantry_moving")),
            gap_ns,
            min_event_ns,
            merge_ns,
        )
        for segment in gantry_segments:
            fields = row_values_for_segment(segment, bag.execution, bag.gantry)
            fields["状态/步骤"] = with_exec_steps("PLC 大车速度反馈", fields)
            axis_values = ordered_unique(sample.values.get("_motion_axis", "") for sample in segment.samples)
            fields["说明"] = ""
            rows.append(
                event_row(
                    bag,
                    "大车移动",
                    segment.start,
                    segment.end,
                    "/rtg/plc_topic_35 PlcMsg[gantry_velocity_1..4,gantry_pos]；关联 GantryPosMsg[bay_no]",
                    f"大车运动轴={ '、'.join(axis_values) or '大车' }；贝位 {fields['大车贝位_起'] or '未知'}→{fields['大车贝位_止'] or '未知'}。",
                    fields,
                )
            )

        stationary_base = lambda sample: (
            not bool(sample.values.get("_trolley_moving"))
            and not bool(sample.values.get("_gantry_moving"))
        )
        stationary_specs = (
            (
                "小车吊具都静止",
                lambda sample: stationary_base(sample)
                and not hoist_is_moving(sample, args.hoist_speed_threshold),
            ),
            (
                "小车静止吊具不静止",
                lambda sample: stationary_base(sample)
                and hoist_is_moving(sample, args.hoist_speed_threshold),
            ),
        )
        for record_type, predicate in stationary_specs:
            stationary_segments = contiguous_segments(
                bag.plc,
                predicate,
                gap_ns,
                min_stationary_ns,
                merge_ns,
            )
            for segment in stationary_segments:
                fields = row_values_for_segment(segment, bag.execution, bag.gantry)
                fields["状态/步骤"] = with_exec_steps(record_type, fields)
                fields["载箱状态"] = box_state_for_segment(segment)
                rows.append(
                    event_row(
                        bag,
                        record_type,
                        segment.start,
                        segment.end,
                        "/rtg/plc_topic_35 PlcMsg[trolley_velocity,gantry_velocity_1..4,hoist_velocity]；关联 GantryPosMsg",
                        f"小车和大车速度低于阈值；吊具 hoist_velocity 按 {args.hoist_speed_threshold:g} 阈值判定；"
                        f"位置为 {fields['大车贝位_起'] or '贝位未知'}，小车位置 {fields['小车位置_起'] or '未知'}；"
                        f"{fields['载箱状态']}。",
                        fields,
                    )
                )

        previous_feedback: Optional[tuple[bool, bool, bool]] = None
        previous_timestamp: Optional[int] = None
        for sample in bag.plc:
            if previous_timestamp is not None and sample.timestamp - previous_timestamp > gap_ns:
                previous_feedback = None
            current_state = str(sample.values.get("_box_state", ""))
            current_feedback = (
                bool(sample.values.get("spreader_lock")),
                bool(sample.values.get("spreader_landed")),
                bool(sample.values.get("_loaded_strong")),
            )
            if previous_feedback is not None and current_feedback != previous_feedback:
                nearest_exec = nearest_sample(bag.execution, sample.timestamp, 2_000_000_000)
                action = ""
                if nearest_exec is not None:
                    action = ACTION_NAMES.get(int(nearest_exec.values.get("action_type", 0)), "")
                previous_carry = previous_feedback[2] or (previous_feedback[0] and not previous_feedback[1])
                current_carry = current_feedback[2] or (current_feedback[0] and not current_feedback[1])
                if current_carry and not previous_carry:
                    direction = "可能抓箱完成/进入带箱状态"
                elif previous_carry and not current_carry:
                    direction = "可能放箱/解除带箱状态"
                else:
                    direction = "着箱/离箱反馈变化"
                changes = []
                if previous_feedback[0] != current_feedback[0]:
                    changes.append(f"spreader_lock={'是' if current_feedback[0] else '否'}")
                if previous_feedback[1] != current_feedback[1]:
                    changes.append(f"spreader_landed={'是' if current_feedback[1] else '否'}")
                if previous_feedback[2] != current_feedback[2]:
                    changes.append(f"称重带箱={'是' if current_feedback[2] else '否'}")
                fields = row_values_for_segment(Segment([sample]), bag.execution, bag.gantry)
                fields["事件消息数"] = "1"
                fields["动作类型"] = action
                fields["抓放信号"] = action or direction
                rows.append(
                    event_row(
                        bag,
                        "吊具反馈变化",
                        sample.timestamp,
                        sample.timestamp,
                        "/rtg/plc_topic_35 PlcMsg[spreader_weight,spreader_lock,spreader_landed]；关联 ExecStatusMsg[action_type]",
                        f"{'; '.join(changes)}；当前状态={current_state}；{direction}。这是反馈推断，不等同于箱体视觉确认。",
                        fields,
                    )
                )
            previous_feedback = current_feedback
            previous_timestamp = sample.timestamp

    # 直接按任务 ID 汇总 ExecStatus，保留抓箱/放箱任务的动作、排、层和步骤。
    task_groups: dict[tuple[str, int, int], list[Sample]] = defaultdict(list)
    for sample in bag.execution:
        action_type = int(sample.values.get("action_type", 0))
        if action_type in (2, 3):
            key = (
                str(sample.values.get("task_id", "")),
                int(sample.values.get("action_index", 0)),
                action_type,
            )
            task_groups[key].append(sample)
    for (task_id, action_index, action_type), samples in sorted(
        task_groups.items(), key=lambda item: item[1][0].timestamp
    ):
        first = samples[0]
        last = samples[-1]
        values = first.values
        action_name = ACTION_NAMES[action_type]
        step_names = ordered_unique(
            MOVE_STEP_NAMES.get(str(sample.values.get("move_step", "")), str(sample.values.get("move_step", "")))
            for sample in samples
        )
        plc_in_task = samples_between(bag.plc, first.timestamp, last.timestamp)
        task_segment = Segment(plc_in_task) if plc_in_task else Segment(samples)
        extra = row_values_for_segment(task_segment, bag.execution, bag.gantry)
        extra.update(
            {
                "事件消息数": str(len(samples)),
                "层": values.get("tier_no", "") or extra.get("层", ""),
                "任务ID": task_id,
                "动作类型": f"{action_type}={action_name}",
                "状态/步骤": "、".join(step_names),
                "Block": extra.get("Block") or values.get("block", ""),
                "大车贝位_起": extra.get("大车贝位_起") or f"{values.get('block', '')}-{values.get('bay_no', '')}".strip("-"),
                "大车贝位_止": extra.get("大车贝位_止") or f"{last.values.get('block', '')}-{last.values.get('bay_no', '')}".strip("-"),
                "大车坐标_起": extra.get("大车坐标_起") or position_text(values.get("gantry_pos")),
                "大车坐标_止": extra.get("大车坐标_止") or position_text(last.values.get("gantry_pos")),
                "小车位置_起": extra.get("小车位置_起") or position_text(values.get("trolley_pos")),
                "小车位置_止": extra.get("小车位置_止") or position_text(last.values.get("trolley_pos")),
                "抓放信号": f"ExecStatus action_type={action_type}（{action_name}）",
            }
        )
        task_row_note = row_motion_note(extra)
        rows.append(
            event_row(
                bag,
                f"{action_name}任务",
                first.timestamp,
                last.timestamp,
                "/rtg/task_exec_status ExecStatusMsg[action_type,row_no,tier_no,move_step]",
                f"任务 {task_id or '未知'}#{action_index}；动作={action_name}；"
                f"步骤={ '、'.join(step_names) or '未知' }；{task_row_note}。",
                extra,
            )
        )

    # GantryPosMsg 中的贝位变化单独列出，避免只在概览中留下一个首尾贝位。
    previous_bay: Optional[str] = None
    previous_sample: Optional[Sample] = None
    for sample in bag.gantry:
        current_bay = bay_text(sample)
        if not current_bay:
            continue
        if previous_bay is not None and current_bay != previous_bay:
            extra = {
                "大车贝位_起": previous_bay,
                "大车贝位_止": current_bay,
                "Block": sample.values.get("block", ""),
                "大车坐标_起": position_text(previous_sample.values.get("gantry_pos")) if previous_sample else "",
                "大车坐标_止": position_text(sample.values.get("gantry_pos")),
                "事件消息数": "2",
                "状态/步骤": "GantryPosMsg 贝位字段变化",
            }
            rows.append(
                event_row(
                    bag,
                    "贝位变化",
                    previous_sample.timestamp if previous_sample else sample.timestamp,
                    sample.timestamp,
                    "/algl/GantryPosMsg GantryPosMsg[block,bay_no,bay_size,gantry_pos]",
                    f"大车贝位字段由 {previous_bay} 变化为 {current_bay}。",
                    extra,
                )
            )
        previous_bay = current_bay
        previous_sample = sample

    return sorted(
        rows,
        key=lambda row: (row.get("开始时间（北京时间）", ""), row.get("记录类型", "")),
    )


def direct_db3_files(root: Path) -> list[Path]:
    result: list[Path] = []
    with os.scandir(root) as entries:
        for entry in entries:
            if entry.name.lower().endswith(".db3") and entry.is_file(follow_symlinks=False):
                result.append(Path(entry.path))
    return sorted(result)


def find_db3_files(root: Path, batch: bool = False) -> list[Path]:
    """按单数据集或全量批处理范围查找 DB3。"""

    if not batch and not direct_db3_files(root):
        print(
            f"  [数据集路径] {root} 没有直接 DB3；单数据集模式不会继续扫描其它数据集。",
            flush=True,
        )
        return []

    db3_files: list[Path] = []
    started = time.monotonic()
    last_report = started
    visited_dirs = 0
    visited_files = 0
    scope = "全量批处理" if batch else "单个数据集"
    print(f"  [目录扫描] 开始查找 DB3（{scope}）：{root}", flush=True)
    for directory, dirnames, filenames in os.walk(root, topdown=True):
        # *_extracted 只存解压后的 JSON，不会再包含需要分析的 DB3；
        # 跳过它们可以避免在网络盘枚举几十万条侧车文件名。
        dirnames[:] = [
            dirname for dirname in dirnames
            if not dirname.lower().endswith("_extracted")
        ]
        visited_dirs += 1
        visited_files += len(filenames)
        for filename in filenames:
            if filename.lower().endswith(".db3"):
                db3_files.append(Path(directory) / filename)
        now = time.monotonic()
        if now - last_report >= 5:
            elapsed = max(now - started, 0.001)
            print(
                f"  [目录扫描] 已检查 {visited_dirs} 个目录、约 {visited_files} 个文件；"
                f"已发现 {len(db3_files)} 个 DB3；耗时 {elapsed:.0f}s。",
                flush=True,
            )
            last_report = now
    elapsed = max(time.monotonic() - started, 0.001)
    print(
        f"  [目录扫描] 完成：{visited_dirs} 个目录、约 {visited_files} 个文件、"
        f"{len(db3_files)} 个 DB3；耗时 {elapsed:.1f}s。",
        flush=True,
    )
    return sorted(db3_files)


def cached_db3(source: Path, cache_dir: Optional[Path]) -> Path:
    """可选地把网络盘 DB3 顺序缓存到本地，减少 SQLite 在网络盘上的随机读。"""

    if cache_dir is None:
        return source
    cache_dir.mkdir(parents=True, exist_ok=True)
    identity = hashlib.sha1(str(source).encode("utf-8")).hexdigest()[:16]
    cached = cache_dir / f"{identity}_{source.name}"
    source_size = source.stat().st_size
    if cached.is_file() and cached.stat().st_size == source_size:
        return cached
    partial = cached.with_name(cached.name + ".partial")
    if partial.exists():
        partial.unlink()
    print(f"  顺序缓存到本地: {source_size / 1024**3:.2f} GiB -> {cached}", flush=True)
    shutil.copyfile(source, partial)
    os.replace(partial, cached)
    return cached


def try_ros2_decoder(mode: str) -> Optional[Ros2Decoder]:
    if mode == "manual":
        return None
    try:
        decoder = Ros2Decoder()
        print("已启用 ROS2 类型支持解码。", flush=True)
        return decoder
    except Exception as exc:
        if mode == "ros2":
            raise RuntimeError(f"--decoder ros2 要求可用 ROS2 类型支持，但加载失败: {exc}") from exc
        print(f"ROS2 类型支持不可用，使用内置 CDR 解码: {exc}", flush=True)
        return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="递归分析 rosbag2 SQLite DB3 的时间、贝位、大小车运动、抓放箱和静止区间"
    )
    parser.add_argument("input", nargs="?", default=DEFAULT_INPUT, help="目标目录或 Windows UNC 路径")
    parser.add_argument(
        "--batch",
        action="store_true",
        help="递归处理输入根目录下的全部数据集；默认只处理传入的单个数据集目录",
    )
    parser.add_argument("--output", type=str, help="CSV 输出路径；默认写到输入目录")
    parser.add_argument(
        "--cache-dir",
        type=str,
        help="可选本地 DB3 缓存目录；网络盘建议使用 /tmp/db3_cache",
    )
    parser.add_argument(
        "--decoder",
        choices=("auto", "manual", "ros2"),
        default="auto",
        help="auto 优先 ROS2；manual 只用内置 CDR；ros2 强制 ROS2",
    )
    parser.add_argument(
        "--data-source",
        choices=("auto", "extracted", "db3"),
        default="auto",
        help="auto 优先使用完整 *_extracted JSON；extracted 强制侧车；db3 逐条扫描 SQLite",
    )
    parser.add_argument("--trolley-speed-threshold", type=float, default=0.05, help="小车运动阈值，默认 0.05")
    parser.add_argument("--gantry-speed-threshold", type=float, default=0.05, help="大车运动阈值，默认 0.05")
    parser.add_argument("--hoist-speed-threshold", type=float, default=0.05, help="吊具运动阈值，默认 0.05")
    parser.add_argument("--box-weight-threshold", type=float, default=5.0, help="称重带箱阈值，吨；默认 5")
    parser.add_argument("--min-event-seconds", type=float, default=0.2, help="运动事件最短时长，默认 0.2 秒")
    parser.add_argument("--min-stationary-seconds", type=float, default=1.0, help="静止事件最短时长，默认 1 秒")
    parser.add_argument("--max-sample-gap-seconds", type=float, default=2.0, help="样本间隔超过此值就断开事件")
    parser.add_argument("--merge-gap-seconds", type=float, default=0.5, help="短暂阈值抖动的合并间隔")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (
        args.trolley_speed_threshold <= 0
        or args.gantry_speed_threshold <= 0
        or args.hoist_speed_threshold <= 0
    ):
        raise SystemExit("速度阈值必须大于 0")
    if args.box_weight_threshold <= 0:
        raise SystemExit("称重阈值必须大于 0")

    print(f"输入路径: {args.input}", flush=True)
    root = normalize_path(args.input)
    print(f"解析后的目标路径: {root}", flush=True)
    if not root.exists():
        print(f"目标路径不存在或当前环境不可访问: {root}", file=sys.stderr)
        return 2
    if root.is_file():
        if root.suffix.lower() != ".db3":
            print(f"输入文件不是 DB3: {root}", file=sys.stderr)
            return 2
        dataset_root = root.parent
        db3_files = [root]
    elif root.is_dir():
        dataset_root = root
        db3_files = find_db3_files(root, args.batch)
    else:
        print(f"目标路径不是目录或 DB3 文件: {root}", file=sys.stderr)
        return 2
    if not db3_files:
        print(
            f"未找到可处理的 DB3: {root}；请传入具体数据集目录，或对总目录使用 --batch。",
            file=sys.stderr,
        )
        return 2

    if args.output:
        output = normalize_path(args.output)
    elif args.batch:
        output = dataset_root / DEFAULT_OUTPUT_NAME
    else:
        output = dataset_root / f"{dataset_root.name}_DB3事件汇总.csv"
    if output.exists() and output.is_dir():
        print(f"输出路径是目录，不是 CSV 文件: {output}", file=sys.stderr)
        return 2
    output.parent.mkdir(parents=True, exist_ok=True)
    cache_dir = normalize_path(args.cache_dir) if args.cache_dir else None
    ros2_decoder = try_ros2_decoder(args.decoder)

    all_rows: list[dict[str, str]] = []
    print(f"目标: {root}", flush=True)
    print(f"发现 DB3: {len(db3_files)} 个", flush=True)
    for index, db3 in enumerate(db3_files, start=1):
        print(f"[{index}/{len(db3_files)}] 分析 {db3}", flush=True)
        try:
            analysis_path = cached_db3(db3, cache_dir)
            sidecar_db3 = db3 if cache_dir and args.data_source in ("auto", "extracted") else None
            bag = load_bag(analysis_path, ros2_decoder, args.data_source, sidecar_db3)
        except Exception as exc:
            bag = BagData(path=db3)
            bag.warnings.append(f"处理失败: {type(exc).__name__}: {exc}")
        bag.path = db3
        all_rows.extend(build_event_rows(bag, args))

    with output.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(all_rows)

    print(f"CSV 已保存: {output}", flush=True)
    print(f"DB3 数量: {len(db3_files)}；CSV 行数: {len(all_rows)}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
