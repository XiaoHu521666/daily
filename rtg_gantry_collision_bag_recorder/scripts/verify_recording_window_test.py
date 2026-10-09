#!/usr/bin/env python3

from pathlib import Path
import sqlite3


ROOT = Path("/home/dnt/ws_ros2_rtg/virtual_test_output/gantry_recording_window")
MARKER = Path("/tmp/gantry_recording_window_test_start")
EXPECTED_TOPICS = {
    "/rtg/plc_topic_35",
    "/algl/GantryColDetMsg",
    "/livox/lidar_3GGDJA600101431",
    "/livox/lidar_3GGDJA500101151",
    "/livox/lidar_3GGDJ9S00100741",
    "/livox/lidar_3GGDJA600101201",
}


def read_event(path):
    values = {}
    for line in (path / "event.txt").read_text().splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            values[key] = value
    return values


def read_db(path):
    counts = {}
    first = None
    for database in sorted(path.glob("*.db3")):
        connection = sqlite3.connect(database)
        for name, count in connection.execute(
            "SELECT topics.name, COUNT(messages.id) "
            "FROM topics LEFT JOIN messages ON topics.id=messages.topic_id "
            "GROUP BY topics.id"
        ):
            counts[name] = counts.get(name, 0) + count
        current_first = connection.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]
        connection.close()
        if current_first is not None:
            first = current_first if first is None else min(first, current_first)
    return counts, first


def main():
    marker_ns = MARKER.stat().st_mtime_ns
    bags = []
    for path in ROOT.glob("*/*"):
        if not path.is_dir() or path.stat().st_mtime_ns <= marker_ns:
            continue
        event = read_event(path)
        bags.append((int(event["start_receive_time_ns"]), path, event))
    bags.sort()

    left = [item for item in bags if "_left_" in item[1].name or item[1].name.endswith("_left")]
    right = [item for item in bags if "_right_" in item[1].name or item[1].name.endswith("_right")]
    passed = len(bags) == 5 and len(left) == 4 and len(right) == 1
    print(f"事件总数={len(bags)}，left={len(left)}，right={len(right)}，预期=5/4/1")

    expected = []
    if len(left) == 4:
        expected.extend([
            (left[0], "capture_window_complete", 4.8, 5.2, 1, "单次报警"),
            (left[1], "capture_window_complete", 7.5, 8.8, 2, "二次报警续期"),
            (left[2], "max_record_duration", 119.9, 120.1, None, "120秒旧包"),
            (left[3], "capture_window_complete", 9.0, 11.5, None, "超限后新包"),
        ])
    if len(right) == 1:
        expected.append((right[0], "capture_window_complete", 4.8, 5.2, 1, "right独立报警"))

    for item, expected_status, min_window, max_window, expected_alarm_count, label in expected:
        _, path, event = item
        counts, first = read_db(path)
        start = int(event["start_receive_time_ns"])
        requested_from = int(event["requested_record_from_ns"])
        requested_until = int(event["requested_record_until_ns"])
        requested_pre = (start - requested_from) / 1_000_000_000
        requested_window = (requested_until - start) / 1_000_000_000
        actual_pre = (start - first) / 1_000_000_000 if first is not None else -1.0
        alarm_count = counts.get("/algl/GantryColDetMsg", 0)
        status = event.get("status")
        topics_ok = set(counts) == EXPECTED_TOPICS and all(value > 0 for value in counts.values())
        alarm_ok = expected_alarm_count is None or alarm_count == expected_alarm_count
        bag_ok = (
            status == expected_status
            and 1.5 <= actual_pre <= 2.3
            and 1.999 <= requested_pre <= 2.001
            and min_window <= requested_window <= max_window
            and topics_ok
            and alarm_ok
        )
        passed = passed and bag_ok
        print(
            f"{label}: {path.name}, status={status}, "
            f"实际预录={actual_pre:.3f}s, 请求预录={requested_pre:.3f}s, "
            f"请求触发后窗口={requested_window:.3f}s, 报警数={alarm_count}, "
            f"Topic数={len(counts)}, 结果={'通过' if bag_ok else '失败'}"
        )

    print("总结论：" + ("通过" if passed else "失败"))
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
