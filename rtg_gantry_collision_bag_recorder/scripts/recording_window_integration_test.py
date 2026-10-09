#!/usr/bin/env python3

import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rtg_algorithm_interfaces.msg import SafetyDetectMsg


TOPIC = "/algl/GantryColDetMsg"
FRAME = "gantry_collision_detect_frame"


def main():
    rclpy.init()
    node = rclpy.create_node("gantry_recording_window_test")
    publisher = node.create_publisher(SafetyDetectMsg, TOPIC, qos_profile_sensor_data)

    def wait(seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, remaining)))

    def alarm(area):
        message = SafetyDetectMsg()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = FRAME
        message.area_code = area
        message.has_object = True
        message.object_distance = 10.0
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.1)

    try:
        print("等待 left/right 两个订阅者发现", flush=True)
        deadline = time.monotonic() + 15.0
        while publisher.get_subscription_count() < 2 and time.monotonic() < deadline:
            wait(0.1)
        count = publisher.get_subscription_count()
        print(f"已发现订阅者数={count}，预期至少 2", flush=True)
        if count < 2:
            raise RuntimeError("未发现 left/right 两个录包节点")
        wait(3.0)

        print("用例 1/4：单条 left 报警，应生成 1 个前 2 秒、后 5 秒的包", flush=True)
        alarm(1)
        wait(7.0)

        print("用例 2/4：left 在第一条后 3 秒再报警，应合并为 1 个包", flush=True)
        alarm(1)
        wait(3.0)
        alarm(1)
        wait(7.0)

        print("用例 3/4：left 每秒报警一次，持续 126 秒，应生成旧包和新包", flush=True)
        start = time.monotonic()
        for index in range(126):
            target = start + index
            if time.monotonic() < target:
                wait(target - time.monotonic())
            alarm(1)
        wait(7.0)

        print("用例 4/4：单条 right 报警，应独立生成 1 个包", flush=True)
        alarm(2)
        wait(7.0)
        print("测试序列完成，请执行终端 4 验收", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
