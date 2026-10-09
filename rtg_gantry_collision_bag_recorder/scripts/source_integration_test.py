#!/usr/bin/env python3

import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rtg_algorithm_interfaces.msg import SafetyDetectMsg


TOPIC = "/algl/GantryColDetMsg"
GANTRY_FRAME = "gantry_collision_detect_frame"
SIDE_FRAME = "gantry_side_truck_detect_frame"


def main():
    rclpy.init()
    node = rclpy.create_node("gantry_collision_source_test")
    side_pub = node.create_publisher(SafetyDetectMsg, TOPIC, qos_profile_sensor_data)
    gantry_pub = None

    def wait(seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

    def send(pub, frame_id, area, distance):
        msg = SafetyDetectMsg()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.area_code = area
        msg.has_object = distance < 16.0
        msg.object_distance = float(distance)
        for _ in range(3):
            pub.publish(msg)
            wait(0.15)

    def step(text):
        print(text, flush=True)

    try:
        step("1/11 仅侧面发布者：区域1-4来源应保持未确认")
        wait(2)
        send(side_pub, SIDE_FRAME, 7, 9999.0)
        send(side_pub, SIDE_FRAME, 8, 9999.0)
        wait(6)

        step("2/11 侧面冒用区域1/3：不能触发left")
        send(side_pub, SIDE_FRAME, 1, 10.0)
        send(side_pub, SIDE_FRAME, 3, 10.0)
        wait(6)

        step("3/11 创建防撞发布者并初始化区域1-4")
        gantry_pub = node.create_publisher(SafetyDetectMsg, TOPIC, qos_profile_sensor_data)
        wait(3)
        for area in (1, 2, 3, 4):
            send(gantry_pub, GANTRY_FRAME, area, 20.0)
        wait(6)

        step("4/11 正确frame但区域7/8：不能触发")
        send(gantry_pub, GANTRY_FRAME, 7, 10.0)
        send(gantry_pub, GANTRY_FRAME, 8, 10.0)
        wait(3)

        step("5/11 区域1报警：仅left开包")
        send(gantry_pub, GANTRY_FRAME, 1, 10.0)
        wait(6)

        step("6/11 left应已在最后报警后5秒关闭；侧面消息不触发，随后正确报警可新开")
        send(side_pub, SIDE_FRAME, 1, 20.0)
        send(side_pub, SIDE_FRAME, 3, 20.0)
        wait(5)
        send(gantry_pub, GANTRY_FRAME, 1, 10.0)
        wait(6)

        step("7/11 正常消息不触发；间隔超过5秒后正确报警应产生新left包")
        send(gantry_pub, GANTRY_FRAME, 1, 20.0)
        send(gantry_pub, GANTRY_FRAME, 3, 20.0)
        wait(5)
        send(gantry_pub, GANTRY_FRAME, 1, 10.0)
        wait(6)

        step("8/11 区域2报警：仅right开包")
        send(gantry_pub, GANTRY_FRAME, 2, 10.0)
        wait(0.5)

        step("9/11 销毁防撞发布端：应报告离线，right仍按固定窗口关闭")
        node.destroy_publisher(gantry_pub)
        gantry_pub = None
        wait(8)

        step("10/11 重建发布端：无需区域4解除，静默超过5秒后区域2报警可开新包")
        gantry_pub = node.create_publisher(SafetyDetectMsg, TOPIC, qos_profile_sensor_data)
        wait(3)
        for area in (1, 3, 2):
            send(gantry_pub, GANTRY_FRAME, area, 20.0)
        wait(5)
        send(gantry_pub, GANTRY_FRAME, 2, 10.0)
        wait(6)

        step("11/11 正常消息不触发；后续区域2报警可产生新right包")
        send(gantry_pub, GANTRY_FRAME, 2, 20.0)
        send(gantry_pub, GANTRY_FRAME, 4, 20.0)
        wait(6)
        send(gantry_pub, GANTRY_FRAME, 2, 10.0)
        wait(6)
        step("测试序列完成；核对录包日志后按 Ctrl+C 退出")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if gantry_pub is not None:
            node.destroy_publisher(gantry_pub)
        node.destroy_publisher(side_pub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
