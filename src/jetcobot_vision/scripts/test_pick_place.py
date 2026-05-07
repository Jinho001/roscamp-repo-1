#!/usr/bin/env python3
"""
Pick-Place 통합 테스트

Pick 완료 후 PlaceCommand 토픽으로 Place 신호 전송.

Usage:
    python3 test_pick_place.py tray pinky_tray_place
    python3 test_pick_place.py tray pinky_tray_place --pick-box 0 --place-box 1
"""

import argparse
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from jetcobot_vision_msgs.msg import PlaceCommand
except ImportError as e:
    print(f"[ERR] ROS2 setup필요: {e}")
    sys.exit(1)


class PickPlaceTest(Node):
    def __init__(self):
        super().__init__("pick_place_test")
        self.place_pub = self.create_publisher(PlaceCommand, "/pick_place_coordinator/place_command", 10)
        self.get_logger().info("PlaceCommand Publisher 준비됨")

    def send_place_command(self, location: str, box_index: int = -1):
        msg = PlaceCommand()
        msg.location = location
        msg.box_index = box_index

        self.get_logger().info(f"PlaceCommand 발행: location={location} box_index={box_index}")
        self.place_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Test Pick-Place Coordinator")
    parser.add_argument("pick_location", help="Pick location name")
    parser.add_argument("place_location", help="Place location name")
    parser.add_argument("--pick-box", type=int, default=-1, help="Pick box index")
    parser.add_argument("--place-box", type=int, default=-1, help="Place box index")
    parser.add_argument("--pick-delay", type=float, default=5.0, help="Delay before starting pick (s)")
    parser.add_argument("--place-delay", type=float, default=5.0, help="Delay after pick before place (s)")

    args = parser.parse_args()

    rclpy.init()
    try:
        node = PickPlaceTest()

        # Pick 시작 (coordinator.start_pick 호출)
        node.get_logger().info(
            f"Pick 시작 준비... ({args.pick_delay}초 후)"
            f"\n  Pick Location: {args.pick_location}, Box Index: {args.pick_box}"
            f"\n  Place Location: {args.place_location}, Box Index: {args.place_box}"
        )
        time.sleep(args.pick_delay)

        # NOTE: coordinator.start_pick는 ROS2 action 호출이므로 별도 클라이언트 필요
        # 여기서는 coordinator가 실행 중이고 pick을 먼저 시작했다고 가정
        node.get_logger().info("Pick 진행 중... (coordinator에서 처리)")

        # Place 신호 전송 대기
        node.get_logger().info(f"Pick 완료 대기 중... ({args.place_delay}초 후 Place 신호 전송)")
        time.sleep(args.place_delay)

        # Place Command 발행
        node.send_place_command(args.place_location, args.place_box)

        node.get_logger().info("테스트 완료. coordinator 로그 확인...")
        time.sleep(2)

        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
