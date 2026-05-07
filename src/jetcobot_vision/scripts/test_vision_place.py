#!/usr/bin/env python3
"""
VisionPlace Action Server 테스트

Usage:
    python3 test_vision_place.py pinky_tray_place
    python3 test_vision_place.py pinky_tray_place --box 1
"""

import argparse
import sys
import time
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from jetcobot_vision_msgs.action import VisionPlace
except ImportError as e:
    print(f"[ERR] ROS2 setup필요: {e}")
    sys.exit(1)

class VisionPlaceTestClient(Node):
    def __init__(self):
        super().__init__("vision_place_test_client")
        self.action_client = ActionClient(self, VisionPlace, "/vision_place")
        self.get_logger().info("VisionPlace Action Client 준비 중...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("[ERR] /vision_place action server 미응답")
        self.get_logger().info("[OK] /vision_place action server 연결됨")

    def send_goal(self, location: str, box_index: int = -1):
        goal = VisionPlace.Goal()
        goal.location = location
        if hasattr(goal, 'box_index'):
            goal.box_index = box_index

        idx_str = f"(index={box_index})" if box_index >= 0 else "(최고 confidence)"
        self.get_logger().info(f"Goal 송신: location={location} {idx_str}")
        future = self.action_client.send_goal_async(goal)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goal 거절됨")
                return

            self.get_logger().info("Goal 수락됨")

            result_future = goal_handle.get_result_async()

            def result_callback(future):
                result = future.result()
                self.get_logger().info(f"[RESULT]")
                self.get_logger().info(f"  success: {result.success}")
                self.get_logger().info(f"  message: {result.message}")
                if result.place_point_base:
                    self.get_logger().info(f"  place_point_base: ({result.place_point_base.x:.3f}, "
                                         f"{result.place_point_base.y:.3f}, {result.place_point_base.z:.3f})")
                rclpy.shutdown()

            result_future.add_done_callback(result_callback)

        future.add_done_callback(goal_response_callback)

def main():
    parser = argparse.ArgumentParser(description="Test VisionPlace Action Server")
    parser.add_argument("location", help="Location name (pinky_tray_place, warehouse_place, etc.)")
    parser.add_argument("--box", type=int, default=-1, help="Box index (-1 = highest confidence)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Timeout in seconds")

    args = parser.parse_args()

    rclpy.init()
    try:
        client = VisionPlaceTestClient()
        client.send_goal(args.location, args.box)

        # Spin with timeout
        start = time.time()
        while time.time() - start < args.timeout:
            rclpy.spin_once(client, timeout_sec=0.1)

        client.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        client.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"[ERR] {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
