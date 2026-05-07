#!/usr/bin/env python3
"""
VisionPick Action Server 테스트

Usage:
    python3 test_vision_pick.py tray
    python3 test_vision_pick.py receiving_zone
    python3 test_vision_pick.py --list
"""

import argparse
import sys
import time
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from jetcobot_vision_msgs.action import VisionPick
except ImportError as e:
    print(f"[ERR] ROS2 setup필요: {e}")
    sys.exit(1)

class VisionPickTestClient(Node):
    def __init__(self):
        super().__init__("vision_pick_test_client")
        self.action_client = ActionClient(self, VisionPick, "/vision_pick")
        self.get_logger().info("VisionPick Action Client 준비 중...")
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("[ERR] /vision_pick action server 미응답")
        self.get_logger().info("[OK] /vision_pick action server 연결됨")

    def send_goal(self, location: str):
        goal = VisionPick.Goal()
        goal.location = location

        self.get_logger().info(f"Goal 송신: location={location}")
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
                if result.pick_point_base:
                    self.get_logger().info(f"  pick_point_base: ({result.pick_point_base.x:.3f}, "
                                         f"{result.pick_point_base.y:.3f}, {result.pick_point_base.z:.3f})")
                rclpy.shutdown()

            result_future.add_done_callback(result_callback)

            # Feedback 콜백
            def feedback_callback(feedback_msg):
                feedback = feedback_msg.feedback
                self.get_logger().info(f"  [{feedback.phase}] {feedback.progress*100:.0f}%")

            goal_handle.feedback_handle.add_callback(feedback_callback)

        future.add_done_callback(goal_response_callback)

def main():
    parser = argparse.ArgumentParser(description="Test VisionPick Action Server")
    parser.add_argument("location", help="Location name (tray, receiving_zone, etc.)")
    parser.add_argument("--list", action="store_true", help="List all locations (TODO)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Timeout in seconds")

    args = parser.parse_args()

    rclpy.init()
    try:
        client = VisionPickTestClient()
        client.send_goal(args.location)

        # Spin with timeout
        start = time.time()
        while time.time() - start < args.timeout:
            rclpy.spin_once(client, timeout_sec=0.1)

        client.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"[ERR] {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
