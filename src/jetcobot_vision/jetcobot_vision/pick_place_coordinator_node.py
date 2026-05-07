#!/usr/bin/env python3
"""
PickPlaceCoordinatorNode
========================
Pick 완료 후 Place 신호를 받으면 자동으로 Place 수행.

Flow:
  1. VisionPick action 호출 → pick 완료 대기
  2. pick 완료 시 상태 저장
  3. PlaceCommand 토픽 구독 → place 신호 수신
  4. place 신호 수신 시 VisionPlace action 호출

Topic:
  /pick_place_coordinator/place_command [PlaceCommand]

Action:
  /vision_pick  [VisionPick]
  /vision_place [VisionPlace]
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from jetcobot_vision_msgs.action import VisionPick, VisionPlace
from jetcobot_vision_msgs.msg import PlaceCommand


class PickPlaceCoordinatorNode(Node):

    def __init__(self) -> None:
        super().__init__("pick_place_coordinator_node")

        self._cb_group = ReentrantCallbackGroup()

        # Action 클라이언트
        self._pick_client = ActionClient(self, VisionPick, "/vision_pick", callback_group=self._cb_group)
        self._place_client = ActionClient(self, VisionPlace, "/vision_place", callback_group=self._cb_group)

        # 상태 추적
        self._pick_done = False
        self._pick_location = None

        # PlaceCommand 구독
        self.create_subscription(
            PlaceCommand,
            "/pick_place_coordinator/place_command",
            self._on_place_command,
            10,
            callback_group=self._cb_group,
        )

        self.get_logger().info("PickPlaceCoordinatorNode 시작")

    def _on_place_command(self, msg: PlaceCommand) -> None:
        """PlaceCommand 토픽 수신."""
        if not self._pick_done:
            self.get_logger().warn("Pick이 완료되지 않았습니다. Place 스킵.")
            return

        self.get_logger().info(f"PlaceCommand 수신: location={msg.location} box_index={msg.box_index}")
        self._do_place(msg.location, msg.box_index)

    def _do_place(self, location: str, box_index: int = -1) -> None:
        """Place Action 호출."""
        if not self._place_client.server_is_ready():
            self.get_logger().error("VisionPlace action server 미준비")
            return

        goal = VisionPlace.Goal()
        goal.location = location
        if hasattr(goal, 'box_index'):
            goal.box_index = box_index

        self.get_logger().info(f"Place Goal 송신: location={location} box_index={box_index}")
        future = self._place_client.send_goal_async(goal)
        future.add_done_callback(self._place_response_callback)

    def _place_response_callback(self, future) -> None:
        """Place Goal 응답 처리."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Place Goal 거절됨")
            return

        self.get_logger().info("Place Goal 수락됨")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._place_result_callback)

        # Feedback 콜백
        def feedback_callback(msg):
            feedback = msg.feedback
            self.get_logger().info(f"  [Place {feedback.phase}] {feedback.progress*100:.0f}%")

        goal_handle.feedback_handle.add_callback(feedback_callback)

    def _place_result_callback(self, future) -> None:
        """Place 결과 처리."""
        result = future.result()
        self.get_logger().info(f"[Place 완료] success={result.success}  message={result.message}")
        self._pick_done = False  # 다음 pick을 위해 리셋

    def start_pick(self, location: str, box_index: int = -1) -> None:
        """Pick Action 시작."""
        if not self._pick_client.server_is_ready():
            self.get_logger().error("VisionPick action server 미준비")
            return

        goal = VisionPick.Goal()
        goal.location = location
        goal.box_index = box_index

        self._pick_location = location
        self._pick_done = False

        self.get_logger().info(f"Pick Goal 송신: location={location} box_index={box_index}")
        future = self._pick_client.send_goal_async(goal)
        future.add_done_callback(self._pick_response_callback)

    def _pick_response_callback(self, future) -> None:
        """Pick Goal 응답 처리."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Pick Goal 거절됨")
            return

        self.get_logger().info("Pick Goal 수락됨")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._pick_result_callback)

        # Feedback 콜백
        def feedback_callback(msg):
            feedback = msg.feedback
            self.get_logger().info(f"  [Pick {feedback.phase}] {feedback.progress*100:.0f}%")

        goal_handle.feedback_handle.add_callback(feedback_callback)

    def _pick_result_callback(self, future) -> None:
        """Pick 결과 처리."""
        result = future.result()
        self.get_logger().info(f"[Pick 완료] success={result.success}  message={result.message}")

        if result.success:
            self._pick_done = True
            self.get_logger().info(f"Pick 완료 → PlaceCommand 대기 중...")
        else:
            self._pick_done = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPlaceCoordinatorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
