#!/usr/bin/env python3
"""
DetectBridgeNode
================
역할
  cv_detect_server.py의 /latest 엔드포인트를 주기적으로 폴링하여
  검출 결과를 ObbBoxArray ROS2 토픽으로 변환해 발행.

토픽
  ~/obb_boxes  [jetcobot_vision_msgs/ObbBoxArray]  QoS: BestEffort / KeepLast 1

파라미터
  server_url       str    "http://localhost:8081/latest"
  poll_hz          float  10.0
  request_timeout  float  1.0
"""

import requests
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import Header
from jetcobot_vision_msgs.msg import ObbBox, ObbBoxArray


class DetectBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__("detect_bridge_node")

        self.declare_parameter("server_url",      "http://localhost:8081/latest")
        self.declare_parameter("poll_hz",         10.0)
        self.declare_parameter("request_timeout", 1.0)

        self._server_url = self.get_parameter("server_url").value
        self._timeout    = self.get_parameter("request_timeout").value
        hz               = self.get_parameter("poll_hz").value

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self._pub = self.create_publisher(ObbBoxArray, "~/obb_boxes", sensor_qos)

        _cb = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / hz, self._poll_and_publish, callback_group=_cb)
        self.get_logger().info(
            f"DetectBridgeNode 시작  (server={self._server_url}, hz={hz})"
        )

    def _poll_and_publish(self) -> None:
        try:
            resp = requests.get(self._server_url, timeout=self._timeout)
            resp.raise_for_status()
            data = resp.json()
        except requests.exceptions.ConnectionError:
            self.get_logger().warn(
                f"cv_detect_server 연결 실패 — 실행 중인지 확인: {self._server_url}",
                throttle_duration_sec=5.0,
            )
            return
        except Exception as exc:
            self.get_logger().warn(f"폴링 오류: {exc}", throttle_duration_sec=5.0)
            return

        msg = ObbBoxArray()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="camera_link",
        )

        if data.get("detected") and data.get("detections"):
            for det in data["detections"]:
                msg.boxes.append(ObbBox(
                    cx=float(det["cx"]),
                    cy=float(det["cy"]),
                    w=float(det["w"]),
                    h=float(det["h"]),
                    theta=float(det["theta"]),
                    id=int(det["id"]),
                    confidence=float(det["confidence"]),
                ))

        self._pub.publish(msg)

        if msg.boxes:
            self.get_logger().debug(
                f"브리지: {len(msg.boxes)}개 "
                f"(best_conf={max(b.confidence for b in msg.boxes):.2f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
