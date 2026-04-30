#!/usr/bin/env python3
"""
VisionDetectorNode
==================
역할
  1. RemoteCapture 로 UDP 영상 스트림 수신 (jetcam.py 프로토콜, 대역폭 유지)
  2. YOLO OBB 모델로 상자 검출
  3. 결과를 ObbBoxArray 토픽으로 Publish

토픽
  ~/obb_boxes  [jetcobot_vision_msgs/ObbBoxArray]  QoS: BestEffort / KeepLast 1

파라미터
  udp_host        str   "0.0.0.0"
  udp_port        int   7007
  robot_id        int   5
  model_path      str   "yolov8n-obb-box.pt"
  conf_threshold  float 0.5
  publish_hz      float 10.0
"""

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
from .remote_capture import RemoteCapture

try:
    from ultralytics import YOLO as _YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False


class VisionDetectorNode(Node):

    def __init__(self) -> None:
        super().__init__("vision_detector_node")

        # ── 파라미터 ──────────────────────────────────────────────────────────
        self.declare_parameter("udp_host",       "0.0.0.0")
        self.declare_parameter("udp_port",       7007)
        self.declare_parameter("robot_id",       5)
        self.declare_parameter("model_path",     "yolov8n-obb-box.pt")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("publish_hz",     10.0)

        udp_host   = self.get_parameter("udp_host").value
        udp_port   = self.get_parameter("udp_port").value
        robot_id   = self.get_parameter("robot_id").value
        model_path = self.get_parameter("model_path").value
        self._conf = self.get_parameter("conf_threshold").value
        hz         = self.get_parameter("publish_hz").value

        # ── QoS: 센서 스트림 — BestEffort, KeepLast 1 ────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(ObbBoxArray, "~/obb_boxes", sensor_qos)

        # ── YOLO 모델 로딩 ────────────────────────────────────────────────────
        if _YOLO_OK:
            self.get_logger().info(f"YOLO 모델 로딩: {model_path}")
            self._model = _YOLO(model_path)
            self.get_logger().info("YOLO 모델 로딩 완료")
        else:
            self._model = None
            self.get_logger().warn(
                "ultralytics 미설치 — YOLO 추론 불가. 모의 검출 모드로 동작합니다."
            )

        # ── UDP 수신기 ────────────────────────────────────────────────────────
        self._capture = RemoteCapture(udp_host, udp_port, robot_id)
        self._capture.start()
        self.get_logger().info(
            f"UDP 수신 시작: {udp_host}:{udp_port}  robot_id={robot_id}"
        )

        # ── 검출 타이머 (MutuallyExclusive: 타이머 콜백이 중첩되지 않음) ───
        _cb = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / hz, self._detect_and_publish, callback_group=_cb)

        self.get_logger().info(
            f"VisionDetectorNode 시작 완료  (publish_hz={hz})"
        )

    # ── 검출 + 발행 ───────────────────────────────────────────────────────────

    def _detect_and_publish(self) -> None:
        ok, frame = self._capture.read()
        if not ok or frame is None:
            return
        if self._model is None:
            return

        results = self._model(frame, verbose=False)
        result  = results[0]

        msg = ObbBoxArray()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id="camera_link",
        )

        if result.obb is not None and result.obb.xywhr is not None:
            xywhr   = result.obb.xywhr.cpu().numpy()           # (N, 5)
            confs   = result.obb.conf.cpu().numpy()            # (N,)
            cls_ids = result.obb.cls.cpu().numpy().astype(int) # (N,)

            for row, conf, cls_id in zip(xywhr, confs, cls_ids):
                if conf < self._conf:
                    continue
                cx, cy, w, h, theta = row
                box = ObbBox(
                    cx=float(cx),
                    cy=float(cy),
                    w=float(w),
                    h=float(h),
                    theta=float(theta),
                    id=int(cls_id),
                    confidence=float(conf),
                )
                msg.boxes.append(box)

        self._pub.publish(msg)

        if msg.boxes:
            self.get_logger().debug(
                f"OBB 검출: {len(msg.boxes)}개 발행 "
                f"(best_conf={max(b.confidence for b in msg.boxes):.2f})"
            )

    # ── 소멸 시 정리 ──────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._capture.stop()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
