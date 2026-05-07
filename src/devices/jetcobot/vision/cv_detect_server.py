#!/usr/bin/env python3
"""
OpenCV HSV 기반 상자 검출 서버 (메인 PC 실행)
=============================================
처리 파이프라인:
  UDP 수신 (stream_sender.py가 보낸 JPEG)
  → GaussianBlur → HSV 마스킹 → 형태학적 연산
  → findContours → 면적 필터 → minAreaRect
  → ObbBoxArray ROS2 토픽 발행 (/detect_bridge_node/obb_boxes)

실행 예:
    python3 src/devices/jetcobot/vision/cv_detect_server.py
    python3 src/devices/jetcobot/vision/cv_detect_server.py --preview
"""

import argparse
import math
import socket
import sys
import threading
import time
from typing import Optional

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse
from pydantic import BaseModel

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
    from std_msgs.msg import Header
    sys.path.insert(0, "/home/addinedu/roscamp-repo-1/install/jetcobot_vision_msgs/local/lib/python3.12/dist-packages")
    from jetcobot_vision_msgs.msg import ObbBox, ObbBoxArray
    _ROS2_OK = True
except ImportError:
    _ROS2_OK = False
    print("[WARN] rclpy 또는 jetcobot_vision_msgs 미설치 — ROS2 토픽 발행 비활성화")

# ── 전역 파라미터 ──────────────────────────────────────────────────────────────
HSV_LOWER  = np.array([0,   0,  208],  dtype=np.uint8)
HSV_UPPER  = np.array([158, 30, 255],  dtype=np.uint8)
MIN_AREA   = 1000
MAX_AREA   = 80000
MIN_W, MAX_W = 110, 200
MIN_H, MAX_H = 110, 200
MORPH_K    = 7
# ────────────────────────────────────────────────────────────────────────────

app = FastAPI(title="CV Detect Server (OpenCV OBB)")

# 최신 검출 결과 (timestamp 포함)
latest_result = {"detected": False, "detections": [], "timestamp": 0.0}


class DetectConfig(BaseModel):
    hsv_lower: Optional[list[int]] = None
    hsv_upper: Optional[list[int]] = None
    min_area:  Optional[int] = None
    max_area:  Optional[int] = None
    min_w:     Optional[int] = None
    max_w:     Optional[int] = None
    min_h:     Optional[int] = None
    max_h:     Optional[int] = None
    morph_k:   Optional[int] = None


@app.post("/config")
async def update_config(cfg: DetectConfig):
    """런타임 파라미터 변경. 전달된 필드만 업데이트."""
    global HSV_LOWER, HSV_UPPER, MIN_AREA, MAX_AREA, MIN_W, MAX_W, MIN_H, MAX_H, MORPH_K
    if cfg.hsv_lower is not None:
        HSV_LOWER = np.array(cfg.hsv_lower, dtype=np.uint8)
    if cfg.hsv_upper is not None:
        HSV_UPPER = np.array(cfg.hsv_upper, dtype=np.uint8)
    if cfg.min_area  is not None: MIN_AREA = cfg.min_area
    if cfg.max_area  is not None: MAX_AREA = cfg.max_area
    if cfg.min_w     is not None: MIN_W    = cfg.min_w
    if cfg.max_w     is not None: MAX_W    = cfg.max_w
    if cfg.min_h     is not None: MIN_H    = cfg.min_h
    if cfg.max_h     is not None: MAX_H    = cfg.max_h
    if cfg.morph_k   is not None: MORPH_K  = cfg.morph_k
    print(f"[CONFIG] HSV: {HSV_LOWER.tolist()} ~ {HSV_UPPER.tolist()}  "
          f"크기: {MIN_W}~{MAX_W} x {MIN_H}~{MAX_H}")
    return {"ok": True}


def detect_box_cv(img: np.ndarray) -> dict:
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask    = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_K, MORPH_K))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    raw_detections = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue

        rect = cv2.minAreaRect(cnt)
        cx, cy = float(rect[0][0]), float(rect[0][1])
        w,  h  = float(rect[1][0]), float(rect[1][1])
        angle  = rect[2]

        size_ok = (MIN_W <= w <= MAX_W and MIN_H <= h <= MAX_H) or \
                  (MIN_W <= h <= MAX_W and MIN_H <= w <= MAX_H)
        if not size_ok:
            continue

        if w < h:
            w, h = h, w
            angle += 90.0
        while angle >= 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0

        theta_rad  = math.radians(angle)
        hull_area  = cv2.contourArea(cv2.convexHull(cnt))
        confidence = min(area / hull_area, 1.0) if hull_area > 0 else 0.0

        raw_detections.append({
            "cx": cx, "cy": cy, "w": w, "h": h,
            "theta": theta_rad, "confidence": round(float(confidence), 4),
        })

    if not raw_detections:
        return {
            "detected": False, "count": 0, "detections": [],
            "cx": 0.0, "cy": 0.0, "w": 0.0, "h": 0.0, "theta": 0.0, "confidence": 0.0
        }

    raw_detections.sort(key=lambda d: d["cx"])
    detections = []
    for i, d in enumerate(raw_detections):
        d["id"] = i
        detections.append(d)

    first = detections[0]
    return {
        "detected":   True,
        "count":      len(detections),
        "detections": detections,
        "cx":         first["cx"],
        "cy":         first["cy"],
        "w":          first["w"],
        "h":          first["h"],
        "theta":      first["theta"],
        "confidence": first["confidence"],
    }


_latest_frame: Optional[np.ndarray] = None
_frame_lock = threading.Lock()

# ── ROS2 퍼블리셔 ─────────────────────────────────────────────────────────────
_ros2_pub = None

def _init_ros2_publisher(topic: str) -> None:
    global _ros2_pub
    if not _ROS2_OK:
        return
    rclpy.init()
    node = rclpy.create_node("cv_detect_server")
    sensor_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=1,
    )
    _ros2_pub = node.create_publisher(ObbBoxArray, topic, sensor_qos)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    print(f"[ROS2] 퍼블리셔 초기화 완료: {topic}")


def _publish_obb(detections: list) -> None:
    if _ros2_pub is None:
        return
    msg = ObbBoxArray()
    msg.header = Header()
    msg.header.stamp = rclpy.clock.Clock().now().to_msg()
    msg.header.frame_id = "camera_link"
    for i, det in enumerate(detections):
        msg.boxes.append(ObbBox(
            cx=float(det["cx"]),
            cy=float(det["cy"]),
            w=float(det["w"]),
            h=float(det["h"]),
            theta=float(det["theta"]),
            id=i,
            confidence=float(det["confidence"]),
        ))
    _ros2_pub.publish(msg)


def _draw_overlay(img: np.ndarray, result: dict) -> np.ndarray:
    """검출 결과를 이미지 위에 오버레이해서 반환."""
    display = img.copy()
    if result.get("detected"):
        for det in result["detections"]:
            cx   = int(det["cx"])
            cy   = int(det["cy"])
            w    = float(det["w"])
            h    = float(det["h"])
            theta = float(det["theta"])
            conf  = float(det["confidence"])

            # OBB 박스
            rect  = ((cx, cy), (w, h), math.degrees(theta))
            box   = cv2.boxPoints(rect).astype(np.int32)
            cv2.polylines(display, [box], True, (0, 255, 0), 2)

            # 중심점
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)

            # 텍스트 정보
            lines = [
                f"cx={cx} cy={cy}",
                f"w={w:.0f} h={h:.0f}",
                f"theta={math.degrees(theta):.1f}deg",
                f"conf={conf:.2f}",
            ]
            for i, txt in enumerate(lines):
                cv2.putText(display, txt, (cx + 10, cy - 40 + i * 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        count = result["count"]
        cv2.putText(display, f"Detected: {count}", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    else:
        cv2.putText(display, "SCANNING...", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return display


def _udp_receiver_loop(udp_port: int) -> None:
    """stream_sender.py가 보내는 UDP JPEG 프레임을 수신해서 즉시 검출 처리."""
    global latest_result, _latest_frame
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", udp_port))
    sock.settimeout(1.0)
    print(f"[UDP] 수신 대기 중: 0.0.0.0:{udp_port}")

    while True:
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] 수신 오류: {e}")
            continue

        nparr = np.frombuffer(data, np.uint8)
        img   = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if img is None:
            continue

        result = detect_box_cv(img)
        result["timestamp"] = time.time()
        latest_result = result

        _publish_obb(result.get("detections", []))

        with _frame_lock:
            _latest_frame = (img, result)


def _preview_loop() -> None:
    """메인 스레드에서 호출. cv2.imshow로 검출 결과 시각화."""
    cv2.namedWindow("CV Detect Preview", cv2.WINDOW_NORMAL)
    print("[PREVIEW] 'q' 키로 프리뷰 종료 (서버는 계속 실행됨)")
    while True:
        with _frame_lock:
            data = _latest_frame

        if data is not None:
            img, result = data
            display = _draw_overlay(img, result)
            cv2.imshow("CV Detect Preview", display)

        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


@app.get("/latest")
async def get_latest():
    return latest_result


@app.post("/detect")
async def detect(image: UploadFile = File(...)):
    """HTTP로 직접 이미지를 올려서 검출할 때 사용 (수동 테스트용)."""
    global latest_result
    try:
        contents = await image.read()
        nparr    = np.frombuffer(contents, np.uint8)
        img      = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("이미지 디코딩 실패")

        result = detect_box_cv(img)
        result["timestamp"] = time.time()
        latest_result = result
        return JSONResponse(result)

    except Exception as e:
        print(f"[ERROR] {e}")
        raise HTTPException(status_code=500, detail=str(e))


def main() -> None:
    global HSV_LOWER, HSV_UPPER, MIN_AREA, MAX_AREA, MIN_W, MAX_W, MIN_H, MAX_H, MORPH_K

    parser = argparse.ArgumentParser(description="OpenCV HSV 상자 검출 서버")
    parser.add_argument("--hsv-lower", type=int, nargs=3, default=[0, 0, 208],
                        metavar=("H", "S", "V"), help="HSV 하한 (기본: 0 0 208)")
    parser.add_argument("--hsv-upper", type=int, nargs=3, default=[158, 30, 255],
                        metavar=("H", "S", "V"), help="HSV 상한 (기본: 158 30 255)")
    parser.add_argument("--min-area",  type=int, default=1000)
    parser.add_argument("--max-area",  type=int, default=80000)
    parser.add_argument("--min-w",     type=int, default=110)
    parser.add_argument("--max-w",     type=int, default=200)
    parser.add_argument("--min-h",     type=int, default=110)
    parser.add_argument("--max-h",     type=int, default=200)
    parser.add_argument("--morph-k",   type=int, default=7)
    parser.add_argument("--host",      default="0.0.0.0")
    parser.add_argument("--port",      type=int, default=8081)
    parser.add_argument("--udp-port",  type=int, default=5000,
                        help="stream_sender.py UDP 수신 포트 (기본: 5000)")
    parser.add_argument("--preview",   action="store_true",
                        help="cv2.imshow로 검출 결과 실시간 표시")
    parser.add_argument("--ros2-topic", default="/detect_bridge_node/obb_boxes",
                        help="ObbBoxArray 발행 토픽 (기본: /detect_bridge_node/obb_boxes)")
    args = parser.parse_args()

    HSV_LOWER = np.array(args.hsv_lower, dtype=np.uint8)
    HSV_UPPER = np.array(args.hsv_upper, dtype=np.uint8)
    MIN_AREA  = args.min_area
    MAX_AREA  = args.max_area
    MIN_W, MAX_W = args.min_w, args.max_w
    MIN_H, MAX_H = args.min_h, args.max_h
    MORPH_K   = args.morph_k

    print(f"[INFO] OpenCV HSV 상자 검출 서버 시작")
    print(f"  HSV 범위: lower={args.hsv_lower}  upper={args.hsv_upper}")
    print(f"  크기 범위: {args.min_w}~{args.max_w} x {args.min_h}~{args.max_h} px")
    print(f"  HTTP 서버: http://{args.host}:{args.port}")
    print(f"  UDP 수신:  0.0.0.0:{args.udp_port}")
    print(f"  ROS2 토픽: {args.ros2_topic}")
    print(f"  프리뷰:    {'ON' if args.preview else 'OFF'}")

    _init_ros2_publisher(args.ros2_topic)

    udp_thread = threading.Thread(
        target=_udp_receiver_loop, args=(args.udp_port,), daemon=True
    )
    udp_thread.start()

    if args.preview:
        # uvicorn을 백그라운드 스레드로 돌리고 메인 스레드에서 imshow 실행
        # (cv2.imshow는 메인 스레드에서만 동작)
        server_thread = threading.Thread(
            target=uvicorn.run,
            kwargs={"app": app, "host": args.host, "port": args.port},
            daemon=True,
        )
        server_thread.start()
        _preview_loop()
    else:
        uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
