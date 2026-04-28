#!/usr/bin/env python3
"""
YOLO OBB 상자 검출 HTTP 클라이언트  (TASK-V03-B)
================================================
GPU 서버의 obb_server.py (포트 8080) 로 이미지를 POST 하고
검출 결과 (cx, cy, theta) 를 반환한다.

단독 테스트:
    python yolo_detect_client.py [--device /dev/jetcocam0]
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import requests


# ── 기본 설정 ─────────────────────────────────────────────────────────────────
GPU_SERVER_URL  = os.environ.get(
    "OBB_SERVER_URL", "http://192.168.1.120:8080/detect"
)
CAMERA_DEVICE   = "/dev/jetcocam0"
REQUEST_TIMEOUT = 3.0       # 초
JPEG_QUALITY    = 90        # POST 이미지 품질
# ─────────────────────────────────────────────────────────────────────────────


def detect_object(frame: np.ndarray, timeout: float = REQUEST_TIMEOUT) -> dict:
    """
    카메라 프레임을 GPU 서버로 전송해 OBB 검출 결과를 반환한다.

    Parameters
    ----------
    frame   : np.ndarray  — BGR 이미지 (cv2.imread / cap.read 결과)
    timeout : float       — HTTP 요청 타임아웃 (초)

    Returns
    -------
    dict with keys:
        detected   : bool
        cx         : float  — 픽셀 좌표 (미검출 시 0.0)
        cy         : float
        theta      : float  — OBB 장축 각도 (rad, 미검출 시 0.0)
        confidence : float  — 신뢰도 (미검출 시 0.0)

    Raises
    ------
    requests.RequestException : 서버 통신 오류
    """
    ok, buf = cv2.imencode(
        ".jpg", frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
    )
    if not ok:
        raise RuntimeError("JPEG 인코딩 실패")

    resp = requests.post(
        GPU_SERVER_URL,
        files={"image": ("frame.jpg", buf.tobytes(), "image/jpeg")},
        timeout=timeout,
    )
    resp.raise_for_status()
    return resp.json()


# ── 단독 테스트 ───────────────────────────────────────────────────────────────

def _run_live_test(device: str) -> None:
    """카메라 영상을 실시간으로 서버에 전송해 검출 결과를 화면에 표시"""
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"[ERROR] 카메라 열기 실패: {device}")
        sys.exit(1)

    print(f"[INFO] OBB 서버: {GPU_SERVER_URL}")
    print(f"[INFO] 카메라  : {device}")
    print("  'q' 로 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        display = frame.copy()

        try:
            t0 = time.time()
            result = detect_object(frame)
            elapsed_ms = (time.time() - t0) * 1000.0

            if result["detected"]:
                cx = int(result["cx"])
                cy = int(result["cy"])
                theta = result["theta"]
                conf  = result["confidence"]

                # 중심점 표시
                cv2.circle(display, (cx, cy), 6, (0, 255, 0), -1)

                # OBB 장축 방향 화살표
                length = 40
                ex = int(cx + length * np.cos(theta))
                ey = int(cy + length * np.sin(theta))
                cv2.arrowedLine(display, (cx, cy), (ex, ey), (0, 255, 0), 2)

                info = f"cx={cx} cy={cy} theta={np.degrees(theta):.1f}deg conf={conf:.2f}"
                cv2.putText(display, info, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(display, "NO OBJECT", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.putText(display, f"{elapsed_ms:.0f} ms", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        except requests.RequestException as e:
            cv2.putText(display, f"SERVER ERROR: {e}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        except Exception as e:
            cv2.putText(display, f"ERROR: {e}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow("OBB Detect Client [TASK-V03]", display)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="YOLO OBB 검출 클라이언트 테스트 (TASK-V03-B)"
    )
    parser.add_argument("--device", default=CAMERA_DEVICE)
    parser.add_argument(
        "--server",
        default=GPU_SERVER_URL,
        help=f"OBB HTTP 서버 URL (기본: {GPU_SERVER_URL})",
    )
    args = parser.parse_args()

    global GPU_SERVER_URL
    GPU_SERVER_URL = args.server

    _run_live_test(args.device)


if __name__ == "__main__":
    main()
