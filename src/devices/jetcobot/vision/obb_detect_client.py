#!/usr/bin/env python3
"""
OBB (Oriented Bounding Box) 검출 API 클라이언트
=============================================
FastAPI 기반 OBB 서버(YOLO 또는 OpenCV)에 이미지를 보내
검출 결과(cx, cy, w, h, theta)를 받아오는 범용 클라이언트 모듈입니다.

단독 테스트:
    python3 src/devices/jetcobot/vision/obb_detect_client.py [--device /dev/jetcocam0]
"""

import argparse
import os
import sys
import os

# src 폴더를 찾을 수 있도록 프로젝트 루트 경로를 sys.path에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))

import cv2
import numpy as np
import time
import requests

from src.devices.jetcobot.vision.remote_capture import RemoteCapture
# ── 기본 설정 ─────────────────────────────────────────────────────────────────
GPU_SERVER_URL  = os.environ.get(
    "OBB_SERVER_URL", "http://localhost:8080/detect"
)
CAMERA_DEVICE   = "/dev/jetcocam0"
REQUEST_TIMEOUT = 3.0       # 초
JPEG_QUALITY    = 90        # POST 이미지 품질
# ─────────────────────────────────────────────────────────────────────────────


def detect_object(frame: np.ndarray, timeout: float = REQUEST_TIMEOUT, server_url: str = None) -> dict:
    """
    카메라 프레임을 GPU 서버로 전송해 OBB 검출 결과를 반환한다.
    """
    if server_url is None:
        server_url = GPU_SERVER_URL
        
    ok, buf = cv2.imencode(
        ".jpg", frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
    )
    if not ok:
        raise RuntimeError("JPEG 인코딩 실패")

    resp = requests.post(
        server_url,
        files={"image": ("frame.jpg", buf.tobytes(), "image/jpeg")},
        timeout=timeout,
    )
    resp.raise_for_status()
    return resp.json()


def get_latest_result(server_url: str = None, timeout: float = REQUEST_TIMEOUT) -> dict:
    """
    서버가 이미 처리 중인 가장 최신 검출 결과(/latest)를 가져온다.
    제어 PC에서 직접 촬영하지 않을 때 사용.
    """
    if server_url is None:
        server_url = GPU_SERVER_URL
    
    # /detect -> /latest 로 경로 변경
    base_url = server_url.rsplit('/', 1)[0]
    latest_url = f"{base_url}/latest"
    
    try:
        resp = requests.get(latest_url, timeout=timeout)
        resp.raise_for_status()
        return resp.json()
    except Exception as e:
        print(f"[ERROR] 실시간 데이터 획득 실패: {e}")
        return {"detected": False, "detections": []}


# ── 단독 테스트 ───────────────────────────────────────────────────────────────

def _run_live_test(device: str, server_url: str, use_remote: bool = False) -> None:
    """카메라 영상을 실시간으로 서버에 전송해 검출 결과를 화면에 표시"""
    if use_remote:
        print("[INFO] 원격 영상 수신 중 (포트 5000)...")
        cap = RemoteCapture(port=5000)
        time.sleep(0.5)
    else:
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            print(f"[ERROR] 카메라 열기 실패: {device}")
            sys.exit(1)

    print(f"[INFO] OBB 서버: {server_url}")
    print(f"[INFO] 카메라  : {device}")
    print("  'q' 로 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        display = frame.copy()

        try:
            t0 = time.time()
            result = detect_object(frame, server_url=server_url)
            elapsed_ms = (time.time() - t0) * 1000.0

            if result["detected"]:
                # 서버에서 detections 리스트를 가져옴 (없으면 단일 결과로 리스트 생성)
                detections = result.get("detections", [])
                if not detections and "cx" in result:
                    detections = [{
                        "cx": result["cx"], "cy": result["cy"],
                        "w": result.get("w",0), "h": result.get("h",0),
                        "theta": result["theta"], "confidence": result["confidence"],
                        "id": 0
                    }]

                for det in detections:
                    cx = int(det["cx"])
                    cy = int(det["cy"])
                    w  = float(det.get("w", 0))
                    h  = float(det.get("h", 0))
                    theta = det["theta"]
                    conf  = det["confidence"]
                    idx   = det.get("id", 0)

                    # ── OBB 4개 꼭짓점 계산 ──────────────────────
                    cos_t, sin_t = np.cos(theta), np.sin(theta)
                    hw, hh = w / 2, h / 2
                    corners_local = np.array([[-hw, -hh], [hw, -hh], [hw, hh], [-hw, hh]], dtype=np.float32)
                    rot = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
                    corners = (corners_local @ rot.T + np.array([cx, cy])).astype(np.int32)

                    # OBB 윤곽선 (초록색)
                    cv2.polylines(display, [corners], isClosed=True, color=(0, 255, 0), thickness=2)

                    # 중심점 (노란색 십자선)
                    cv2.circle(display, (cx, cy), 5, (0, 255, 255), -1)
                    
                    # 박스 구분 번호 표시 (상단에 Box #N)
                    label = f"Box #{idx}"
                    cv2.putText(display, label, (cx - 20, cy - int(h/2) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                # 전체 정보 텍스트 (첫 번째 상자 기준 요약)
                count = len(detections)
                cv2.putText(display, f"Detected: {count} boxes", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
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
    parser.add_argument(
        "--remote",
        action="store_true",
        help="원격 영상 수신(RemoteCapture) 사용 여부",
    )
    args = parser.parse_args()

    _run_live_test(args.device, args.server, args.remote)


if __name__ == "__main__":
    main()
