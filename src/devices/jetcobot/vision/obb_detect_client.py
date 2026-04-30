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
import time
from threading import Thread, Lock

# src 폴더를 찾을 수 있도록 프로젝트 루트 경로를 sys.path에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))

import cv2
import numpy as np
import requests

from src.devices.jetcobot.vision.remote_capture import RemoteCapture
# ── 기본 설정 ─────────────────────────────────────────────────────────────────
GPU_SERVER_URL  = os.environ.get(
    "OBB_SERVER_URL", "http://localhost:8081/detect"
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
    """카메라 영상을 실시간으로 서버에 전송해 검출 결과를 화면에 표시 (동결 방지 쓰레딩 버전)"""
    if use_remote:
        print("[INFO] 원격 영상 수신 중 (포트 5000)...")
        cap = RemoteCapture(port=5000)
        time.sleep(0.5)
    else:
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            print(f"[ERROR] 카메라 열기 실패: {device}")
            return

    print(f"[INFO] OBB 서버: {server_url}")
    print("  'q' 로 종료 (No-Freeze Mode)")

    # ── 공유 데이터 공간 ──────────────────────
    state = {
        "latest_result": {"detected": False, "detections": []},
        "is_processing": False,
        "elapsed_ms": 0.0
    }
    lock = Lock()

    def detection_task(img):
        """백그라운드에서 서버에 검출 요청을 보내는 함수"""
        try:
            t0 = time.time()
            res = detect_object(img, server_url=server_url)
            dt = (time.time() - t0) * 1000.0
            with lock:
                state["latest_result"] = res
                state["elapsed_ms"] = dt
        except Exception as e:
            print(f"[DEBUG] 검출 실패: {e}")
        finally:
            with lock:
                state["is_processing"] = False

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 1. 검출 쓰레드가 쉬고 있다면 새로운 프레임 투입
        with lock:
            if not state["is_processing"]:
                state["is_processing"] = True
                Thread(target=detection_task, args=(frame.copy(),), daemon=True).start()

            # 현재까지의 최신 결과 복사
            result = state["latest_result"].copy()
            latency = state["elapsed_ms"]

        # 2. 시각화 (항상 최신 프레임 위에 마지막 결과를 그림)
        display = frame.copy()
        if result.get("detected"):
            detections = result.get("detections", [])
            for det in detections:
                cx, cy = int(det["cx"]), int(det["cy"])
                w, h = float(det.get("w", 0)), float(det.get("h", 0))
                theta = det["theta"]
                
                # OBB 박스 계산 및 그리기
                rect = ((cx, cy), (w, h), np.degrees(theta))
                box = cv2.boxPoints(rect)
                box = np.int32(box)
                cv2.polylines(display, [box], True, (0, 255, 0), 2)
                cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(display, f"{w:.0f}x{h:.0f}", (cx-20, cy-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

            cv2.putText(display, f"Detected: {len(detections)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "SCANNING...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # 3. FPS/Latency 정보
        cv2.putText(display, f"Server: {latency:.0f}ms", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        # 4. 화면 출력 (여기서 멈추지 않음)
        cv2.imshow("OBB Client (Threaded)", display)
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
