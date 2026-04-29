#!/usr/bin/env python3
"""
HSV 색상 범위 실시간 튜닝 도구  (옵션 B - PHASE B-1)
====================================================
원격 영상 스트림을 수신하면서 HSV 슬라이더를 조작하여
상자만 흰색(마스크)으로 남는 최적 파라미터를 찾는다.

실행:
    python3 src/devices/jetcobot/vision/hsv_tuner.py
    python3 src/devices/jetcobot/vision/hsv_tuner.py --local --device /dev/video0

조작 방법:
    - 슬라이더를 움직여 마스크 창에서 상자만 흰색으로 남도록 조정
    - 'p' 키: 현재 HSV 범위 터미널에 출력
    - 'q' 키: 종료
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

# RemoteCapture 임포트
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
from src.devices.jetcobot.vision.remote_capture import RemoteCapture

WINDOW_SRC   = "HSV Tuner - Source"
WINDOW_MASK  = "HSV Tuner - Mask (상자만 흰색으로 남기세요)"
WINDOW_CTRL  = "HSV Tuner - Controls"


def create_trackbars(window: str) -> None:
    """HSV 슬라이더 6개 생성."""
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 400, 300)

    # 초기값: 흰색/밝은 계열 (상자에 맞게 조정 필요)
    cv2.createTrackbar("H Lower", window,   0, 179, lambda v: None)
    cv2.createTrackbar("S Lower", window,   0, 255, lambda v: None)
    cv2.createTrackbar("V Lower", window, 100, 255, lambda v: None)
    cv2.createTrackbar("H Upper", window, 179, 179, lambda v: None)
    cv2.createTrackbar("S Upper", window,  80, 255, lambda v: None)
    cv2.createTrackbar("V Upper", window, 255, 255, lambda v: None)


def get_hsv_range(window: str) -> tuple[np.ndarray, np.ndarray]:
    """슬라이더 현재값을 읽어 (lower, upper) 반환."""
    hl = cv2.getTrackbarPos("H Lower", window)
    sl = cv2.getTrackbarPos("S Lower", window)
    vl = cv2.getTrackbarPos("V Lower", window)
    hu = cv2.getTrackbarPos("H Upper", window)
    su = cv2.getTrackbarPos("S Upper", window)
    vu = cv2.getTrackbarPos("V Upper", window)
    return np.array([hl, sl, vl]), np.array([hu, su, vu])


def apply_mask(frame: np.ndarray,
               hsv_lower: np.ndarray,
               hsv_upper: np.ndarray,
               morph_ksize: int = 5) -> tuple[np.ndarray, np.ndarray]:
    """
    HSV 마스킹 및 형태학적 연산 적용.
    Returns: (masked_frame, binary_mask)
    """
    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lower, hsv_upper)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_ksize, morph_ksize))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 구멍 메우기
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)  # 잡음 제거

    masked = cv2.bitwise_and(frame, frame, mask=mask)
    return masked, mask


def draw_contours_on_frame(display: np.ndarray, mask: np.ndarray,
                            min_area: int = 500) -> np.ndarray:
    """마스크에서 윤곽선을 찾아 minAreaRect로 OBB를 표시."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        rect = cv2.minAreaRect(cnt)
        box_pts = cv2.boxPoints(rect).astype(np.int32)
        cv2.polylines(display, [box_pts], True, (0, 255, 0), 2)
        cx, cy = int(rect[0][0]), int(rect[0][1])
        theta = rect[2]
        cv2.circle(display, (cx, cy), 5, (0, 255, 255), -1)
        cv2.putText(display, f"θ={theta:.1f}°", (cx + 8, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return display


def main() -> None:
    parser = argparse.ArgumentParser(description="HSV 실시간 튜닝 도구")
    parser.add_argument("--local",  action="store_true", help="로컬 카메라 사용")
    parser.add_argument("--device", default="/dev/video0", help="로컬 카메라 장치")
    parser.add_argument("--port",   type=int, default=5000, help="원격 수신 UDP 포트")
    parser.add_argument("--min-area", type=int, default=500, help="검출 최소 면적 (px²)")
    args = parser.parse_args()

    # 카메라 초기화
    if args.local:
        cap = cv2.VideoCapture(args.device)
        print(f"[INFO] 로컬 카메라 사용: {args.device}")
    else:
        cap = RemoteCapture(port=args.port)
        print(f"[INFO] 원격 스트림 수신 중 (포트: {args.port})")
        time.sleep(0.5)

    create_trackbars(WINDOW_CTRL)
    cv2.namedWindow(WINDOW_SRC,  cv2.WINDOW_NORMAL)
    cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)

    print("\n[조작 방법]")
    print("  슬라이더 조작 → 마스크 창에서 상자만 흰색으로 남도록 조정")
    print("  'p' 키         → 현재 HSV 범위 터미널에 출력")
    print("  'q' 키         → 종료\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            continue

        hsv_lower, hsv_upper = get_hsv_range(WINDOW_CTRL)
        masked, mask = apply_mask(frame, hsv_lower, hsv_upper)

        display = frame.copy()
        display = draw_contours_on_frame(display, mask, args.min_area)

        cv2.imshow(WINDOW_SRC,  display)
        cv2.imshow(WINDOW_MASK, mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("p"):
            lo = hsv_lower.tolist()
            hi = hsv_upper.tolist()
            print(f"\n[RESULT] 현재 HSV 범위")
            print(f"  --hsv-lower {lo[0]} {lo[1]} {lo[2]}")
            print(f"  --hsv-upper {hi[0]} {hi[1]} {hi[2]}\n")
            print("  workspace_config.yaml 또는 cv_detect_server.py 인자로 사용하세요.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
