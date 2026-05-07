#!/usr/bin/env python3
"""
Location별 HSV/W,H 파라미터 통합 튜닝 도구
================================================
UDP 스트림을 수신하며 슬라이더로 각 location의 파라미터를 실시간 조정.
'p' 키로 pick_place_profiles.yaml에 자동 저장.

실행:
    python3 src/devices/jetcobot/vision/profile_tuner.py --location tray
    python3 src/devices/jetcobot/vision/profile_tuner.py --list

조작:
    - 슬라이더로 HSV/W/H 조정
    - '1'~'5': location 전환 (tray, receiving_zone, pinky_tray_place, ...)
    - 'p': pick_place_profiles.yaml에 저장
    - 'q': 종료
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))
from remote_capture import RemoteCapture

WINDOW_SRC  = "Profile Tuner - Source"
WINDOW_MASK = "Profile Tuner - Mask"
WINDOW_CTRL = "Profile Tuner - Controls"

LOCATIONS = ["tray", "receiving_zone", "pinky_tray_place", "warehouse_pick", "warehouse_place"]
VISION_LOCATIONS = ["tray", "receiving_zone", "pinky_tray_place"]


class ProfileTuner:
    def __init__(self, location: str, profiles_path: str):
        self.location = location
        self.profiles_path = Path(profiles_path)
        self.profiles = self._load_profiles()
        self.cap = RemoteCapture(port=5000)

        self._build_windows()
        self._init_trackbars()

        print(f"\n[ProfileTuner] Location: {self.location}")
        print(f"[ProfileTuner] 슬라이더로 값 조정 후:")
        print(f"  'p' - 저장")
        print(f"  '1'~'5' - location 전환")
        print(f"  'q' - 종료")
        print()

    def _load_profiles(self) -> dict:
        """pick_place_profiles.yaml 로드."""
        if not self.profiles_path.exists():
            raise FileNotFoundError(f"Config not found: {self.profiles_path}")
        with open(self.profiles_path) as f:
            return yaml.safe_load(f)

    def _build_windows(self) -> None:
        """슬라이더 창 생성."""
        for win in [WINDOW_SRC, WINDOW_MASK, WINDOW_CTRL]:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_CTRL, 400, 600)

    def _init_trackbars(self) -> None:
        """profile 초기값으로 슬라이더 설정."""
        profile = self.profiles.get(self.location, {})

        # HSV 슬라이더
        h_lower = profile.get("hsv_lower", [0, 0, 208])[0]
        s_lower = profile.get("hsv_lower", [0, 0, 208])[1]
        v_lower = profile.get("hsv_lower", [0, 0, 208])[2]
        h_upper = profile.get("hsv_upper", [158, 30, 255])[0]
        s_upper = profile.get("hsv_upper", [158, 30, 255])[1]
        v_upper = profile.get("hsv_upper", [158, 30, 255])[2]

        cv2.createTrackbar("H Lower", WINDOW_CTRL, h_lower, 179, lambda v: None)
        cv2.createTrackbar("S Lower", WINDOW_CTRL, s_lower, 255, lambda v: None)
        cv2.createTrackbar("V Lower", WINDOW_CTRL, v_lower, 255, lambda v: None)
        cv2.createTrackbar("H Upper", WINDOW_CTRL, h_upper, 179, lambda v: None)
        cv2.createTrackbar("S Upper", WINDOW_CTRL, s_upper, 255, lambda v: None)
        cv2.createTrackbar("V Upper", WINDOW_CTRL, v_upper, 255, lambda v: None)

        # W/H 슬라이더
        min_w = profile.get("min_w", 100)
        max_w = profile.get("max_w", 300)
        min_h = profile.get("min_h", 100)
        max_h = profile.get("max_h", 300)

        cv2.createTrackbar("Min W", WINDOW_CTRL, min_w, 640, lambda v: None)
        cv2.createTrackbar("Max W", WINDOW_CTRL, max_w, 640, lambda v: None)
        cv2.createTrackbar("Min H", WINDOW_CTRL, min_h, 640, lambda v: None)
        cv2.createTrackbar("Max H", WINDOW_CTRL, max_h, 640, lambda v: None)

    def _clear_trackbars(self) -> None:
        """슬라이더 제거."""
        cv2.destroyWindow(WINDOW_CTRL)
        self._build_windows()

    def _read_trackbars(self) -> dict:
        """현재 슬라이더 값 읽기."""
        return {
            "hsv_lower": [
                cv2.getTrackbarPos("H Lower", WINDOW_CTRL),
                cv2.getTrackbarPos("S Lower", WINDOW_CTRL),
                cv2.getTrackbarPos("V Lower", WINDOW_CTRL),
            ],
            "hsv_upper": [
                cv2.getTrackbarPos("H Upper", WINDOW_CTRL),
                cv2.getTrackbarPos("S Upper", WINDOW_CTRL),
                cv2.getTrackbarPos("V Upper", WINDOW_CTRL),
            ],
            "min_w": cv2.getTrackbarPos("Min W", WINDOW_CTRL),
            "max_w": cv2.getTrackbarPos("Max W", WINDOW_CTRL),
            "min_h": cv2.getTrackbarPos("Min H", WINDOW_CTRL),
            "max_h": cv2.getTrackbarPos("Max H", WINDOW_CTRL),
        }

    def _detect_boxes(self, frame: np.ndarray, params: dict) -> dict:
        """cv_detect_server.py의 detect_box_cv 로직."""
        hsv_lower = np.array(params["hsv_lower"], dtype=np.uint8)
        hsv_upper = np.array(params["hsv_upper"], dtype=np.uint8)
        min_w, max_w = params["min_w"], params["max_w"]
        min_h, max_h = params["min_h"], params["max_h"]
        min_area = 500
        morph_k = 7

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_lower, hsv_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (morph_k, morph_k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            rect = cv2.minAreaRect(cnt)
            cx, cy = rect[0]
            w, h = rect[1]
            angle = rect[2]

            # W/H 필터 (회전 고려)
            size_ok = (min_w <= w <= max_w and min_h <= h <= max_h) or \
                      (min_w <= h <= max_w and min_h <= w <= max_h)
            if not size_ok:
                continue

            # W > H로 정규화
            if w < h:
                w, h = h, w
                angle += 90.0
            while angle >= 90.0:
                angle -= 180.0
            while angle < -90.0:
                angle += 180.0

            detections.append({
                "cx": cx,
                "cy": cy,
                "w": w,
                "h": h,
                "angle": angle,
                "area": area,
            })

        return mask, detections

    def _draw_frame(self, frame: np.ndarray, detections: list, params: dict) -> np.ndarray:
        """프레임에 OBB 오버레이."""
        display = frame.copy()
        for det in detections:
            cx, cy = int(det["cx"]), int(det["cy"])
            w, h = det["w"], det["h"]
            angle = det["angle"]

            # 좌표계: OpenCV (x는 오른쪽, y는 아래)
            cos_a = math.cos(math.radians(angle))
            sin_a = math.sin(math.radians(angle))

            half_w = w / 2.0
            half_h = h / 2.0

            corners = [
                (cx + half_w * cos_a - half_h * sin_a, cy + half_w * sin_a + half_h * cos_a),
                (cx - half_w * cos_a - half_h * sin_a, cy - half_w * sin_a + half_h * cos_a),
                (cx - half_w * cos_a + half_h * sin_a, cy - half_w * sin_a - half_h * cos_a),
                (cx + half_w * cos_a + half_h * sin_a, cy + half_w * sin_a - half_h * cos_a),
            ]

            pts = np.array([(int(x), int(y)) for x, y in corners], np.int32)
            cv2.polylines(display, [pts], True, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 5, (0, 255, 255), -1)
            cv2.putText(display, f"θ={angle:.1f}°", (cx + 8, cy),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # 파라미터 텍스트
        info_text = [
            f"Location: {self.location}",
            f"Detections: {len(detections)}",
            f"HSV: ({params['hsv_lower'][0]}-{params['hsv_upper'][0]}, "
            f"{params['hsv_lower'][1]}-{params['hsv_upper'][1]}, "
            f"{params['hsv_lower'][2]}-{params['hsv_upper'][2]})",
            f"W: {params['min_w']}-{params['max_w']}, "
            f"H: {params['min_h']}-{params['max_h']}",
        ]
        y_offset = 30
        for text in info_text:
            cv2.putText(display, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 25

        return display

    def _change_location(self, new_loc: str) -> None:
        """location 전환."""
        if new_loc not in LOCATIONS:
            return

        if new_loc not in VISION_LOCATIONS:
            print(f"[WARN] {new_loc}은 비전 미사용 (고정 coords)")
            return

        if new_loc == self.location:
            return

        self.location = new_loc
        self.profiles = self._load_profiles()
        self._clear_trackbars()
        self._init_trackbars()
        print(f"[Location] {self.location}로 전환")

    def _save(self, params: dict) -> None:
        """pick_place_profiles.yaml에 저장."""
        profile = self.profiles.get(self.location, {})

        # HSV/W/H만 업데이트, 나머지는 유지
        profile["hsv_lower"] = params["hsv_lower"]
        profile["hsv_upper"] = params["hsv_upper"]
        profile["min_w"] = params["min_w"]
        profile["max_w"] = params["max_w"]
        profile["min_h"] = params["min_h"]
        profile["max_h"] = params["max_h"]

        self.profiles[self.location] = profile

        with open(self.profiles_path, 'w') as f:
            yaml.dump(self.profiles, f, default_flow_style=False, sort_keys=False)

        print(f"\n[SAVED] {self.location}")
        print(f"  HSV: {params['hsv_lower']} ~ {params['hsv_upper']}")
        print(f"  W: {params['min_w']}~{params['max_w']}, H: {params['min_h']}~{params['max_h']}")
        print()

    def run(self) -> None:
        """메인 루프."""
        while True:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            params = self._read_trackbars()
            mask, detections = self._detect_boxes(frame, params)
            display_frame = self._draw_frame(frame, detections, params)

            cv2.imshow(WINDOW_SRC, display_frame)
            cv2.imshow(WINDOW_MASK, mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[EXIT] Quit")
                break
            elif key == ord('p'):
                self._save(params)
            elif key == ord('1'):
                self._change_location("tray")
            elif key == ord('2'):
                self._change_location("receiving_zone")
            elif key == ord('3'):
                self._change_location("pinky_tray_place")
            elif key == ord('4'):
                self._change_location("warehouse_pick")
            elif key == ord('5'):
                self._change_location("warehouse_place")

        self.cap.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Location별 HSV/W,H 파라미터 튜닝")
    parser.add_argument("--location", default="tray", choices=LOCATIONS,
                       help="시작 location")
    parser.add_argument("--list", action="store_true", help="Location 목록 출력")
    parser.add_argument("--config",
                       default="/home/addinedu/roscamp-repo-1/src/jetcobot_vision/config/pick_place_profiles.yaml",
                       help="pick_place_profiles.yaml 경로")

    args = parser.parse_args()

    if args.list:
        print("Available locations:")
        for i, loc in enumerate(LOCATIONS, 1):
            vision = " (vision)" if loc in VISION_LOCATIONS else " (fixed_coords)"
            print(f"  {i}. {loc}{vision}")
        return

    print(f"\n{'='*60}")
    print(f"Profile Tuner — Location: {args.location}")
    print(f"{'='*60}\n")

    tuner = ProfileTuner(args.location, args.config)
    tuner.run()


if __name__ == "__main__":
    main()
