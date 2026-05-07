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
    - 슬라이더로 HSV/W/H 조정 (Controls 창)
    - '1'~'5': location 전환 (Source 창에서)
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

WINDOW_SRC  = "Profile Tuner - Source (P:save, Q:quit, 1-5:location)"
WINDOW_MASK = "Profile Tuner - Mask"
WINDOW_CTRL = "Profile Tuner - Controls (슬라이더 조정)"

LOCATIONS = ["tray", "receiving_zone", "pinky_tray_place", "warehouse_pick", "warehouse_place"]
VISION_LOCATIONS = ["tray", "receiving_zone", "pinky_tray_place"]

# 슬라이더 UI: (슬라이더 이름, 최솟값, 최댓값, 파라미터 키)
TRACKBARS = [
    ("H Lower", 0, 179, ("hsv_lower", 0)),
    ("S Lower", 0, 255, ("hsv_lower", 1)),
    ("V Lower", 0, 255, ("hsv_lower", 2)),
    ("H Upper", 0, 179, ("hsv_upper", 0)),
    ("S Upper", 0, 255, ("hsv_upper", 1)),
    ("V Upper", 0, 255, ("hsv_upper", 2)),
    ("Min W", 0, 640, ("min_w", None)),
    ("Max W", 0, 640, ("max_w", None)),
    ("Min H", 0, 640, ("min_h", None)),
    ("Max H", 0, 640, ("max_h", None)),
]


class ProfileTuner:
    def __init__(self, location: str, profiles_path: str, fx: float = 990.81, fy: float = 987.62, z_surface_mm: float = 95.0):
        self.location = location
        self.profiles_path = Path(profiles_path)
        self.profiles = self._load_profiles()

        # 카메라 캘리브레이션 (camera_info.yaml 2026-05 기준)
        self.fx = fx
        self.fy = fy
        self.z_surface_mm = z_surface_mm

        self.cap = RemoteCapture(port=5000)

        # 파라미터 로드
        self.params = self._load_params_from_profile(self.location)

        # 창 생성
        cv2.namedWindow(WINDOW_SRC, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WINDOW_CTRL, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_CTRL, 600, 800)

        print("\n[Ready] 영상을 기다리는 중... (stream_sender.py 실행 필요)")
        print(f"Location: {self.location} | Camera: fx={self.fx:.2f}, fy={self.fy:.2f}, z={self.z_surface_mm:.1f}mm\n")

    def _load_profiles(self) -> dict:
        """pick_place_profiles.yaml 로드."""
        if not self.profiles_path.exists():
            raise FileNotFoundError(f"Config not found: {self.profiles_path}")
        with open(self.profiles_path) as f:
            return yaml.safe_load(f)

    def _load_params_from_profile(self, location: str) -> dict:
        """location 프로파일에서 파라미터 로드."""
        profile = self.profiles.get(location, {})
        return {
            "hsv_lower": profile.get("hsv_lower", [0, 0, 208]),
            "hsv_upper": profile.get("hsv_upper", [158, 30, 255]),
            "min_w": profile.get("min_w", 100),
            "max_w": profile.get("max_w", 300),
            "min_h": profile.get("min_h", 100),
            "max_h": profile.get("max_h", 300),
        }


    def _detect_boxes(self, frame: np.ndarray, params: dict) -> tuple:
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

            # 픽셀 → mm 변환 (z_surface 기반)
            w_mm = (w / self.fx) * self.z_surface_mm
            h_mm = (h / self.fy) * self.z_surface_mm

            detections.append({
                "cx": cx,
                "cy": cy,
                "w": w,
                "h": h,
                "w_mm": w_mm,
                "h_mm": h_mm,
                "angle": angle,
                "area": area,
            })

        return mask, detections

    def _draw_frame(self, frame: np.ndarray, detections: list, params: dict) -> np.ndarray:
        """프레임에 OBB 오버레이."""
        display = frame.copy()
        for i, det in enumerate(detections):
            cx, cy = int(det["cx"]), int(det["cy"])
            w, h = det["w"], det["h"]
            w_mm = det["w_mm"]
            h_mm = det["h_mm"]
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

            # 픽셀과 mm 함께 표시
            label = f"#{i} θ={angle:.1f}°\n{w:.0f}px({w_mm:.1f}mm) × {h:.0f}px({h_mm:.1f}mm)"
            y_text = cy - 20
            for line in label.split('\n'):
                cv2.putText(display, line, (cx + 8, y_text),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                y_text += 15

        # 헤더: 설정값 및 예상값
        header_text = [
            f"Location: {self.location} | z={self.z_surface_mm:.1f}mm | Expected: 30×20mm (box)",
            f"Detections: {len(detections)} | HSV: {params['hsv_lower']}-{params['hsv_upper']}",
            f"W: {params['min_w']}-{params['max_w']}px | H: {params['min_h']}-{params['max_h']}px",
        ]
        y_offset = 25
        for text in header_text:
            cv2.putText(display, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            y_offset += 20

        return display

    def _draw_controls(self, detections: list, params: dict) -> np.ndarray:
        """Controls 창: 파라미터 값과 검출 결과 표시."""
        # 검은 배경 (800x600)
        img = np.zeros((800, 600, 3), dtype=np.uint8)
        img.fill(30)

        y = 20
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = 0.5
        color_label = (180, 180, 180)
        color_value = (0, 255, 255)
        color_header = (100, 200, 255)

        # 헤더
        cv2.putText(img, f"Location: {self.location} | z={self.z_surface_mm:.1f}mm", (10, y), font, 0.7, color_header, 2)
        y += 35
        cv2.putText(img, "Expected box size: 30×20mm", (10, y), font, 0.5, (150, 150, 200), 1)
        y += 20

        # HSV 값
        h_l, s_l, v_l = params["hsv_lower"]
        h_u, s_u, v_u = params["hsv_upper"]

        cv2.putText(img, "HSV Lower:", (10, y), font, font_size, color_label, 1)
        y += 20
        cv2.putText(img, f"  H: {h_l:3d} (0-179)", (20, y), font, font_size, color_value, 1)
        y += 18
        cv2.putText(img, f"  S: {s_l:3d} (0-255)", (20, y), font, font_size, color_value, 1)
        y += 18
        cv2.putText(img, f"  V: {v_l:3d} (0-255)", (20, y), font, font_size, color_value, 1)
        y += 25

        cv2.putText(img, "HSV Upper:", (10, y), font, font_size, color_label, 1)
        y += 20
        cv2.putText(img, f"  H: {h_u:3d} (0-179)", (20, y), font, font_size, color_value, 1)
        y += 18
        cv2.putText(img, f"  S: {s_u:3d} (0-255)", (20, y), font, font_size, color_value, 1)
        y += 18
        cv2.putText(img, f"  V: {v_u:3d} (0-255)", (20, y), font, font_size, color_value, 1)
        y += 25

        # W/H 값
        cv2.putText(img, "Size Filters:", (10, y), font, font_size, color_label, 1)
        y += 20
        cv2.putText(img, f"  W: {params['min_w']:3d} ~ {params['max_w']:3d} px", (20, y), font, font_size, color_value, 1)
        y += 18
        cv2.putText(img, f"  H: {params['min_h']:3d} ~ {params['max_h']:3d} px", (20, y), font, font_size, color_value, 1)
        y += 30

        # 검출 결과
        cv2.putText(img, f"DETECTIONS ({len(detections)}):", (10, y), font, 0.6, color_header, 1)
        y += 25

        if len(detections) == 0:
            cv2.putText(img, "No detections", (20, y), font, font_size, (100, 100, 100), 1)
        else:
            for i, det in enumerate(detections[:5]):  # 최대 5개만 표시
                text = f"#{i}: {det['w']:.0f}px({det['w_mm']:.1f}mm) × {det['h']:.0f}px({det['h_mm']:.1f}mm)"
                cv2.putText(img, text, (20, y), font, 0.45, (100, 255, 100), 1)
                y += 18
            if len(detections) > 5:
                cv2.putText(img, f"... and {len(detections)-5} more", (20, y), font, 0.4, (100, 150, 100), 1)

        y = 700
        # 조작 안내
        cv2.putText(img, "KEYBOARD SHORTCUTS:", (10, y), font, 0.55, (255, 180, 100), 1)
        y += 20
        for shortcut, desc in [("P", "Save"), ("1-5", "Location"), ("Q", "Quit")]:
            text = f"[{shortcut}] {desc}"
            cv2.putText(img, text, (20, y), font, 0.45, (180, 180, 180), 1)
            y += 16

        return img

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
        self.params = self._load_params_from_profile(self.location)

        print(f"[✓] Location: {self.location}로 전환")

    def _save(self) -> None:
        """pick_place_profiles.yaml에 저장."""
        profile = self.profiles.get(self.location, {})

        # HSV/W/H만 업데이트, 나머지는 유지
        profile["hsv_lower"] = self.params["hsv_lower"]
        profile["hsv_upper"] = self.params["hsv_upper"]
        profile["min_w"] = self.params["min_w"]
        profile["max_w"] = self.params["max_w"]
        profile["min_h"] = self.params["min_h"]
        profile["max_h"] = self.params["max_h"]

        self.profiles[self.location] = profile

        with open(self.profiles_path, 'w') as f:
            yaml.dump(self.profiles, f, default_flow_style=False, sort_keys=False)

        print(f"\n[✓ SAVED] {self.location}")
        print(f"  HSV: {self.params['hsv_lower']} ~ {self.params['hsv_upper']}")
        print(f"  W: {self.params['min_w']}~{self.params['max_w']}, H: {self.params['min_h']}~{self.params['max_h']}")
        print()

    def run(self) -> None:
        """메인 루프."""
        while True:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # 검출
            mask, detections = self._detect_boxes(frame, self.params)

            # 프레임 그리기
            display_frame = self._draw_frame(frame, detections, self.params)
            controls_img = self._draw_controls(detections, self.params)

            cv2.imshow(WINDOW_SRC, display_frame)
            cv2.imshow(WINDOW_MASK, mask)
            cv2.imshow(WINDOW_CTRL, controls_img)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[EXIT] Quit")
                break
            elif key == ord('p'):
                self._save()
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
            elif key in [ord(str(i)) for i in range(10)]:
                # 숫자 키: 파라미터 조정용
                # 예: 0 누르면 h-lower +1, 1 누르면 h-lower -1, etc.
                pass

        self.cap.release()
        cv2.destroyAllWindows()


def print_help():
    """헬프 메시지."""
    print("""
【 HSV 파라미터 】
  h-lower <값>     - HSV H Lower (0-179)
  s-lower <값>     - HSV S Lower (0-255)
  v-lower <값>     - HSV V Lower (0-255)
  h-upper <값>     - HSV H Upper (0-179)
  s-upper <값>     - HSV S Upper (0-255)
  v-upper <값>     - HSV V Upper (0-255)

【 크기 필터 】
  min-w <값>       - Min W (0-640px)
  max-w <값>       - Max W (0-640px)
  min-h <값>       - Min H (0-640px)
  max-h <값>       - Max H (0-640px)

【 신뢰성 튜닝 】
  z-surface <값>   - 작업면 높이 (mm, 소수점 지원)
                     검출 크기(mm)가 실제 크기와 일치할 때까지 조정
                     현재값 확인: Controls 창 상단 참고

【 단축키 (영상 창에서) 】
  P: 저장 | Q: 종료 | 1-5: location 전환

【 예시 】
  > v-lower 215
  > min-w 100
  > z-surface 95.5
""")


def main():
    parser = argparse.ArgumentParser(description="Location별 HSV/W,H 파라미터 튜닝")
    parser.add_argument("--location", default="tray", choices=LOCATIONS,
                       help="시작 location")
    parser.add_argument("--list", action="store_true", help="Location 목록 출력")
    parser.add_argument("--config",
                       default="/home/addinedu/roscamp-repo-1/src/jetcobot_vision/config/pick_place_profiles.yaml",
                       help="pick_place_profiles.yaml 경로")
    parser.add_argument("--fx", type=float, default=990.8120239698858,
                       help="카메라 fx (기본: camera_info.yaml 2026-05 값)")
    parser.add_argument("--fy", type=float, default=987.6160053978392,
                       help="카메라 fy")
    parser.add_argument("--z-surface", type=float, default=95.0,
                       help="작업면 높이 mm (profile의 z_surface_mm)")

    args = parser.parse_args()

    if args.list:
        print("Available locations:")
        for i, loc in enumerate(LOCATIONS, 1):
            vision = " (vision)" if loc in VISION_LOCATIONS else " (fixed_coords)"
            print(f"  {i}. {loc}{vision}")
        return

    print(f"\n{'='*70}")
    print(f"Profile Tuner — {args.location}")
    print(f"Camera: fx={args.fx:.2f}, fy={args.fy:.2f}, z={args.z_surface:.1f}mm")
    print(f"{'='*70}\n")

    tuner = ProfileTuner(args.location, args.config, fx=args.fx, fy=args.fy, z_surface_mm=args.z_surface)

    # 사용자 입력 루프 (별도 스레드)
    import threading
    def input_loop():
        print_help()
        while tuner.cap.is_running:
            try:
                cmd = input("> ").strip().lower()
                if not cmd:
                    continue

                parts = cmd.split()
                if len(parts) == 2:
                    key, val = parts[0], parts[1]
                    try:
                        val = int(val) if '.' not in val else float(val)
                        if key == "h-lower":
                            tuner.params["hsv_lower"][0] = max(0, min(179, int(val)))
                        elif key == "s-lower":
                            tuner.params["hsv_lower"][1] = max(0, min(255, int(val)))
                        elif key == "v-lower":
                            tuner.params["hsv_lower"][2] = max(0, min(255, int(val)))
                        elif key == "h-upper":
                            tuner.params["hsv_upper"][0] = max(0, min(179, int(val)))
                        elif key == "s-upper":
                            tuner.params["hsv_upper"][1] = max(0, min(255, int(val)))
                        elif key == "v-upper":
                            tuner.params["hsv_upper"][2] = max(0, min(255, int(val)))
                        elif key == "min-w":
                            tuner.params["min_w"] = max(0, min(640, int(val)))
                        elif key == "max-w":
                            tuner.params["max_w"] = max(0, min(640, int(val)))
                        elif key == "min-h":
                            tuner.params["min_h"] = max(0, min(640, int(val)))
                        elif key == "max-h":
                            tuner.params["max_h"] = max(0, min(640, int(val)))
                        elif key == "z-surface":
                            tuner.z_surface_mm = max(1.0, float(val))
                            print(f"✓ z_surface_mm = {tuner.z_surface_mm:.1f}mm")
                            continue
                        else:
                            print("[ERR] 알 수 없는 파라미터")
                            continue
                        print(f"✓ {key} = {val}")
                    except ValueError:
                        print("[ERR] 숫자를 입력해주세요")
            except EOFError:
                break
            except KeyboardInterrupt:
                break

    thread = threading.Thread(target=input_loop, daemon=True)
    thread.start()

    tuner.run()


if __name__ == "__main__":
    main()
