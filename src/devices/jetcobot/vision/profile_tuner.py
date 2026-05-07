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

WINDOW_SRC  = "Profile Tuner - Source (P:save, Q:quit, 1-5:location, H:help)"
WINDOW_MASK = "Profile Tuner - Mask"

LOCATIONS = ["tray", "receiving_zone", "pinky_tray_place", "warehouse_pick", "warehouse_place"]
VISION_LOCATIONS = ["tray", "receiving_zone", "pinky_tray_place"]


class ProfileTuner:
    def __init__(self, location: str, profiles_path: str, fx: float = 990.81, fy: float = 987.62, z_surface_mm: float = 95.0):
        self.location = location
        self.profiles_path = Path(profiles_path)
        self.profiles = self._load_profiles()
        self.cap = RemoteCapture(port=5000)

        # 카메라 캘리브레이션 (camera_info.yaml 2026-05 기준)
        self.fx = fx
        self.fy = fy
        self.z_surface_mm = z_surface_mm

        # 초기 파라미터 로드
        self.params = self._load_params_from_profile(self.location)

        # 창 생성
        cv2.namedWindow(WINDOW_SRC, cv2.WINDOW_NORMAL)
        cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)

        self._print_help()

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

    def _print_help(self) -> None:
        """헬프 메시지 출력."""
        print(f"\n{'='*70}")
        print(f"Profile Tuner — {self.location}")
        print(f"Camera: fx={self.fx:.2f}, fy={self.fy:.2f}, z={self.z_surface_mm:.1f}mm")
        print(f"{'='*70}")
        print("\n【 파라미터 조정 】")
        print("  h-lower <값>     - HSV H Lower (0-179)")
        print("  s-lower <값>     - HSV S Lower (0-255)")
        print("  v-lower <값>     - HSV V Lower (0-255)")
        print("  h-upper <값>     - HSV H Upper (0-179)")
        print("  s-upper <값>     - HSV S Upper (0-255)")
        print("  v-upper <값>     - HSV V Upper (0-255)")
        print("  min-w <값>       - 최소 너비 (0-640px)")
        print("  max-w <값>       - 최대 너비 (0-640px)")
        print("  min-h <값>       - 최소 높이 (0-640px)")
        print("  max-h <값>       - 최대 높이 (0-640px)")
        print("\n【 단축키 】")
        print("  p                - pick_place_profiles.yaml에 저장")
        print("  1-5              - location 전환")
        print("  h                - 이 헬프 메시지")
        print("  q                - 종료")
        print(f"\n현재값: {self.params}\n")

    def _print_params(self) -> None:
        """현재 파라미터 출력."""
        h_low, s_low, v_low = self.params["hsv_lower"]
        h_up, s_up, v_up = self.params["hsv_upper"]
        print(f"\n[Current Parameters]")
        print(f"  HSV Lower: [{h_low:3d}, {s_low:3d}, {v_low:3d}]")
        print(f"  HSV Upper: [{h_up:3d}, {s_up:3d}, {v_up:3d}]")
        print(f"  W Range:   {self.params['min_w']:3d} ~ {self.params['max_w']:3d} px")
        print(f"  H Range:   {self.params['min_h']:3d} ~ {self.params['max_h']:3d} px")
        print()

    def _update_param(self, key: str, value) -> bool:
        """파라미터 업데이트. 성공 시 True."""
        try:
            if key == "h-lower":
                self.params["hsv_lower"][0] = max(0, min(179, int(value)))
            elif key == "s-lower":
                self.params["hsv_lower"][1] = max(0, min(255, int(value)))
            elif key == "v-lower":
                self.params["hsv_lower"][2] = max(0, min(255, int(value)))
            elif key == "h-upper":
                self.params["hsv_upper"][0] = max(0, min(179, int(value)))
            elif key == "s-upper":
                self.params["hsv_upper"][1] = max(0, min(255, int(value)))
            elif key == "v-upper":
                self.params["hsv_upper"][2] = max(0, min(255, int(value)))
            elif key == "min-w":
                self.params["min_w"] = max(0, min(640, int(value)))
            elif key == "max-w":
                self.params["max_w"] = max(0, min(640, int(value)))
            elif key == "min-h":
                self.params["min_h"] = max(0, min(640, int(value)))
            elif key == "max-h":
                self.params["max_h"] = max(0, min(640, int(value)))
            else:
                return False
            self._print_params()
            return True
        except (ValueError, IndexError):
            print("[ERR] 숫자를 입력해주세요")
            return False

    def _detect_boxes(self, frame: np.ndarray) -> dict:
        """cv_detect_server.py의 detect_box_cv 로직."""
        hsv_lower = np.array(self.params["hsv_lower"], dtype=np.uint8)
        hsv_upper = np.array(self.params["hsv_upper"], dtype=np.uint8)
        min_w, max_w = self.params["min_w"], self.params["max_w"]
        min_h, max_h = self.params["min_h"], self.params["max_h"]
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

    def _draw_frame(self, frame: np.ndarray, detections: list) -> np.ndarray:
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
            f"Detections: {len(detections)} | HSV: {self.params['hsv_lower']}-{self.params['hsv_upper']}",
            f"W: {self.params['min_w']}-{self.params['max_w']}px | H: {self.params['min_h']}-{self.params['max_h']}px",
        ]
        y_offset = 25
        for text in header_text:
            cv2.putText(display, text, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            y_offset += 20

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
        self.params = self._load_params_from_profile(self.location)
        print(f"[Location] {self.location}로 전환")
        self._print_params()

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

        print(f"\n[SAVED] {self.location}")
        print(f"  HSV: {self.params['hsv_lower']} ~ {self.params['hsv_upper']}")
        print(f"  W: {self.params['min_w']}~{self.params['max_w']}, H: {self.params['min_h']}~{self.params['max_h']}")
        print()

    def run(self) -> None:
        """메인 루프."""
        print("\n[Ready] 영상을 기다리는 중... (stream_sender.py가 보내는 UDP 확인)")

        while True:
            # 비디오 프레임 처리
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            # 박스 검출 및 화면 그리기
            mask, detections = self._detect_boxes(frame)
            display_frame = self._draw_frame(frame, detections)

            cv2.imshow(WINDOW_SRC, display_frame)
            cv2.imshow(WINDOW_MASK, mask)

            # 키 입력 처리 (cv2.waitKey 블로킹이므로 비동기 입력 별도 처리)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("[EXIT] Quit")
                break
            elif key == ord('p'):
                self._save()
            elif key == ord('h'):
                self._print_help()
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

    def interactive_loop(self) -> None:
        """터미널 입력 처리 루프 (별도 스레드에서 실행)."""
        while True:
            try:
                cmd = input("> ").strip().lower()
                if not cmd:
                    continue

                parts = cmd.split()
                if len(parts) == 1:
                    if parts[0] == 'q':
                        break
                    elif parts[0] == 'h':
                        self._print_help()
                    elif parts[0] in ['1', '2', '3', '4', '5']:
                        locs = ["tray", "receiving_zone", "pinky_tray_place", "warehouse_pick", "warehouse_place"]
                        self._change_location(locs[int(parts[0])-1])
                    elif parts[0] == 'p':
                        self._save()
                    elif parts[0] == 'status':
                        self._print_params()
                    else:
                        print("[ERR] 알 수 없는 명령어. 'h'로 도움말 보기")
                elif len(parts) == 2:
                    key, val = parts[0], parts[1]
                    if self._update_param(key, val):
                        print(f"✓ {key} = {val}")
                else:
                    print("[ERR] 형식: <파라미터> <값> 또는 명령어")
            except EOFError:
                break
            except Exception as e:
                print(f"[ERR] {e}")


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

    print(f"\n{'='*60}")
    print(f"Profile Tuner — Location: {args.location}")
    print(f"Camera: fx={args.fx:.2f}, fy={args.fy:.2f}")
    print(f"Z-Surface: {args.z_surface:.1f}mm")
    print(f"{'='*60}\n")

    tuner = ProfileTuner(args.location, args.config, fx=args.fx, fy=args.fy, z_surface_mm=args.z_surface)

    # 비디오 표시 + 터미널 입력을 동시에 처리
    import threading
    input_thread = threading.Thread(target=tuner.interactive_loop, daemon=True)
    input_thread.start()

    try:
        tuner.run()
    except KeyboardInterrupt:
        print("\n[EXIT] Interrupted")
    finally:
        tuner.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
