#!/usr/bin/env python3
"""
화면 클릭 기반 좌표 변환 테스트 스크립트
YOLO 서버 없이, 화면의 상자를 마우스로 클릭하면 해당 픽셀의 물리적 로봇 Base 좌표를 반환합니다.
"""

import cv2
import argparse
import sys
import os

# src 폴더를 찾을 수 있도록 프로젝트 루트 경로를 sys.path에 추가
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))

from src.devices.jetcobot.vision.remote_capture import RemoteCapture
from src.devices.jetcobot.vision.handeye_calibration import RemoteRobot
from src.devices.jetcobot.vision.coord_transform import get_object_coords_in_base, DEFAULT_CFG

# 전역 변수
mc = None
config_dir = DEFAULT_CFG

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"\n[CLICK] 🖱️ 클릭한 픽셀: cx={x}, cy={y}")
        if mc is not None:
            try:
                # 상자가 수평하게 놓여있다고 가정 (theta=0)
                P_base, yaw_deg = get_object_coords_in_base(mc, float(x), float(y), 0.0, "tray", config_dir)
                print(f"  👉 로봇이 이동할 3D 좌표: X={P_base[0]:.1f}, Y={P_base[1]:.1f}, Z={P_base[2]:.1f} (mm)")
            except Exception as e:
                print(f"  ❌ 좌표 변환 실패: {e}")

def main():
    global mc, config_dir
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-host", required=True, help="제어 PC IP")
    args = parser.parse_args()

    print(f"[INFO] 원격 로봇 서버에 연결 중: {args.robot_host}:5001")
    mc = RemoteRobot(args.robot_host, 5001)

    print("[INFO] 영상 스트림 수신 대기 중 (포트 5000)...")
    cap = RemoteCapture(port=5000)

    cv2.namedWindow("Camera Stream")
    cv2.setMouseCallback("Camera Stream", mouse_callback)

    print("\n" + "="*60)
    print(" 🎯 [마우스 클릭 테스트]")
    print(" 카메라 화면이 뜨면, 물건의 중심을 마우스로 클릭해 보세요!")
    print(" YOLO 없이도 클릭한 지점의 실제 로봇 3D 좌표를 계산해 줍니다.")
    print(" (종료하려면 화면 클릭 후 'q' 키를 누르세요)")
    print("="*60 + "\n")

    while True:
        ret, frame = cap.read()
        if ret and frame is not None:
            # 정중앙 가이드선 그리기
            h, w = frame.shape[:2]
            cv2.line(frame, (w//2, 0), (w//2, h), (0, 255, 0), 1)
            cv2.line(frame, (0, h//2), (w, h//2), (0, 255, 0), 1)
            
            cv2.imshow("Camera Stream", frame)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
