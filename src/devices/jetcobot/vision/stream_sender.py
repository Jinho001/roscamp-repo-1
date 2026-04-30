#!/usr/bin/env python3
"""
제어 PC용 영상 스트리머 (Sender)
===============================
젯코봇 제어 PC에서 실행하여 메인 PC로 영상을 전송합니다.
"""
import cv2
import socket
import struct
import time
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True, help="메인 PC의 IP 주소")
    parser.add_argument("--port", type=int, default=5000, help="전송 포트")
    parser.add_argument("--device", default="/dev/jetcocam0", help="카메라 장치")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[ERROR] 카메라를 열 수 없습니다: {args.device}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.port)

    print(f"[STREAM] 전송 시작: {args.host}:{args.port}")

    try:
        while True:
            if not cap.isOpened():
                print("[WARN] 카메라가 닫혀있습니다. 재연결 시도...")
                time.sleep(1.0)
                cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
                continue

            ret, frame = cap.read()
            if not ret:
                print("[WARN] 프레임 획득 실패. (케이블 확인 필요)")
                time.sleep(0.5)  # CPU 100% 방지
                # 3번 이상 실패 시 재연결을 위해 닫기
                cap.release()
                continue

            # JPEG 압축 (품질을 동적으로 낮춰 UDP 64KB 제한 초과 방지)
            quality = 50
            while True:
                _, img_encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
                data = img_encoded.tobytes()
                
                if len(data) <= 65000 or quality <= 10:
                    break
                quality -= 10  # 용량이 크면 화질을 더 낮춤

            if len(data) > 65000:
                print(f"[WARN] 프레임 크기 초과 ({len(data)} bytes) - 전송 포기")
                continue

            sock.sendto(data, dest)
            print(f"[DEBUG] 전송 성공 ({len(data)} bytes)")
            time.sleep(0.03) # 약 30 FPS
    except KeyboardInterrupt:
        print("[STOP] 스트리밍 중단")
    finally:
        cap.release()
        sock.close()

if __name__ == "__main__":
    main()
