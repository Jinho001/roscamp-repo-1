#!/usr/bin/env python3
"""
제어 PC용 로봇 좌표 서버 (UDP)
메인 PC에서 원격으로 로봇(젯코봇)의 좌표를 요청할 때 사용합니다.
"""
import socket
import json
import argparse
from pymycobot import MyCobot280

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyJETCOBOT", help="로봇 포트")
    parser.add_argument("--baud", type=int, default=1000000, help="보드레이트")
    parser.add_argument("--server-port", type=int, default=5001, help="UDP 대기 포트")
    args = parser.parse_args()

    print(f"[INFO] 로봇 연결 중: {args.port} @ {args.baud}")
    try:
        mc = MyCobot280(args.port, args.baud)
    except Exception as e:
        print(f"[ERROR] 로봇 연결 실패: {e}")
        return
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.server_port))
    print(f"[SERVER] UDP {args.server_port} 포트에서 메인 PC의 좌표 요청을 대기합니다.")
    
    try:
        while True:
            data, addr = sock.recvfrom(1024)
            raw_str = data.decode('utf-8').strip()
            
            # 구버전 단순 텍스트 요청 호환성
            if raw_str == 'get_coords':
                coords = mc.get_coords()
                if coords is None:
                    coords = []
                sock.sendto(json.dumps(coords).encode('utf-8'), addr)
                continue
                
            # JSON 형태의 원격 명령 처리
            try:
                msg = json.loads(raw_str)
                cmd = msg.get("cmd")
                args_list = msg.get("args", [])
                
                res = None
                if cmd == 'get_coords':
                    res = mc.get_coords()
                    if res is None: res = []
                elif cmd == 'send_coords':
                    res = mc.send_coords(*args_list)
                elif cmd == 'is_moving':
                    res = mc.is_moving()
                elif cmd == 'set_gripper_value':
                    res = mc.set_gripper_value(*args_list)
                    
                sock.sendto(json.dumps(res).encode('utf-8'), addr)
            except Exception as e:
                print(f"[WARN] 명령 처리 실패: {e}")
    except KeyboardInterrupt:
        print("\n[STOP] 서버 종료")
    finally:
        sock.close()

if __name__ == "__main__":
    main()
