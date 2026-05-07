import cv2
import socket
import numpy as np
import threading

class RemoteCapture:
    """
    네트워크(UDP)를 통해 전송되는 JPEG 프레임을 수신하여 
    cv2.VideoCapture와 유사한 인터페이스를 제공하는 클래스
    """
    def __init__(self, port=5000):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(1.0)
        
        self.frame = None
        self.is_running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def _receive_loop(self):
        print(f"[RemoteCapture] 수신 루프 시작 (Port: {self.port})")
        packet_count = 0
        last_log_time = 0

        while self.is_running:
            try:
                # 65535는 UDP 최대 크기
                data, addr = self.sock.recvfrom(65535)
                packet_count += 1
                
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    self.frame = frame
                
                # 5초마다 수신 현황 출력 (로깅 비활성화)
                # import time
                # now = time.time()
                # if now - last_log_time > 5.0:
                #     print(f"[RemoteCap] 수신 중... (누적 {packet_count} 패킷, 최근 발신지: {addr})")
                #     last_log_time = now

            except socket.timeout:
                # timeout 발생 시 아무 로그도 안 찍으면 알 수 없으므로 출력 추가 고려 가능
                # 여기서는 루프를 유지함
                continue
            except Exception as e:
                if self.is_running:
                    print(f"[RemoteCap] Error: {e}")
                break

    def read(self):
        if self.frame is None:
            return False, None
        return True, self.frame.copy()

    def isOpened(self):
        return True

    def release(self):
        self.is_running = False
        self.sock.close()
        self.thread.join()
