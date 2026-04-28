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
        while self.is_running:
            try:
                data, _ = self.sock.recvfrom(65535)
                nparr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if frame is not None:
                    self.frame = frame
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[RemoteCap] Error: {e}")

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
