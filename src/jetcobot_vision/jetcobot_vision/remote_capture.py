#!/usr/bin/env python3
"""
RemoteCapture
=============
jetcam.py 송신 프로토콜과 쌍을 이루는 UDP 프레임 수신기.

헤더 형식 (!HIHH):
  robot_id     H  2 bytes
  frame_id     I  4 bytes
  total_chunks H  2 bytes
  chunk_index  H  2 bytes
"""

import socket
import struct
import threading
import time
from typing import Optional

import cv2
import numpy as np

_HEADER_FMT  = "!HIHH"
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)


class RemoteCapture:
    """
    스레드 안전한 UDP 멀티청크 프레임 수신기.

    Parameters
    ----------
    host : str
        수신 바인드 주소
    port : int
        수신 포트
    robot_id : int
        수신 대상 robot_id 필터 (다른 ID는 무시)
    frame_timeout : float
        불완전 프레임 폐기 시간 (초)
    """

    def __init__(
        self,
        host: str,
        port: int,
        robot_id: int,
        frame_timeout: float = 2.0,
    ) -> None:
        self._host          = host
        self._port          = port
        self._robot_id      = robot_id
        self._frame_timeout = frame_timeout

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.settimeout(1.0)

        self._latest_frame: Optional[np.ndarray] = None
        self._lock    = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ── 공개 API ──────────────────────────────────────────────────────────────

    def start(self) -> None:
        """백그라운드 수신 스레드 시작."""
        if self._running:
            return
        self._running = True
        self._thread  = threading.Thread(
            target=self._recv_loop, name="RemoteCapture", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """수신 스레드 정지 및 소켓 해제."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        try:
            self._sock.close()
        except OSError:
            pass

    def read(self) -> tuple[bool, Optional[np.ndarray]]:
        """최신 프레임을 (success, frame) 튜플로 반환. 복사본을 돌려줍니다."""
        with self._lock:
            if self._latest_frame is None:
                return False, None
            return True, self._latest_frame.copy()

    # ── 내부 수신 루프 ────────────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        pending: dict = {}  # frame_id → {"total", "chunks", "ts"}

        while self._running:
            try:
                packet, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                self._expire(pending)
                continue
            except OSError:
                break

            if len(packet) < _HEADER_SIZE:
                continue

            robot_id, frame_id, total_chunks, chunk_idx = struct.unpack(
                _HEADER_FMT, packet[:_HEADER_SIZE]
            )

            if robot_id != self._robot_id:
                continue

            chunk = packet[_HEADER_SIZE:]

            if frame_id not in pending:
                pending[frame_id] = {
                    "total":  total_chunks,
                    "chunks": {},
                    "ts":     time.monotonic(),
                }

            pending[frame_id]["chunks"][chunk_idx] = chunk

            if len(pending[frame_id]["chunks"]) == pending[frame_id]["total"]:
                self._assemble(pending.pop(frame_id))

            self._expire(pending)

    def _assemble(self, entry: dict) -> None:
        """청크를 순서대로 이어 붙여 JPEG → BGR ndarray 로 디코딩."""
        data = b"".join(
            entry["chunks"][i] for i in range(entry["total"])
        )
        img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is not None:
            with self._lock:
                self._latest_frame = img

    def _expire(self, pending: dict) -> None:
        """frame_timeout 을 초과한 불완전 프레임을 폐기."""
        now     = time.monotonic()
        expired = [k for k, v in pending.items()
                   if now - v["ts"] > self._frame_timeout]
        for k in expired:
            del pending[k]
