#!/usr/bin/env python3
"""
Hand-Eye 캘리브레이션 도구  (TASK-V02, Eye-in-Hand)
====================================================
수학적 배경:
    AX = XB  문제
    A : T_base2ee   — mc.get_coords() 로 수집
    B : T_cam2board — cv2.solvePnP 로 수집
    X : T_ee2cam    — 구하는 값 (카메라→EE 변환행렬)

사용법:
    python handeye_calibration.py [--device /dev/jetcocam0] [--port /dev/ttyJETCOBOT]

조작키:
    스페이스바 : 현재 자세에서 (A, B) 쌍 수집
    'c'        : 캘리브레이션 실행 (MIN_POSES 이상 필요)
    'v'        : 검증 모드 — 고정 마커 3자세에서 재현 오차 측정
    'r'        : 수집 데이터 초기화
    'q'        : 종료

완료 조건: 고정 마커 3자세 재현 오차 < 5 mm
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import yaml
import json
import socket
from pymycobot import MyCobot280
from .remote_capture import RemoteCapture

class RemoteRobot:
    def __init__(self, host: str, port: int = 5001):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(2.0)
        
    def _send_cmd(self, cmd, *args):
        payload = json.dumps({"cmd": cmd, "args": args})
        try:
            self.sock.sendto(payload.encode('utf-8'), (self.host, self.port))
            data, _ = self.sock.recvfrom(1024)
            return json.loads(data.decode('utf-8'))
        except Exception as e:
            print(f"[ERROR] 원격 로봇 응답 없음 ({cmd}): {e}")
            return None

    def get_coords(self) -> list:
        res = self._send_cmd('get_coords')
        return res if res is not None else []
        
    def send_coords(self, coords, speed, mode):
        return self._send_cmd('send_coords', coords, speed, mode)
        
    def is_moving(self):
        res = self._send_cmd('is_moving')
        return res if res is not None else 0
        
    def set_gripper_value(self, value, speed):
        return self._send_cmd('set_gripper_value', value, speed)



# ── 기본 설정 ─────────────────────────────────────────────────────────────────
CHECKERBOARD   = (9, 6)
SQUARE_SIZE_MM = 25.0          # camera_calibration.py와 동일 값 사용
CAMERA_DEVICE  = "/dev/jetcocam0"
ROBOT_PORT     = "/dev/ttyJETCOBOT"
ROBOT_BAUD     = 1_000_000
MIN_POSES      = 15
VIBRATION_WAIT = 0.5           # 이동 후 진동 안정화 대기 (초)

_CAMERA_INFO_PATH = os.path.join(
    os.path.dirname(__file__), "..", "config", "front_jet", "camera_info.yaml"
)
DEFAULT_OUT = os.path.join(
    os.path.dirname(__file__), "..", "config", "front_jet", "handeye_result.yaml"
)

SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30, 0.001,
)
# ─────────────────────────────────────────────────────────────────────────────


# ── YAML 로더 ─────────────────────────────────────────────────────────────────

def load_camera_info(path: str) -> tuple[np.ndarray, np.ndarray]:
    """
    camera_info.yaml → (K, D)
    K : camera matrix (3×3)
    D : distortion coefficients (1×5)
    """
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(data["dist_coeffs"]["data"],   dtype=np.float64).reshape(1, -1)
    return K, D


# ── 변환 헬퍼 ─────────────────────────────────────────────────────────────────

def euler_zyx_to_rotation(rz_deg: float, ry_deg: float, rx_deg: float) -> np.ndarray:
    """
    MyCobot280 get_coords() 반환 rx/ry/rz → 회전행렬 (ZYX Euler, 내인 회전)
    get_coords: [x, y, z, rx, ry, rz]  (mm, deg)
    적용 순서: Rz * Ry * Rx  (ZYX 외인 회전 = XYZ 내인 회전)
    """
    rx = np.radians(rx_deg)
    ry = np.radians(ry_deg)
    rz = np.radians(rz_deg)

    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz),  np.cos(rz), 0],
        [0,           0,          1],
    ])
    Ry = np.array([
        [ np.cos(ry), 0, np.sin(ry)],
        [0,           1, 0         ],
        [-np.sin(ry), 0, np.cos(ry)],
    ])
    Rx = np.array([
        [1, 0,          0         ],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx),  np.cos(rx)],
    ])
    return Rz @ Ry @ Rx


def coords_to_matrix(coords: list) -> np.ndarray:
    """
    mc.get_coords() → [x, y, z, rx, ry, rz] (mm, deg)
    → 4×4 동차변환행렬 T_base2ee (단위: m)
    """
    x, y, z, rx, ry, rz = coords
    R = euler_zyx_to_rotation(rz, ry, rx)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = np.array([x, y, z]) / 1000.0   # mm → m
    return T


def make_4x4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """R (3×3), t (3×1 or 3,) → 4×4 동차행렬"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = t.flatten()
    return T


# ── 체커보드 검출 ─────────────────────────────────────────────────────────────

def _build_board_points(square_mm: float) -> np.ndarray:
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[
        0:CHECKERBOARD[0], 0:CHECKERBOARD[1]
    ].T.reshape(-1, 2) * square_mm
    return objp


def detect_board_pose(
    frame_gray: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    square_mm: float,
) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
    """
    체커보드 검출 + solvePnP → (rvec, tvec)
    실패 시 (None, None) 반환
    """
    objp = _build_board_points(square_mm)
    ret, corners = cv2.findChessboardCorners(
        frame_gray, CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
    )
    if not ret:
        return None, None

    corners = cv2.cornerSubPix(
        frame_gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA
    )
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
    if not ok:
        return None, None
    return rvec, tvec


# ── 캘리브레이션 ──────────────────────────────────────────────────────────────

def run_handeye(
    R_base2ee_list: list[np.ndarray],
    t_base2ee_list: list[np.ndarray],
    R_cam2board_list: list[np.ndarray],
    t_cam2board_list: list[np.ndarray],
) -> np.ndarray:
    """
    cv2.calibrateHandEye(TSAI) → T_ee2cam (4×4, 단위: m)
    """
    R, t = cv2.calibrateHandEye(
        R_base2ee_list,
        t_base2ee_list,
        R_cam2board_list,
        t_cam2board_list,
        method=cv2.CALIB_HAND_EYE_TSAI,
    )
    return make_4x4(R, t)


def save_result(T_ee2cam: np.ndarray, rms_verify_mm: float | None, path: str) -> None:
    """
    handeye_result.yaml 저장

    형식
    ----
    T_ee2cam:
      rows: 4
      cols: 4
      data: [16개 원소, row-major]
    verify_rms_mm: <float or null>
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    data = {
        "T_ee2cam": {
            "rows": 4,
            "cols": 4,
            "data": T_ee2cam.flatten().tolist(),
        },
        "verify_rms_mm": rms_verify_mm,
    }
    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    print(f"[SAVE] handeye_result.yaml → {os.path.abspath(path)}")


# ── 검증 ──────────────────────────────────────────────────────────────────────

def verify_reprojection(
    mc,
    cap: cv2.VideoCapture,
    T_ee2cam: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    square_mm: float,
    n_poses: int = 3,
) -> float:
    """
    n_poses 개 자세에서 체커보드 기준점의 base 좌표를 계산하고
    첫 번째 자세 기준 재현 오차 (mm) 를 측정.

    Returns
    -------
    rms_mm : float  — 재현 오차 RMS (mm)
    """
    print(f"\n[VERIFY] {n_poses}개 자세에서 체커보드 코너 base 좌표 측정")
    print("  → 각 자세로 로봇을 이동한 뒤 스페이스바를 누르세요.")

    base_coords_list: list[np.ndarray] = []

    collected = 0
    while collected < n_poses:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rvec, tvec = detect_board_pose(gray, K, D, square_mm)

        display = frame.copy()
        status_color = (0, 255, 0) if rvec is not None else (0, 0, 255)
        status_text  = f"VERIFY {collected+1}/{n_poses} | " + (
            "BOARD OK" if rvec is not None else "NO BOARD"
        )
        cv2.putText(display, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.imshow("Camera Calibration [TASK-V01]", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            if rvec is None:
                print("[WARN] 체커보드 미검출")
                continue
            time.sleep(VIBRATION_WAIT)

            # 현재 EE pose
            coords = mc.get_coords()
            if not coords or len(coords) != 6:
                print("[WARN] get_coords() 실패, 스킵")
                continue

            T_base2ee = coords_to_matrix(coords)
            T_base2cam = T_base2ee @ T_ee2cam

            # 체커보드 원점을 base 좌표로 변환
            R_cam2board, _ = cv2.Rodrigues(rvec)
            tvec_m = tvec.flatten() / 1000.0   # mm → m
            P_board_in_cam = np.array([*tvec_m, 1.0])
            P_board_in_base = T_base2cam @ P_board_in_cam

            base_coords_list.append(P_board_in_base[:3] * 1000.0)  # m → mm
            print(f"  [{collected+1}] base = {base_coords_list[-1].round(2)} mm")
            collected += 1
        elif key == ord("q"):
            break

    if len(base_coords_list) < 2:
        print("[WARN] 검증 데이터 부족")
        return float("nan")

    arr = np.array(base_coords_list)
    mean = arr.mean(axis=0)
    errors = np.linalg.norm(arr - mean, axis=1)
    rms_mm = float(np.sqrt(np.mean(errors**2)))
    print(f"\n[VERIFY RESULT] 재현 오차 RMS = {rms_mm:.2f} mm  (목표: < 5 mm)")
    if rms_mm < 5.0:
        print("[OK] 검증 통과 ✅")
    else:
        print("[WARN] 오차 과다 — 더 많은 자세 수집 또는 재캘리브레이션 권장")
    return rms_mm


# ── 메인 루프 ─────────────────────────────────────────────────────────────────

def run(device: str, robot_port: str, output_path: str, use_remote: bool = False, square_mm: float = 25.0, robot_host: str = "") -> None:
    # 카메라 intrinsic 로드 (V01 산출물)
    cam_info_path = os.path.abspath(_CAMERA_INFO_PATH)
    if not os.path.exists(cam_info_path):
        print(f"[ERROR] camera_info.yaml 미존재: {cam_info_path}")
        print("  → 먼저 TASK-V01 (camera_calibration.py) 을 실행하세요.")
        sys.exit(1)
    K, D = load_camera_info(cam_info_path)
    print(f"[INFO] camera_info 로드: {cam_info_path}")

    # 카메라 열기
    if use_remote:
        print(f"[INFO] 원격 카메라 수신 대기 중 (Port: 5000)...")
        cap = RemoteCapture(port=5000)
    else:
        cap = cv2.VideoCapture(device)

    if not cap.isOpened():
        print(f"[ERROR] 카메라 열기 실패: {device}")
        sys.exit(1)

    # 로봇 연결
    if robot_host:
        print(f"[INFO] 원격 로봇 서버에 연결 중: {robot_host}:5001")
        mc = RemoteRobot(robot_host, 5001)
        time.sleep(0.5)
        print("[INFO] 원격 로봇 준비 완료")
    else:
        print(f"[INFO] 로봇 연결 중: {robot_port} @ {ROBOT_BAUD}")
        mc = MyCobot280(robot_port, ROBOT_BAUD)
        time.sleep(0.5)
        print("[INFO] 로봇 연결 완료")

    # 수집 버퍼
    R_base2ee_list:   list[np.ndarray] = []
    t_base2ee_list:   list[np.ndarray] = []
    R_cam2board_list: list[np.ndarray] = []
    t_cam2board_list: list[np.ndarray] = []

    T_ee2cam: np.ndarray | None = None
    rms_verify: float | None    = None

    print("=" * 60)
    print("  Hand-Eye 캘리브레이션  (TASK-V02)")
    print(f"  체커보드: {CHECKERBOARD[0]}×{CHECKERBOARD[1]}, {square_mm} mm")
    print(f"  목표    : {MIN_POSES}자세 이상 수집 후 'c' 입력")
    print("=" * 60)
    print("  [스페이스] 자세 수집  [c] 캘리브레이션  [v] 검증  [r] 초기화  [q] 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rvec, tvec = detect_board_pose(gray, K, D, square_mm)

        display = frame.copy()
        if rvec is not None:
            cv2.drawFrameAxes(display, K, D, rvec, tvec, square_mm * 2)
            cv2.putText(display, "BOARD DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(display, "NO BOARD", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        info = f"poses: {len(R_base2ee_list)}"
        if T_ee2cam is not None:
            info += "  [calibrated]"
        cv2.putText(display, info, (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("Hand-Eye Calibration [TASK-V02]", display)

        key = cv2.waitKey(1) & 0xFF

        # ── 스페이스: 자세 수집 ───────────────────────────────────────────
        if key == ord(" "):
            if rvec is None:
                print("[WARN] 체커보드 미검출 — 스킵")
                continue

            time.sleep(VIBRATION_WAIT)
            coords = mc.get_coords()
            if not coords or len(coords) != 6:
                print("[WARN] get_coords() 실패 — 스킵")
                continue

            T_base2ee = coords_to_matrix(coords)
            R_b2ee = T_base2ee[:3, :3]
            t_b2ee = T_base2ee[:3,  3].reshape(3, 1)

            R_c2b, _ = cv2.Rodrigues(rvec)
            t_c2b     = tvec / 1000.0   # mm → m

            R_base2ee_list.append(R_b2ee)
            t_base2ee_list.append(t_b2ee)
            R_cam2board_list.append(R_c2b)
            t_cam2board_list.append(t_c2b)

            print(f"[CAPTURE] {len(R_base2ee_list)}자세 수집  "
                  f"coords={[round(v,1) for v in coords]}")

        # ── 'c': 캘리브레이션 실행 ────────────────────────────────────────
        elif key == ord("c"):
            n = len(R_base2ee_list)
            if n < MIN_POSES:
                print(f"[WARN] {n}자세. 최소 {MIN_POSES} 필요.")
            else:
                print(f"[INFO] {n}자세로 Hand-Eye 캘리브레이션 실행...")
                T_ee2cam = run_handeye(
                    R_base2ee_list, t_base2ee_list,
                    R_cam2board_list, t_cam2board_list,
                )
                print("[RESULT] T_ee2cam =")
                print(np.round(T_ee2cam, 4))
                save_result(T_ee2cam, rms_verify, output_path)

        # ── 'v': 검증 모드 ────────────────────────────────────────────────
            if T_ee2cam is None:
                print("[WARN] 캘리브레이션 먼저 실행하세요 ('c').")
            else:
                rms_verify = verify_reprojection(mc, cap, T_ee2cam, K, D, square_mm)
                save_result(T_ee2cam, rms_verify, output_path)

        # ── 'r': 초기화 ───────────────────────────────────────────────────
        elif key == ord("r"):
            R_base2ee_list.clear()
            t_base2ee_list.clear()
            R_cam2board_list.clear()
            t_cam2board_list.clear()
            T_ee2cam   = None
            rms_verify = None
            print("[RESET] 수집 데이터 초기화")

        # ── 'q': 종료 ─────────────────────────────────────────────────────
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="Hand-Eye 캘리브레이션 (TASK-V02)"
    )
    parser.add_argument("--device",    default=CAMERA_DEVICE)
    parser.add_argument("--port",      default=ROBOT_PORT)
    parser.add_argument("--out",       default=DEFAULT_OUT)
    parser.add_argument("--square-mm", type=float, default=SQUARE_SIZE_MM)
    parser.add_argument("--remote",    action="store_true", help="원격 영상 수신")
    parser.add_argument("--board",     default="9x6", help="체커보드 내부 코너 수 가로x세로")
    parser.add_argument("--robot-host", default="", help="로봇 제어 PC IP (원격 좌표 수신)")
    args = parser.parse_args()

    global CHECKERBOARD
    if args.board:
        parts = args.board.split("x")
        if len(parts) == 2:
            CHECKERBOARD = (int(parts[0]), int(parts[1]))

    run(args.device, args.port, args.out, args.remote, args.square_mm, args.robot_host)


if __name__ == "__main__":
    main()
