#!/usr/bin/env python3
"""
카메라 Intrinsic 캘리브레이션 도구  (TASK-V01)
================================================
사용법:
    python camera_calibration.py [--device /dev/jetcocam0] [--out config/front_jet/camera_info.yaml]

조작키:
    스페이스바 : 현재 프레임을 캡처 (체커보드 검출 성공 시만 저장)
    'c'        : 캘리브레이션 실행 (MIN_IMAGES 장 이상 필요)
    'r'        : 수집 이미지 초기화
    'q'        : 종료

완료 조건: RMS reprojection error < 0.5 px
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import yaml
from .remote_capture import RemoteCapture


# ── 기본 설정 ─────────────────────────────────────────────────────────────────
CHECKERBOARD   = (9, 6)        # 내부 코너 수 (반드시 홀수×짝수)
SQUARE_SIZE_MM = 25.0          # ← 인쇄된 체커보드 정사각형 한 변 실측값 (mm) 수정 필요
CAMERA_DEVICE  = "/dev/jetcocam0"
MIN_IMAGES     = 20
RMS_THRESHOLD  = 0.5           # px
DEFAULT_OUT    = os.path.join(
    os.path.dirname(__file__),
    "..", "config", "front_jet", "camera_info.yaml"
)

# SubPix 정밀화 파라미터
SUBPIX_CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30, 0.001
)
# ─────────────────────────────────────────────────────────────────────────────


def _build_object_points() -> np.ndarray:
    """3D 체커보드 코너 좌표 (Z=0 평면, mm 단위)"""
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[
        0:CHECKERBOARD[0], 0:CHECKERBOARD[1]
    ].T.reshape(-1, 2) * SQUARE_SIZE_MM
    return objp


def detect_corners(frame_gray: np.ndarray):
    """
    체커보드 코너 검출 + SubPix 정밀화.

    Returns
    -------
    corners : np.ndarray or None
        검출 성공 시 정밀화된 코너 배열, 실패 시 None
    """
    ret, corners = cv2.findChessboardCorners(
        frame_gray,
        CHECKERBOARD,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
    )
    if not ret:
        return None

    corners_sub = cv2.cornerSubPix(
        frame_gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA
    )
    return corners_sub


def calibrate(
    obj_points_list: list,
    img_points_list: list,
    img_size: tuple,
) -> tuple[np.ndarray, np.ndarray, float]:
    """
    cv2.calibrateCamera 실행.

    Parameters
    ----------
    obj_points_list : list of np.ndarray  — 3D 물체 좌표 (N장 × M코너 × 3)
    img_points_list : list of np.ndarray  — 2D 이미지 좌표 (N장 × M코너 × 2)
    img_size        : (width, height)

    Returns
    -------
    camera_matrix : np.ndarray (3×3)
    dist_coeffs   : np.ndarray (1×5)
    rms           : float — reprojection error (px)
    """
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points_list,
        img_points_list,
        img_size,
        None, None,
    )
    return camera_matrix, dist_coeffs, rms


def save_camera_info(
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    img_size: tuple,
    rms: float,
    path: str,
) -> None:
    """
    camera_info.yaml 저장.

    형식
    ----
    image_size: [width, height]
    camera_matrix:
      rows: 3
      cols: 3
      data: [fx, 0, ppx, 0, fy, ppy, 0, 0, 1]
    dist_coeffs:
      data: [k1, k2, p1, p2, k3]
    rms_error: <float>
    """
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

    data = {
        "image_size": list(img_size),           # [width, height]
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": camera_matrix.flatten().tolist(),
        },
        "dist_coeffs": {
            "data": dist_coeffs.flatten().tolist(),
        },
        "rms_error": float(rms),
    }

    with open(path, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    print(f"[SAVE] camera_info.yaml → {os.path.abspath(path)}")


def run(device: str, output_path: str, use_remote: bool = False) -> None:
    """메인 인터랙티브 캘리브레이션 루프"""
    if use_remote:
        print(f"[INFO] 원격 카메라 수신 대기 중 (Port: 5000)...")
        cap = RemoteCapture(port=5000)
    else:
        cap = cv2.VideoCapture(device)

    if not cap.isOpened():
        print(f"[ERROR] 카메라 열기 실패: {device}")
        sys.exit(1)

    objp = _build_object_points()
    obj_points_list: list[np.ndarray] = []
    img_points_list: list[np.ndarray] = []
    img_size = None

    print("=" * 60)
    print("  카메라 Intrinsic 캘리브레이션")
    print(f"  체커보드: {CHECKERBOARD[0]}×{CHECKERBOARD[1]}, 정사각형: {SQUARE_SIZE_MM} mm")
    print(f"  카메라  : {device}")
    print(f"  목표    : {MIN_IMAGES}장 이상 캡처 후 'c' 입력")
    print("=" * 60)
    print("  [스페이스] 캡처  [c] 캘리브레이션  [r] 초기화  [q] 종료")

    camera_matrix = None
    dist_coeffs   = None
    rms           = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] 프레임 읽기 실패")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray.shape[1], gray.shape[0])

        # 실시간 코너 미리보기
        corners = detect_corners(gray)
        display = frame.copy()
        if corners is not None:
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners, True)
            cv2.putText(display, "DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            cv2.putText(display, "NO BOARD", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        # 캡처 수 / RMS 표시
        status = f"captured: {len(img_points_list)}"
        if rms is not None:
            status += f"  RMS: {rms:.4f} px"
        cv2.putText(display, status, (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

        cv2.imshow("Camera Calibration [TASK-V01]", display)
        key = cv2.waitKey(1) & 0xFF

        # ── 스페이스바: 캡처 ───────────────────────────────────────────────
        if key == ord(" "):
            if corners is None:
                print("[WARN] 체커보드 미검출 — 캡처 스킵")
            else:
                obj_points_list.append(objp)
                img_points_list.append(corners)
                print(f"[CAPTURE] {len(img_points_list)}장 수집 완료")

        # ── 'c': 캘리브레이션 실행 ────────────────────────────────────────
        elif key == ord("c"):
            n = len(img_points_list)
            if n < MIN_IMAGES:
                print(f"[WARN] {n}장 수집됨. 최소 {MIN_IMAGES}장 필요.")
            else:
                print(f"[INFO] {n}장으로 캘리브레이션 실행 중...")
                camera_matrix, dist_coeffs, rms = calibrate(
                    obj_points_list, img_points_list, img_size
                )
                print(f"[RESULT] RMS error = {rms:.4f} px")
                if rms < RMS_THRESHOLD:
                    print(f"[OK] RMS < {RMS_THRESHOLD} px — 품질 양호")
                    save_camera_info(
                        camera_matrix, dist_coeffs, img_size, rms, output_path
                    )
                    print("  fx  =", camera_matrix[0, 0])
                    print("  fy  =", camera_matrix[1, 1])
                    print("  ppx =", camera_matrix[0, 2])
                    print("  ppy =", camera_matrix[1, 2])
                    print("  dist=", dist_coeffs.flatten().tolist())
                else:
                    print(f"[WARN] RMS >= {RMS_THRESHOLD} px — 이미지를 더 수집하거나 품질을 개선하세요.")

        # ── 'r': 초기화 ───────────────────────────────────────────────────
        elif key == ord("r"):
            obj_points_list.clear()
            img_points_list.clear()
            rms = None
            print("[RESET] 수집 데이터 초기화")

        # ── 'q': 종료 ─────────────────────────────────────────────────────
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    if camera_matrix is None:
        print("[EXIT] 캘리브레이션 미완료 상태로 종료")
    else:
        print(f"[DONE] 결과 저장 위치: {os.path.abspath(output_path)}")


def main():
    parser = argparse.ArgumentParser(
        description="카메라 Intrinsic 캘리브레이션 (TASK-V01)"
    )
    parser.add_argument(
        "--device", default=CAMERA_DEVICE,
        help=f"카메라 디바이스 (기본: {CAMERA_DEVICE})"
    )
    parser.add_argument(
        "--out", default=DEFAULT_OUT,
        help=f"출력 YAML 경로 (기본: {DEFAULT_OUT})"
    )
    parser.add_argument(
        "--square-mm", type=float, default=SQUARE_SIZE_MM,
        help=f"체커보드 정사각형 한 변 크기 mm (기본: {SQUARE_SIZE_MM})"
    )
    parser.add_argument(
        "--remote", action="store_true",
        help="제어 PC로부터 네트워크 영상 수신"
    )
    args = parser.parse_args()

    global SQUARE_SIZE_MM
    SQUARE_SIZE_MM = args.square_mm

    run(args.device, args.out, args.remote)


if __name__ == "__main__":
    main()
