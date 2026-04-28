#!/usr/bin/env python3
"""
핀홀 역산 + Hand-Eye 변환 파이프라인  (TASK-V04)
================================================
YOLO OBB 결과 (cx, cy, theta) → robot base 3D 좌표 변환

주요 API:
    get_object_coords_in_base(mc, cx, cy, theta, location, config_dir)
    → (P_base [x,y,z] mm, yaw_deg)

단독 검증 테스트:
    python coord_transform.py --port /dev/ttyJETCOBOT
"""

import argparse
import os
import time

import cv2
import numpy as np
import yaml
from pymycobot import MyCobot280


# ── 기본 경로 ─────────────────────────────────────────────────────────────────
_HERE       = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(_HERE, "..", "config", "front_jet")
# ─────────────────────────────────────────────────────────────────────────────


# ══════════════════════════════════════════════════════════════════════════════
# YAML 로더
# ══════════════════════════════════════════════════════════════════════════════

def load_camera_info(config_dir: str) -> tuple[np.ndarray, np.ndarray]:
    """
    camera_info.yaml → (K, D)
    K : camera matrix (3×3, float64)
    D : distortion coefficients (1×N, float64)
    """
    path = os.path.join(config_dir, "camera_info.yaml")
    with open(path) as f:
        data = yaml.safe_load(f)
    K = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(data["dist_coeffs"]["data"],   dtype=np.float64).reshape(1, -1)
    return K, D


def load_handeye(config_dir: str) -> np.ndarray:
    """
    handeye_result.yaml → T_ee2cam (4×4, float64, 단위: m)
    """
    path = os.path.join(config_dir, "handeye_result.yaml")
    with open(path) as f:
        data = yaml.safe_load(f)
    T = np.array(data["T_ee2cam"]["data"], dtype=np.float64).reshape(4, 4)
    return T


def load_workspace_config(config_dir: str) -> dict:
    """
    workspace_config.yaml → dict
    {
      locations: { tray: {z_fixed: mm}, shelf_A1: {z_fixed: mm}, ... },
      grasp_rp: { roll: deg, pitch: deg },
      observe_pose: { tray: [x,y,z,rx,ry,rz], ... },
    }
    """
    path = os.path.join(config_dir, "workspace_config.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


# ══════════════════════════════════════════════════════════════════════════════
# 변환 헬퍼
# ══════════════════════════════════════════════════════════════════════════════

def euler_zyx_to_rotation(rz_deg: float, ry_deg: float, rx_deg: float) -> np.ndarray:
    """
    MyCobot280 get_coords() rx/ry/rz (deg) → 3×3 회전행렬
    적용 순서: Rz * Ry * Rx  (ZYX 외인 회전)
    """
    rx, ry, rz = map(np.radians, [rx_deg, ry_deg, rz_deg])

    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0,           0,          1]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                   [0,           1, 0         ],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rx = np.array([[1, 0,           0          ],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]])
    return Rz @ Ry @ Rx


def coords_to_homogeneous(coords: list) -> np.ndarray:
    """
    mc.get_coords() → [x, y, z, rx, ry, rz] (mm, deg)
    → T_base2ee (4×4, 단위: m)

    handeye_calibration.py 의 coords_to_matrix() 와 동일 구현.
    """
    x, y, z, rx, ry, rz = coords
    R = euler_zyx_to_rotation(rz, ry, rx)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3,  3] = np.array([x, y, z]) / 1000.0   # mm → m
    return T


def theta_cam_to_robot(theta_cam: float, T_base2cam: np.ndarray) -> float:
    """
    카메라 픽셀 좌표계의 OBB 장축 각도(rad) → 로봇 base yaw (deg).

    카메라 이미지 평면의 2D 방향벡터를 T_base2cam 의 R 성분으로 회전시켜
    base 좌표계의 XY 평면 상 yaw 를 구한다.

    Parameters
    ----------
    theta_cam    : float  — OBB 장축 각도 (rad, 카메라 픽셀 좌표 기준)
    T_base2cam   : np.ndarray (4×4)

    Returns
    -------
    yaw_deg : float  — base 좌표계 yaw (deg)
    """
    # 카메라 이미지 평면의 방향벡터 (z=0)
    d_cam = np.array([np.cos(theta_cam), np.sin(theta_cam), 0.0])

    # base 좌표계로 회전 (평행이동 무시)
    R_base2cam = T_base2cam[:3, :3]
    d_base = R_base2cam @ d_cam

    # XY 평면 투영 → atan2
    yaw_rad = np.arctan2(d_base[1], d_base[0])
    return float(np.degrees(yaw_rad))


# ══════════════════════════════════════════════════════════════════════════════
# 메인 변환 API
# ══════════════════════════════════════════════════════════════════════════════

def get_object_coords_in_base(
    mc: MyCobot280,
    cx: float,
    cy: float,
    theta: float,
    location: str = "tray",
    config_dir: str = DEFAULT_CFG,
) -> tuple[np.ndarray, float]:
    """
    YOLO OBB 결과 → robot base 3D 좌표 변환.

    Parameters
    ----------
    mc         : MyCobot280 인스턴스
    cx, cy     : 픽셀 좌표 (float)
    theta      : OBB 장축 각도 (rad, 카메라 픽셀 좌표 기준)
    location   : 'tray' | 'shelf_A1' | 'shelf_A2' (workspace_config.yaml 키)
    config_dir : YAML 파일들이 있는 디렉토리

    Returns
    -------
    P_base  : np.ndarray (3,)  — [x, y, z] (mm, robot base 기준)
    yaw_deg : float            — 파지 yaw 각도 (deg, robot base 기준)
    """
    cfg      = load_workspace_config(config_dir)
    K, D     = load_camera_info(config_dir)
    T_ee2cam = load_handeye(config_dir)

    # 1. Z 고정값 로드 (mm)
    z_fixed_mm = cfg["locations"][location]["z_fixed"]
    if z_fixed_mm is None:
        raise ValueError(
            f"workspace_config.yaml: locations.{location}.z_fixed 가 null 입니다. "
            "실측 후 값을 채우세요."
        )

    # 핀홀 역산은 m 단위로 수행 (T_ee2cam 이 m 단위이므로)
    Z_m = z_fixed_mm / 1000.0

    # 2. 핀홀 역산 — 카메라 좌표계 (m)
    fx, fy   = K[0, 0], K[1, 1]
    ppx, ppy = K[0, 2], K[1, 2]

    X_cam = (cx - ppx) * Z_m / fx
    Y_cam = (cy - ppy) * Z_m / fy
    P_cam = np.array([X_cam, Y_cam, Z_m, 1.0])

    # 3. 현재 EE pose → T_base2cam
    coords = mc.get_coords()
    if not coords or len(coords) != 6:
        raise RuntimeError("mc.get_coords() 실패 — 로봇 연결 상태를 확인하세요.")

    T_base2ee  = coords_to_homogeneous(coords)
    T_base2cam = T_base2ee @ T_ee2cam

    # 4. base 좌표 (m → mm)
    P_base_m = T_base2cam @ P_cam
    P_base   = P_base_m[:3] * 1000.0   # m → mm

    # 5. yaw 변환
    yaw_deg = theta_cam_to_robot(theta, T_base2cam)

    return P_base, yaw_deg


# ══════════════════════════════════════════════════════════════════════════════
# 단독 검증 테스트
# ══════════════════════════════════════════════════════════════════════════════

def _run_verify(robot_port: str, config_dir: str, n_trials: int = 5) -> None:
    """
    알려진 위치의 상자를 n_trials 회 측정해 평균 오차를 출력한다.
    실행 전: workspace_config.yaml 의 z_fixed 를 실측 입력 완료 상태여야 함.

    사용법:
        로봇을 observe_pose 로 이동한 뒤 스페이스바 → cx/cy 를 직접 입력
    """
    print(f"[VERIFY] {n_trials}회 좌표 측정 테스트")
    print("  z_fixed 와 T_ee2cam 이 올바르게 설정됐는지 확인합니다.")

    mc = MyCobot280(robot_port, 1_000_000)
    time.sleep(0.5)

    cfg = load_workspace_config(config_dir)

    results: list[np.ndarray] = []
    for i in range(n_trials):
        cx_str = input(f"\n  [{i+1}/{n_trials}] cx 입력 (픽셀): ")
        cy_str = input(f"  [{i+1}/{n_trials}] cy 입력 (픽셀): ")
        th_str = input(f"  [{i+1}/{n_trials}] theta 입력 (rad): ")

        try:
            cx    = float(cx_str)
            cy    = float(cy_str)
            theta = float(th_str)
        except ValueError:
            print("  [WARN] 숫자 입력 오류, 스킵")
            continue

        P_base, yaw = get_object_coords_in_base(mc, cx, cy, theta, "tray", config_dir)
        print(f"  → base = {P_base.round(2)} mm,  yaw = {yaw:.2f} deg")
        results.append(P_base)

    if len(results) >= 2:
        arr  = np.array(results)
        mean = arr.mean(axis=0)
        errs = np.linalg.norm(arr - mean, axis=1)
        rms  = float(np.sqrt(np.mean(errs**2)))
        print(f"\n[RESULT] {len(results)}회 측정  RMS 오차 = {rms:.2f} mm  (목표: < 10 mm)")
        if rms < 10.0:
            print("[OK] 검증 통과 ✅")
        else:
            print("[WARN] 오차 과다 — handeye_result 또는 z_fixed 재확인 필요")


def main():
    parser = argparse.ArgumentParser(
        description="좌표 변환 파이프라인 검증 (TASK-V04)"
    )
    parser.add_argument("--port",       default="/dev/ttyJETCOBOT")
    parser.add_argument("--config-dir", default=DEFAULT_CFG)
    parser.add_argument("--trials",     type=int, default=5)
    args = parser.parse_args()

    _run_verify(args.port, args.config_dir, args.trials)


if __name__ == "__main__":
    main()
