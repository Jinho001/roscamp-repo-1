#!/usr/bin/env python3
"""
비전 기반 픽앤플레이스 통합 모듈  (TASK-V05)
============================================
카메라 촬영 → YOLO OBB 검출 → 좌표 변환 → send_coords 픽업 1사이클

safe_pick 미사용 — mc.send_coords() 직접 호출 (pymycobot 내부 IK).

단독 테스트:
    python vision_pick.py --location tray --port /dev/ttyJETCOBOT
"""

import argparse
import os
import time

import cv2
import numpy as np
from pymycobot import MyCobot280

from .coord_transform    import get_object_coords_in_base, load_workspace_config
from .yolo_detect_client import detect_object
from .remote_capture     import RemoteCapture
from .handeye_calibration import RemoteRobot


# ── 기본 설정 ─────────────────────────────────────────────────────────────────
CAMERA_DEVICE   = "/dev/jetcocam0"
ROBOT_PORT      = "/dev/ttyJETCOBOT"
ROBOT_BAUD      = 1_000_000
MOVE_SPEED      = 30            # mm/s (관측 자세 / pre-pick 이동)
DESCENT_SPEED   = 20            # mm/s (픽업 하강)
PRE_PICK_OFFSET = 50.0          # mm  (픽업 위치 위 대기 고도)
SETTLE_TIME     = 1.0           # 초  (이동 완료 후 안정화 대기)
GRIP_CLOSE      = 0             # 그리퍼 닫힘 값 (0 = 완전 닫힘)
GRIP_OPEN       = 100           # 그리퍼 열림 값
GRIP_SPEED      = 50

_HERE       = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CFG = os.path.join(_HERE, "..", "config", "front_jet")
# ─────────────────────────────────────────────────────────────────────────────


def _wait_move(mc, timeout: float = 30.0) -> None:
    """is_moving() 폴링으로 이동 완료 대기. timeout 초 초과 시 강제 진행."""
    deadline = time.time() + timeout
    while True:
        try:
            if not mc.is_moving():
                break
        except Exception:
            break
        if time.time() > deadline:
            break
        time.sleep(0.1)


def capture_frame(device: str = CAMERA_DEVICE, use_remote: bool = False) -> np.ndarray | None:
    """
    카메라에서 단일 프레임을 캡처해 반환한다.
    실패 시 None 반환.
    """
    if use_remote:
        cap = RemoteCapture(port=5000)
        # 네트워크 지연을 고려하여 잠시 대기 후 첫 프레임 획득
        time.sleep(0.5)
    else:
        cap = cv2.VideoCapture(device)
    
    if not cap.isOpened():
        return None
    ret, frame = cap.read()
    cap.release()
    return frame if ret else None


def vision_pick(
    mc,
    location: str = "tray",
    config_dir: str = DEFAULT_CFG,
    camera_device: str = CAMERA_DEVICE,
    use_remote: bool = False,
) -> tuple[bool, str]:
    """
    비전 픽업 1사이클.

    Steps
    -----
    1. observe_pose 로 이동
    2. 프레임 캡처 → YOLO OBB 검출
    3. 좌표 변환 (get_object_coords_in_base)
    4. pre_pick 고도 이동 → 하강 → 그리퍼 닫기 → 상승

    Parameters
    ----------
    mc            : MyCobot280 인스턴스
    location      : 'tray' | 'shelf_A1' | 'shelf_A2'
    config_dir    : YAML 파일 디렉토리
    camera_device : 카메라 디바이스 경로

    Returns
    -------
    (success: bool, message: str)
    """
    cfg = load_workspace_config(config_dir)

    # ── 1. 관측 자세 이동 ─────────────────────────────────────────────────────
    obs_pose = cfg.get("observe_pose", {}).get(location)
    if obs_pose is None:
        return False, f"observe_pose.{location} 미설정 — workspace_config.yaml 확인"

    print(f"[VISION_PICK] {location} 관측 자세 이동...")
    mc.send_coords(obs_pose, MOVE_SPEED, 0)
    _wait_move(mc)
    time.sleep(SETTLE_TIME)

    # ── 2. 촬영 + 검출 ────────────────────────────────────────────────────────
    print("[VISION_PICK] 프레임 캡처...")
    frame = capture_frame(camera_device, use_remote)
    if frame is None:
        return False, f"카메라 캡처 실패: {camera_device}"

    try:
        result = detect_object(frame)
    except Exception as e:
        return False, f"YOLO 서버 오류: {e}"

    if not result["detected"]:
        return False, "상자 미검출"

    cx    = result["cx"]
    cy    = result["cy"]
    theta = result["theta"]
    print(f"[VISION_PICK] 검출: cx={cx:.1f} cy={cy:.1f} "
          f"theta={np.degrees(theta):.1f}deg conf={result['confidence']:.2f}")

    # ── 3. 좌표 변환 ──────────────────────────────────────────────────────────
    try:
        P_base, yaw_deg = get_object_coords_in_base(
            mc, cx, cy, theta, location, config_dir
        )
    except Exception as e:
        return False, f"좌표 변환 오류: {e}"

    x, y, z = P_base
    roll  = cfg["grasp_rp"]["roll"]
    pitch = cfg["grasp_rp"]["pitch"]
    yaw_offset = cfg["grasp_rp"].get("yaw_offset", 0.0)

    if roll is None or pitch is None:
        return False, "grasp_rp.roll/pitch 미설정 — workspace_config.yaml 확인"
    
    final_yaw = yaw_deg + yaw_offset

    print(f"[VISION_PICK] base 좌표: x={x:.1f} y={y:.1f} z={z:.1f} "
          f"roll={roll} pitch={pitch} yaw={yaw_deg:.1f} (offset 적용 후 RZ: {final_yaw:.1f}deg)")

    # ── 4. 픽업 모션 ──────────────────────────────────────────────────────────
    pre_z = z + PRE_PICK_OFFSET

    # 4-a. pre-pick 고도 이동
    print("[VISION_PICK] pre-pick 이동...")
    mc.send_coords([x, y, pre_z, roll, pitch, final_yaw], MOVE_SPEED, 0)
    _wait_move(mc)
    time.sleep(SETTLE_TIME)

    # 4-b. 픽업 위치로 하강
    print("[VISION_PICK] 픽업 위치 하강...")
    mc.send_coords([x, y, z, roll, pitch, final_yaw], DESCENT_SPEED, 0)
    _wait_move(mc)
    time.sleep(0.5)

    # 4-c. 그리퍼 닫기 (파지)
    print("[VISION_PICK] 그리퍼 닫기 (파지)...")
    mc.set_gripper_value(GRIP_CLOSE, GRIP_SPEED)
    time.sleep(1.0)

    # 4-d. pre-pick 고도로 복귀
    print("[VISION_PICK] 상승 복귀...")
    mc.send_coords([x, y, pre_z, roll, pitch, final_yaw], MOVE_SPEED, 0)
    _wait_move(mc)
    time.sleep(SETTLE_TIME)

    msg = f"픽업 성공: {location} ({x:.1f}, {y:.1f}, {z:.1f}) yaw={yaw_deg:.1f}deg"
    print(f"[VISION_PICK] {msg}")
    return True, msg


def vision_place(
    mc,
    target_coords: list,
) -> tuple[bool, str]:
    """
    선반 적재 (Place) 모션.

    Parameters
    ----------
    mc            : MyCobot280 인스턴스
    target_coords : [x, y, z, rx, ry, rz] (mm, deg) — 수동 티칭 좌표

    Returns
    -------
    (success: bool, message: str)
    """
    if len(target_coords) != 6:
        return False, f"target_coords 길이 오류: {len(target_coords)} (6 필요)"

    x, y, z   = target_coords[:3]
    rx, ry, rz = target_coords[3:]
    pre_z = z + PRE_PICK_OFFSET

    # pre-place 고도 이동
    print("[VISION_PLACE] pre-place 이동...")
    mc.send_coords([x, y, pre_z, rx, ry, rz], MOVE_SPEED, 0)
    _wait_move(mc)
    time.sleep(SETTLE_TIME)

    # 적재 위치로 하강
    print("[VISION_PLACE] 적재 위치 하강...")
    mc.send_coords([x, y, z, rx, ry, rz], DESCENT_SPEED, 0)
    _wait_move(mc)
    time.sleep(0.5)

    # 그리퍼 열기 (해제)
    print("[VISION_PLACE] 그리퍼 열기 (해제)...")
    mc.set_gripper_value(GRIP_OPEN, GRIP_SPEED)
    time.sleep(1.0)

    # pre-place 고도 복귀
    print("[VISION_PLACE] 상승 복귀...")
    mc.send_coords([x, y, pre_z, rx, ry, rz], MOVE_SPEED, 0)
    _wait_move(mc)
    time.sleep(SETTLE_TIME)

    msg = f"적재 성공: ({x:.1f}, {y:.1f}, {z:.1f})"
    print(f"[VISION_PLACE] {msg}")
    return True, msg


# ══════════════════════════════════════════════════════════════════════════════
# 단독 테스트
# ══════════════════════════════════════════════════════════════════════════════

def _run_test(robot_port: str, location: str, config_dir: str, n: int = 3, use_remote: bool = False, robot_host: str = "") -> None:
    """픽업 n 회 연속 테스트 (place 좌표는 workspace_config 에서 자동 로드)."""
    if robot_host:
        print(f"[INFO] 원격 로봇 서버에 연결 중: {robot_host}:5001")
        mc = RemoteRobot(robot_host, 5001)
    else:
        print(f"[INFO] 로봇 연결 중: {robot_port} @ {ROBOT_BAUD}")
        mc = MyCobot280(robot_port, ROBOT_BAUD)
    time.sleep(0.5)

    cfg = load_workspace_config(config_dir)

    success_count = 0
    for i in range(n):
        print(f"\n{'='*50}")
        print(f"  시도 {i+1}/{n}  (location={location})")
        print(f"{'='*50}")
        ok, msg = vision_pick(mc, location, config_dir, use_remote=use_remote)
        if ok:
            success_count += 1
            # 선반 place 좌표가 config 에 있다면 자동 실행
            place_coords = cfg.get("place_pose", {}).get(location)
            if place_coords:
                vision_place(mc, place_coords)
        else:
            print(f"[FAIL] {msg}")
        time.sleep(1.0)

    print(f"\n[TEST RESULT] {success_count}/{n} 성공")


def main():
    parser = argparse.ArgumentParser(
        description="비전 픽앤플레이스 통합 테스트 (TASK-V05)"
    )
    parser.add_argument("--port",       default=ROBOT_PORT)
    parser.add_argument("--location",   default="tray",
                        choices=["tray", "shelf_A1", "shelf_A2"])
    parser.add_argument("--config-dir", default=DEFAULT_CFG)
    parser.add_argument("--trials",     type=int, default=3)
    parser.add_argument("--remote",     action="store_true", help="원격 영상 수신")
    parser.add_argument("--robot-host", default="", help="로봇 제어 PC IP (원격 제어용)")
    args = parser.parse_args()

    _run_test(args.port, args.location, args.config_dir, args.trials, args.remote, args.robot_host)


if __name__ == "__main__":
    main()
