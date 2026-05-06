#!/usr/bin/env python3
"""
observe_pose 이동 스크립트
==========================
pick_place_profiles.yaml에서 observe_pose를 읽어 로봇을 이동시킨다.
좌표 변환 검증 및 z_surface 튜닝 시 사용.

사용법:
    python3 src/devices/jetcobot/tools/move_to_observe.py
    python3 src/devices/jetcobot/tools/move_to_observe.py --location receiving_zone
    python3 src/devices/jetcobot/tools/move_to_observe.py --location tray --speed 20
"""

import argparse
import os
import time

import yaml

try:
    from pymycobot import MyCobot280
except ImportError:
    print("[ERROR] pymycobot 미설치")
    raise

_HERE         = os.path.dirname(os.path.abspath(__file__))
_PROFILES_PATH = os.path.join(_HERE, "../../../jetcobot_vision/config/pick_place_profiles.yaml")


def load_observe_pose(location: str) -> list:
    path = os.path.normpath(_PROFILES_PATH)
    with open(path) as f:
        profiles = yaml.safe_load(f)
    if location not in profiles:
        raise ValueError(f"location '{location}' 없음. 가능한 목록: {list(profiles.keys())}")
    pose = profiles[location].get("observe_pose")
    if pose is None:
        raise ValueError(f"'{location}'에 observe_pose가 없습니다.")
    return pose


def wait_move(mc: MyCobot280, settle: float = 1.5) -> None:
    time.sleep(settle)
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        try:
            if not mc.is_moving():
                break
        except Exception:
            break
        time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser(description="observe_pose 이동")
    parser.add_argument("--location", default="tray",
                        help="프로파일 location 이름 (기본: tray)")
    parser.add_argument("--port",  default="/dev/ttyJETCOBOT")
    parser.add_argument("--baud",  type=int, default=1_000_000)
    parser.add_argument("--speed", type=int, default=20)
    args = parser.parse_args()

    pose = load_observe_pose(args.location)
    print(f"[INFO] location={args.location}  observe_pose={pose}")

    mc = MyCobot280(args.port, args.baud)
    time.sleep(0.5)
    print(f"[INFO] 연결 완료. 이동 중...")

    mc.send_coords(pose, args.speed, 0)
    wait_move(mc)

    actual = mc.get_coords()
    if actual and len(actual) == 6:
        print(f"[INFO] 이동 완료")
        print(f"  목표:  {pose[:3]}")
        print(f"  실제:  {[round(v, 1) for v in actual[:3]]}")
        dx = actual[0] - pose[0]
        dy = actual[1] - pose[1]
        dz = actual[2] - pose[2]
        print(f"  오차:  dx={dx:+.1f}  dy={dy:+.1f}  dz={dz:+.1f} mm")
    else:
        print("[WARN] get_coords() 실패")


if __name__ == "__main__":
    main()
