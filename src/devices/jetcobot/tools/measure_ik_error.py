#!/usr/bin/env python3
"""
IK 오차 측정 스크립트 (오차 B: 입력 좌표 vs 실제 좌표)
======================================================
고정 좌표로 send_coords 후 get_coords를 읽어 입력/실제 차이를 표로 출력.

사용법:
    python3 src/devices/jetcobot/tools/measure_ik_error.py
    python3 src/devices/jetcobot/tools/measure_ik_error.py --port /dev/ttyJETCOBOT --speed 20
"""

import argparse
import time

try:
    from pymycobot import MyCobot280
except ImportError:
    print("[ERROR] pymycobot 미설치")
    raise

# ── 측정할 좌표 목록 [x, y, z, rx, ry, rz] (mm, deg) ──────────────────────
# 실제 작업 영역 내 대표 좌표로 수정해서 사용
TEST_COORDS = [
    [-62.4, -87.9, 314.2, -161.81, 0.94, -179.37],  # observe_pose (tray)
    [-62.4, -87.9, 200.0, -161.81, 0.94, -179.37],  # z 낮춤
    [-62.4, -87.9, 150.0, -161.81, 0.94, -179.37],  # z 더 낮춤
    [  0.0,   0.0, 300.0, -161.81, 0.94, -179.37],  # 중심 위치
]

SETTLE_SEC  = 1.5   # 이동 완료 후 안정화 대기 (초)
MAX_WAIT    = 15.0  # is_moving() 폴링 최대 시간


def wait_move(mc: MyCobot280) -> None:
    time.sleep(SETTLE_SEC)
    deadline = time.monotonic() + MAX_WAIT
    while time.monotonic() < deadline:
        try:
            if not mc.is_moving():
                break
        except Exception:
            break
        time.sleep(0.1)


def measure(mc: MyCobot280, speed: int) -> list[dict]:
    results = []
    for i, cmd in enumerate(TEST_COORDS):
        print(f"\n[{i+1}/{len(TEST_COORDS)}] 이동 중: {cmd}")
        mc.send_coords(cmd, speed, 0)
        wait_move(mc)

        actual = mc.get_coords()
        if not actual or len(actual) != 6:
            print("  [WARN] get_coords() 실패 — 스킵")
            continue

        dx = actual[0] - cmd[0]
        dy = actual[1] - cmd[1]
        dz = actual[2] - cmd[2]
        import math
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        results.append({
            "idx":    i + 1,
            "cmd":    cmd,
            "actual": actual,
            "dx": dx, "dy": dy, "dz": dz,
            "dist": dist,
        })
        print(f"  입력:  x={cmd[0]:.1f}  y={cmd[1]:.1f}  z={cmd[2]:.1f}")
        print(f"  실제:  x={actual[0]:.1f}  y={actual[1]:.1f}  z={actual[2]:.1f}")
        print(f"  오차:  dx={dx:+.1f}  dy={dy:+.1f}  dz={dz:+.1f}  dist={dist:.2f} mm")

    return results


def print_table(results: list[dict]) -> None:
    print("\n" + "=" * 80)
    print(f"{'#':>2}  {'입력 X':>8} {'입력 Y':>8} {'입력 Z':>8}  "
          f"{'실제 X':>8} {'실제 Y':>8} {'실제 Z':>8}  "
          f"{'dX':>7} {'dY':>7} {'dZ':>7}  {'dist':>7}")
    print("-" * 80)
    for r in results:
        c, a = r["cmd"], r["actual"]
        print(f"{r['idx']:>2}  {c[0]:>8.1f} {c[1]:>8.1f} {c[2]:>8.1f}  "
              f"{a[0]:>8.1f} {a[1]:>8.1f} {a[2]:>8.1f}  "
              f"{r['dx']:>+7.1f} {r['dy']:>+7.1f} {r['dz']:>+7.1f}  {r['dist']:>7.2f}")
    print("=" * 80)

    if results:
        import statistics
        dists = [r["dist"] for r in results]
        dxs   = [r["dx"]   for r in results]
        dys   = [r["dy"]   for r in results]
        dzs   = [r["dz"]   for r in results]
        print(f"\n평균 오차:  dx={statistics.mean(dxs):+.2f}  dy={statistics.mean(dys):+.2f}  "
              f"dz={statistics.mean(dzs):+.2f}  dist={statistics.mean(dists):.2f} mm")
        print(f"최대 오차:  dist={max(dists):.2f} mm")
        print(f"오차 편향:  dx 평균={statistics.mean(dxs):+.2f} mm  "
              f"→ {'일정한 편향 있음' if abs(statistics.mean(dxs)) > 2 else '편향 없음'}")


def main() -> None:
    parser = argparse.ArgumentParser(description="IK 오차 측정")
    parser.add_argument("--port",  default="/dev/ttyJETCOBOT")
    parser.add_argument("--baud",  type=int, default=1_000_000)
    parser.add_argument("--speed", type=int, default=20,
                        help="이동 속도 (기본: 20)")
    args = parser.parse_args()

    print(f"[INFO] 연결 중: {args.port} @ {args.baud}")
    mc = MyCobot280(args.port, args.baud)
    time.sleep(0.5)
    print("[INFO] 연결 완료. 측정 시작...\n")

    results = measure(mc, args.speed)
    print_table(results)


if __name__ == "__main__":
    main()
