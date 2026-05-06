#!/usr/bin/env python3
"""
IK 오차 측정 스크립트 (오차 B: 입력 좌표 vs 실제 좌표)
======================================================
tcp_offset을 적용한 실제 픽업 좌표 기준으로 send_coords 후
get_coords를 읽어 오차를 표로 출력. tcp_offset 보정값 검증에 사용.

사용법:
    python3 src/devices/jetcobot/tools/measure_ik_error.py
    python3 src/devices/jetcobot/tools/measure_ik_error.py --tcp-offset 0 20 110
    python3 src/devices/jetcobot/tools/measure_ik_error.py --speed 20 --settle 2.0
"""

import argparse
import math
import statistics
import time

try:
    from pymycobot import MyCobot280
except ImportError:
    print("[ERROR] pymycobot 미설치")
    raise

# ── 측정할 픽업 목표 좌표 [x, y, z, rx, ry, rz] (mm, deg) ─────────────────
# vision에서 받아온 pick_point 기준 좌표로 수정해서 사용
# tcp_offset은 아래 argparse로 별도 입력 (기본: [0, 20, 110])
TARGET_COORDS = [
    [-62.4, -87.9, 314.2, -161.81, 0.94, -179.37],  # observe_pose (tray)
    [-42, -200.5, 230.3, -177.46, 0.27, -179.82],  # sshopy_1_pre_z
    [-42, -200.5, 200.3, -177.46, 0.27, -179.82],  # sshopy_1
    [-42, -240.5, 230.3, -177.46, 0.27, -179.82],  # sshopy_2_pre_z
    [-42, -240.5, 200.3, -177.46, 0.27, -179.82],  # sshopy_2
]

SETTLE_SEC = 1.5
MAX_WAIT   = 15.0


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


def apply_tcp(coord: list, tcp: list) -> list:
    """픽업 목표 좌표에 tcp_offset 적용."""
    return [
        coord[0]
        coord[1]
        coord[2]
        coord[3], coord[4], coord[5],
    ]


def measure(mc: MyCobot280, speed: int, tcp: list) -> list[dict]:
    results = []
    for i, target in enumerate(TARGET_COORDS):
        cmd = apply_tcp(target, tcp)
        print(f"\n[{i+1}/{len(TARGET_COORDS)}] 목표: {target[:3]}  →  입력(+tcp): {cmd[:3]}")
        mc.send_coords(cmd, speed, 0)
        wait_move(mc)

        actual = mc.get_coords()
        if not actual or len(actual) != 6:
            print("  [WARN] get_coords() 실패 — 스킵")
            continue

        # 실제 좌표에서 tcp_offset을 빼면 → 도달한 target 좌표
        actual_target = [actual[j] - tcp[j] if j < 3 else actual[j] for j in range(6)]

        dx = actual_target[0] - target[0]
        dy = actual_target[1] - target[1]
        dz = actual_target[2] - target[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        results.append({
            "idx": i + 1,
            "target": target,
            "cmd": cmd,
            "actual": actual,
            "actual_target": actual_target,
            "dx": dx, "dy": dy, "dz": dz,
            "dist": dist,
        })
        print(f"  목표:  x={target[0]:.1f}  y={target[1]:.1f}  z={target[2]:.1f}")
        print(f"  실제:  x={actual_target[0]:.1f}  y={actual_target[1]:.1f}  z={actual_target[2]:.1f}")
        print(f"  오차:  dx={dx:+.1f}  dy={dy:+.1f}  dz={dz:+.1f}  dist={dist:.2f} mm")

    return results


def print_table(results: list[dict], tcp: list) -> None:
    print(f"\n[tcp_offset 적용값: x={tcp[0]} y={tcp[1]} z={tcp[2]}]")
    print("\n" + "=" * 88)
    print(f"{'#':>2}  {'목표 X':>8} {'목표 Y':>8} {'목표 Z':>8}  "
          f"{'실제 X':>8} {'실제 Y':>8} {'실제 Z':>8}  "
          f"{'dX':>7} {'dY':>7} {'dZ':>7}  {'dist':>7}")
    print("-" * 88)
    for r in results:
        t, a = r["target"], r["actual_target"]
        print(f"{r['idx']:>2}  {t[0]:>8.1f} {t[1]:>8.1f} {t[2]:>8.1f}  "
              f"{a[0]:>8.1f} {a[1]:>8.1f} {a[2]:>8.1f}  "
              f"{r['dx']:>+7.1f} {r['dy']:>+7.1f} {r['dz']:>+7.1f}  {r['dist']:>7.2f}")
    print("=" * 88)

    if results:
        dists = [r["dist"] for r in results]
        dxs   = [r["dx"]   for r in results]
        dys   = [r["dy"]   for r in results]
        dzs   = [r["dz"]   for r in results]
        print(f"\n평균 오차:  dx={statistics.mean(dxs):+.2f}  dy={statistics.mean(dys):+.2f}  "
              f"dz={statistics.mean(dzs):+.2f}  dist={statistics.mean(dists):.2f} mm")
        print(f"최대 오차:  dist={max(dists):.2f} mm")

        bias_x = abs(statistics.mean(dxs)) > 2
        bias_y = abs(statistics.mean(dys)) > 2
        bias_z = abs(statistics.mean(dzs)) > 2
        if bias_x or bias_y or bias_z:
            print("\n[보정 제안]")
            if bias_x:
                print(f"  tcp_offset[0] += {-statistics.mean(dxs):+.1f}  "
                      f"(현재 {tcp[0]} → 권장 {tcp[0] - statistics.mean(dxs):.1f})")
            if bias_y:
                print(f"  tcp_offset[1] += {-statistics.mean(dys):+.1f}  "
                      f"(현재 {tcp[1]} → 권장 {tcp[1] - statistics.mean(dys):.1f})")
            if bias_z:
                print(f"  tcp_offset[2] += {-statistics.mean(dzs):+.1f}  "
                      f"(현재 {tcp[2]} → 권장 {tcp[2] - statistics.mean(dzs):.1f})")
        else:
            print("\n[결과] tcp_offset 보정 양호 — 추가 보정 불필요")


def main() -> None:
    parser = argparse.ArgumentParser(description="IK 오차 측정 (tcp_offset 검증)")
    parser.add_argument("--port",       default="/dev/ttyJETCOBOT")
    parser.add_argument("--baud",       type=int, default=1_000_000)
    parser.add_argument("--speed",      type=int, default=20)
    parser.add_argument("--settle",     type=float, default=1.5,
                        help="이동 후 안정화 대기 시간 (초, 기본: 1.5)")
    parser.add_argument("--tcp-offset", type=float, nargs=3, default=[0.0, 20.0, 110.0],
                        metavar=("OX", "OY", "OZ"),
                        help="tcp_offset [x y z] (기본: 0 20 110)")
    args = parser.parse_args()

    global SETTLE_SEC
    SETTLE_SEC = args.settle
    tcp = args.tcp_offset

    print(f"[INFO] 연결 중: {args.port} @ {args.baud}")
    mc = MyCobot280(args.port, args.baud)
    time.sleep(0.5)
    print(f"[INFO] 연결 완료  tcp_offset={tcp}  speed={args.speed}\n")

    results = measure(mc, args.speed, tcp)
    print_table(results, tcp)


if __name__ == "__main__":
    main()
