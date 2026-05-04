#!/usr/bin/env python3
"""
OpenCV 고전 비전 기반 상자 검출 서버  (옵션 B - PHASE B-2)
=========================================================
obb_server.py 와 완전히 동일한 HTTP 인터페이스를 제공하므로
obb_detect_client.py 및 vision_pick.py 코드 수정 없이
--server 인자만 변경하면 바로 교체 사용 가능.

처리 파이프라인:
  GaussianBlur → HSV 마스킹 → 형태학적 연산
  → findContours → 면적 필터 → minAreaRect → JSON 반환

실행 예:
    # 먼저 hsv_tuner.py 로 HSV 범위를 파악한 뒤 아래 인자를 채운다
    python3 src/devices/jetcobot/vision/cv_detect_server.py \\
        --hsv-lower 0 30 80 --hsv-upper 20 255 255 \\
        --min-area 1000 --max-area 80000 --port 8081
"""

import argparse
import math

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse

# ── 전역 파라미터 (argparse → globals 에 저장) ──────────────────────────────
HSV_LOWER  = np.array([0,   0,  208],  dtype=np.uint8)
HSV_UPPER  = np.array([158, 30, 255],  dtype=np.uint8)
MIN_AREA   = 1000      # px²
MAX_AREA   = 80000     # px²
MIN_W, MAX_W = 110, 200 # 상자 가로 범위 (px)
MIN_H, MAX_H = 110, 200 # 상자 세로 범위 (px)
MORPH_K    = 7         # 형태학적 연산 커널 크기
# ────────────────────────────────────────────────────────────────────────────

app = FastAPI(title="CV Detect Server (OpenCV OBB)")


def detect_box_cv(img: np.ndarray) -> dict:
    """
    OpenCV 고전 비전으로 상자를 검출하여 OBB 정보 반환.

    Returns
    -------
    dict with keys: detected, cx, cy, w, h, theta, confidence
    """
    # 1. 전처리
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask    = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    # 2. 형태학적 연산 (구멍 메우기 + 잡음 제거)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_K, MORPH_K))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    # 3. 윤곽선 검출
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 4. 모든 유효한 윤곽선 검출 (면적 필터 적용)
    raw_detections = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue

        rect = cv2.minAreaRect(cnt)
        cx, cy = float(rect[0][0]), float(rect[0][1])
        w,  h  = float(rect[1][0]), float(rect[1][1])
        angle  = rect[2]

        # 가로/세로 크기 필터링
        size_ok = (MIN_W <= w <= MAX_W and MIN_H <= h <= MAX_H) or \
                  (MIN_W <= h <= MAX_W and MIN_H <= w <= MAX_H)
        
        if not size_ok:
            continue

        # 무조건 긴 변을 w(가로)로 설정하여 장축 각도 기준 맞추기
        if w < h:
            w, h = h, w
            angle += 90.0

        # 각도를 -90 ~ 90도 범위로 정규화
        while angle >= 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0

        theta_rad = math.radians(angle)

        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        confidence = min(area / hull_area, 1.0) if hull_area > 0 else 0.0

        raw_detections.append({
            "cx": cx, "cy": cy, "w":  w, "h":  h,
            "theta": theta_rad, "confidence": round(float(confidence), 4),
        })

    if not raw_detections:
        return {
            "detected":   False, "count": 0, "detections": [],
            "cx": 0.0, "cy": 0.0, "w": 0.0, "h": 0.0, "theta": 0.0, "confidence": 0.0
        }

    # 5. X 좌표(cx) 기준으로 정렬하여 번호(ID) 고정
    # 이렇게 하면 상자가 가만히 있는 한 왼쪽 것이 항상 0번, 오른쪽 것이 1번이 됩니다.
    raw_detections.sort(key=lambda d: d["cx"])
    
    detections = []
    for i, d in enumerate(raw_detections):
        d["id"] = i
        detections.append(d)

    # 6. 결과 반환
    first = detections[0]
    return {
        "detected":   True,
        "count":      len(detections),
        "detections": detections,
        "cx":         first["cx"],
        "cy":         first["cy"],
        "w":          first["w"],
        "h":          first["h"],
        "theta":      first["theta"],
        "confidence": first["confidence"],
    }


# 최신 검출 결과를 저장할 전역 변수
latest_result = {"detected": False, "detections": []}

@app.get("/latest")
async def get_latest():
    """가장 최근에 처리된 검출 결과를 반환"""
    return latest_result

@app.post("/detect")
async def detect(image: UploadFile = File(...)):
    global latest_result
    """
    obb_server.py 와 완전히 동일한 엔드포인트.
    클라이언트 코드 변경 없이 --server URL 만 바꾸면 교체 사용 가능.
    """
    try:
        contents = await image.read()
        nparr    = np.frombuffer(contents, np.uint8)
        img      = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("이미지 디코딩 실패")

        result = detect_box_cv(img)
        # 전역 변수에 결과 업데이트
        latest_result = result
        
        return JSONResponse(result)

    except Exception as e:
        print(f"[ERROR] {e}")
        raise HTTPException(status_code=500, detail=str(e))


def main() -> None:
    global HSV_LOWER, HSV_UPPER, MIN_AREA, MAX_AREA, MORPH_K

    parser = argparse.ArgumentParser(description="OpenCV 고전 비전 상자 검출 서버")
    parser.add_argument("--hsv-lower", type=int, nargs=3, default=[0, 30, 80],
                        metavar=("H", "S", "V"), help="HSV 하한 (기본: 0 30 80)")
    parser.add_argument("--hsv-upper", type=int, nargs=3, default=[20, 255, 255],
                        metavar=("H", "S", "V"), help="HSV 상한 (기본: 20 255 255)")
    parser.add_argument("--min-area",  type=int, default=1000,
                        help="검출 최소 면적 px² (기본: 1000)")
    parser.add_argument("--max-area",  type=int, default=80000,
                        help="검출 최대 면적 px² (기본: 80000)")
    parser.add_argument("--min-w", type=int, default=30, help="상자 최소 가로 px (기본: 30)")
    parser.add_argument("--max-w", type=int, default=400, help="상자 최대 가로 px (기본: 400)")
    parser.add_argument("--min-h", type=int, default=30, help="상자 최소 세로 px (기본: 30)")
    parser.add_argument("--max-h", type=int, default=400, help="상자 최대 세로 px (기본: 400)")
    parser.add_argument("--morph-k",   type=int, default=7,
                        help="형태학적 연산 커널 크기 (기본: 7)")
    parser.add_argument("--host",      default="0.0.0.0", help="서버 바인딩 IP")
    parser.add_argument("--port",      type=int, default=8081, help="서버 포트 (기본: 8081)")
    args = parser.parse_args()

    # 전역 파라미터 업데이트
    global HSV_LOWER, HSV_UPPER, MIN_AREA, MAX_AREA, MIN_W, MAX_W, MIN_H, MAX_H, MORPH_K
    HSV_LOWER = np.array(args.hsv_lower, dtype=np.uint8)
    HSV_UPPER = np.array(args.hsv_upper, dtype=np.uint8)
    MIN_AREA  = args.min_area
    MAX_AREA  = args.max_area
    MIN_W, MAX_W = args.min_w, args.max_w
    MIN_H, MAX_H = args.min_h, args.max_h
    MORPH_K   = args.morph_k

    print(f"[INFO] OpenCV 고전 비전 서버 시작")
    print(f"  HSV 범위: lower={args.hsv_lower}  upper={args.hsv_upper}")
    print(f"  면적 범위: {args.min_area} ~ {args.max_area} px²")
    print(f"  서버 주소: http://{args.host}:{args.port}/detect")
    print(f"  클라이언트 명령어 예:")
    print(f"    python3 src/devices/jetcobot/vision/obb_detect_client.py \\")
    print(f"      --remote --server http://localhost:{args.port}/detect")

    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
