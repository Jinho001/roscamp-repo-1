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
    python3 src/devices/jetcobot/vision/cv_detect_server.py
"""

import argparse
import math
import time
from typing import Optional

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse
from pydantic import BaseModel

# ── 전역 파라미터 ──────────────────────────────────────────────────────────────
HSV_LOWER  = np.array([0,   0,  208],  dtype=np.uint8)
HSV_UPPER  = np.array([158, 30, 255],  dtype=np.uint8)
MIN_AREA   = 1000
MAX_AREA   = 80000
MIN_W, MAX_W = 110, 200
MIN_H, MAX_H = 110, 200
MORPH_K    = 7
# ────────────────────────────────────────────────────────────────────────────

app = FastAPI(title="CV Detect Server (OpenCV OBB)")


class DetectConfig(BaseModel):
    hsv_lower: Optional[list[int]] = None
    hsv_upper: Optional[list[int]] = None
    min_area:  Optional[int] = None
    max_area:  Optional[int] = None
    min_w:     Optional[int] = None
    max_w:     Optional[int] = None
    min_h:     Optional[int] = None
    max_h:     Optional[int] = None
    morph_k:   Optional[int] = None


@app.post("/config")
async def update_config(cfg: DetectConfig):
    """런타임 파라미터 변경. 전달된 필드만 업데이트."""
    global HSV_LOWER, HSV_UPPER, MIN_AREA, MAX_AREA, MIN_W, MAX_W, MIN_H, MAX_H, MORPH_K
    if cfg.hsv_lower is not None:
        HSV_LOWER = np.array(cfg.hsv_lower, dtype=np.uint8)
    if cfg.hsv_upper is not None:
        HSV_UPPER = np.array(cfg.hsv_upper, dtype=np.uint8)
    if cfg.min_area  is not None: MIN_AREA = cfg.min_area
    if cfg.max_area  is not None: MAX_AREA = cfg.max_area
    if cfg.min_w     is not None: MIN_W    = cfg.min_w
    if cfg.max_w     is not None: MAX_W    = cfg.max_w
    if cfg.min_h     is not None: MIN_H    = cfg.min_h
    if cfg.max_h     is not None: MAX_H    = cfg.max_h
    if cfg.morph_k   is not None: MORPH_K  = cfg.morph_k
    print(f"[CONFIG] HSV: {HSV_LOWER.tolist()} ~ {HSV_UPPER.tolist()}  "
          f"크기: {MIN_W}~{MAX_W} x {MIN_H}~{MAX_H}")
    return {"ok": True}


def detect_box_cv(img: np.ndarray) -> dict:
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask    = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (MORPH_K, MORPH_K))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    raw_detections = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA or area > MAX_AREA:
            continue

        rect = cv2.minAreaRect(cnt)
        cx, cy = float(rect[0][0]), float(rect[0][1])
        w,  h  = float(rect[1][0]), float(rect[1][1])
        angle  = rect[2]

        size_ok = (MIN_W <= w <= MAX_W and MIN_H <= h <= MAX_H) or \
                  (MIN_W <= h <= MAX_W and MIN_H <= w <= MAX_H)
        if not size_ok:
            continue

        if w < h:
            w, h = h, w
            angle += 90.0
        while angle >= 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0

        theta_rad  = math.radians(angle)
        hull_area  = cv2.contourArea(cv2.convexHull(cnt))
        confidence = min(area / hull_area, 1.0) if hull_area > 0 else 0.0

        raw_detections.append({
            "cx": cx, "cy": cy, "w": w, "h": h,
            "theta": theta_rad, "confidence": round(float(confidence), 4),
        })

    if not raw_detections:
        return {
            "detected": False, "count": 0, "detections": [],
            "cx": 0.0, "cy": 0.0, "w": 0.0, "h": 0.0, "theta": 0.0, "confidence": 0.0
        }

    raw_detections.sort(key=lambda d: d["cx"])
    detections = []
    for i, d in enumerate(raw_detections):
        d["id"] = i
        detections.append(d)

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


# 최신 검출 결과 (timestamp 포함)
latest_result = {"detected": False, "detections": [], "timestamp": 0.0}


@app.get("/latest")
async def get_latest():
    return latest_result


@app.post("/detect")
async def detect(image: UploadFile = File(...)):
    global latest_result
    try:
        contents = await image.read()
        nparr    = np.frombuffer(contents, np.uint8)
        img      = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("이미지 디코딩 실패")

        result = detect_box_cv(img)
        result["timestamp"] = time.time()
        latest_result = result
        return JSONResponse(result)

    except Exception as e:
        print(f"[ERROR] {e}")
        raise HTTPException(status_code=500, detail=str(e))


def main() -> None:
    global HSV_LOWER, HSV_UPPER, MIN_AREA, MAX_AREA, MIN_W, MAX_W, MIN_H, MAX_H, MORPH_K

    parser = argparse.ArgumentParser(description="OpenCV 고전 비전 상자 검출 서버")
    parser.add_argument("--hsv-lower", type=int, nargs=3, default=[0, 0, 208],
                        metavar=("H", "S", "V"), help="HSV 하한 (기본: 0 0 208)")
    parser.add_argument("--hsv-upper", type=int, nargs=3, default=[158, 30, 255],
                        metavar=("H", "S", "V"), help="HSV 상한 (기본: 158 30 255)")
    parser.add_argument("--min-area",  type=int, default=1000)
    parser.add_argument("--max-area",  type=int, default=80000)
    parser.add_argument("--min-w",     type=int, default=110)
    parser.add_argument("--max-w",     type=int, default=200)
    parser.add_argument("--min-h",     type=int, default=110)
    parser.add_argument("--max-h",     type=int, default=200)
    parser.add_argument("--morph-k",   type=int, default=7)
    parser.add_argument("--host",      default="0.0.0.0")
    parser.add_argument("--port",      type=int, default=8081)
    args = parser.parse_args()

    HSV_LOWER = np.array(args.hsv_lower, dtype=np.uint8)
    HSV_UPPER = np.array(args.hsv_upper, dtype=np.uint8)
    MIN_AREA  = args.min_area
    MAX_AREA  = args.max_area
    MIN_W, MAX_W = args.min_w, args.max_w
    MIN_H, MAX_H = args.min_h, args.max_h
    MORPH_K   = args.morph_k

    print(f"[INFO] OpenCV 고전 비전 서버 시작")
    print(f"  HSV 범위: lower={args.hsv_lower}  upper={args.hsv_upper}")
    print(f"  크기 범위: {args.min_w}~{args.max_w} x {args.min_h}~{args.max_h} px")
    print(f"  서버 주소: http://{args.host}:{args.port}")

    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
