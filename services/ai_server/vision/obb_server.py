#!/usr/bin/env python3
"""
Jinho YOLO OBB 상자 검출 HTTP 서버 (임시)
==========================================
GPU 서버에서 실행. FastAPI + uvicorn.

엔드포인트:
    POST /detect
        Body : multipart/form-data, field name = "image" (JPEG bytes)
        Response: {
            "detected"  : bool,
            "cx"        : float,  # 픽셀 중심 X
            "cy"        : float,  # 픽셀 중심 Y
            "theta"     : float,  # OBB 장축 각도 (rad)
            "confidence": float
        }

    GET  /health
        Response: {"status": "ok", "model": "<모델 경로>"}

실행:
    pip install fastapi uvicorn ultralytics
    python obb_server.py [--model yolov8n-obb-box.pt] [--port 8080]

참고:
    기존 cv_server.py (UDP, 포트 6006) 와 완전히 별도 프로세스.
    포트 충돌 없음.
"""

import argparse
import io
import math
import os
import sys
import time

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, File, HTTPException, UploadFile
from fastapi.responses import JSONResponse
from ultralytics import YOLO


# ── 기본 설정 ─────────────────────────────────────────────────────────────────
DEFAULT_MODEL = os.environ.get("OBB_MODEL_PATH", "yolov8n-obb-box.pt")
CONF_THRESHOLD = float(os.environ.get("OBB_CONF", "0.5"))
DEFAULT_PORT   = int(os.environ.get("OBB_PORT",   "8080"))
# ─────────────────────────────────────────────────────────────────────────────

app   = FastAPI(title="YOLO OBB Box Detection Server", version="1.0.0")
model: YOLO | None = None


# ── 헬퍼 ─────────────────────────────────────────────────────────────────────

def _decode_image(data: bytes) -> np.ndarray | None:
    """JPEG bytes → BGR ndarray. 실패 시 None."""
    buf = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return img


def _extract_best_obb(result, conf_threshold: float) -> dict:
    """
    ultralytics OBB result 에서 가장 신뢰도 높은 검출 결과를 추출한다.

    OBB 속성:
        result.obb.xywhr : [cx, cy, w, h, angle_rad]  (각 행이 하나의 박스)
        result.obb.conf  : 신뢰도

    Returns
    -------
    dict with keys: detected, cx, cy, theta, confidence
    """
    empty = {"detected": False, "cx": 0.0, "cy": 0.0,
             "theta": 0.0, "confidence": 0.0}

    if result.obb is None or result.obb.xywhr is None:
        return empty

    xywhr = result.obb.xywhr.cpu().numpy()   # (N, 5)
    confs  = result.obb.conf.cpu().numpy()    # (N,)

    # conf 필터링
    mask = confs >= conf_threshold
    if not mask.any():
        return empty

    xywhr = xywhr[mask]
    confs  = confs[mask]

    # 가장 신뢰도 높은 박스 선택
    best_idx = int(np.argmax(confs))
    cx, cy, w, h, angle_rad = xywhr[best_idx]

    return {
        "detected"  : True,
        "cx"        : float(cx),
        "cy"        : float(cy),
        "theta"     : float(angle_rad),   # rad
        "confidence": float(confs[best_idx]),
    }


# ── 엔드포인트 ────────────────────────────────────────────────────────────────

@app.get("/health")
async def health():
    return {"status": "ok", "model": DEFAULT_MODEL}


@app.post("/detect")
async def detect(image: UploadFile = File(...)):
    """
    JPEG 이미지를 받아 YOLO OBB 추론을 실행하고 결과를 반환한다.
    """
    if model is None:
        raise HTTPException(status_code=503, detail="모델 로딩 중")

    raw = await image.read()
    img = _decode_image(raw)
    if img is None:
        raise HTTPException(status_code=400, detail="이미지 디코딩 실패")

    t0 = time.time()
    results = model(img, verbose=False)
    elapsed_ms = (time.time() - t0) * 1000.0

    # config에서 threshold를 가져오는 대신 전역 변수나 인자를 사용할 수 있게 수정 가능
    result_dict = _extract_best_obb(results[0], 0.5) 
    result_dict["process_ms"] = round(elapsed_ms, 2)

    return JSONResponse(content=result_dict)


# ── 메인 ─────────────────────────────────────────────────────────────────────

def main():
    global model, DEFAULT_MODEL, CONF_THRESHOLD, DEFAULT_PORT

    parser = argparse.ArgumentParser(
        description="YOLO OBB Box Detection HTTP Server (TASK-V03-A)"
    )
    parser.add_argument(
        "--model", default=DEFAULT_MODEL,
        help=f"YOLO OBB 모델 경로 (기본: {DEFAULT_MODEL})"
    )
    parser.add_argument(
        "--conf", type=float, default=CONF_THRESHOLD,
        help=f"신뢰도 임계값 (기본: {CONF_THRESHOLD})"
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"HTTP 포트 (기본: {DEFAULT_PORT})"
    )
    args = parser.parse_args()

    print(f"[INFO] 모델 로딩: {args.model}")
    model = YOLO(args.model)
    print(f"[INFO] OBB 서버 시작 — 포트 {args.port}")
    print(f"[INFO] 신뢰도 임계값: {args.conf}")

    uvicorn.run(app, host="0.0.0.0", port=args.port)


if __name__ == "__main__":
    main()
