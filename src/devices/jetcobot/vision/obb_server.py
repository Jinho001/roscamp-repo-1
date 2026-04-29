#!/usr/bin/env python3
"""
YOLOv8 OBB (Oriented Bounding Box) 추론 API 서버
==============================================
FastAPI를 사용하여 POST /detect 엔드포인트를 제공합니다.
클라이언트가 이미지를 보내면 가장 신뢰도가 높은 상자의 OBB 검출 결과를 반환합니다.
"""

import argparse
import io
import math
import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.responses import JSONResponse

# ultralytics 모듈은 설치가 필요합니다 (pip install ultralytics)
try:
    from ultralytics import YOLO
except ImportError:
    print("[ERROR] ultralytics 모듈이 없습니다. 'pip install ultralytics' 를 실행하세요.")
    YOLO = None

app = FastAPI(title="YOLO OBB Server")
model = None

@app.post("/detect")
async def detect(image: UploadFile = File(...)):
    global model
    if model is None:
        raise HTTPException(status_code=500, detail="Model not loaded.")

    try:
        # 1. 클라이언트가 보낸 이미지 바이트 읽기 및 디코딩
        contents = await image.read()
        nparr = np.frombuffer(contents, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError("Failed to decode image.")

        # 2. YOLO OBB 추론 수행
        results = model(img, verbose=False)
        
        # 3. 결과 파싱 (신뢰도 가장 높은 객체 1개 선택)
        best_conf = -1.0
        best_box = None
        
        for r in results:
            if r.obb is not None and len(r.obb) > 0:
                for obb in r.obb:
                    conf = float(obb.conf[0])
                    if conf > best_conf:
                        best_conf = conf
                        best_box = obb

        if best_box is None:
            return JSONResponse({
                "detected": False,
                "cx": 0.0,
                "cy": 0.0,
                "theta": 0.0,
                "confidence": 0.0
            })

        # ultralytics OBB 포맷: xywhr (x_center, y_center, width, height, rotation_radian)
        xywhr = best_box.xywhr[0].cpu().numpy()
        cx = float(xywhr[0])
        cy = float(xywhr[1])
        w  = float(xywhr[2])
        h  = float(xywhr[3])
        theta = float(xywhr[4])

        return JSONResponse({
            "detected": True,
            "cx": cx,
            "cy": cy,
            "theta": theta,
            "confidence": best_conf
        })

    except Exception as e:
        print(f"[ERROR] Inference failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


def main():
    global model
    parser = argparse.ArgumentParser(description="YOLO OBB FastAPI Server")
    parser.add_argument("--weights", type=str, default="yolov8n-obb.pt", help="학습된 YOLO OBB 가중치 파일 경로")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="서버 바인딩 IP")
    parser.add_argument("--port", type=int, default=8080, help="서버 바인딩 포트")
    args = parser.parse_args()

    if YOLO is None:
        return

    print(f"[INFO] 모델 로드 중: {args.weights}")
    try:
        model = YOLO(args.weights)
    except Exception as e:
        print(f"[ERROR] 모델 로드 실패: {e}")
        print("  👉 YOLO OBB 모델 파일이 존재하는지 확인하세요.")
        return

    print(f"[INFO] FastAPI 서버 시작: {args.host}:{args.port}")
    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
