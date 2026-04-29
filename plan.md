# 체커보드 인식 개선 계획 (Plan)

## 1. 실제 체커보드 규격 확인 (Human-in-the-loop 완료)
* [x] 사용자에게 현재 사용 중인 체커보드의 **가로/세로 사각형 칸 개수** 또는 **내부 교차점 코너 개수**를 확인받기. (사용자: "9*6개가 맞음")
* [ ] 조명 상태나 출력된 종이 여백에 문제가 없는지 확인.

## 2. `camera_calibration.py` 코드 수정
* [x] `CHECKERBOARD` 변수를 스크립트 실행 인자로 받을 수 있도록 `argparse`에 `--board` 옵션 추가 (예: `--board 9x6` 또는 `--board 8x5`). 이를 통해 코드 내 하드코딩된 (9, 6)을 유연하게 변경할 수 있도록 조치.
* [x] 인식 속도 및 안정성 향상을 위해 `cv2.findChessboardCorners` 플래그에 `cv2.CALIB_CB_FAST_CHECK` 추가 고려.

## 3. `handeye_calibration.py` 동기화 (옵션)
* [x] `camera_calibration.py`와 마찬가지로 Hand-Eye 캘리브레이션 코드에도 `--board` 옵션을 추가하여 일관성 유지.

## 4. 검증 및 캘리브레이션 재수행
* [x] 변경된 코드로 `stream_sender.py` 및 `camera_calibration.py` 재가동 완료.
* [x] (V01 완료) RMS 0.21px 달성 및 `camera_info.yaml` 생성 완료.
* [x] (V02 완료) 검증 오차 RMS 2.53mm 달성 및 `handeye_result.yaml` 생성 완료.

## 5. 좌표 변환 파이프라인 검증 (TASK-V04)
* [x] 트레이(Tray) 표면의 높이(`z_fixed`) 실측 및 `workspace_config.yaml` 입력 완료.
* [x] (V04 완료) `coord_transform.py` 검증 스크립트를 통한 픽셀 → Base 3D 좌표 변환 수학 모델 검증 완료.

## 6. 비전 픽앤플레이스 통합 (TASK-V05) 준비
* [ ] 파지 각도 설정: `workspace_config.yaml`의 `grasp_rp` (roll, pitch) 값 입력.
* [ ] YOLO OBB 서버 기동 확인 및 물체 인식 연동.
* [ ] `vision_pick.py` 구동 테스트.

---
> **다음 단계**: 실제로 물건을 집어들 때 사용할 로봇 손목의 각도(`grasp_rp`의 roll, pitch)를 확인하여 입력해 주세요. (일반적으로 바닥을 수직으로 내려다보며 집을 경우 pitch: 0, roll: -90 근처입니다.)
