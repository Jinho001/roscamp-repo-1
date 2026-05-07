# Vision Pick & Place 구현 진행 현황 (2026-05-07)

## 🎯 프로젝트 목표
ROS2 Jetcobot 비전 시스템에서 상자 픽업(pick)과 배치(place)를 자동화하는 파이프라인 구축.

---

## ✅ 완료된 작업

### 1. 시스템 아키텍처 설계 및 구현
- **HTTP → ROS2 마이그레이션 완료**
  - `cv_detect_server.py`: OBB 검출 → ROS2 ObbBoxArray 토픽 발행
  - `detect_bridge_node` 제거 (기능 통합)
  - `coord_transform_node`: 카메라 좌표 → base_link 3D 변환

- **통합 Pick-Place 노드 구현**
  - `vision_pick_place_node.py`: Pick/Place 액션 통합 (Serial port 충돌 방지)
  - MyCobot280 제어 (단일 instance)
  - 뮤텍스를 통한 동시 실행 방지

### 2. 비전 검출 신뢰성 검증
- **Profile Tuner 구현** (`profile_tuner.py`)
  - 실시간 HSV/W,H 파라미터 조정
  - 터미널 입력 기반 UI
  - 픽셀 ↔ mm 변환 표시
  - Location별 독립적 튜닝

- **카메라 내부 파라미터 보정**
  - `_default_K` 오류 수정 (구 캘리브레이션 값 → 최신 실측값)
  - intrinsics: [990.81, 987.62, 301.20, 238.38]
  - RMS error: 1.25mm (hand-eye 캘리브레이션)

### 3. 좌표 변환 최적화
- **Ray-Plane 교점 계산** (핀홀 역산)
  - z_surface_mm 기반 고정 높이 처리
  - 모든 상자에 대해 3D 좌표 계산

- **Location별 보정값 적용**
  - `receiving_zone`: x=-17.6mm, y=-9.9mm 오차 보정
  - `tray`: 보정 없음 (오차 허용 범위 내)
  - `pick_offset_mm` 파라미터로 location별 관리

### 4. 다중 상자 지원
- **Box Index 선택 기능**
  - VisionPick.Goal에 `box_index` 필드 추가
  - `-1`: 최고 confidence (기본값)
  - `0~N`: 특정 상자 선택
  - PickPoint 메시지: `box_index`, `confidence` 필드 추가

- **coord_transform_node 개선**
  - 모든 검출 상자에 대해 PickPoint 발행
  - 각 상자의 신뢰도(confidence) 함께 전달

### 5. Pick-Place 연속 실행
- **Pick-Place Coordinator 구현** (`pick_place_coordinator_node.py`)
  - Pick 완료 후 PlaceCommand 토픽 대기
  - Place 신호 수신 → 자동 Place 액션 호출
  - 비동기 action client 기반

- **메시지 정의**
  - `PlaceCommand`: location, box_index

### 6. 파라미터 튜닝 완료
- **Location별 프로파일** (`pick_place_profiles.yaml`)
  - `tray`: z_surface=90mm, HSV=[0,0,208]~[158,30,255]
  - `receiving_zone`: z_surface=50mm, W/H 범위 조정
  - `pinky_tray_place`: z_surface=70mm, 색상 마커 감지 (신규)

### 7. 동시 실행 방지
- **뮤텍스 기반 제어**
  - `_action_lock` (threading.Lock)
  - Pick 실행 중 Place 요청 → 거절
  - Place 실행 중 Pick 요청 → 거절
  - 안전한 Serial port 접근

### 8. 테스트 인프라 구축
- **테스트 스크립트**
  - `test_vision_pick.py`: Pick action 테스트
  - `test_vision_place.py`: Place action 테스트
  - `test_pick_place.py`: Coordinator 테스트

- **테스트 가이드 문서**
  - `PICK_PLACE_TEST.md`: 5가지 시나리오
  - 체크리스트, 로그 포인트, 트러블슈팅

---

## 📊 현재 상태

### 구현된 노드

| 노드 | 역할 | 상태 |
|------|------|------|
| `coord_transform_node` | Static TF + 역투영 | ✅ 완료 |
| `vision_pick_place_node` | Pick/Place 액션 | ✅ 완료 |
| `pick_place_coordinator_node` | Pick-Place 조율 | ✅ 완료 |
| `cv_detect_server.py` | OBB 검출 | ✅ 완료 |

### 메시지 정의

| 메시지 | 필드 | 상태 |
|--------|------|------|
| `PickPoint` | x, y, z, yaw_deg, box_index, confidence | ✅ 완료 |
| `PlaceCommand` | location, box_index | ✅ 완료 |
| `ObbBoxArray` | (기존) | ✅ 사용 중 |

### Action 인터페이스

| Action | Goal | Feedback | Result | 상태 |
|--------|------|----------|--------|------|
| `/vision_pick` | location, box_index | phase, progress | success, message, pick_point_base | ✅ 완료 |
| `/vision_place` | location, box_index | phase, progress | success, message, place_point_base | ✅ 완료 |

---

## 🔧 기술 스택

| 계층 | 기술 |
|------|------|
| **OS** | Ubuntu 24.04 + ROS2 Jazzy |
| **비전** | OpenCV, YOLO OBB |
| **로봇 제어** | MyCobot280, pymycobot |
| **좌표 변환** | TF2, numpy |
| **네트워크** | UDP (영상 스트림), HTTP (detect_server), ROS2 Topics/Actions |

---

## 📈 테스트 준비 상태

### 구현된 테스트

- ✅ **시나리오 1**: Pick (tray)
- ✅ **시나리오 2**: Pick (receiving_zone) + 보정값 확인
- ✅ **시나리오 3**: Place (pinky_tray_place)
- ✅ **시나리오 4**: Pick → Place 연속 (Coordinator)
- ✅ **시나리오 5**: 동시 실행 방지 검증

### 테스트 커맨드

```bash
# 1. ROS2 노드 시작 (메인 PC)
python3 src/devices/jetcobot/vision/cv_detect_server.py
python3 src/devices/jetcobot/vision/stream_sender.py --host 192.168.1.4

# 2. ROS2 노드 시작 (제어 PC)
ros2 launch jetcobot_vision jetcobot_vision.launch.py

# 3. Pick 테스트
python3 src/jetcobot_vision/scripts/test_vision_pick.py tray
python3 src/jetcobot_vision/scripts/test_vision_pick.py receiving_zone

# 4. Place 테스트
python3 src/jetcobot_vision/scripts/test_vision_place.py pinky_tray_place

# 5. Place 신호 발송 (Pick 완료 후)
ros2 topic pub /pick_place_coordinator/place_command \
  jetcobot_vision_msgs/PlaceCommand \
  "location: 'pinky_tray_place'
   box_index: -1"
```

---

## 📝 주요 변경사항 (Git Commit 요약)

| 번호 | 커밋 | 설명 |
|------|------|------|
| 1 | Fix: receiving_zone 좌표 오차 보정 | pick_offset_mm 추가 |
| 2 | Feat: vision_place_node에 보정값 적용 | place도 오차 보정 |
| 3 | Refactor: 불필요한 pick/place 파라미터 제거 | 1차 범위 정리 |
| 4 | Feat: 상자 index 선택 기능 추가 | box_index 필드 |
| 5 | Feat: Pick-Place Coordinator 노드 추가 | 연속 실행 지원 |
| 6 | Refactor: Pick와 Place를 통합 노드로 관리 | vision_pick_place_node |
| 7 | Fix: Pick/Place 동시 실행 방지 | 뮤텍스 추가 |
| 8 | Docs: Pick & Place 통합 테스트 가이드 | PICK_PLACE_TEST.md |
| 9 | Fix: ROS2 Action 클라이언트 피드백 처리 제거 | test_vision_place.py 추가 |

---

## 🚀 다음 단계

### 즉시 필요한 작업
1. **실제 로봇 테스트** (PICK_PLACE_TEST.md 시나리오 실행)
2. **각 location별 파라미터 검증**
   - `warehouse_pick`: fixed_coords 측정 필요
   - `warehouse_place`: fixed_coords 측정 필요

### 향후 개선사항
- [ ] Coordinator에 `/start_pick` service 추가
- [ ] Pick-Place 자동 loop 지원
- [ ] 상태 조회 service (`/status`)
- [ ] 에러 recovery 메커니즘
- [ ] 로그 시각화 (rqt dashboard)

---

## 📂 주요 파일 구조

```
src/jetcobot_vision/
├── jetcobot_vision/
│   ├── coord_transform_node.py          # 좌표 변환
│   ├── vision_pick_place_node.py        # Pick/Place 통합
│   └── pick_place_coordinator_node.py   # Pick-Place 조율
├── config/
│   ├── pick_place_profiles.yaml         # Location별 파라미터
│   └── vision_params.yaml
├── launch/
│   └── jetcobot_vision.launch.py        # 노드 시작
└── scripts/
    ├── test_vision_pick.py              # Pick 테스트
    └── test_vision_place.py             # Place 테스트

src/jetcobot_vision_msgs/
├── action/
│   ├── VisionPick.action                # Pick action
│   └── VisionPlace.action               # Place action
└── msg/
    ├── PickPoint.msg                    # 픽업 좌표
    ├── PlaceCommand.msg                 # Place 신호
    └── ObbBoxArray.msg

docs/
├── PICK_PLACE_TEST.md                   # 테스트 가이드
├── PICK_PLACE_COORDINATOR.md            # Coordinator 설명
├── RECEIVING_ZONE_DIAGNOSIS.md          # 진단 가이드
└── TUNING_GUIDE.md                      # 파라미터 튜닝 가이드
```

---

## 🎓 학습 포인트

### 해결한 문제들

1. **Serial Port 충돌**: 두 노드가 같은 포트 접근 → 통합 노드로 해결
2. **좌표 변환 오차**: Camera intrinsics 불일치 → 실측값으로 보정
3. **오버피팅 위험**: 모든 상자에 같은 HSV → location별 프로파일 분리
4. **카메라 앵글 차이**: 각 location마다 pick_offset_mm 적용
5. **ROS2 호환성**: Action feedback 처리 방식 개선

### 핵심 설계 결정

1. **통합 노드 선택**: 분산 제어보다 단일 제어점
2. **Coordinator 패턴**: Pick-Place 간 느슨한 결합
3. **토픽 기반 신호**: 명령형 vs 반응형 제어
4. **뮤텍스 기반 배제**: 동시성 제어의 단순성

---

## 📞 문의 및 협업

- **브랜치**: `claude/ros2-robot-migration` (develop에 merge 예정)
- **테스트 상태**: 준비 완료 (실제 로봇 테스트 대기)
- **주요 담당자**: kcloud (git user)

---

**최종 업데이트**: 2026-05-07 23:30 KST
**상태**: ✅ 구현 완료 → 테스트 준비 중
