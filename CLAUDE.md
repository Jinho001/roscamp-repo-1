# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## 프로젝트 한 줄 요약

무신사 매장 내 자율주행 로봇을 이용한 시착품 자동 전달 시스템.
고객이 UI에서 상품을 선택하면, AMR(SShopy)이 ARM 로봇(FrontJet/WareJet) 사이를 오가며 상자를 픽업·적재·배달한다.

---

## 시스템 아키텍처

```
┌─────────────────────────────────────────────────────┐
│ Interface Layer                                      │
│   Mobile Web UI (React)    KIOSK UI    Monitoring UI │
└───────────────────┬─────────────────────────────────┘
                    │ HTTP
┌───────────────────▼─────────────────────────────────┐
│ Server Layer                                         │
│   Moosinsa Service (b)  ←→  MSS DB                  │
│        ↕ HTTP/ROS2                                   │
│   Moosinsa AI Server: Yolo26 / M_LLM                 │
└──────┬──────────────┬──────────────┬────────────────┘
       │ HTTP         │ HTTP         │ HTTP
┌──────▼──────┐ ┌─────▼──────┐ ┌────▼───────┐
│  SShopy Pi  │ │ FrontJet Pi│ │ WareJet Pi │
│  :8003      │ │  :8001     │ │  :8002     │
│  Nav2       │ │  v4l2_cam  │ │  v4l2_cam  │
│  FastAPI    │ │  YOLO node │ │  YOLO node │
│             │ │  ARM ctrl  │ │  ARM ctrl  │
│             │ │  FastAPI   │ │  FastAPI   │
└─────────────┘ └────────────┘ └────────────┘
```

### 통신 프로토콜

| 구간 | 프로토콜 |
|------|---------|
| UI ↔ Moosinsa Service | HTTP (React fetch) |
| Moosinsa Service ↔ 각 Pi | HTTP (FastAPI REST) |
| Pi 내부 노드 간 | ROS2 Topic / Action |
| Moosinsa Service ↔ AI Server | HTTP |
| Moosinsa Service ↔ DB | MySQL |

---

## 컴포넌트 식별자

| 약어 | 컴포넌트 | 역할 |
|------|---------|------|
| f | UI | 사용자 인터페이스 (React) |
| b | Moosinsa Service | 중앙 오케스트레이터 |
| mllm | M_LLM | 키워드 기반 상품 필터링 |
| yolo | Yolo26 | 회수존 박스 유무 확인 |
| db | MSS DB | 재고(stock) 관리 |
| sp | SShopy | AMR (Pinky Pro) |
| fj | FrontJet | 고정 ARM 로봇 — 수령/반납 허브 |
| wj | WareJet | 고정 ARM 로봇 — 창고 선반 적재 |

---

## 연동 시나리오 (1차 구현 목표 플로우)

```
f  → b    : 사용자 키워드 입력 → 결과 요청
b  → mllm : 키워드 기반 상품 필터링 요청
b  → yolo : top-view cam 이미지 전달 → 회수존 박스 유무 확인
mllm → b  : 필터링된 상품 1개 응답
yolo → b  : 회수존 박스 있음 응답
b  → db   : 해당 상품 재고(stock ≥ 1) 조회
b  → f    : 결과 상품 응답

f  → b    : 시착품 전달 요청 (상품 클릭)
b  → sp   : FrontJet 위치로 이동 요청
sp → b    : 이동 완료 응답
b  → fj   : 카메라 이미지 1장 요청
fj → b    : 이미지 응답
b  → fj   : SShopy 트레이에 상자 싣기 요청
fj → b    : 싣기 완료 응답

b  → sp   : WareJet 위치로 이동 요청
sp → b    : 이동 완료 응답
b  → wj   : 카메라 이미지 1장 요청
wj → b    : 이미지 응답
b  → wj   : SShopy 트레이에 상자 싣기 요청
wj → b    : 싣기 완료 응답

b  → sp   : 고객 좌석으로 이동 요청
sp → b    : 이동 완료 응답
b  → f    : 시착품 전달 완료 응답
```

---

## 하드웨어 스펙

| 항목 | 값 |
|------|---|
| OS / 미들웨어 | Ubuntu 24.04 + ROS2 Jazzy |
| ARM 로봇 | MyCobot 280 × 2 (FrontJet, WareJet) |
| AMR | Pinky Pro (SShopy) |
| 컨트롤러 | Raspberry Pi (로봇별), 별도 GPU 서버 |
| 카메라 | USB 캠, Eye-in-Hand 방식 |
| 상자 크기 | 30 × 20 × 15 mm (모형) |
| 그리퍼 | 최대 70 mm 개구, 옆면 파지 |
| 비전 전략 | YOLO OBB + 핀홀 역산 (Z 고정값) |
| AMR 트레이 | 고정 홈 구조 |

---

## 로봇 및 PC 네트워크

| 장비 | IP | ROS Domain | rosbridge 포트 |
|------|----|-----------:|---------------|
| SShopy1 | 192.168.1.111 | 11 | 9091 |
| SShopy2 | 메인 PC 경유 | 12 | 9092 |
| SShopy3 | 메인 PC 경유 | 13 | 9093 |
| front_jet | 192.168.1.114 | 14 | 9094 |
| ware_jet | 192.168.1.115 | 15 | 9095 |
| 메인 PC | 192.168.1.4 | - | - |

---

## ROS2 노드 구조 (Pi별)

### FrontJet Pi (:8001) / WareJet Pi (:8002) — 동일 구조

```
[v4l2_camera]       → /fj/camera/image_raw  (sensor_msgs/Image)
[yolo_client]       → GPU 서버 HTTP 추론 요청
[coord_calc_node]   → /fj/object/pose       (geometry_msgs/PoseStamped)
[pick_place_server] → /fj/pick_and_place    (Action Server)
[http_bridge]       → :8001 FastAPI
```

### SShopy Pi (:8003)

```
[nav2 stack]   (Pinky Pro 기본 제공)
[http_bridge]  → :8003 FastAPI
```

### ROS2 Action 인터페이스 (PickAndPlace)

```
Action: /fj/pick_and_place
  Goal:
    geometry_msgs/PoseStamped target_pose  # base_link 기준
    uint8 operation  # 0=PICK, 1=PLACE
  Result:
    bool   success
    string message
  Feedback:
    string state  # APPROACHING / GRASPING / RETREATING / DONE
```

---

## HTTP API 엔드포인트 (FastAPI, 각 Pi)

### FrontJet / WareJet

```
POST /pick
  body: { "operation": "pick"|"place", "shelf_id": "A1" }
  resp: { "success": true, "cycle_time_ms": 5200 }

POST /image
  resp: { "image_base64": "..." }
```

### SShopy

```
POST /move
  body: { "waypoint": "frontjet"|"warejet"|"seat_<id>" }
  resp: { "success": true }
```

---

## 디렉토리 구조

```
apps/           UI 클라이언트
  phone_ui/       React/TypeScript — 고객 모바일 앱
  admin_ui/       React — 관리자 UI

services/       백엔드 서버 (Docker)
  main_server/
    api_server/   FastAPI — 메인 API (moosinsa_service.py)
    fms/          Fleet Management System (FastAPI + roslibpy)
    db/           MySQL 스키마/시드
  ai_server/
    vision/       OBB 검출 서버 (obb_server.py, cv_server.py)
    llm/          LLM/MLLM 서버

src/            ROS2 패키지
  devices/
    jetcobot/
      roles/
        front_jet/      매장 Jetcobot 컨트롤러 노드
        warehouse_jet/  창고 Jetcobot 컨트롤러 노드
      vision/           레거시 비전 파이프라인 (HTTP 기반)
      config/           handeye, camera_info, workspace_config YAML
    sshopy/             SShopy(Pinky) 관련 패키지
  jetcobot_vision/      ROS2 비전 파이프라인 패키지 (마이그레이션 진행 중)
  jetcobot_vision_msgs/ 커스텀 메시지/액션 패키지
```

---

## 구현 로드맵

| Phase | TASK | 내용 |
|-------|------|------|
| 0 | TASK-000 | AMR 정차 정밀도 실측 (±2cm 기준) |
| 0 | TASK-001 | AMR 트레이 설계·제작 |
| 0 | TASK-002 | 작업 공간 레이아웃 측정 → workspace_config.yaml |
| 0 | TASK-003 | ROS2 네트워크 통일 (ROS_DOMAIN_ID=42) |
| 1 | TASK-010 | USB 캠 ROS2 노드 (v4l2_camera) |
| 1 | TASK-011 | Camera Intrinsic 캘리브레이션 |
| 1 | TASK-012 | YOLO OBB 파인튜닝 + ROS2 래핑 |
| 1 | TASK-013 | Hand-Eye 캘리브레이션 (T_ee2cam) |
| 1 | TASK-014 | 핀홀 XY 역산 + 고정 Z 파이프라인 |
| 2 | TASK-020 | MyCobot ROS2 Action Server |
| 2 | TASK-021 | 픽앤플레이스 MVP 통합 테스트 |
| 3 | TASK-030 | FrontJet HTTP 브릿지 (FastAPI) |
| 3 | TASK-031 | SShopy AMR 이동 연동 (Nav2) |
| 3 | TASK-032 | WareJet 연동 (FrontJet 코드 재사용) |
| 3 | TASK-033 | 전체 시나리오 E2E 테스트 |

---

## 주요 리스크

| ID | 리스크 | 대응 |
|----|--------|------|
| R1 | AMR 정차 ±2cm 초과 | AprilTag 도킹 마커 추가 (일정 +2일) |
| R2 | Hand-Eye 캘리브 오차 | 25자세 이상, Tsai 방식, 검증 루틴 필수 |
| R3 | YOLO 데이터 부족 | 50장 이상 직접 촬영, 다양한 조명 |
| R4 | Reach 부족 | TASK-002 레이아웃 측정 후 배치 조정 |
| R5 | 카메라 모션 블러 | 정지 후 0.5초 대기 후 촬영 강제화 |
| R6 | 시리얼 끊김 | watchdog 2초, 자동 재연결 |

---

## 빌드 및 실행

```bash
# ROS2 패키지 빌드 (메시지 패키지 먼저)
cd /home/addinedu/roscamp-repo-1
colcon build --packages-select jetcobot_vision_msgs && source install/setup.bash
colcon build --packages-select jetcobot_vision && source install/setup.bash

# Main Server
cd services/main_server && docker compose up -d

# AI Server
cd services/ai_server && docker compose up -d

# 비전 파이프라인 (수동)
python3 src/devices/jetcobot/vision/stream_sender.py --host 192.168.1.4  # 제어 PC
python3 services/ai_server/vision/cv_server.py                            # 메인 PC
ros2 launch jetcobot_vision jetcobot_vision.launch.py                     # 메인 PC

# Phone UI
cd apps/phone_ui && npm install && npm run dev
```

---

## 아키텍처 핵심 패턴

### FMS 로봇 제어 흐름
`services/main_server/fms/RobotManager`가 핵심.
- `config.py`에 로봇별 rosbridge 포트·SSH 정보 정의
- Pinky(SShopy): roslibpy로 `/goal_pose`, `/cmd_vel` publish
- Jetcobot 팔: SSH → pymycobot 스크립트 직접 실행 (ROS2 미사용)
- 배달 시나리오는 `/amcl_pose` 구독으로 도착 감지 후 자동 stage 전환

### ROS2 비전 파이프라인 (진행 중)
`src/jetcobot_vision/`의 3노드 체인:
```
cv_detect_server.py (/latest 폴링)
    → detect_bridge_node → /detect_bridge/obb_boxes (ObbBoxArray)
    → coord_transform_node → /coord_transform/pick_point_base (PointStamped)
    → vision_pick_node (Action Server /vision_pick)
```
파라미터는 `src/jetcobot_vision/config/vision_params.yaml` 단일 파일로 관리.

---

## 개발 워크플로우 규칙

1. **Research First** — 작업 전 `docs/` 에 문서 작성
2. **Plan Before Code** — 플랜 승인 전 코드 작성 금지
3. **Human-in-the-loop** — 명시적 승인 후 구현
4. **Verify Before Done** — 테스트 없이 완료 표시 금지
5. **Minimal Impact** — 필요한 부분만 수정, 사이드 이펙트 최소화
6. **Stop If Unsure** — 불명확하면 추측 구현 대신 질문

- 브랜치: `feature/[이름]-[컴포넌트]-[기능]` → develop → main
- 커밋: `Feat:`, `Fix:`, `Docs:`, `Refactor:`, `Chore:` 접두사 사용
- 플랜 문서: `docs/{구체적인작업내용}_{YYYYMMDD}.md`
- 작업 추적: `tasks/todo.md`, 교훈 기록: `tasks/lessons.md`
- `tasks/`, `docs/`는 .gitignore 포함 (로컬 전용)

---

## 주요 설정 파일

| 파일 | 용도 |
|------|------|
| `services/main_server/fms/config.py` | 로봇 fleet IP/포트/도메인 설정 |
| `src/jetcobot_vision/config/vision_params.yaml` | 비전 노드 파라미터 (HSV, handeye, observe_pose 등) |
| `src/devices/jetcobot/config/front_jet/workspace_config.yaml` | front_jet 작업 공간 설정 |
| `src/devices/jetcobot/config/front_jet/handeye_result.yaml` | 핸드아이 캘리브레이션 행렬 |
| `src/devices/jetcobot/config/front_jet/camera_info.yaml` | 카메라 내부 파라미터 |
