# Pick-Place Coordinator

Pick 동작 완료 후 Place 신호를 받으면 자동으로 Place 동작을 수행하는 코디네이터.

## 아키텍처

```
┌─────────────────────────────────────────┐
│ pick_place_coordinator_node             │
│  - /vision_pick Action Client           │
│  - /vision_place Action Client          │
│  - /pick_place_coordinator/place_command│
│    Topic Subscriber                     │
└─────────────────────────────────────────┘
         ↓                    ↑
    ┌────────────┐    ┌──────────────┐
    │ vision_pick│    │ vision_place │
    │   node     │    │    node      │
    └────────────┘    └──────────────┘
```

## 동작 흐름

### 1. Pick-Place 자동 연속 수행

```bash
# Terminal 1: ROS2 노드 시작
ros2 launch jetcobot_vision jetcobot_vision.launch.py

# Terminal 2: Coordinator에서 Pick 시작 요청 (현재는 수동)
# - coordinator가 이미 실행 중이고 대기 중

# Terminal 3: Pick 진행
python3 src/jetcobot_vision/scripts/test_vision_pick.py tray

# Pick 완료 후 Place 신호 전송
ros2 topic pub /pick_place_coordinator/place_command \
  jetcobot_vision_msgs/PlaceCommand \
  "location: 'pinky_tray_place'"
```

### 2. 협조(Coordinator) 노드 사용

`pick_place_coordinator_node`는 두 가지 역할:

#### a. Pick Action 클라이언트
- `/vision_pick` action 호출 가능
- 추후 `start_pick()` 메서드 공개 예정

#### b. PlaceCommand 토픽 구독
- `/pick_place_coordinator/place_command` 토픽 구독
- Pick 완료 상태일 때만 Place 실행

## 메시지 형식

### PlaceCommand
```yaml
location: str      # Place 위치 (예: "pinky_tray_place")
box_index: int32   # 상자 인덱스 (-1 = 최고 confidence)
```

## 사용 예시

### 예시 1: 단순 Place 신호

```bash
# tray에서 pick, pinky_tray_place에 place
ros2 topic pub /pick_place_coordinator/place_command \
  jetcobot_vision_msgs/PlaceCommand \
  "location: 'pinky_tray_place'
   box_index: -1"
```

### 예시 2: 특정 상자 선택

```bash
# 색상 위치 index 1에 place
ros2 topic pub /pick_place_coordinator/place_command \
  jetcobot_vision_msgs/PlaceCommand \
  "location: 'pinky_tray_place'
   box_index: 1"
```

## 로그 확인

Coordinator 노드의 진행 상황 확인:

```bash
# Pick 완료 메시지
[INFO] [Pick 완료] success=true

# Place 대기 중
[INFO] Pick 완료 → PlaceCommand 대기 중...

# Place 신호 수신
[INFO] PlaceCommand 수신: location=pinky_tray_place box_index=-1

# Place 실행 중
[INFO] Place Goal 송신: location=pinky_tray_place box_index=-1
[INFO]   [Place moving] 10%
[INFO]   [Place detecting] 40%
...
[INFO] [Place 완료] success=true
```

## 향후 개선

- [ ] Coordinator에 `/start_pick` service 추가 (Python/shell에서 pick 시작 가능)
- [ ] Pick-Place 연속 loop 지원 (자동 반복)
- [ ] 타임아웃 처리 (Place 신호 없으면 자동 cancel)
- [ ] 상태 조회 service (`/status`)

## 파일

- `pick_place_coordinator_node.py` — 메인 노드
- `PlaceCommand.msg` — 메시지 정의
- `jetcobot_vision.launch.py` — Coordinator 포함 launch 파일
