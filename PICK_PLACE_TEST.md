# Pick & Place 통합 테스트

실제 로봇으로 pick과 place를 연속 수행하는 통합 테스트.

## 준비 사항

### 1. 메인 PC (192.168.1.4)

**cv_detect_server.py 실행:**
```bash
cd /home/addinedu/roscamp-repo-repo-1
python3 src/devices/jetcobot/vision/cv_detect_server.py
```

**stream_sender.py 실행 (제어 PC에서):**
```bash
python3 src/devices/jetcobot/vision/stream_sender.py --host 192.168.1.4
```

### 2. 제어 PC (Raspberry Pi)

**ROS2 노드 시작:**
```bash
cd Jino/roscamp-repo-1
source install/setup.bash
ros2 launch jetcobot_vision jetcobot_vision.launch.py
```

이 명령으로 다음 노드들이 시작됨:
- coord_transform_node
- vision_pick_place_node (pick + place 통합)
- pick_place_coordinator_node

## 테스트 시나리오

### 시나리오 1: Pick 테스트 (tray)

**목표:** Tray에서 상자 픽업

```bash
# Terminal: 제어 PC
python3 src/jetcobot_vision/scripts/test_vision_pick.py tray
```

**예상 결과:**
```
[OK] /vision_pick action server 연결됨
Goal 송신: location=tray (최고 confidence)
Goal 수락됨
  [moving] 10%
  [detecting] 40%
  [transforming] 70%
  [picking] 90%
  [done] 100%
[RESULT]
  success: true
  message: location=tray 픽업 완료
```

### 시나리오 2: Pick 테스트 (receiving_zone)

**목표:** 회수존에서 상자 픽업 (보정값 적용 확인)

```bash
python3 src/jetcobot_vision/scripts/test_vision_pick.py receiving_zone
```

**예상 결과:**
- 보정값 적용 로그: `[Pick] 보정값 적용: dx=-17.6 dy=-9.9 dz=0.0 mm`
- Pick 성공

### 시나리오 3: Place 테스트 (pinky_tray_place)

**목표:** 핑키 트레이의 색상 마커에 place

```bash
python3 src/jetcobot_vision/scripts/test_vision_place.py pinky_tray_place
```

**예상 결과:**
```
[OK] /vision_place action server 연결됨
Goal 송신: location=pinky_tray_place
Goal 수락됨
  [moving] 10%
  [detecting] 40%
  [placing] 70%
  [done] 100%
[RESULT]
  success: true
  message: location=pinky_tray_place place 완료
```

### 시나리오 4: Pick → Place 연속 (Coordinator 사용)

**목표:** Pick 완료 후 자동으로 Place 신호를 받아 place 수행

```bash
# Terminal 1: ROS2 노드 시작 (coordinator 포함)
ros2 launch jetcobot_vision jetcobot_vision.launch.py

# Terminal 2: Pick 시작
python3 src/jetcobot_vision/scripts/test_vision_pick.py tray

# Terminal 3: Pick 완료 후 Place 신호 전송
# (pick 완료 로그를 확인한 후 실행)
ros2 topic pub /pick_place_coordinator/place_command \
  jetcobot_vision_msgs/PlaceCommand \
  "location: 'pinky_tray_place'
   box_index: -1"
```

**예상 흐름:**
1. [Pick] 실행 시작 location=tray
2. [Pick] 픽업 완료 (상자 집음)
3. [Place] Goal 송신 location=pinky_tray_place
4. [Place] place 좌표 감지
5. [Place] place 완료 (상자 놓음)

### 시나리오 5: 동시 실행 방지 테스트

**목표:** Pick 실행 중 Place 요청 → 거절 확인

```bash
# Terminal 1: ROS2 노드
ros2 launch jetcobot_vision jetcobot_vision.launch.py

# Terminal 2: Pick 시작
python3 src/jetcobot_vision/scripts/test_vision_pick.py tray

# Terminal 3: Pick 진행 중 (detecting 단계 등) Place 요청
python3 src/jetcobot_vision/scripts/test_vision_place.py pinky_tray_place
```

**예상 결과:**
```
[RESULT]
  success: false
  message: 다른 동작(Pick/Place) 진행 중. 기다려주세요.
```

## 테스트 체크리스트

- [ ] cv_detect_server 정상 작동 (포트 8000)
- [ ] stream_sender UDP 프레임 전송 확인 (포트 5000)
- [ ] coord_transform_node Static TF 발행 확인
- [ ] vision_pick_place_node 시작 성공
- [ ] pick_place_coordinator_node 시작 성공
- [ ] **시나리오 1:** tray pick 성공
- [ ] **시나리오 2:** receiving_zone pick 성공 (보정값 확인)
- [ ] **시나리오 3:** pinky_tray_place place 성공
- [ ] **시나리오 4:** Pick → Place 연속 수행 성공
- [ ] **시나리오 5:** 동시 실행 방지 정상 작동

## 로그 확인 포인트

### Pick 성공 로그
```
[vision_pick_place_node] [Pick] 실행 시작  location=tray  box_index=-1
[vision_pick_place_node] 선택된 상자: index=0 conf=0.94
[vision_pick_place_node] [Pick] 픽업 좌표: x=162.6 y=4.1 z=50.0 mm  yaw=-2.6 deg
[vision_pick_place_node] [Pick] 보정값 적용: dx=0.0 dy=0.0 dz=0.0 mm  (tray는 보정 없음)
```

### Receiving Zone Pick (보정값 적용)
```
[vision_pick_place_node] [Pick] 보정값 적용: dx=-17.6 dy=-9.9 dz=0.0 mm
```

### 동시 실행 방지
```
[vision_pick_place_node] [warn] 다른 동작(Pick/Place) 진행 중. 기다려주세요.
```

## 트러블슈팅

### "검출 타임아웃"
→ cv_detect_server 실행 확인
→ stream_sender 실행 확인
→ HSV 파라미터 확인 (profile_tuner.py로 튜닝)

### Place 좌표 감지 실패
→ pinky_tray_place의 observe_pose 확인
→ z_surface_mm 조정 (현재 70mm)
→ 색상 마커 명도 확인

### Pick 중 Place 거절 (정상)
→ 동시 실행 방지 메커니즘 작동 중
→ Pick 완료 후 Place 시도

## 추가 테스트 (선택사항)

### 여러 상자 선택 테스트
```bash
# 상자 index 0 선택
python3 src/jetcobot_vision/scripts/test_vision_pick.py tray --box 0

# 상자 index 1 선택
python3 src/jetcobot_vision/scripts/test_vision_pick.py tray --box 1
```

### 다양한 위치 테스트
- warehouse_pick (고정 좌표, 아직 미설정)
- warehouse_place (고정 좌표, 아직 미설정)

---

**테스트 날짜:** 2026-05-07
**상태:** 준비 완료 ✓
**다음 단계:** 실제 로봇 테스트 진행
