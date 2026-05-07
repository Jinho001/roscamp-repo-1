# Vision Pipeline 테스트 스크립트

Location별 파라미터 조정 및 vision pick/place 액션 테스트용 유틸리티.

## 스크립트

### 1. `test_cv_config.py` — CV Detect Server 파라미터 테스트

cv_detect_server의 `/config` 엔드포인트로 HSV/W/H 파라미터를 전송하고 응답 확인.

**사용법:**
```bash
# tray location 파라미터 전송
python3 test_cv_config.py tray

# receiving_zone 파라미터 전송
python3 test_cv_config.py receiving_zone

# 사용 가능한 location 목록
python3 test_cv_config.py --list

# 커스텀 서버 주소
python3 test_cv_config.py tray --server http://192.168.1.4:8000
```

**출력:**
```
[LOCATION] tray
[SERVER] http://192.168.1.4:8000
[PROFILE]
  observe_pose: [-62.4, -87.9, 314.2, -161.81, 0.94, -179.37]
  z_surface_mm: 95.0
  hsv_lower: [0, 0, 208]
  hsv_upper: [158, 30, 255]
  min_w: 110
  max_w: 200
  min_h: 110
  max_h: 200

[POST] http://192.168.1.4:8000/config
[BODY] {
  "hsv_lower": [0, 0, 208],
  ...
}
[OK] {"ok": true}
```

### 2. `test_vision_pick.py` — VisionPick Action 테스트

vision_pick_node의 `/vision_pick` 액션을 호출하고 피드백/결과 모니터링.

**사용법:**
```bash
# tray 위치에서 pick
python3 test_vision_pick.py tray

# 30초 타임아웃
python3 test_vision_pick.py tray --timeout 30

# receiving_zone 위치에서 pick
python3 test_vision_pick.py receiving_zone
```

**출력:**
```
[OK] /vision_pick action server 연결됨
Goal 송신: location=tray
Goal 수락됨
  [moving] 10%
  [detecting] 40%
  [transforming] 70%
  [picking] 90%
  [done] 100%
[RESULT]
  success: true
  message: location=tray pick 완료
  pick_point_base: (0.123, 0.456, 0.789)
```

### 3. `test_topics.sh` — ROS2 토픽 모니터링

ObbBoxArray, PickPoint 등 토픽 구독.

**사용법:**
```bash
# ObbBoxArray 토픽 모니터링 (메인 PC → 제어 PC)
./test_topics.sh obb

# PickPoint 토픽 모니터링 (coord_transform_node 발행)
./test_topics.sh pick

# 모든 토픽 목록
./test_topics.sh all
```

## 테스트 워크플로우

### 1단계: 메인 PC에서 cv_detect_server 시작
```bash
# 메인 PC (192.168.1.4)
cd /home/addinedu/roscamp-repo-1
python3 src/devices/jetcobot/vision/cv_detect_server.py
```

### 2단계: 제어 PC에서 ROS2 노드 시작
```bash
# 제어 PC (Raspberry Pi)
cd Jino/roscamp-repo-1
colcon build --packages-select jetcobot_vision
source install/setup.bash

# Terminal 1: stream_sender
python3 src/devices/jetcobot/vision/stream_sender.py --host 192.168.1.4

# Terminal 2: ROS2 launch
ros2 launch jetcobot_vision jetcobot_vision.launch.py
```

### 3단계: 파라미터 테스트
```bash
# Terminal 3: cv_detect_server 파라미터 테스트
cd src/jetcobot_vision
python3 scripts/test_cv_config.py tray
python3 scripts/test_cv_config.py receiving_zone
```

### 4단계: 토픽 모니터링
```bash
# Terminal 4: ObbBoxArray 수신 확인
./scripts/test_topics.sh obb

# Terminal 5: PickPoint 수신 확인
./scripts/test_topics.sh pick
```

### 5단계: Action 테스트 (실제 로봇 없이)
```bash
# Terminal 6: VisionPick Action 호출
python3 scripts/test_vision_pick.py tray --timeout 30
```

## 트러블슈팅

### "cv_detect_server 연결 실패"
```
[ERR] HTTPConnectionError: ('Unable to connect to http://192.168.1.4:8000')
```
→ 메인 PC에서 cv_detect_server가 실행 중인지 확인
→ 네트워크 연결 확인 (ping 192.168.1.4)
→ 포트 8000이 방화벽에 차단되지 않았는지 확인

### "ObbBoxArray 토픽 없음"
```
  (없음)
```
→ cv_detect_server가 실행 중인지 확인
→ stream_sender.py가 UDP 프레임을 보내고 있는지 확인
→ `ros2 topic list -t | grep Obb` 로 확인

### "Action server 미응답"
```
[ERR] /vision_pick action server 미응답
```
→ launch 파일로 vision_pick_node가 실행됐는지 확인
→ `ros2 node list` 로 확인: `/vision_pick_node`
→ `ros2 action list` 로 확인: `/vision_pick`

## 파라미터 조정

각 location의 파라미터는 `config/pick_place_profiles.yaml`에 정의됨:

```yaml
tray:
  observe_pose: [-62.4, -87.9, 314.2, -161.81, 0.94, -179.37]
  z_surface_mm: 95.0
  hsv_lower: [0, 0, 208]        # H, S, V 하한
  hsv_upper: [158, 30, 255]     # H, S, V 상한
  min_w: 110
  max_w: 200
  min_h: 110
  max_h: 200
  ...
```

`test_cv_config.py`로 파라미터를 빠르게 테스트하고, 
원하는 값을 YAML에 기록하면 된다.

## 예시: tray 위치 HSV 미세조정

1. YAML에서 현재 값 확인
   ```bash
   python3 scripts/test_cv_config.py tray
   ```

2. 값 조정 (pick_place_profiles.yaml)
   ```yaml
   tray:
     hsv_lower: [0, 0, 210]  # V 하한 208 → 210
     hsv_upper: [158, 30, 255]
   ```

3. 다시 전송해서 확인
   ```bash
   python3 scripts/test_cv_config.py tray
   ```

4. 원하는 결과 확인 후 git commit
