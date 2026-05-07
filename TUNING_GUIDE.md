# HSV/W,H 파라미터 튜닝 가이드

각 로봇 위치(location)에서 상자를 정확히 검출하기 위한 HSV 색상 범위와 W,H 필터 파라미터를 설정하는 방법.

## 개요

```
메인 PC (192.168.1.4)
├── cv_detect_server.py (포트 8000)      ← HTTP /config로 파라미터 수신
├── profile_tuner.py (포트 5000)         ← UDP 스트림 수신, 슬라이더로 조정

제어 PC (Raspi)
├── stream_sender.py --host 192.168.1.4  ← UDP 5000으로 영상 전송
├── coord_transform_node (ROS2)
└── vision_pick_node / vision_place_node (ROS2)
```

## 준비

### 1단계: 환경 설정

**메인 PC:**
```bash
cd /home/addinedu/roscamp-repo-1

# 필수 패키지
pip3 install opencv-python numpy pyyaml requests
```

**제어 PC:**
```bash
cd Jino/roscamp-repo-1
colcon build --packages-select jetcobot_vision_msgs jetcobot_vision
source install/setup.bash
```

### 2단계: 스트림 전송 시작

**제어 PC에서:**
```bash
python3 src/devices/jetcobot/vision/stream_sender.py --host 192.168.1.4
```

이 명령은 USB 카메라의 영상을 메인 PC의 포트 5000으로 UDP 스트림 전송.

### 3단계: Profile Tuner 실행

**메인 PC에서:**
```bash
python3 src/devices/jetcobot/vision/profile_tuner.py --location tray
```

3개 창이 열림:
- **Source**: 원본 영상 + 검출된 상자 경계 (녹색) + 중심 (노란 원)
- **Mask**: HSV 마스킹 결과 (흰색 = 검출됨)
- **Controls**: HSV/W,H 슬라이더

## 튜닝 방법

### 목표

마스크(Mask) 창에서 **상자만 흰색으로 남고** 배경, 노이즈는 검은색이 되도록 조정.

### 슬라이더

| 슬라이더 | 범위 | 설명 |
|---------|------|------|
| **H Lower** | 0–179 | 색상 범위 하한 (0=빨강, 60=초록, 120=파랑) |
| **S Lower** | 0–255 | 채도 범위 하한 (0=회색, 255=순색) |
| **V Lower** | 0–255 | 밝기 범위 하한 (0=검은색, 255=흰색) |
| **H Upper** | 0–179 | 색상 범위 상한 |
| **S Upper** | 0–255 | 채도 범위 상한 |
| **V Upper** | 0–255 | 밝기 범위 상한 |
| **Min W** | 0–640 | 상자 너비 최소값 (px) |
| **Max W** | 0–640 | 상자 너비 최대값 |
| **Min H** | 0–640 | 상자 높이 최소값 |
| **Max H** | 0–640 | 상자 높이 최대값 |

### 단계별 조정

#### 1. HSV 범위 조정

1. 마스크 창을 보면서 상자 영역이 흰색이 되도록 조정
   - V Lower를 먼저 낮춰서 상자가 모두 흰색이 되게
   - H/S Lower를 천천히 올려서 잡음 제거
   - V Upper를 낮춰서 밝기 상한 조정

2. 배경과 노이즈 제거
   - H Lower/Upper로 색상 범위 좁히기
   - S Lower로 채도 하한 올리기 (회색 제거)

3. 반복하며 미세조정

#### 2. W,H 필터 조정

1. Source 창에서 검출된 상자의 크기 확인
2. Min W/Max W로 너비 범위 설정
3. Min H/Max H로 높이 범위 설정

**예: 상자가 120×130 px일 때**
```
Min W: 100    (너비 최소)
Max W: 200    (너비 최대)
Min H: 100    (높이 최소)
Max H: 200    (높이 최대)
```

### 결과 확인

**좋은 튜닝:**
- Mask: 상자 영역만 흰색, 배경 검은색
- Source: 상자마다 녹색 경계 + 노란 중심점 표시
- Detections 수: 1~3 (화면에 보이는 상자 개수와 일치)

**나쁜 튜닝:**
- Mask: 배경도 흰색 (HSV 범위 너무 넓음)
- Mask: 상자가 검은색 (HSV 범위 너무 좁음)
- Source: 경계가 부자연스러움 (W/H 필터 잘못됨)

## 각 Location별 튜닝

### tray (핑키 적재함)

**특징:** 흰색 상자, 밝은 조명

**초기값:**
```yaml
hsv_lower: [0, 0, 208]
hsv_upper: [158, 30, 255]
min_w: 110
max_w: 200
min_h: 110
max_h: 200
```

**예상 조정:**
- V Lower: 200–220 (어두운 흰색 필터)
- S Upper: 30–50 (포화도 낮게, 회색 배경 제외)

### receiving_zone (회수존)

**특징:** 다양한 조명, 배경 노이즈 가능

**초기값:**
```yaml
hsv_lower: [0, 0, 208]
hsv_upper: [158, 30, 255]
min_w: 90
max_w: 220
min_h: 80
max_h: 220
```

**예상 조정:**
- H Lower/Upper: 색상 범위 좁혀서 배경 제외
- S Lower: 높여서 회색/금속 배경 제외

### pinky_tray_place (색상 스티커)

**특징:** 형광 녹색/노란색 스티커

**초기값:**
```yaml
hsv_lower: [20, 100, 100]  # 초록~노랑
hsv_upper: [40, 255, 255]
min_w: 30
max_w: 150
min_h: 30
max_h: 150
```

**예상 조정:**
- H Lower/Upper: 20–50 범위 (초록/노랑)
- S Lower: 100–150 (포화도 높게, 순색만)

## 저장 및 적용

### 저장하기

Profile Tuner에서 조정 후 **'p' 키**를 누르면:
1. 현재 값이 터미널에 출력
2. `pick_place_profiles.yaml`에 자동 저장

```
[SAVED] tray
  HSV: [0, 0, 210] ~ [158, 30, 255]
  W: 110~200, H: 110~200
```

### 실제 로봇 테스트

저장 후 제어 PC에서:

```bash
# 1. 파일 업데이트 (git pull 또는 직접 수정)
git pull origin claude/ros2-robot-migration

# 2. ROS2 노드 시작
ros2 launch jetcobot_vision jetcobot_vision.launch.py

# 3. Vision Pick Action 테스트
ros2 action send_goal /vision_pick jetcobot_vision_msgs/action/VisionPick \
  "location: 'tray'"
```

## 트러블슈팅

### "UDP 수신 중... (누적 0 패킷)"

→ stream_sender.py가 실행 중인지 확인
→ 네트워크 연결 확인: `ping 192.168.1.4`
→ 방화벽 5000 포트 차단 확인

### 마스크가 검은색만 나옴

→ HSV 범위 너무 좁음
→ V Lower를 50~100으로 낮춰서 재시도
→ S Upper를 255로 올려서 채도 상한 제거

### 마스크가 흰색만 나옴

→ HSV 범위 너무 넓음
→ H Lower/Upper를 좁혀서 색상 범위 제한
→ S Lower를 올려서 채도 하한 제한

### Source에서 상자 경계가 안 보임

→ W/H 필터가 너무 좁음 (min > actual size)
→ Min W/H를 작게, Max W/H를 크게 설정

### 여러 객체가 연합됨 (하나의 상자가 여러 박스로 분리됨)

→ 모폴로지 커널이 작음 (기본값 7)
→ HSV 범위에서 노이즈가 있는지 확인
→ Min Area를 올려서 작은 노이즈 제외

## 모범 사례

1. **한 번에 하나씩 조정**
   - 모든 슬라이더를 동시에 움직이지 말 것
   - HSV 먼저, 그 다음 W/H 필터

2. **배경과 조명 고려**
   - 여러 각도에서 촬영해보기
   - 조명 변화에 강건하게 (S Lower 활용)

3. **저장 전 확인**
   - 3~5프레임 연속으로 마스크가 깨끗한지 확인
   - 상자가 없을 때도 배경이 검은색인지 확인

4. **git으로 버전 관리**
   ```bash
   git add src/jetcobot_vision/config/pick_place_profiles.yaml
   git commit -m "Tune: tray location HSV/W,H parameters"
   git push
   ```

## 예시: tray 튜닝 과정

```
초기값:
  HSV: [0, 0, 208] ~ [158, 30, 255]
  W/H: 110~200 / 110~200

Step 1: V Lower 조정
  HSV: [0, 0, 210] ~ [158, 30, 255]
  → 상자가 더 명확하게 흰색

Step 2: S Upper 조정
  HSV: [0, 0, 210] ~ [158, 30, 255]  (변경 없음)
  → S Upper를 40으로 낮춤 (배경 회색 제외)

Step 3: W/H 미세조정
  W: 105~205, H: 105~205
  → 실제 크기: 110×130 → Min H를 100으로 낮춤

최종값:
  HSV: [0, 0, 210] ~ [158, 30, 255]
  W: 105~205, H: 100~210
  
→ 'p' 키로 저장 ✓
```

## 참고

- `src/devices/jetcobot/vision/cv_detect_server.py`: 검출 엔진
- `src/devices/jetcobot/vision/remote_capture.py`: UDP 수신
- `src/jetcobot_vision/config/pick_place_profiles.yaml`: 저장 파일
- `src/jetcobot_vision/jetcobot_vision/vision_pick_node.py`: 실제 액션 서버
