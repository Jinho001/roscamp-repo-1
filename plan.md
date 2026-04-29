# Plan: vision_pick 실행 위치 재구성 (제어 PC 직접 실행)

## 목표
`vision_pick.py`를 제어 PC에서 단독으로 실행 가능하게 만들고,
메인 PC의 검출 서버(cv_detect_server.py)와 HTTP로 통신하는 구조를 완성한다.

---

## 최종 역할 분담

| 구분 | 메인 PC (192.168.1.11) | 제어 PC (192.168.1.114, 젯슨) |
|---|---|---|
| **실행 파일** | `cv_detect_server.py` | `vision_pick.py` |
| **역할** | OpenCV 상자 검출 서버 | 카메라 촬영 → 검출 요청 → 로봇 제어 |
| **입출력** | 이미지 수신 → (cx,cy,theta) 반환 | 검출 결과 수신 → 좌표 변환 → 로봇 이동 |
| **하드웨어** | 없음 (순수 연산) | 카메라 + 로봇 serial |

---

## PHASE 1: 상대 임포트 문제 해결

### 1-1. vision_pick.py 수정
- 파일 상단에 `sys.path` 동적 조정 코드를 추가하여 **상대 임포트 없이도 실행 가능**하게 변경
- `.` 상대 임포트를 절대 임포트로 교체

**변경 대상 임포트 (4줄):**
```python
# Before (상대 임포트 - 단독 실행 불가)
from .coord_transform    import get_object_coords_in_base, load_workspace_config
from .obb_detect_client  import detect_object
from .remote_capture     import RemoteCapture
from .handeye_calibration import RemoteRobot

# After (절대 임포트 - 단독 실행 가능)
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
from src.devices.jetcobot.vision.coord_transform    import get_object_coords_in_base, load_workspace_config
from src.devices.jetcobot.vision.obb_detect_client  import detect_object
from src.devices.jetcobot.vision.remote_capture     import RemoteCapture
from src.devices.jetcobot.vision.handeye_calibration import RemoteRobot
```

### 1-2. 동일 패턴 파일들도 함께 수정
`coord_transform.py`도 단독 실행 시 `from .handeye_calibration import RemoteRobot` 때문에 실패함. 동일한 방식으로 수정.

---

## PHASE 2: 기본 서버 URL 변경

### 2-1. vision_pick.py 기본값 수정
```python
# Before
server_url: str = "http://localhost:8081/detect"

# After
server_url: str = "http://192.168.1.11:8081/detect"  # 메인 PC IP
```

---

## PHASE 3: 제어 PC 코드 동기화

### 3-1. 제어 PC에서 git pull
```bash
# 제어 PC 터미널에서
cd ~/roscamp-repo-1
git pull
```

---

## 실행 가이드 (최종)

### 메인 PC 터미널 (1개)
```bash
python3 src/devices/jetcobot/vision/cv_detect_server.py \
  --hsv-lower 0 0 203 --hsv-upper 179 37 255 --port 8081
```

### 제어 PC 터미널 (1개)
```bash
cd ~/roscamp-repo-1
python3 src/devices/jetcobot/vision/vision_pick.py \
  --location tray --trials 1
```
> 서버 URL 기본값이 `http://192.168.1.11:8081`이므로 별도 인자 불필요

---

## 변경 범위 요약

| 파일 | 변경 내용 |
|---|---|
| `vision_pick.py` | 상대 임포트 → 절대 임포트 + sys.path 추가, 기본 server_url 변경 |
| `coord_transform.py` | 상대 임포트 → 절대 임포트 + sys.path 추가 |
| `cv_detect_server.py` | 변경 없음 |
| `obb_detect_client.py` | 변경 없음 |

---

## 체크리스트
- [x] `vision_pick.py` 임포트 수정
- [x] `coord_transform.py` 임포트 수정
- [x] 기본 server_url 메인 PC IP로 변경 (192.168.1.11:8081)
- [ ] 제어 PC에서 git pull
- [ ] 제어 PC에서 단독 실행 테스트
- [ ] 실제 픽업 동작 검증
