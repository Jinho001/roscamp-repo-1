# Jetcobot 실측 및 파라미터 튜닝 상세 가이드

Phase 1(캘리브레이션)은 생략하며, Phase 2부터 시작하여 `pick_place_profiles.yaml`을 완성하기 위한 PC별 명령어와 세부 절차입니다.

---

## Phase 2: 로봇 자세 티칭 및 Z축 높이 실측 (프리 드라이브 활용)

로봇의 모터 힘을 풀고 손으로 직접 움직여(프리 드라이브) 원하는 위치의 좌표를 따는 과정입니다.

### 💻 제어 PC (젯슨/라즈베리파이) 조작
1. 터미널을 열고 파이썬 대화형 쉘(REPL)을 실행합니다.
   ```bash
   python3
   ```
2. 로봇을 연결하고 모터 힘을 풉니다.
   ```python
   from pymycobot import MyCobot280
   mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
   mc.release_all_servos()
   ```

### 📋 세부 측정 절차
**[1] 관측 자세 (`observe_pose`) 측정**
1. 로봇을 손으로 잡고 카메라가 회수존(`receiving_zone`) 바닥을 직각으로 잘 내려다보게 만듭니다. (시야에 상자가 잘 들어오게)
2. 파이썬 쉘에 아래 명령어를 입력하여 좌표를 확인합니다.
   ```python
   mc.get_coords()
   ```
3. 출력된 6개의 숫자 `[x, y, z, rx, ry, rz]`를 `pick_place_profiles.yaml`의 `receiving_zone` -> `observe_pose`에 기입합니다.
4. 동일한 방법으로 핑키 적재함(`pinky_tray_place`)을 바라보는 관측 자세를 잡아 측정하고 기입합니다.

**[2] 바닥면 고도 (`z_surface_mm`) 측정**
1. 로봇 그리퍼 끝단이 회수존의 상자가 놓이는 **"맨바닥"**에 닿을락 말락 할 때까지 손으로 내립니다. (상자 위가 아닙니다!)
2. 파이썬 쉘에서 다시 좌표를 확인합니다.
   ```python
   mc.get_coords()
   ```
3. 출력된 값 중 3번째 값(Z)을 `receiving_zone` -> `z_surface_mm`에 기입합니다.
4. 동일하게 핑키 적재함 바닥면까지 내려서 Z값을 측정해 `pinky_tray_place` -> `z_surface_mm`에 기입합니다.

> 측정 완료 후 파이썬 쉘을 `exit()` 또는 `Ctrl+D`로 종료하여 포트를 닫아주세요.

---

## Phase 3: 비전 파라미터 튜닝 (HSV & 크기 필터)

카메라 영상을 메인 PC로 받아오며 색상과 크기 범위를 찾습니다.

### 🤖 제어 PC (젯슨/라즈베리파이)
카메라 영상을 메인 PC(예: `192.168.1.4`)로 송출합니다.
```bash
python3 src/devices/jetcobot/vision/stream_sender.py --host 192.168.1.4
```

### 🖥️ 메인 PC (관제 PC)
HSV 튜너를 실행하여 원격 영상을 받습니다.
```bash
python3 src/devices/jetcobot/vision/hsv_tuner.py --remote
```

### 📋 세부 튜닝 절차
**[1] 회수존 상자 (`receiving_zone`) 튜닝**
1. 실제 작업 환경과 동일한 조명 아래에 빨간 상자를 둡니다.
2. 튜너 창의 슬라이더를 조절하여 상자만 선명한 하얀색으로 보이게 만듭니다.
3. 결정된 H, S, V의 Lower/Upper 값을 YAML에 기입합니다.
4. 화면에 인식된 상자의 `W`와 `H` 픽셀 크기를 참고하여 약간의 여유를 두고 `min_w`, `max_w`, `min_h`, `max_h`를 YAML에 기입합니다.

**[2] 핑키 적재함 (`pinky_tray_place`) 형광 마커 튜닝**
1. 핑키 모바일 로봇을 가져다 두고 관측 자세에서 형광 스티커를 비춥니다.
2. 튜너를 조절하여 스티커 색상을 맞춥니다.
3. 스티커 크기에 맞게 가로세로(W, H) 필터를 아주 작게(예: `30~80`) 세팅하여 YAML에 기입합니다.

---

## Phase 4: 고정 위치 티칭 (Warehouse Fixed Pose)

창고 등 카메라 없이 티칭된 고정 좌표로만 움직이는 구역의 좌표를 땁니다.

### 💻 제어 PC (젯슨/라즈베리파이) 조작
Phase 2와 동일하게 파이썬 쉘을 열고 프리 드라이브를 켭니다.
```bash
python3
```
```python
from pymycobot import MyCobot280
mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)
mc.release_all_servos()
```

### 📋 세부 티칭 절차
**[1] 창고 픽업/적재 고정점 측정**
1. 창고 선반의 특정 상자 픽업 위치(`warehouse_pick`)로 로봇 팔을 정밀하게 가져갑니다.
2. `mc.get_coords()`를 입력해 6개의 축 값을 기록하고 YAML의 `fixed_coords`에 기입합니다.
3. 창고 적재 위치(`warehouse_place`)도 동일하게 수행하여 기록합니다.

> 모든 작업이 끝나면 작성된 `pick_place_profiles.yaml`을 저장하고, 다음 Phase(서버 코드와 YAML 연동)를 진행하면 됩니다.
