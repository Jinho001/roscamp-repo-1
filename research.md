# Research: vision_pick 실행 위치 및 아키텍처 분석

## 문제 진단

### 현재 아키텍처의 구조적 문제
vision_pick.py의 핵심 로직은 다음을 요구한다:
1. `mc.get_coords()` — 로봇의 현재 자세 읽기 (serial: /dev/ttyJETCOBOT)
2. `mc.send_coords()` — 로봇에 이동 명령 전달
3. `mc.set_gripper_value()` — 그리퍼 제어

→ 이 세 가지 모두 **로봇 serial 포트에 직접 접근**해야 하며,
  serial 포트(/dev/ttyJETCOBOT)는 **제어 PC(젯슨)**에만 연결되어 있다.

결론: **vision_pick.py는 반드시 제어 PC에서 실행되어야 한다.**

### coord_transform의 mc 의존성
- `get_object_coords_in_base(mc, cx, cy, theta, ...)` 는
  내부에서 `mc.get_coords()` 를 호출하여 현재 EE 자세를 읽는다.
- 이 역시 serial 포트가 필요하므로 **제어 PC에서만 실행 가능**하다.

### 현재 상대 임포트 문제
- vision_pick.py는 `from .coord_transform import ...` 와 같이
  **상대 임포트**를 사용하고 있어, 단독 실행이 불가능하다.
- 해결: 파일 내부에서 sys.path를 조작하거나, -m 모드로 실행해야 한다.

## 올바른 실전 아키텍처

```
[제어 PC (젯슨)]                    [메인 PC]
카메라 (/dev/jetcocam0)             cv_detect_server.py (포트 8081)
로봇 (/dev/ttyJETCOBOT)                  │
        │                                 │
   vision_pick.py ──── HTTP POST ────────▶│
   1. 카메라에서 프레임 캡처               │ 상자 검출
   2. 메인PC 서버로 이미지 전송 ◀── (cx,cy,theta) 반환
   3. coord_transform (mc.get_coords)      │
   4. mc.send_coords → 로봇 이동
   5. mc.set_gripper_value → 그리퍼
```

## 해결해야 할 과제

1. **상대 임포트 문제**: 제어 PC에서 `vision_pick.py`를 단독 실행할 수 있어야 함
   - 방법 A: `sys.path`를 동적으로 추가
   - 방법 B: `python3 -m src.devices.jetcobot.vision.vision_pick` 으로 실행

2. **서버 URL 설정**: 메인 PC의 IP와 포트(8081)를 가리켜야 함
   - 현재: `http://localhost:8081/detect` (기본값)
   - 변경 필요: `http://192.168.1.11:8081/detect` (메인 PC IP)

3. **카메라**: 제어 PC의 로컬 카메라를 사용 (--remote 불필요)
   - 기본값 `/dev/jetcocam0` 그대로 사용

4. **레포 동기화**: 제어 PC에도 동일한 코드가 있어야 함
   - git pull 또는 rsync로 최신 코드 동기화 필요

## 실행 방법 (결론)

제어 PC에서:
```bash
# 프로젝트 루트에서 실행 (패키지 모드)
cd ~/roscamp-repo-1
python3 -m src.devices.jetcobot.vision.vision_pick \
  --server-url http://192.168.1.11:8081/detect \
  --location tray \
  --trials 1
```

메인 PC에서:
```bash
# 검출 서버만 실행
python3 src/devices/jetcobot/vision/cv_detect_server.py \
  --hsv-lower 0 0 203 --hsv-upper 179 37 255 --port 8081
```
