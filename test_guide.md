# Vision Pick-and-Place 테스트 지침서 (단계별 모드)

본 문서는 메인 PC와 제어 PC(로봇) 간의 연동 테스트를 위한 실행 순서 및 명령어를 정리합니다.

## 1. 사전 준비
- **메인 PC IP**: `192.168.1.4` (현재 서버 실행 환경)
- **로봇 포트**: `/dev/ttyJETCOBOT` (라즈베리파이 연결)
- **카메라 장치**: `/dev/jetcocam0` (라즈베리파이 연결)

---

## 2. 메인 PC (검출 서버) 실행
메인 PC에서 검출 서버를 먼저 실행합니다. `--show` 옵션을 사용하여 로봇이 보내는 화면을 실시간으로 확인합니다.

```bash
# 프로젝트 루트 디렉토리로 이동
cd ~/roscamp-repo-1

# OpenCV 기반 검출 서버 실행 (시각화 모드)
python3 src/devices/jetcobot/vision/cv_detect_server.py --port 8081 --show
```
*   **확인 사항**: 실행 후 `latest_result.jpg` 창이 뜨는지 확인합니다.

---

## 3. 제어 PC (로봇/라즈베리파이) 실행
로봇에서는 단계별(Step) 모드를 활성화하여 안전하게 1회 테스트를 진행합니다.

```bash
# 프로젝트 루트 디렉토리로 이동
cd ~/Jino/roscamp-repo-1

# 비전 픽업 실행 (단계별 모드 + 메인 PC 서버 연결)
python3 src/devices/jetcobot/vision/vision_pick.py \
  --server-url http://192.168.1.4:8081/detect \
  --location tray \
  --trials 1 \
  --step
```

---

## 4. 테스트 단계별 확인 포인트 (Step 모드)

| 단계 | 확인 사항 |
|---|---|
| **[1/4] 관측 자세** | 로봇이 트레이를 정면으로 바라보는 위치로 가는지 확인. |
| **[2/4] 캡처 & 검출** | 메인 PC 화면에 로봇이 찍은 사진이 뜨고, 상자에 초록색 박스가 쳐지는지 확인. |
| **[3/4] 픽업 이동** | 출력된 베이스 좌표(`x, y, z`)가 실제 물체 위로 가는지 확인 (Pre-pick 고도 확인). |
| **[4/4] 파지** | 그리퍼가 물체를 정확히 잡는지 확인. (중단하고 싶으면 `q` 입력) |

## 5. 문제 해결 (Troubleshooting)
- **Connection Refused**: 메인 PC의 서버(`cv_detect_server.py`)가 실행 중인지, 포트 번호(`8081`)가 일치하는지 확인하세요.
- **Camera Capture Fail**: `/dev/video0`를 다른 프로세스(예: `stream_sender.py`)가 사용 중인지 확인하세요. (`sudo fuser -k /dev/video0`로 정리 가능)
- **좌표 오차**: `workspace_config.yaml`의 `z_fixed` 및 `tcp_offset` 값이 실제 환경과 맞는지 확인하세요.
