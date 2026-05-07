# 문서 색인 (Documentation Index)

Vision Pick & Place 프로젝트의 모든 문서 및 저장 경로.

---

## 📚 주요 문서

### 프로젝트 개요

| 문서 | 경로 | 목적 | 대상 |
|------|------|------|------|
| **PROGRESS_SUMMARY.md** | `/home/addinedu/roscamp-repo-1/` | 전체 프로젝트 진행 현황 | 전체 팀 |
| **CLAUDE.md** | `/home/addinedu/roscamp-repo-1/` | 프로젝트 개발 가이드라인 | 개발자 |

### 테스트 & 운영

| 문서 | 경로 | 목적 | 대상 |
|------|------|------|------|
| **PICK_PLACE_TEST.md** | Git 커밋: `8225d95` | 실제 로봇 테스트 시나리오 (5가지) | 테스터 |
| **PICK_PLACE_COORDINATOR.md** | Git 커밋: `bfb597e` | Coordinator 노드 상세 설명 | 개발자 |
| **RECEIVING_ZONE_DIAGNOSIS.md** | Git 커밋: `ed03f8c 이전` | 좌표 오차 진단 가이드 | 디버거 |
| **TUNING_GUIDE.md** | Git 커밋: `6aba0ed 이전` | 파라미터 튜닝 방법 | 작업자 |

---

## 💾 메모리 (자동 로드)

### Claude Memory System
위치: `/home/addinedu/.claude/projects/-home-addinedu-roscamp-repo-1/memory/`

| 파일 | 설명 | 자동로드 |
|------|------|---------|
| **MEMORY.md** | 메모리 인덱스 | ✅ 항상 |
| **project_vision_pick_place_status.md** | Pick/Place 프로젝트 상태 | ✅ 항상 |
| **project_ros2_migration.md** | ROS2 마이그레이션 진행상황 | ✅ 항상 |
| **user_profile.md** | 사용자 역할 및 선호도 | ✅ 항상 |
| **project_cv_detect_server_location.md** | cv_detect_server 위치 이동 항목 | ✅ 항상 |

---

## 📂 코드 파일 매핑

### 핵심 노드

| 파일 | 경로 | 역할 |
|------|------|------|
| **vision_pick_place_node.py** | `src/jetcobot_vision/jetcobot_vision/` | Pick/Place 액션 (통합) |
| **coord_transform_node.py** | `src/jetcobot_vision/jetcobot_vision/` | 좌표 변환 |
| **pick_place_coordinator_node.py** | `src/jetcobot_vision/jetcobot_vision/` | Pick-Place 조율 |

### 설정 파일

| 파일 | 경로 | 용도 |
|------|------|------|
| **pick_place_profiles.yaml** | `src/jetcobot_vision/config/` | Location별 파라미터 |
| **vision_params.yaml** | `src/jetcobot_vision/config/` | 노드 파라미터 |

### 테스트 스크립트

| 파일 | 경로 | 용도 |
|------|------|------|
| **test_vision_pick.py** | `src/jetcobot_vision/scripts/` | Pick 액션 테스트 |
| **test_vision_place.py** | `src/jetcobot_vision/scripts/` | Place 액션 테스트 |
| **test_pick_place.py** | `src/jetcobot_vision/scripts/` | Coordinator 테스트 |

---

## 🔍 문서 검색 방법

### 로컬에서 문서 찾기

```bash
# 프로젝트 루트 문서
cd /home/addinedu/roscamp-repo-1
ls -la *.md

# Git 히스토리에서 찾기
git log --oneline --all | grep -i "pick\|place\|test"

# 특정 커밋의 파일 확인
git show 8225d95:PICK_PLACE_TEST.md

# 메모리 파일
ls -la ~/.claude/projects/-home-addinedu-roscamp-repo-1/memory/
```

### Git 커밋 기준

| 기능 | 커밋 | 문서 |
|------|------|------|
| Coordinator 추가 | `bfb597e` | PICK_PLACE_COORDINATOR.md |
| 테스트 가이드 | `8225d95` | PICK_PLACE_TEST.md |
| 파라미터 튜닝 | `9d510a7` | (PICK_PLACE_TEST.md 참조) |
| 동시 실행 방지 | `8c95cc7` | (PROGRESS_SUMMARY.md 참조) |

---

## 📖 읽는 순서 (추천)

### 👤 새로운 팀원

1. **PROGRESS_SUMMARY.md** (프로젝트 전체 이해)
2. **project_vision_pick_place_status.md** (현재 상태 파악)
3. **PICK_PLACE_TEST.md** (테스트 방법 학습)

### 🔧 개발자

1. **CLAUDE.md** (개발 가이드)
2. **PROGRESS_SUMMARY.md** (아키텍처 이해)
3. **PICK_PLACE_COORDINATOR.md** (Coordinator 설계)
4. **코드 읽기**: vision_pick_place_node.py → coord_transform_node.py

### 🧪 테스터

1. **PICK_PLACE_TEST.md** (테스트 시나리오)
2. **PROGRESS_SUMMARY.md** (기술 이해)
3. **실제 로봇 테스트 실행**

### 🔨 작업자 (파라미터 튜닝)

1. **TUNING_GUIDE.md** (기존 문서, git에서)
2. **pick_place_profiles.yaml** (현재 파라미터)
3. **profile_tuner.py** (실시간 튜닝)

---

## 🎯 빠른 접근

### "전체 상황이 뭐지?"
→ `PROGRESS_SUMMARY.md` 읽기

### "실제로 테스트는 어떻게?"
→ `PICK_PLACE_TEST.md` (Git 8225d95)

### "코디네이터는 어떻게 작동?"
→ `PICK_PLACE_COORDINATOR.md` (Git bfb597e)

### "파라미터 어떻게 설정?"
→ `TUNING_GUIDE.md` (Git 6aba0ed 이전) 또는 `pick_place_profiles.yaml`

### "로봇이 왜 안 움직이지?"
→ `RECEIVING_ZONE_DIAGNOSIS.md` (진단 가이드)

### "다음 세션에 뭘 해야 되지?"
→ Memory 자동 로드 (project_vision_pick_place_status.md)

---

## 📊 문서 상태

| 문서 | 상태 | 최신 업데이트 |
|------|------|-------------|
| PROGRESS_SUMMARY.md | ✅ 활성 | 2026-05-07 |
| PICK_PLACE_TEST.md | ✅ Git 히스토리 | 2026-05-07 (커밋) |
| PICK_PLACE_COORDINATOR.md | ✅ Git 히스토리 | 2026-05-07 (커밋) |
| TUNING_GUIDE.md | ✅ Git 히스토리 | 2026-04-30 (추정) |
| Memory 파일들 | ✅ 활성 | 2026-05-07 |

---

## 🔗 관련 파일

- **소스코드**: `src/jetcobot_vision/`
- **메시지 정의**: `src/jetcobot_vision_msgs/`
- **설정**: `src/jetcobot_vision/config/`
- **Launch**: `src/jetcobot_vision/launch/`

---

**마지막 업데이트**: 2026-05-07 23:45 KST
