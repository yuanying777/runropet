# MOVE MY ROBOT 프로젝트 진행 상황 요약

## 프로젝트 개요
Naver Maps API를 활용한 Turtlebot 자동 경로 계획 및 제어 대시보드 시스템

## 주요 기능
1. **차선도로 모드**: Directions 15 API 사용 (곡선 최적화 경로)
2. **도보도로 모드**: 네이버 지도 크롤링 + 사용자 입력 횡단보도 방향
3. **경로 스케일링**: 실제 지도 경로를 2m x 3m (기본값) 교실 공간으로 축소
4. **실시간 대시보드**: Flask 기반 웹 인터페이스

## 최근 완료된 작업 (2024-11-26)

### 1. 직선 로직 제거 및 곡선 로직 통합 ✅
- **배경**: 곡선 최적화 경로가 직선 로직까지 모두 커버하므로 직선 로직 불필요
- **변경 사항**:
  - UI에서 직선/곡선 선택 버튼 제거 (곡선으로 고정)
  - `dashboard.py`: 곡선 최적화 경로만 사용하도록 수정
  - `app.js`: 경로 타입 선택 로직 제거
  - `move_full_path.py`: 직선 로직 분기 제거, 곡선 고정
  - `api_to_position.py` import 제거 (곡선 모듈만 사용)
- **효과**: 코드 단순화, 유지보수 용이, 사용자 경험 개선

## 이전 완료 작업 (2024-11-25)

### 1. SSH 연동 기능 통합 ✅
- **파일**: `dashboard/dashboard.py`, `dashboard/static/js/app.js`
- **기능**:
  - 브링업 상태 자동 확인 (`/api/check-status`)
  - 페이지 로드 시 & 입력 필드 변경 시 자동 상태 확인
  - 연결 성공 시 버튼 텍스트 자동 변경 ("로봇 연동" → "로봇 연동됨")
  - ssh_test.py 로직을 대시보드에 완전 통합
- **테스트 버튼**: 30cm 직진 테스트 기능 (STEP 03 "TEST" 버튼)

### 2. 회전 각도 부호 보정 ✅
- **파일**: `dashboard/dashboard.py` (generate_move_path_py 함수)
- **수정**: 다운로드되는 `move_path.py` 파일의 `rotate()` 함수에 각도 부호 반전 추가
- **효과**: 좌회전/우회전이 올바른 방향으로 실행됨
- **코드**: `angle_deg = -angle_deg  # 좌표계 보정`

### 3. 파일 경로 재정리 ✅
- **robot_connect.py**: move_path_9.py 경로 수정 (dashboard 폴더 구조 반영)
- **test_move_command.py**: 로봇 PC 경로 업데이트 (`~/robot_ws/src/move_turtle/`)
- **move_full_path.py**: 로봇 PC용 import 활성화, move_path_9 import 주석 처리

### 4. move_path_9.py 제거 준비 ✅
- **robot_connect.py**: `run_precomputed_move()` 함수 주석 처리
- **dashboard.py**: `/api/run-path` 엔드포인트 주석 처리
- **영향**: move_path_9.py 삭제 가능, 테스트 기능은 대시보드의 정규 경로 생성으로 대체

### 5. 이전 완료 작업 (유지)
- 곡선 최적화 경로 구현 (포인트 수 제한 20개, 최소 거리 0.1m)
- 속도/스케일 동기화
- API 재호출 방지 (좌표 파일 저장)
- 이미지 관리 최적화 (고정 파일명)

## 현재 파일 구조

### 핵심 파일 (VMware 이동 필수)
- `api_to_position_curve.py`: 곡선 최적화 경로 계획 (차선도로)
- `api_to_position.py`: Geocoding 유틸리티 (geocode 함수만 사용)
- `crawl_to_position.py`: 도보 경로 계획 (도보도로)
- `test_naver_map_crawl.py`: 네이버 지도 크롤링
- `move_full_path.py`: ROS2 노드, 실제 로봇 제어 (로봇 PC에 배포)
- `test_move_command.py`: 30cm 직진 테스트 명령 (로봇 PC에 배포)
- `dashboard/`: 전체 폴더 (dashboard.py, templates, static 포함)

### 데이터 파일
- `dashboard/current_route_coordinates.json`: 현재 경로 좌표 (API 재호출 방지용)
- `dashboard/static/routes/current_route_visualization.png`: 현재 경로 이미지 (한 장만 유지)
- `motion_settings.json`: 모션 설정 (속도, 가용면적 등) - 선택사항

### 삭제 가능 파일
- `walking_route_*.json`: 이전 크롤링 캐시 파일들
- `ssh_test.py`: SSH 테스트용 독립 파일 (기능이 대시보드에 통합됨)
- `move_path_9.py`: 사전 정의 경로 (사용 중단)
- `SSH_TEST_README.md`: 가이드 문서
- ~~`api_to_position.py`~~: 직선 로직 제거됨 (geocode 함수는 유지)

## 주요 함수 및 로직

### 곡선 최적화 (`api_to_position_curve.py`)
1. `scale_path()`: 위경도 좌표를 목표 공간(2m x 3m)으로 스케일링
2. `compute_angles()`: 세 점 간 회전각 계산
3. `compute_line_deviation()`: 직선으로부터의 편차 계산
4. `classify_segments()`: 직선/부드러운 곡선/급격한 곡선 구간 분류
5. `simplify_curve_path()`: 포인트 수 제한 (최대 20개) 및 최소 거리 보장 (0.1m)
6. `generate_spline_for_curves()`: 곡선 구간 Spline 보간
7. `generate_cmd_vel_sequence()`: Turtlebot 명령 생성 (linear, angular, duration 포함)
8. `get_robot_path_from_coordinates()`: 원본 경로 좌표로부터 명령 생성 (API 호출 없음)
9. **참고**: 직선 구간도 곡선 최적화 경로에서 자동으로 처리됨 (type: "straight")

### 대시보드 (`dashboard/dashboard.py`)
- `/api/plan-route`: 경로 생성 (차선도로)
- `/api/process-walking-route`: 도보 경로 처리
- `/api/connect`: 로봇 브링업 시작
- `/api/check-status`: 브링업 상태 확인 ⭐ NEW
- `/api/test-move`: 30cm 직진 테스트
- `/api/start-drive`: 경로 주행 시작
- `/api/stop-drive`: 경로 주행 강제 종료
- `/api/save-motion-settings`: 속도/스케일 설정 저장 및 재계산
- `/api/download-move-path`: `move_path.py` 파일 다운로드 (회전 각도 보정 적용)
- `recalculate_scale_factor()`: 스케일 팩터 재계산 (파일에서 좌표 읽기)
- `generate_move_path_py()`: 다운로드 파일 생성 (각도 부호 반전 포함)

## 현재 상태 및 확인 사항

### ✅ 완료된 기능
- [x] 곡선 최적화 경로 구현
- [x] 포인트 수 제한 (20개)
- [x] 최소 거리 보장 (0.1m)
- [x] 속도/스케일 동기화
- [x] API 재호출 방지 (파일 저장)
- [x] 이미지 관리 (한 장만 유지)
- [x] STEP 04 표시 개선
- [x] SSH 연동 기능 통합
- [x] 브링업 상태 자동 확인
- [x] 회전 각도 부호 보정
- [x] 파일 경로 재정리
- [x] 직선 로직 제거 및 곡선 로직 통합 ⭐ NEW

### 🚀 VMware 이동 준비 완료
- 불필요한 테스트 파일 제거 가능 상태
- 경로 구조 정리 완료
- 대시보드 통합 완료

### 📝 주의사항
- **회전 각도**: 다운로드되는 `move_path.py`에서 자동으로 부호 반전 적용됨
- **브링업 상태**: 로봇 IP/ID/PW 입력 시 자동으로 연동 상태 확인
- **테스트 버튼**: 로봇 연동 후 TEST 버튼으로 30cm 직진 테스트 가능
- **경로 타입**: 곡선 최적화 경로로 고정됨 (직선 구간도 자동으로 포함)

## 다음 작업 시 참고사항

### 테스트 체크리스트
1. 차선도로 → 곡선 최적화 경로 생성 → 포인트 수 확인 (20개 이하)
2. 스케일 세팅 변경 → API 재호출 없이 스케일 팩터 업데이트 확인
3. 속도 변경 → STEP 04의 duration 자동 업데이트 확인
4. 새 경로 생성 → 기존 이미지 자동 삭제 확인
5. `current_route_coordinates.json` 파일 생성 확인
6. UI에서 경로 타입 선택 버튼이 제거되었는지 확인
7. 곡선 최적화 경로가 직선 구간도 올바르게 처리하는지 확인

### 주요 설정값
- 기본 가용면적: 2m x 3m
- 기본 속도: 0.1 m/s (SCALE), 0.2 m/s (REAL)
- 최대 포인트 수: 20개
- 최소 포인트 간 거리: 0.1m (10cm)
- 곡선 분류 기준: 30° (부드러운 곡선 < 30°, 급격한 곡선 ≥ 30°)

### 파일 위치
- 경로 좌표 파일: `robot_ws/src/move_turtle/dashboard/current_route_coordinates.json`
- 경로 이미지: `robot_ws/src/move_turtle/dashboard/static/routes/current_route_visualization.png`
- 모션 설정: `robot_ws/src/move_turtle/motion_settings.json`

## 코드 변경 이력 (최근)

### 2024-11-26 업데이트

#### 직선 로직 제거 및 곡선 로직 통합
- **`dashboard/templates/index.html`**
  - 직선/곡선 경로 선택 버튼 제거 (line 61-72)
  
- **`dashboard/static/js/app.js`**
  - `straightPathBtn`, `curvePathBtn` 참조 제거
  - `currentPathType`을 const로 변경하고 "curve"로 고정
  - 경로 타입 버튼 클릭 이벤트 리스너 전체 제거 (line 1461-1507)
  
- **`dashboard/dashboard.py`**
  - `api_to_position` import에서 `get_robot_path_from_addresses` 제거
  - `CURVE_MODULE_AVAILABLE` 체크 제거 (곡선 모듈 필수)
  - `/api/plan-route`: path_type을 "curve"로 고정, 직선 로직 분기 제거
  - `recalculate_scale_factor()`: 직선 로직 분기 제거, 곡선 함수만 사용
  
- **`move_full_path.py`**
  - `from api_to_position import get_robot_path_from_addresses` 제거
  - `execute_path()`: path_type 기본값을 "curve"로 변경, 직선 로직 분기 제거
  - `--path-type` 인자: choices=['curve']로 제한, 기본값 "curve"
  
- **`PROJECT_SUMMARY.md`**
  - 주요 기능 섹션 업데이트 (직선/곡선 → 곡선 최적화 경로)
  - 파일 구조 섹션 업데이트 (`api_to_position.py`는 geocode만 사용)
  - 완료된 기능 리스트에 "직선 로직 제거" 추가
  - 주의사항 및 테스트 체크리스트 업데이트

### 2024-11-25 업데이트

#### `move_path.py` vs `move_full_path.py` 개요 (NEW)
- **move_path.py (다운로드 스크립트)**  
  - 사용자가 웹 대시보드에서 경로 생성 후 다운로드하는 독립 실행형 ROS2 스크립트  
  - `commands = [...]`가 파일 내에 포함되어 있어, 로컬/오프라인 환경에서 즉시 실행 가능  
  - 테스트나 공유용으로 설계, `python3 move_path.py`로 단독 실행

- **move_full_path.py (원격 실행 본체)**  
  - 대시보드에서 STEP 03/04 버튼으로 원격 로봇(SSH)에 실행시키는 메인 ROS2 노드  
  - start/goal 혹은 JSON 명령을 API에서 다시 계산하여 실시간으로 실행  
  - 속도/스케일 변경, JSON 업로드 등 모든 대시보드 기능과 연동되는 핵심 스크립트

#### `dashboard/dashboard.py`
- `/api/check-status` 엔드포인트 추가: 브링업 상태 확인
- `/api/run-path` 엔드포인트 주석 처리: move_path_9.py 삭제 대비
- `run_precomputed_move` import 주석 처리
- `generate_move_path_py()`: 회전 각도 부호 반전 로직 추가 (`angle_deg = -angle_deg`)
- `flatten_commands()`/`store_route_commands()` 추가: commands 중첩 리스트 평탄화 및 세션 저장 시 일관성 유지 (NEW)

#### `dashboard/static/js/app.js`
- `checkBringupStatus()` 함수 추가: 브링업 상태 자동 확인
- 페이지 로드 시 상태 확인 로직 추가
- 입력 필드 변경 시 debounce 적용한 상태 확인
- 연결 성공 시 버튼 텍스트 자동 변경 ("로봇 연동됨")

#### `dashboard/robot_connect.py`
- `move_path_9.py` 경로 수정: `parent.parent / "move_path_9.py"`
- `test_move_command.py` 경로 수정: `~/robot_ws/src/move_turtle/`
- `run_precomputed_move()` 함수 전체 주석 처리

#### `move_full_path.py`
- 로봇 PC용 import 활성화 (api_to_position, api_to_position_curve, test_naver_map_crawl)
- move_path_9 import 주석 처리 (로컬 PC 전용)

### 이전 업데이트 (2024-11-23)
- 곡선 최적화, 포인트 수 제한, API 재호출 방지 등 (상세 내역 생략)

## 다음 작업 시 시작점

현재 프로젝트는 **VMware 이동 준비 완료** 상태입니다.

### VMware 이동 체크리스트
1. ✅ 핵심 파일 목록 확인 완료
2. ✅ 불필요한 파일 식별 완료
3. ✅ 경로 구조 정리 완료
4. ✅ SSH 연동 기능 통합 완료
5. ✅ 회전 각도 보정 완료

### VMware 이동 후 작업
1. **환경 설정**: Naver API 키 설정 (환경변수)
2. **로봇 PC 파일 배포**: `move_full_path.py`, `test_move_command.py`
3. **대시보드 실행**: `cd dashboard && python3 dashboard.py`
4. **테스트**: 경로 생성 → 다운로드 → 로봇 실행 전체 플로우 확인

## 주의사항

- **API 키**: Naver Maps API 키는 환경변수로 관리 (`NAVER_CLIENT_ID`, `NAVER_CLIENT_SECRET`)
- **파일 경로**: WSL 환경에서 작업 중 (`/Ubuntu-22.04/home/yuanying/robot_ws/...`)
- **세션 관리**: 현재는 단일 유저 환경에 최적화됨 (다중 유저 환경은 로그인 기능 필요)

