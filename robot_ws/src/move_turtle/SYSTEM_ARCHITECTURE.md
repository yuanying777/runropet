# 시스템 아키텍처 문서

## 개요

이 문서는 로봇 경로 계획 및 실행 시스템의 전체 구조를 기술적으로 설명합니다.

---

## 1. 파일 위치 및 역할

### 1.1 로컬 PC (개발 환경)

#### 대시보드 관련
```
robot_ws/src/move_turtle/dashboard/
├── dashboard.py              # Flask 서버 (메인)
├── robot_connect.py          # SSH/SFTP 통신 모듈
├── templates/
│   └── index.html            # 프론트엔드 UI
└── static/
    ├── js/app.js             # JavaScript 클라이언트
    └── css/styles.css        # 스타일시트
```

#### 경로 생성 로직
```
robot_ws/src/move_turtle/
├── api_to_position.py        # 차선도로 직선 로직
├── api_to_position_curve.py  # 차선도로 곡선 로직
├── test_naver_map_crawl.py   # 도보도로 크롤링 로직
└── crawl_to_position.py      # 도보도로 경로 변환
```

#### 설정 및 데이터 파일
```
robot_ws/src/move_turtle/
├── motion_settings.json       # 모션 설정 (스케일, 속도 등)
└── dashboard/
    ├── current_route_coordinates.json      # 차선도로 경로 좌표 (원본)
    └── current_walking_route_data.json     # 도보도로 경로 데이터 (원본)
```

### 1.2 로봇 PC (실행 환경)

```
~/robot_ws/src/move_turtle/
├── move_full_path.py         # 메인 실행 스크립트 (ROS2 노드)
├── motion_settings.json      # 모션 설정 (로컬 PC와 동기화)
└── /tmp/
    └── walking_route.json    # 도보도로 JSON 파일 (SFTP로 업로드)
```

---

## 2. 경로 생성 로직별 구조

### 2.1 차선도로 - 직선 로직

**파일**: `api_to_position.py`

**주요 함수**:
- `get_robot_path_from_addresses()`: 주소 → 경로 생성
- `get_robot_path_from_coordinates()`: 좌표 → 경로 생성
- `extract_keypoints()`: Keypoint 추출
- `fit_path_to_area()`: 스케일링
- `calculate_robot_commands()`: 로봇 명령 생성

**출력 형식**:
```python
{
    "commands": [
        {
            "type": "straight",
            "linear": 0.1,
            "angular": 0.0,
            "distance": 0.5,
            "turn_angle": 90.0,
            "original_distance": 150.0,
            ...
        }
    ],
    "scale_factor": 0.003,
    "bbox": {"width": 0.001, "height": 0.001},
    ...
}
```

### 2.2 차선도로 - 곡선 로직

**파일**: `api_to_position_curve.py`

**주요 함수**:
- `get_robot_path_from_addresses()`: 주소 → 경로 생성 (곡선)
- `get_robot_path_from_coordinates()`: 좌표 → 경로 생성 (곡선)
- `generate_curve_commands()`: 곡선 명령 생성
- `is_straight_segment()`: 직선/곡선 판단 (임계값: 2.75m)
- `generate_cmd_vel_sequence()`: 최종 명령 생성

**출력 형식**:
```python
{
    "commands": [
        {
            "type": "straight" or "smooth_curve",  # 곡선→곡선만 smooth_curve
            "linear": 0.1,
            "angular": 0.0 or ±X,  # 곡선→곡선만 각속도 적용
            "distance": 0.5,
            "turn_angle": 90.0,
            "original_distance": 150.0,
            ...
        }
    ],
    ...
}
```

**특수 규칙**:
- **곡선→곡선만** `type: "smooth_curve"`, `angular ≠ 0`
- **나머지 전부** `type: "straight"`, `angular = 0`

### 2.3 도보도로 로직

**파일**: `test_naver_map_crawl.py`, `crawl_to_position.py`

**주요 함수**:
- `parse_route_actions()`: 네이버 지도 경로 텍스트 파싱
- `convert_json_to_robot_commands()`: JSON → 로봇 명령 변환

**입력 형식** (JSON):
```json
{
    "url": "https://map.naver.com/...",
    "actions": [
        {"type": "출발", "coords": [127.xxx, 37.xxx]},
        {"type": "횡단보도_좌회전", "coords": [127.xxx, 37.xxx]},
        ...
    ],
    "crosswalks": [...]
}
```

**출력 형식**: 차선도로와 동일한 commands 구조

---

## 3. 데이터 흐름

### 3.1 경로 생성 흐름

```
[사용자 입력]
    ↓
[대시보드 UI] (app.js)
    ↓ POST /api/plan-route
[dashboard.py]
    ↓ route_mode, path_type 확인
    ├─ "car" + "straight" → api_to_position.py
    ├─ "car" + "curve" → api_to_position_curve.py
    └─ "walk" → test_naver_map_crawl.py
    ↓
[경로 생성 로직]
    ↓ commands 생성
[dashboard.py]
    ├─ 세션 저장 (session["last_route_commands"])
    ├─ 파일 저장 (JSON)
    └─ 브라우저 응답 (commands, scale_factor, ...)
```

### 3.2 파일 저장 위치

#### 차선도로 (직선/곡선)
**로컬 PC**:
- `dashboard/current_route_coordinates.json`
  ```json
  {
      "path": [[lon, lat], ...],
      "start_coords": [lon, lat],
      "goal_coords": [lon, lat]
  }
  ```

**세션 데이터**:
- `session["last_route_mode"]`: "car"
- `session["last_route_path_type"]`: "straight" or "curve"
- `session["last_route_start_address"]`: "경기 성남시..."
- `session["last_route_goal_address"]`: "경기 성남시..."
- `session["last_route_commands"]`: commands 리스트

#### 도보도로
**로컬 PC**:
- `dashboard/current_walking_route_data.json`
  ```json
  {
      "url": "https://map.naver.com/...",
      "actions": [...],
      "crosswalks": [...]
  }
  ```

**세션 데이터**:
- `session["last_route_mode"]`: "walk"
- `session["last_route_actions"]`: actions 리스트
- `session["last_route_url"]`: 네이버 지도 URL
- `session["last_route_crosswalks"]`: 횡단보도 정보

---

## 4. 로봇 PC 통신 구조

### 4.1 "주행 시작" 버튼 클릭 시 흐름

```
[대시보드 UI] (app.js)
    ↓ POST /api/start-move
[dashboard.py]
    ↓ route_mode 확인
    ├─ "walk" → 도보도로 처리
    └─ "car" → 차선도로 처리
    ↓
[robot_connect.py]
    ├─ SSH 연결
    ├─ SFTP 업로드 (도보도로만)
    └─ SSH 명령 실행
    ↓
[로봇 PC]
    └─ move_full_path.py 실행
```

### 4.2 도보도로 통신 상세

**1단계: JSON 파일 생성**
```python
# dashboard.py
json_data = {
    "url": route_url,
    "actions": actions,
    "crosswalks": crosswalks
}
```

**2단계: SFTP 업로드**
```python
# robot_connect.py
upload_json_file(
    robot_ip, username, password,
    json_data=json_data,
    remote_path="/tmp/walking_route.json"
)
```

**3단계: SSH 명령 실행**
```bash
# robot_connect.py에서 생성되는 명령
cd ~/robot_ws && \
source install/local_setup.bash && \
export ROS_DOMAIN_ID=3 && \
PYTHONPATH=~/robot_ws/src:$PYTHONPATH \
python3 src/move_full_path.py \
  --json-path /tmp/walking_route.json \
  --target-width 2.0 \
  --target-height 3.0
```

**4단계: 로봇 PC에서 실행**
```python
# move_full_path.py
mover.execute_path_from_json(
    json_path="/tmp/walking_route.json",
    target_width_m=2.0,
    target_height_m=3.0
)
```

### 4.3 차선도로 통신 상세

**1단계: 주소 정보 전달**
```python
# dashboard.py
start_address = session.get("last_route_start_address")
goal_address = session.get("last_route_goal_address")
path_type = session.get("last_route_path_type", "straight")
```

**2단계: SSH 명령 실행 (JSON 파일 없음)**
```bash
# robot_connect.py에서 생성되는 명령
cd ~/robot_ws && \
source install/local_setup.bash && \
export ROS_DOMAIN_ID=3 && \
PYTHONPATH=~/robot_ws/src:$PYTHONPATH \
python3 src/move_full_path.py \
  --start-address "경기 성남시 수정구 수정로 167" \
  --goal-address "경기 성남시 수정구 공원로 426" \
  --path-type "curve" \
  --target-width 2.0 \
  --target-height 3.0
```

**3단계: 로봇 PC에서 실행**
```python
# move_full_path.py
mover.execute_path(
    start_address="경기 성남시...",
    goal_address="경기 성남시...",
    target_width_m=2.0,
    target_height_m=3.0,
    path_type="curve"  # "straight" or "curve"
)
```

**4단계: 경로 재생성**
```python
# move_full_path.py
if path_type == "curve":
    result = get_robot_path_from_addresses_curve(...)
else:
    result = get_robot_path_from_addresses(...)
```

---

## 5. 함수 호출 체인

### 5.1 차선도로 - 직선 모드

```
dashboard.py::plan_route()
    ↓
api_to_position.py::get_robot_path_from_addresses()
    ├─ geocode() → 네이버 Geocoding API
    ├─ get_directions() → 네이버 Directions 15 API
    ├─ extract_keypoints()
    ├─ fit_path_to_area()
    └─ calculate_robot_commands()
    ↓
commands 반환
```

### 5.2 차선도로 - 곡선 모드

```
dashboard.py::plan_route()
    ↓
api_to_position_curve.py::get_robot_path_from_addresses_curve()
    ├─ geocode() → 네이버 Geocoding API
    ├─ get_directions() → 네이버 Directions 15 API
    ├─ extract_keypoints()
    ├─ fit_path_to_area()
    └─ generate_curve_commands()
        ├─ is_straight_segment() (임계값: 2.75m)
        └─ heading_between() (y축 반전 없음)
    ↓
commands 반환
```

### 5.3 도보도로 모드

```
dashboard.py::process_walking_route()
    ↓
test_naver_map_crawl.py::parse_route_actions()
    ↓
test_naver_map_crawl.py::convert_json_to_robot_commands()
    ├─ fit_path_to_area()
    └─ commands 생성
    ↓
commands 반환
```

### 5.4 로봇 PC 실행 체인

```
move_full_path.py::main()
    ↓ argparse 파싱
    ├─ --json-path → execute_path_from_json()
    │   └─ convert_json_to_robot_commands()
    │
    └─ --start-address/--goal-address → execute_path()
        ├─ path_type == "curve" → get_robot_path_from_addresses_curve()
        └─ path_type == "straight" → get_robot_path_from_addresses()
        ↓
    execute_commands(commands)
        ├─ type == "smooth_curve" → move_curve()
        └─ type == "straight" → rotate() + move_straight()
```

---

## 6. JSON 파일 구조

### 6.1 도보도로 JSON (`/tmp/walking_route.json`)

```json
{
    "url": "https://map.naver.com/v5/directions/...",
    "actions": [
        {
            "type": "출발",
            "coords": [127.1385844, 37.4432127],
            "heading": 0.0
        },
        {
            "type": "횡단보도_좌회전",
            "coords": [127.1436247, 37.448409],
            "heading": 90.0
        },
        ...
    ],
    "crosswalks": [
        {
            "direction": "left",
            "angle": 90
        },
        ...
    ]
}
```

### 6.2 경로 좌표 JSON (`current_route_coordinates.json`)

```json
{
    "path": [
        [127.1385844, 37.4432127],
        [127.1390000, 37.4435000],
        ...
    ],
    "start_coords": [127.1385844, 37.4432127],
    "goal_coords": [127.1436247, 37.448409]
}
```

---

## 7. 통신 프로토콜

### 7.1 SSH 통신

**라이브러리**: `paramiko`

**용도**:
- 원격 명령 실행 (`_run_remote_command()`)
- ROS2 노드 실행
- 파일 업로드/다운로드

**연결 정보**:
- 포트: 22 (기본 SSH)
- 인증: 사용자명/비밀번호
- 타임아웃: 15초

### 7.2 SFTP 통신

**라이브러리**: `paramiko.SFTPClient`

**용도**:
- 도보도로 JSON 파일 업로드
- 원격 파일 경로: `/tmp/walking_route.json`

**업로드 함수**:
```python
upload_json_file(
    robot_ip, username, password,
    json_data=dict,
    remote_path="/tmp/walking_route.json"
)
```

### 7.3 HTTP API 통신

**프로토콜**: HTTP/1.1

**엔드포인트**:
- `POST /api/plan-route`: 경로 생성
- `POST /api/start-move`: 주행 시작
- `POST /api/stop-move`: 주행 중지
- `POST /api/connect`: 로봇 연결
- `POST /api/download-move-path`: move_path.py 다운로드

---

## 8. 스케일 재계산 로직

### 8.1 대시보드에서 스케일 변경 시

```
[사용자] 스케일 변경 (2.0m x 3.0m → 3.0m x 4.0m)
    ↓
[dashboard.py] recalculate_scale_factor()
    ↓ route_mode 확인
    ├─ "car" → 차선도로 재계산
    │   ├─ 파일에서 원본 좌표 읽기
    │   ├─ path_type 확인
    │   ├─ "straight" → api_to_position.py
    │   └─ "curve" → api_to_position_curve.py
    │
    └─ "walk" → 도보도로 재계산
        ├─ 파일에서 원본 데이터 읽기
        └─ convert_json_to_robot_commands()
    ↓
새로운 commands 생성
    ↓
세션 업데이트
```

### 8.2 로봇 PC에서 스케일 적용

**차선도로**:
- `--target-width`, `--target-height` 인자로 전달
- `execute_path()`에서 재계산

**도보도로**:
- `--target-width`, `--target-height` 인자로 전달
- `execute_path_from_json()`에서 재계산

---

## 9. 명령 실행 구조

### 9.1 move_path.py (다운로드용)

**생성 위치**: `dashboard.py::generate_move_path_py()`

**내용**:
- `execute_path()` 함수에 commands 하드코딩
- `rotate()`, `move_straight()`, `move_curve()` 메서드 포함
- 로컬 테스트용

### 9.2 move_full_path.py (로봇 PC 실행용)

**위치**: `~/robot_ws/src/move_turtle/move_full_path.py`

**실행 방식**:
1. SSH로 원격 실행
2. 인자로 경로 정보 전달
3. 로봇 PC에서 경로 재생성
4. commands 실행

**장점**:
- 스케일 변경 시 재계산 가능
- 파일 전송 불필요 (차선도로)
- 실시간 설정 반영

---

## 10. 좌표계 및 각도 처리

### 10.1 좌표계 변환

**지도 좌표계 (위경도)**:
- X축: 경도 (longitude)
- Y축: 위도 (latitude)
- Y 증가: 북쪽

**로봇 좌표계 (ROS2)**:
- X축: 전진 방향
- Y축: 좌측 방향
- Y 증가: 좌측

**변환 함수**:
```python
# 직선 로직
dy = -(next_p[1] - curr[1])  # y축 반전

# 곡선 로직
dy = (p2[1] - p1[1])  # y축 반전 없음 (반대로!)
```

### 10.2 각도 처리

**turn_angle 규칙**:
- 좌회전: 양수 (+)
- 우회전: 음수 (-)
- 범위: -180° ~ +180°

**angular_vel 규칙**:
- 곡선 로직: `angular_vel = -math.radians(turn_angle) / duration`
- 부호 반전 이유: 로봇 좌표계 보정

---

## 11. 에러 처리 및 로깅

### 11.1 로깅 레벨

- `app.logger.info()`: 일반 정보
- `app.logger.warning()`: 경고
- `app.logger.error()`: 오류

### 11.2 예외 처리

**네이버 API 오류**:
- 401/403: API 키 오류
- 210: 서비스 구독 필요
- 네트워크 오류: 타임아웃 처리

**SSH/SFTP 오류**:
- 인증 실패: 사용자명/비밀번호 확인
- 연결 실패: 네트워크 확인
- 타임아웃: 15초

---

## 12. 의존성

### 12.1 Python 패키지

**로컬 PC**:
- Flask: 웹 서버
- paramiko: SSH/SFTP 통신
- requests: HTTP API 호출
- numpy, scipy: 곡선 보간 (선택)
- PIL/Pillow: 이미지 생성 (선택)

**로봇 PC**:
- rclpy: ROS2 Python 클라이언트
- geometry_msgs: Twist 메시지

### 12.2 외부 API

- 네이버 클라우드 플랫폼:
  - Geocoding API
  - Directions 15 API

---

## 13. 파일 동기화

### 13.1 로컬 PC → 로봇 PC

**자동 동기화**:
- `motion_settings.json`: SSH 명령 실행 시 자동 읽기
- 도보도로 JSON: SFTP 업로드

**수동 동기화 필요**:
- `move_full_path.py`: 수동 복사 또는 git
- `api_to_position*.py`: 수동 복사 또는 git
- `test_naver_map_crawl.py`: 수동 복사 또는 git

### 13.2 세션 데이터

**Flask 세션**:
- 브라우저 쿠키 기반
- 서버 재시작 시 초기화
- 파일 저장으로 영구 보존

---

## 14. 보안 고려사항

### 14.1 인증

- SSH: 사용자명/비밀번호
- API 키: 환경변수 또는 하드코딩 (주의 필요)

### 14.2 네트워크

- 로컬 네트워크 내 통신 가정
- 방화벽 설정 필요 시 포트 22, 8088 열기

---

## 15. 확장성

### 15.1 새로운 경로 타입 추가

1. 새로운 로직 파일 생성 (예: `api_to_position_xxx.py`)
2. `dashboard.py`에 라우팅 추가
3. `move_full_path.py`에 실행 로직 추가

### 15.2 새로운 통신 방식

- 현재: SSH/SFTP
- 가능: ROS2 토픽, 웹소켓, MQTT 등

---

## 16. 디버깅 팁

### 16.1 로컬 PC 디버깅

- Flask 서버 로그 확인
- 브라우저 개발자 도구 (F12)
- 세션 데이터 확인: `session.get()`

### 16.2 로봇 PC 디버깅

- SSH로 직접 접속하여 실행
- ROS2 토픽 모니터링: `ros2 topic echo /cmd_vel`
- 로그 확인: `ros2 run move_turtle move_full_path.py ...`

---

## 17. 참고 문서

- `CURVE_PATH_LOGIC.md`: 곡선 로직 상세 설명
- `PROJECT_SUMMARY.md`: 프로젝트 전체 요약
- 네이버 클라우드 플랫폼 API 문서

---

## 18. 변경 이력

### 2025-11-25
- 곡선 로직 추가
- 곡선→곡선 스무스 회전 규칙 적용
- path_type 인자 추가
- move_full_path.py 곡선 로직 지원

---

**작성일**: 2025-11-25  
**작성자**: AI Assistant  
**버전**: 1.0

