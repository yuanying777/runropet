# MOVE MY ROBOT Dashboard

로봇 경로 계획 및 제어 웹 대시보드

## 설치

```bash
cd ~/robot_ws/src/move_turtle/dashboard
pip3 install -r requirements.txt
```

## 네이버 API 키 설정

### 1. 네이버 클라우드 플랫폼에서 API 키 발급

1. [네이버 클라우드 플랫폼](https://www.ncloud.com/) 접속
2. **Console** > **AI·NAVER API** > **Application** 에서 신청
3. 다음 서비스 활성화:
   - **Geocoding API** (주소 → 위경도 변환)
   - **Directions 15 API** (경로 탐색)
   - **Maps JavaScript API** (웹 지도 표시)

### 2. 환경변수 설정

터미널에서:

```bash
# 지오코딩 & 경로 탐색 API 키
export NAVER_CLIENT_ID="your-client-id"
export NAVER_CLIENT_SECRET="your-client-secret"

# 지도 JavaScript API 클라이언트 ID (웹 지도 표시용)
export NAVER_MAP_CLIENT_ID="your-map-client-id"
```

또는 `.bashrc`에 추가:

```bash
echo 'export NAVER_CLIENT_ID="your-client-id"' >> ~/.bashrc
echo 'export NAVER_CLIENT_SECRET="your-client-secret"' >> ~/.bashrc
echo 'export NAVER_MAP_CLIENT_ID="your-map-client-id"' >> ~/.bashrc
source ~/.bashrc
```

**참고**: 
- `NAVER_CLIENT_ID`와 `NAVER_CLIENT_SECRET`은 **같은** Application의 키입니다.
- `NAVER_MAP_CLIENT_ID`는 JavaScript API용이며, **별도**로 발급받아야 합니다.

### 3. API 키 확인

```bash
cd ~/robot_ws/src/move_turtle
python3 test_geocode.py
```

성공하면 API 키가 정상적으로 작동합니다.

## 실행

### 방법 1: dashboard 폴더에서 직접 실행
```bash
cd ~/robot_ws/src/move_turtle/dashboard
python3 dashboard.py
```

### 방법 2: robot_ws/src에서 실행
```bash
cd ~/robot_ws/src
python3 run_dashboard.py
```

### 포트 변경
```bash
DASHBOARD_PORT=5000 python3 dashboard.py
```

## 접속

브라우저에서 `http://localhost:8088` 접속

## 기능

1. **STEP 01**: 출발지/도착지 입력 및 네이버 지도 표시
2. **STEP 02**: 경로 계획 생성 (Python 파일 다운로드)
3. **STEP 03**: 로봇 SSH 연결 및 테스트
4. **STEP 04**: 로봇 실행 및 카메라 스트리밍

## 문제 해결

### 401 Unauthorized 오류
- API 키가 잘못되었거나 만료됨
- 환경변수 설정 확인: `echo $NAVER_CLIENT_ID`
- API 서비스 권한 확인 (Geocoding, Directions 15 활성화 여부)

### 지도가 표시되지 않음
- `NAVER_MAP_CLIENT_ID` 환경변수 확인
- 브라우저 콘솔에서 JavaScript 에러 확인
- Maps JavaScript API 서비스 활성화 여부 확인

