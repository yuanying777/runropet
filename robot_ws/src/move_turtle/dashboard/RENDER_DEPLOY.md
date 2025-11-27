# Render.com 배포 가이드 (Ubuntu Docker)

Render.com에서 Ubuntu 기반 Docker로 배포하는 방법

## 📋 사전 준비

1. **GitHub 저장소 준비**
   - 프로젝트를 GitHub에 푸시
   - `robot_ws/src/move_turtle/` 폴더 전체를 저장소 루트로 설정하거나
   - 또는 `robot_ws/src/move_turtle/` 폴더를 별도 저장소로 만들기

2. **필수 파일 확인**
   - ✅ `dashboard/Dockerfile` (생성됨)
   - ✅ `dashboard/.dockerignore` (생성됨)
   - ✅ `dashboard/requirements.txt`
   - ✅ `dashboard/dashboard.py`

## 🚀 Render.com 배포 단계

### 방법 1: 웹 UI 사용 (추천)

1. **Render.com 가입**
   - https://render.com 접속
   - GitHub 계정으로 로그인

2. **새 Web Service 생성**
   - Dashboard → "New +" → "Web Service" 클릭
   - GitHub 저장소 연결 (처음이면 권한 부여)

3. **서비스 설정**
   - **Name**: `robot-dashboard` (원하는 이름)
   - **Environment**: `Docker` 선택 ⭐
   - **Region**: 가장 가까운 지역 선택
   - **Branch**: `main` (또는 기본 브랜치)
   - **Root Directory**: 
     - 저장소 루트가 `move_turtle/`이면: `.`
     - 저장소 루트가 `robot_ws/`이면: `src/move_turtle`
   - **Dockerfile Path**: `dashboard/Dockerfile`
   - **Docker Context**: `.` (Root Directory 기준)

4. **환경변수 설정**
   - Environment Variables 섹션에서 추가:
     ```
     NAVER_CLIENT_ID=your-client-id
     NAVER_CLIENT_SECRET=your-client-secret
     NAVER_MAP_CLIENT_ID=your-map-client-id
     FLASK_DEBUG=False
     ```
   - ⚠️ **중요**: API 키는 절대 GitHub에 커밋하지 마세요!

5. **고급 설정 (선택)**
   - **Auto-Deploy**: `Yes` (GitHub 푸시 시 자동 배포)
   - **Health Check Path**: `/` (또는 비워두기)

6. **배포 시작**
   - "Create Web Service" 클릭
   - 빌드 로그 확인 (약 2-5분 소요)

### 방법 2: render.yaml 사용 (자동화)

1. **render.yaml 파일 확인**
   - `dashboard/render.yaml` 파일이 프로젝트에 있는지 확인
   - 필요시 수정

2. **Render.com에서 Blueprint 사용**
   - Dashboard → "New +" → "Blueprint" 선택
   - GitHub 저장소 연결
   - `render.yaml` 파일 자동 감지

## 🔍 배포 후 확인

1. **서비스 URL 확인**
   - Render Dashboard에서 서비스 URL 확인
   - 예: `https://robot-dashboard.onrender.com`

2. **로그 확인**
   - Dashboard → 서비스 → "Logs" 탭
   - 에러가 있으면 로그에서 확인

3. **환경변수 확인**
   - Dashboard → 서비스 → "Environment" 탭
   - 모든 환경변수가 설정되었는지 확인

## 🐛 문제 해결

### 빌드 실패
- **원인**: Dockerfile 경로 오류
- **해결**: Dockerfile Path와 Docker Context 확인

### 모듈 import 오류
- **원인**: Python 경로 문제
- **해결**: Dockerfile의 `PYTHONPATH` 확인

### 포트 오류
- **원인**: Render.com은 `PORT` 환경변수 사용
- **해결**: `dashboard.py`가 `PORT` 환경변수를 읽도록 수정됨 ✅

### API 키 오류
- **원인**: 환경변수 미설정
- **해결**: Render Dashboard에서 환경변수 확인

## 📝 Docker Context 설정 가이드

### 케이스 1: 저장소 루트가 `move_turtle/`
```
move_turtle/
├── dashboard/
│   ├── Dockerfile
│   ├── dashboard.py
│   └── ...
├── api_to_position_curve.py
└── ...
```
- **Root Directory**: `.`
- **Dockerfile Path**: `dashboard/Dockerfile`
- **Docker Context**: `.`

### 케이스 2: 저장소 루트가 `robot_ws/`
```
robot_ws/
└── src/
    └── move_turtle/
        ├── dashboard/
        └── ...
```
- **Root Directory**: `src/move_turtle`
- **Dockerfile Path**: `dashboard/Dockerfile`
- **Docker Context**: `.`

## 🎯 최종 체크리스트

- [ ] GitHub에 프로젝트 푸시 완료
- [ ] Dockerfile이 올바른 위치에 있음
- [ ] 환경변수 모두 설정됨
- [ ] Root Directory 경로 확인
- [ ] Dockerfile Path 확인
- [ ] 첫 배포 성공 확인
- [ ] 웹사이트 접속 테스트

## 🔗 유용한 링크

- [Render.com Docker 가이드](https://render.com/docs/docker)
- [Render.com 환경변수](https://render.com/docs/environment-variables)
- [Render.com 로그](https://render.com/docs/log-streaming)

