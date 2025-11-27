# 웹 서버 배포 가이드

Flask 대시보드를 무료로 웹에 배포하는 방법들

## 🆓 무료 호스팅 옵션

### 1. **Render.com** (추천 ⭐)
- **무료 티어**: 무제한
- **특징**: 
  - 자동 HTTPS, 커스텀 도메인 지원
  - GitHub 연동으로 자동 배포
  - 15분 비활성 시 슬립 모드 (첫 요청 시 깨어남)
  - **Ubuntu 기반 Docker 지원** ✅
- **제한사항**: 
  - 무료 티어는 15분 비활성 시 슬립
  - 메모리 512MB
- **배포 방법 (Docker 사용 - Ubuntu 기반)**:
  1. GitHub에 프로젝트 푸시 (Dockerfile 포함)
  2. Render.com 가입 후 "New Web Service" 선택
  3. GitHub 저장소 연결
  4. **Environment**: `Docker` 선택
  5. **Dockerfile Path**: `dashboard/Dockerfile` (또는 프로젝트 루트에 있다면 `Dockerfile`)
  6. **Docker Context**: `.` (프로젝트 루트)
  7. 환경변수 설정:
     - `NAVER_CLIENT_ID`
     - `NAVER_CLIENT_SECRET`
     - `NAVER_MAP_CLIENT_ID`
     - `FLASK_DEBUG=False` (프로덕션 모드)
  8. Deploy 클릭!

- **배포 방법 (표준 Python 환경)**:
  1. GitHub에 프로젝트 푸시
  2. Render.com 가입 후 "New Web Service" 선택
  3. GitHub 저장소 연결
  4. **Environment**: `Python 3` 선택
  5. Build Command: `pip install -r dashboard/requirements.txt`
  6. Start Command: `cd dashboard && python dashboard.py`
  7. 환경변수 설정 (NAVER_CLIENT_ID, NAVER_CLIENT_SECRET 등)

### 2. **Railway.app**
- **무료 티어**: $5 크레딧/월 (약 500시간)
- **특징**: 
  - 빠른 배포, 자동 HTTPS
  - GitHub 연동
  - 슬립 모드 없음
- **제한사항**: 크레딧 소진 시 중단
- **배포 방법**: Render와 유사

### 3. **PythonAnywhere**
- **무료 티어**: 제한적
- **특징**: 
  - Python 전용 호스팅
  - 웹 인터페이스로 쉬운 설정
- **제한사항**: 
  - 외부 API 호출 제한
  - 하루 1회 재로드 필요
- **배포 방법**:
  1. PythonAnywhere 가입
  2. Files 탭에서 프로젝트 업로드
  3. Web 탭에서 Flask 앱 설정
  4. 환경변수 설정

### 4. **Fly.io**
- **무료 티어**: 3개 앱, 256MB RAM/앱
- **특징**: 
  - 전 세계 엣지 배포
  - Docker 기반
- **제한사항**: 리소스 제한
- **배포 방법**: Dockerfile 필요

## 🖥️ Ubuntu 서버 옵션 (무료)

### 1. **Oracle Cloud Free Tier** (가장 추천 ⭐⭐⭐)
- **무료 제공**: 
  - 2개 VM (ARM 또는 x86)
  - 각 1 OCPU, 1GB RAM
  - 200GB 스토리지
  - **영구 무료** (크레딧 소진 없음)
- **특징**: 
  - Ubuntu 22.04 LTS 제공
  - 공인 IP 제공
  - 24/7 무료 운영 가능
- **설정 방법**:
  ```bash
  # 1. Oracle Cloud 가입 (oracle.com/cloud)
  # 2. Always Free VM 인스턴스 생성
  # 3. SSH로 접속 후:
  
  sudo apt update
  sudo apt install python3-pip python3-venv nginx
  
  # 프로젝트 클론
  git clone <your-repo> ~/robot_ws
  cd ~/robot_ws/src/move_turtle/dashboard
  
  # 가상환경 설정
  python3 -m venv venv
  source venv/bin/activate
  pip install -r requirements.txt
  
  # 환경변수 설정
  nano ~/.bashrc
  # 추가:
  export NAVER_CLIENT_ID="your-id"
  export NAVER_CLIENT_SECRET="your-secret"
  export NAVER_MAP_CLIENT_ID="your-map-id"
  export DASHBOARD_PORT=8088
  
  # systemd 서비스 설정 (백그라운드 실행)
  sudo nano /etc/systemd/system/dashboard.service
  ```
  
  **dashboard.service 파일 내용**:
  ```ini
  [Unit]
  Description=Robot Dashboard Flask App
  After=network.target

  [Service]
  User=ubuntu
  WorkingDirectory=/home/ubuntu/robot_ws/src/move_turtle/dashboard
  Environment="PATH=/home/ubuntu/robot_ws/src/move_turtle/dashboard/venv/bin"
  ExecStart=/home/ubuntu/robot_ws/src/move_turtle/dashboard/venv/bin/python dashboard.py
  Restart=always

  [Install]
  WantedBy=multi-user.target
  ```
  
  ```bash
  # 서비스 시작
  sudo systemctl daemon-reload
  sudo systemctl enable dashboard
  sudo systemctl start dashboard
  
  # Nginx 리버스 프록시 설정 (선택사항)
  sudo nano /etc/nginx/sites-available/dashboard
  ```
  
  **Nginx 설정**:
  ```nginx
  server {
      listen 80;
      server_name your-domain.com;  # 또는 IP 주소

      location / {
          proxy_pass http://127.0.0.1:8088;
          proxy_set_header Host $host;
          proxy_set_header X-Real-IP $remote_addr;
      }
  }
  ```
  
  ```bash
  sudo ln -s /etc/nginx/sites-available/dashboard /etc/nginx/sites-enabled/
  sudo nginx -t
  sudo systemctl restart nginx
  ```

### 2. **AWS Free Tier (EC2)**
- **무료 제공**: 
  - 1년간 750시간/월 (t2.micro)
  - 1년 후 유료 전환
- **특징**: 안정적, 널리 사용됨
- **설정**: Oracle Cloud와 유사

### 3. **Google Cloud Free Tier**
- **무료 제공**: 
  - $300 크레딧 (90일)
  - 이후 제한적 무료 티어
- **특징**: GCP 생태계 활용 가능

### 4. **Azure Free Tier**
- **무료 제공**: 
  - $200 크레딧 (30일)
  - 이후 제한적 무료 티어
- **특징**: Microsoft 생태계

## 🚀 빠른 배포 추천 순서

### 옵션 A: 가장 쉬운 방법 (Render.com)
1. GitHub에 코드 푸시
2. Render.com에서 5분 내 배포
3. 환경변수만 설정하면 완료

### 옵션 B: 가장 안정적인 방법 (Oracle Cloud)
1. Oracle Cloud 가입 (무료)
2. Ubuntu VM 생성
3. SSH로 접속하여 수동 설정
4. 24/7 무료 운영 가능

### 옵션 C: 테스트용 (PythonAnywhere)
1. PythonAnywhere 가입
2. 웹 인터페이스로 간단히 배포
3. 외부 API 제한 주의

## 📝 배포 전 체크리스트

- [ ] 환경변수 설정 (NAVER_CLIENT_ID, NAVER_CLIENT_SECRET, NAVER_MAP_CLIENT_ID)
- [ ] `requirements.txt` 확인
- [ ] 포트 설정 확인 (기본: 8088)
- [ ] 정적 파일 경로 확인 (`static/`, `templates/`)
- [ ] 파일 업로드/다운로드 경로 확인
- [ ] CORS 설정 (필요시)
- [ ] 보안 설정 (프로덕션 모드: `debug=False`)

## 🔒 보안 주의사항

1. **프로덕션 모드**: `app.run(debug=False)` 설정
2. **환경변수**: API 키는 절대 코드에 하드코딩하지 않기
3. **HTTPS**: 무료 호스팅은 대부분 자동 HTTPS 제공
4. **방화벽**: Ubuntu 서버 사용 시 포트 열기
   ```bash
   sudo ufw allow 8088/tcp
   sudo ufw allow 80/tcp
   sudo ufw allow 443/tcp
   ```

## 📚 참고 링크

- [Render.com 문서](https://render.com/docs)
- [Oracle Cloud Free Tier](https://www.oracle.com/cloud/free/)
- [Flask 배포 가이드](https://flask.palletsprojects.com/en/latest/deploying/)

