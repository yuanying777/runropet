# Ubuntu 기반 Python 3.10 이미지 사용
FROM ubuntu:22.04

# 환경변수 설정 (비대화형 설치)
ENV DEBIAN_FRONTEND=noninteractive

# 시스템 업데이트 및 필수 패키지 설치
RUN apt-get update && apt-get install -y \
    python3.10 \
    python3-pip \
    python3-venv \
    git \
    && rm -rf /var/lib/apt/lists/*

# 작업 디렉토리 설정
WORKDIR /app

# 프로젝트 전체 구조 복사 (move_turtle 폴더 전체)
# Docker Context가 robot_ws/src/move_turtle일 경우
COPY . /app/

# requirements.txt가 있는 위치로 이동
WORKDIR /app/dashboard

# Python 패키지 설치
RUN pip3 install --no-cache-dir -r requirements.txt

# Python 경로 설정 (상위 디렉토리 모듈 import를 위해)
ENV PYTHONPATH=/app:/app/dashboard:$PYTHONPATH

# 작업 디렉토리를 dashboard로 설정
WORKDIR /app/dashboard

# 포트 노출 (Render.com은 PORT 환경변수를 사용)
EXPOSE 8088

# 환경변수 기본값 설정
ENV DASHBOARD_PORT=8088

# Flask 앱 실행
# Render.com은 PORT 환경변수를 자동으로 설정하므로 이를 사용
WORKDIR /app/dashboard
CMD ["python3", "dashboard.py"]

