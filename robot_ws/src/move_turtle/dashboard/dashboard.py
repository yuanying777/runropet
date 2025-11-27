"""
Flask based dashboard server for MOVE MY ROBOT.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from pprint import pformat
from textwrap import dedent
from typing import Any, Dict

from flask import Flask, jsonify, render_template, request, session, send_file

# Allow importing project modules
BASE_DIR = Path(__file__).resolve().parents[1]
ROOT_SRC_DIR = Path(__file__).resolve().parents[2]  # robot_ws/src
CURRENT_DIR = Path(__file__).resolve().parent
for path in (BASE_DIR, CURRENT_DIR):
    if str(path) not in sys.path:
        sys.path.append(str(path))

# 곡선 최적화 경로만 사용 (직선 로직 제거됨)
from api_to_position_curve import (  # type: ignore  # noqa: E402
    get_robot_path_from_addresses as get_robot_path_from_addresses_curve,
    get_robot_path_from_coordinates as get_robot_path_from_coordinates_curve,
)
from api_to_position import geocode  # type: ignore  # noqa: E402
from crawl_to_position import visualize_walking_route_image  # type: ignore  # noqa: E402
from robot_connect import (  # type: ignore  # noqa: E402
    connect_and_bringup,
    send_test_move,
    start_move_full_path,
    upload_json_file,
    stop_move_full_path,
    check_bringup_status,
    # run_precomputed_move,  # move_path_9.py 삭제로 사용 불가
)
from test_naver_map_crawl import parse_route_actions  # type: ignore  # noqa: E402

app = Flask(__name__, static_folder="static", template_folder="templates")
app.secret_key = os.urandom(24)  # 세션을 위한 시크릿 키
CRAWL_SCRIPT = BASE_DIR / "test_naver_map_crawl.py"
SETTINGS_PATH = BASE_DIR / "motion_settings.json"
ROUTE_COORDINATES_PATH = BASE_DIR / "dashboard" / "current_route_coordinates.json"  # 차선도로 경로 좌표 파일
WALKING_ROUTE_DATA_PATH = BASE_DIR / "dashboard" / "current_walking_route_data.json"  # 도보도로 경로 데이터 파일


def flatten_commands(commands: Any) -> Any:
    """Ensure commands is not wrapped in single-item nested lists."""
    while (
        commands
        and isinstance(commands, list)
        and len(commands) == 1
        and isinstance(commands[0], list)
    ):
        commands = commands[0]
    return commands


def normalize_commands(commands: Any) -> list[Any]:
    """Return flattened list representation for downstream use."""
    if not commands:
        return []
    commands = flatten_commands(commands)
    # JSON에서 넘어온 tuple 등을 list로 강제 전환
    return list(commands) if isinstance(commands, (list, tuple)) else []


def store_route_commands(commands: Any) -> list[Any]:
    """Flatten and store commands in session, returning normalized list."""
    normalized = normalize_commands(commands)
    session["last_route_commands"] = normalized
    return normalized


def get_last_route_commands() -> list[Any]:
    """Fetch stored commands, ensuring normalization at read-time as well."""
    stored = session.get("last_route_commands", [])
    return store_route_commands(stored) if stored else []


def load_motion_settings() -> Dict[str, Any]:
    defaults = {
        "mode": "scale",
        "scale_width": 2.0,
        "scale_height": 3.0,
        "scale_speed": 0.1,
        "real_speed": 0.2,
    }
    if SETTINGS_PATH.exists():
        try:
            with open(SETTINGS_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            for key in defaults:
                if isinstance(defaults[key], float):
                    defaults[key] = float(data.get(key, defaults[key]))
                else:
                    defaults[key] = data.get(key, defaults[key])
        except Exception:
            pass
    return defaults


def save_motion_settings_file(payload: Dict[str, Any]) -> Dict[str, Any]:
    settings = load_motion_settings()
    for key in settings:
        if key in payload:
            try:
                if isinstance(settings[key], float):
                    settings[key] = float(payload[key])
                else:
                    settings[key] = payload[key]
            except (ValueError, TypeError):
                continue
    with open(SETTINGS_PATH, "w", encoding="utf-8") as f:
        json.dump(settings, f, ensure_ascii=False, indent=2)
    return settings


@app.route("/")
def index():
    # 서버 시작/새로고침 시 항상 디폴트 설정 사용
    default_settings = {
        "mode": "scale",
        "scale_width": 2.0,
        "scale_height": 3.0,
        "scale_speed": 0.1,
        "real_speed": 0.2,
    }
    # 캐시 버스팅을 위한 타임스탬프 (서버 시작 시간 기반)
    import time
    cache_bust = int(time.time())
    return render_template("index.html", settings=default_settings, cache_bust=cache_bust)


@app.post("/api/geocode")
def api_geocode():
    """Geocoding API 엔드포인트 (JavaScript에서 사용)"""
    data: Dict[str, Any] = request.get_json(force=True) or {}
    address = data.get("address")
    
    if not address:
        return jsonify({"status": "error", "message": "주소를 입력해주세요."}), 400
    
    try:
        lon, lat = geocode(address)
        return jsonify({
            "status": "ok",
            "coords": [lon, lat]  # [lon, lat] 형식
        })
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 500


@app.post("/api/plan-route")
def plan_route():
    data: Dict[str, Any] = request.get_json(force=True) or {}
    start = data.get("start_address")
    goal = data.get("goal_address")
    # 사용자가 명시적으로 width/height를 제공하지 않으면 디폴트 사용
    # (서버 재시작 시 항상 디폴트로 시작)
    motion_settings = load_motion_settings()
    width = float(data.get("width", motion_settings.get("scale_width", 2.0)))
    height = float(data.get("height", motion_settings.get("scale_height", 3.0)))
    angle_threshold = int(data.get("angle_threshold", 20))
    # path_type은 곡선으로 고정 (직선 로직 제거됨)
    path_type = "curve"

    if not start or not goal:
        return jsonify({"status": "error", "message": "출발지와 도착지를 입력해주세요."}), 400

    try:
        # 곡선 최적화 경로만 사용
        # 현재 속도 설정 읽기
        motion_settings = load_motion_settings()
        current_mode = motion_settings.get("mode", "scale")
        current_speed = motion_settings.get("scale_speed", 0.1) if current_mode == "scale" else motion_settings.get("real_speed", 0.2)
        
        app.logger.info(f"[경로 생성] 곡선 최적화 경로 모드: API 호출 시작")
        result = get_robot_path_from_addresses_curve(
            start_address=start,
            goal_address=goal,
            target_width_m=width,
            target_height_m=height,
            angle_threshold=angle_threshold,
            linear_speed=current_speed,
        )
        app.logger.info(f"[경로 생성] 곡선 최적화 경로 모드: API 호출 완료")
        
        # 세션에 경로 데이터 저장 (다운로드용)
        commands = result.get("commands", [])
        commands = store_route_commands(commands)
        session["last_route_mode"] = "car"
        session["last_route_path_type"] = path_type  # "straight" 또는 "curve"
        
        # 실제 적용된 스케일 팩터 계산 (commands의 distance/original_distance 평균)
        actual_scale_factor = result.get("scale_factor", 1.0)
        if commands:
            scale_factors = []
            for cmd in commands:
                orig_dist = cmd.get("original_distance", 0)
                scaled_dist = cmd.get("distance", 0)
                if orig_dist > 0.001:
                    scale_factors.append(scaled_dist / orig_dist)
            if scale_factors:
                actual_scale_factor = sum(scale_factors) / len(scale_factors)
        
        session["last_route_scale_factor"] = actual_scale_factor
        session["last_route_start_address"] = start  # 출발지 주소 저장 (재계산용)
        session["last_route_goal_address"] = goal  # 도착지 주소 저장 (재계산용)
        session["last_route_angle_threshold"] = angle_threshold  # 각도 임계값 저장
        
        # 원본 경로 좌표를 파일로 저장 (API 재호출 방지용)
        if "path" in result:
            route_data = {
                "path": result["path"],
                "start_coords": result.get("start_coords"),
                "goal_coords": result.get("goal_coords"),
                "start_address": start,
                "goal_address": goal,
                "path_type": path_type,
                "angle_threshold": angle_threshold,
            }
            try:
                with open(ROUTE_COORDINATES_PATH, 'w', encoding='utf-8') as f:
                    json.dump(route_data, f, ensure_ascii=False, indent=2)
                app.logger.info(f"[경로 좌표 저장] 파일 저장 완료: {ROUTE_COORDINATES_PATH}")
            except Exception as e:
                app.logger.warning(f"[경로 좌표 저장] 파일 저장 실패: {e}")
        
        # JavaScript API 사용하므로 Static Map URL 불필요
        # 실제 적용된 스케일 팩터를 반환
        result["scale_factor"] = actual_scale_factor
        return jsonify({"status": "ok", **result})
    except Exception as exc:  # pragma: no cover
        return jsonify({"status": "error", "message": str(exc)}), 500


@app.post("/api/connect")
def connect_robot():
    data = request.get_json(force=True) or {}
    robot_ip = data.get("robot_ip")
    username = data.get("username")
    password = data.get("password")

    if not all([robot_ip, username, password]):
        return jsonify({"status": "error", "message": "로봇 IP, ID, PW를 모두 입력하세요."}), 400

    success, out, err = connect_and_bringup(robot_ip, username, password)
    payload = {"status": "ok" if success else "error", "stdout": out, "stderr": err}
    return jsonify(payload), (200 if success else 500)


@app.post("/api/check-status")
def api_check_status():
    """브링업 상태 확인 API"""
    data = request.get_json(force=True) or {}
    robot_ip = data.get("robot_ip")
    username = data.get("username")
    password = data.get("password")
    
    if not all([robot_ip, username, password]):
        return jsonify({
            "status": "error",
            "message": "로봇 IP, ID, PW를 모두 입력하세요."
        }), 400
    
    try:
        is_running, out, err = check_bringup_status(robot_ip, username, password)
        return jsonify({
            "status": "ok",
            "is_running": is_running,
            "message": "로봇 연동됨" if is_running else "로봇 미연동"
        }), 200
    except Exception as e:
        return jsonify({
            "status": "error",
            "message": str(e),
            "is_running": False
        }), 500


@app.post("/api/test-move")
def test_move():
    data = request.get_json(force=True) or {}
    robot_ip = data.get("robot_ip")
    username = data.get("username")
    password = data.get("password")

    if not all([robot_ip, username, password]):
        return jsonify({"status": "error", "message": "로봇 IP, ID, PW를 모두 입력하세요."}), 400

    success, out, err = send_test_move(robot_ip, username, password)
    payload = {"status": "ok" if success else "error", "stdout": out, "stderr": err}
    return jsonify(payload), (200 if success else 500)


# @app.post("/api/run-path")
# def api_run_path():
#     """사전 정의된 주행 경로 실행 API (move_path_9)
#     
#     ⚠️ move_path_9.py 삭제로 인해 사용 불가
#     """
#     data = request.get_json(force=True) or {}
#     robot_ip = data.get("robot_ip")
#     username = data.get("username")
#     password = data.get("password")
#     domain_id = int(data.get("domain_id", 3))
#     precomputed_path = data.get("precomputed_path", "move_path_9")
# 
#     if not all([robot_ip, username, password]):
#         return jsonify({
#             "status": "error",
#             "message": "로봇 IP, ID, PW를 모두 입력하세요."
#         }), 400
# 
#     try:
#         success, out, err = run_precomputed_move(
#             robot_ip, username, password, domain_id, precomputed_path
#         )
#         return jsonify({
#             "status": "ok" if success else "error",
#             "stdout": out,
#             "stderr": err,
#             "message": "주행 명령이 실행되었습니다." if success else (err or "주행 명령 실행 실패")
#         }), (200 if success else 500)
#     except Exception as e:
#         return jsonify({
#             "status": "error",
#             "message": str(e),
#             "stderr": str(e)
#         }), 500


@app.post("/api/start-drive")
def start_drive():
    data = request.get_json(force=True) or {}
    robot_ip = data.get("robot_ip")
    username = data.get("username")
    password = data.get("password")
    domain_id = data.get("domain_id")

    if not all([robot_ip, username, password]):
        return jsonify({"status": "error", "message": "로봇 IP, ID, PW를 모두 입력하세요."}), 400

    # 모드 확인 (차선도로/도보도로)
    route_mode = session.get("last_route_mode", "car")  # 기본값: car
    motion_settings = load_motion_settings()
    target_width_m = motion_settings.get("scale_width", 2.0)
    target_height_m = motion_settings.get("scale_height", 3.0)
    
    # 도보도로 모드일 경우
    if route_mode == "walk":
        # 로컬 PC에서 이미 생성된 commands 사용 (차선도로와 동일)
        commands = session.get("last_route_commands")
        
        if not commands:
            return jsonify({"status": "error", "message": "도보도로 경로가 생성되지 않았습니다. 먼저 경로를 생성해주세요."}), 400
        
        # commands를 JSON 파일로 저장
        json_data = {
            "commands": commands
        }
        
        # 임시 JSON 파일 경로 (원격)
        remote_json_path = "/tmp/walking_route.json"
        
        # JSON 파일을 원격에 업로드
        upload_success, upload_out, upload_err = upload_json_file(
            robot_ip, username, password, json_data, remote_json_path
        )
        
        if not upload_success:
            return jsonify({
                "status": "error",
                "message": f"JSON 파일 업로드 실패: {upload_err}"
            }), 500
        
        # 원격에서 move_full_path.py 실행 (commands JSON 파일 경로 전달)
        success, out, err = start_move_full_path(
            robot_ip, username, password, 
            domain_id=domain_id,
            commands_json=remote_json_path
        )
    else:
        # 차선도로 모드: 로컬 PC에서 경로 계산 후 JSON으로 전달
        commands = session.get("last_route_commands")
        
        if not commands:
            return jsonify({
                "status": "error",
                "message": "차선도로 경로가 생성되지 않았습니다. 먼저 경로를 생성해주세요."
            }), 400
        
        # commands를 JSON 파일로 저장
        json_data = {
            "commands": commands
        }
        
        # 임시 JSON 파일 경로 (원격)
        remote_json_path = "/tmp/car_route.json"
        
        # JSON 파일을 원격에 업로드
        upload_success, upload_out, upload_err = upload_json_file(
            robot_ip, username, password, json_data, remote_json_path
        )
        
        if not upload_success:
            return jsonify({
                "status": "error",
                "message": f"JSON 파일 업로드 실패: {upload_err}"
            }), 500
        
        # 원격에서 move_full_path.py 실행 (JSON 파일 경로 전달)
        success, out, err = start_move_full_path(
            robot_ip, username, password, 
            domain_id=domain_id,
            commands_json=remote_json_path
        )
    
    payload = {"status": "ok" if success else "error", "stdout": out, "stderr": err}
    return jsonify(payload), (200 if success else 500)


@app.post("/api/stop-drive")
def stop_drive():
    data = request.get_json(force=True) or {}
    robot_ip = data.get("robot_ip")
    username = data.get("username")
    password = data.get("password")

    if not all([robot_ip, username, password]):
        return jsonify({"status": "error", "message": "로봇 IP, ID, PW를 모두 입력하세요."}), 400

    success, out, err = stop_move_full_path(robot_ip, username, password)
    payload = {"status": "ok" if success else "error", "stdout": out, "stderr": err}
    return jsonify(payload), (200 if success else 500)


@app.post("/api/camera")
def camera_controls():
    data = request.get_json(force=True) or {}
    robot_ip = data.get("robot_ip")
    username = data.get("username")
    password = data.get("password")
    topic = data.get("topic", "/image_raw")

    if not all([robot_ip, username, password]):
        return jsonify({"status": "error", "message": "로봇 IP, ID, PW를 모두 입력하세요."}), 400

    success, out, err = start_camera_nodes(robot_ip, username, password)
    payload = {
        "status": "ok" if success else "error",
        "stdout": out,
        "stderr": err,
        "stream_url": f"http://{robot_ip}:8080/stream?topic={topic}",
    }
    return jsonify(payload), (200 if success else 500)


def recalculate_scale_factor(target_width_m: float, target_height_m: float) -> float:
    """
    현재 세션의 경로 데이터를 기반으로 스케일 팩터 재계산
    
    Returns:
        계산된 스케일 팩터 (float)
    """
    route_mode = session.get("last_route_mode", "car")
    
    if route_mode == "car":
        # 차선도로: 파일에서 원본 경로 좌표 읽기, 없으면 API 재호출
        angle_threshold = session.get("last_route_angle_threshold", 20)
        original_path = None
        start_coords = None
        goal_coords = None
        path_type = session.get("last_route_path_type", "straight")
        
        # 파일에서 경로 좌표 읽기
        if ROUTE_COORDINATES_PATH.exists():
            try:
                with open(ROUTE_COORDINATES_PATH, 'r', encoding='utf-8') as f:
                    route_data = json.load(f)
                    original_path = route_data.get("path")
                    start_coords = route_data.get("start_coords")
                    goal_coords = route_data.get("goal_coords")
                    # 경로 타입 확인 (파일에 저장된 타입과 현재 세션 타입 일치 확인)
                    file_path_type = route_data.get("path_type", "straight")
                    if file_path_type != path_type:
                        app.logger.warning(f"[경로 좌표] 파일의 경로 타입({file_path_type})과 세션 타입({path_type})이 다릅니다. API 재호출합니다.")
                        original_path = None
                if original_path:
                    app.logger.info(f"[경로 좌표] 파일에서 읽기 완료: {len(original_path)}개 포인트")
            except Exception as e:
                app.logger.warning(f"[경로 좌표] 파일 읽기 실패: {e}, API 재호출합니다.")
                original_path = None
        
        if original_path:
            # 원본 경로 좌표 재사용 (API 호출 없음)
            try:
                # 현재 속도 설정 읽기
                motion_settings = load_motion_settings()
                current_mode = motion_settings.get("mode", "scale")
                current_speed = motion_settings.get("scale_speed", 0.1) if current_mode == "scale" else motion_settings.get("real_speed", 0.2)
                
                # 곡선 최적화 경로만 사용 (path_type 고정)
                app.logger.info(f"[스케일 팩터 재계산] 곡선 최적화 경로: 파일에서 좌표 읽기 완료, API 재호출 없이 스케일링만 재계산")
                result = get_robot_path_from_coordinates_curve(
                    path=original_path,
                    target_width_m=target_width_m,
                    target_height_m=target_height_m,
                    angle_threshold=angle_threshold,
                    linear_speed=current_speed,
                    start_coords=start_coords,
                    goal_coords=goal_coords,
                )
                
                # 재계산된 commands를 세션에 저장 (API 호출 없이 처리된 경우)
                commands = result.get("commands", [])
                # 실제 적용된 스케일 팩터 계산 (commands의 distance/original_distance 평균)
                actual_scale_factor = result.get("scale_factor", 1.0)
                if commands:
                    scale_factors = []
                    for cmd in commands:
                        orig_dist = cmd.get("original_distance", 0)
                        scaled_dist = cmd.get("distance", 0)
                        if orig_dist > 0.001:
                            scale_factors.append(scaled_dist / orig_dist)
                    if scale_factors:
                        actual_scale_factor = sum(scale_factors) / len(scale_factors)
                # 세션에 업데이트된 commands 저장 (스케일링 적용된 새 명령)
                commands = store_route_commands(commands)
                return actual_scale_factor
            except Exception as e:
                app.logger.warning(f"[스케일 팩터 재계산] 좌표 재사용 실패: {e}, API 재호출 시도")
                original_path = None  # 실패 시 API 재호출로 fallback
        
        if not original_path:
            # 원본 경로 좌표가 없으면 API 재호출 (fallback)
            start_address = session.get("last_route_start_address")
            goal_address = session.get("last_route_goal_address")
            
            if start_address and goal_address:
                try:
                    # 현재 속도 설정 읽기
                    motion_settings = load_motion_settings()
                    current_mode = motion_settings.get("mode", "scale")
                    current_speed = motion_settings.get("scale_speed", 0.1) if current_mode == "scale" else motion_settings.get("real_speed", 0.2)
                    
                    # 곡선 최적화 경로만 사용
                    result = get_robot_path_from_addresses_curve(
                        start_address=start_address,
                        goal_address=goal_address,
                        target_width_m=target_width_m,
                        target_height_m=target_height_m,
                        angle_threshold=angle_threshold,
                        linear_speed=current_speed,
                    )
                    commands = result.get("commands", [])
                    # 실제 적용된 스케일 팩터 계산 (commands의 distance/original_distance 평균)
                    actual_scale_factor = result.get("scale_factor", 1.0)
                    if commands:
                        scale_factors = []
                        for cmd in commands:
                            orig_dist = cmd.get("original_distance", 0)
                            scaled_dist = cmd.get("distance", 0)
                            if orig_dist > 0.001:
                                scale_factors.append(scaled_dist / orig_dist)
                        if scale_factors:
                            actual_scale_factor = sum(scale_factors) / len(scale_factors)
                    # 세션에 업데이트된 commands 저장 (스케일링 적용된 새 명령)
                    commands = store_route_commands(commands)
                    return actual_scale_factor
                except Exception as e:
                    app.logger.warning(f"[스케일 팩터 재계산] 차선도로 재계산 실패: {e}")
                    # 실패 시 기존 스케일 팩터 반환
                    return session.get("last_route_scale_factor", 1.0)
        else:
            # 주소가 없으면 기존 스케일 팩터 반환
            return session.get("last_route_scale_factor", 1.0)
    
    elif route_mode == "walk":
        # 도보도로: 파일에서 원본 경로 데이터 읽기, 없으면 세션에서 읽기
        original_data = None
        
        # 1단계: 파일에서 경로 데이터 읽기 (차선도로와 동일한 로직)
        if WALKING_ROUTE_DATA_PATH.exists():
            try:
                with open(WALKING_ROUTE_DATA_PATH, 'r', encoding='utf-8') as f:
                    original_data = json.load(f)
                app.logger.info(f"[스케일 팩터 재계산] 도보도로: 파일에서 원본 데이터 읽기 완료")
            except Exception as e:
                app.logger.warning(f"[스케일 팩터 재계산] 도보도로: 파일 읽기 실패: {e}, 세션에서 읽기 시도")
                original_data = None
        
        # 2단계: 파일에서 읽기 실패 시 세션에서 읽기
        if not original_data:
            actions = session.get("last_route_actions")
            url = session.get("last_route_url", "")
            crosswalks = session.get("last_route_crosswalks", [])
            
            if not actions:
                app.logger.warning(f"[스케일 팩터 재계산] 도보도로: 세션에도 데이터 없음")
                return 1.0
            
            original_data = {
                "url": url,
                "actions": actions,
                "crosswalks": crosswalks
            }
            app.logger.info(f"[스케일 팩터 재계산] 도보도로: 세션에서 데이터 읽기 완료")
        
        # 3단계: convert_json_to_robot_commands()를 사용하여 새로운 스케일로 commands 재계산
        try:
            actions = original_data.get("actions", [])
            url = original_data.get("url", "")
            crosswalks = original_data.get("crosswalks", [])
            
            if not actions:
                return 1.0
            
            temp_json_data = {
                "url": url,
                "actions": actions,
                "crosswalks": crosswalks
            }
            
            with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False, encoding='utf-8') as tmp_file:
                json.dump(temp_json_data, tmp_file, ensure_ascii=False, indent=2)
                temp_json_path = tmp_file.name
            
            try:
                from test_naver_map_crawl import convert_json_to_robot_commands
                app.logger.info(f"[스케일 팩터 재계산] 도보도로: convert_json_to_robot_commands 호출 (target={target_width_m}m x {target_height_m}m)")
                commands = convert_json_to_robot_commands(
                    json_path=temp_json_path,
                    target_width_m=target_width_m,
                    target_height_m=target_height_m
                )
                app.logger.info(f"[스케일 팩터 재계산] 도보도로: {len(commands)}개 명령 생성 완료")
                
                # 속도 설정 적용 (commands에 linear/duration 업데이트)
                motion_settings = load_motion_settings()
                current_mode = motion_settings.get("mode", "scale")
                current_speed = motion_settings.get("scale_speed", 0.1) if current_mode == "scale" else motion_settings.get("real_speed", 0.2)
                
                for cmd in commands:
                    if cmd.get("distance", 0) > 0.001:
                        cmd["linear"] = current_speed
                        cmd["duration"] = cmd["distance"] / current_speed if current_speed > 0 else 0.0
                
                # 세션에 업데이트된 commands 저장
                commands = store_route_commands(commands)
                
                # 실제 적용된 스케일 팩터 계산
                actual_scale_factor = 1.0
                if commands:
                    scale_factors = []
                    for cmd in commands:
                        orig_dist = cmd.get("original_distance", 0)
                        scaled_dist = cmd.get("distance", 0)
                        if orig_dist > 0.001:
                            scale_factors.append(scaled_dist / orig_dist)
                    if scale_factors:
                        actual_scale_factor = sum(scale_factors) / len(scale_factors)
                
                app.logger.info(f"[스케일 팩터 재계산] 도보도로: 스케일 팩터={actual_scale_factor:.6f}")
                return actual_scale_factor
            finally:
                # 임시 파일 삭제
                try:
                    if os.path.exists(temp_json_path):
                        os.unlink(temp_json_path)
                except Exception:
                    pass
        except Exception as e:
            app.logger.error(f"[스케일 팩터 재계산] 도보도로 재계산 실패: {e}")
            import traceback
            traceback.print_exc()
            # 실패 시 기존 스케일 팩터 반환
            return session.get("last_route_scale_factor", 1.0)
    
    return 1.0


@app.post("/api/save-motion-settings")
def save_motion_settings():
    data = request.get_json(force=True) or {}
    try:
        settings = save_motion_settings_file(data)
        
        # 스케일 팩터 재계산 및 commands 업데이트 (경로가 생성된 경우)
        scale_factor = None
        current_mode = settings.get("mode", "scale")
        current_speed = settings.get("scale_speed", 0.1) if current_mode == "scale" else settings.get("real_speed", 0.2)
        
        stored_commands = get_last_route_commands()
        if stored_commands or session.get("last_route_actions"):
            # REAL 모드일 때는 commands를 original_distance로 변경
            if current_mode == "real":
                if stored_commands:
                    adjusted_commands = []
                    for cmd in stored_commands:
                        adjusted_cmd = cmd.copy()
                        adjusted_cmd["distance"] = cmd.get("original_distance", cmd.get("distance", 0))
                        # 속도 변경 시 duration 재계산
                        if adjusted_cmd.get("distance", 0) > 0.001:
                            adjusted_cmd["duration"] = adjusted_cmd["distance"] / current_speed if current_speed > 0 else 0.0
                        adjusted_cmd["linear"] = current_speed
                        adjusted_commands.append(adjusted_cmd)
                    stored_commands = store_route_commands(adjusted_commands)
                scale_factor = 1.0  # REAL 모드는 100%
            else:
                # SCALE 모드일 때는 스케일 팩터 재계산 및 commands 업데이트
                target_width_m = settings.get("scale_width", 2.0)
                target_height_m = settings.get("scale_height", 3.0)
                scale_factor = recalculate_scale_factor(target_width_m, target_height_m)
                session["last_route_scale_factor"] = scale_factor
                
                # 속도 변경 시 commands의 duration 재계산
                commands = get_last_route_commands()
                if commands:
                    updated_commands = []
                    for cmd in commands:
                        updated_cmd = cmd.copy()
                        # 속도 변경 시 duration 재계산
                        if updated_cmd.get("distance", 0) > 0.001:
                            updated_cmd["duration"] = updated_cmd["distance"] / current_speed if current_speed > 0 else 0.0
                        updated_cmd["linear"] = current_speed
                        updated_commands.append(updated_cmd)
                    store_route_commands(updated_commands)
        
        response_data = {"status": "ok", "settings": settings}
        if scale_factor is not None:
            response_data["scale_factor"] = scale_factor
        
        # 속도 변경 시 commands도 함께 반환 (STEP 04 업데이트용)
        # REAL 모드와 SCALE 모드 모두 commands 반환
        normalized_commands = get_last_route_commands()
        if normalized_commands:
            response_data["commands"] = normalized_commands
        
        return jsonify(response_data)
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 500


@app.post("/api/crawl-walking-route")
def crawl_walking_route():
    """도보도로 크롤링 API"""
    data = request.get_json(force=True) or {}
    url = data.get("url")

    if not url:
        return jsonify({"status": "error", "message": "URL을 입력해주세요."}), 400

    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=".json") as tmp_file:
            output_path = Path(tmp_file.name)
        
        base_cmd = [
            sys.executable,
            str(CRAWL_SCRIPT),
            "--url",
            url,
            "--output-json",
            str(output_path),
            "--wait-time",
            "20",
            "--timeout",
            "90",
        ]
        
        attempts = [
            (base_cmd + ["--headless"], "headless"),
            (base_cmd, "headful"),
        ]
        
        crawl_data = None
        last_error: str | None = None
        
        for cmd, mode in attempts:
            try:
                app.logger.info("[크롤링] (%s) Subprocess 실행: %s", mode, " ".join(cmd))
                result = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    timeout=240,
                )
            except subprocess.TimeoutExpired as exc:
                app.logger.error("[크롤링] (%s) subprocess 시간 초과: %s초", mode, exc.timeout)
                last_error = f"크롤링이 시간 초과되었습니다. (mode={mode})"
                continue
            
            if result.returncode != 0:
                stderr = result.stderr.strip()
                stdout = result.stdout.strip()
                app.logger.error("[크롤링] (%s) subprocess 실패 (code=%s)", mode, result.returncode)
                if stdout:
                    app.logger.error("[크롤링] stdout: %s", stdout)
                if stderr:
                    app.logger.error("[크롤링] stderr: %s", stderr)
                last_error = stderr or stdout or "경로 텍스트 추출 실패"
                continue
            
            with open(output_path, "r", encoding="utf-8") as f:
                crawl_data = json.load(f)
            break
        
        if crawl_data is None:
            return jsonify({
                "status": "error",
                "message": "경로 텍스트 추출 실패",
                "detail": last_error,
            }), 500
        
        return jsonify({
            "status": "ok",
            "route_texts": crawl_data.get("route_texts", []),
            "crosswalks": crawl_data.get("crosswalks", []),
        })
    except Exception as exc:
        app.logger.exception("[크롤링] 처리 중 예외 발생: %s", exc)
        return jsonify({"status": "error", "message": str(exc)}), 500
    finally:
        try:
            if 'output_path' in locals() and output_path.exists():
                output_path.unlink()
        except Exception:
            pass


@app.post("/api/process-walking-route")
def process_walking_route():
    """도보도로 경로 처리 API (횡단보도 방향 포함)"""
    data = request.get_json(force=True) or {}
    route_texts = data.get("route_texts", [])
    crosswalks = data.get("crosswalks", [])
    url = data.get("url", "")

    if not route_texts:
        return jsonify({"status": "error", "message": "경로 텍스트가 없습니다."}), 400

    try:
        # 횡단보도 방향을 포함하여 액션 파싱
        actions = parse_route_actions(route_texts, crosswalks)
        
        # 세션에 경로 데이터 저장 (다운로드용)
        # 도보도로는 actions를 commands 형식으로 변환
        # move_path.py 다운로드용: 스케일링 적용된 commands 생성 (move_full_path.py와 동일한 로직)
        motion_settings = load_motion_settings()
        target_width_m = motion_settings.get("scale_width", 2.0)
        target_height_m = motion_settings.get("scale_height", 3.0)
        current_mode = motion_settings.get("mode", "scale")
        current_speed = motion_settings.get("scale_speed", 0.1) if current_mode == "scale" else motion_settings.get("real_speed", 0.2)
        
        # 원본 경로 데이터를 파일로 저장 (스케일 변경 시 재계산용) - 차선도로와 동일한 로직
        walking_route_data = {
            "url": url,
            "route_texts": route_texts,
            "crosswalks": crosswalks,
            "actions": actions,
        }
        try:
            with open(WALKING_ROUTE_DATA_PATH, 'w', encoding='utf-8') as f:
                json.dump(walking_route_data, f, ensure_ascii=False, indent=2)
            app.logger.info(f"[도보도로 데이터 저장] 파일 저장 완료: {WALKING_ROUTE_DATA_PATH}")
        except Exception as e:
            app.logger.warning(f"[도보도로 데이터 저장] 파일 저장 실패: {e}")
        
        # 임시 JSON 파일 생성하여 convert_json_to_robot_commands() 사용
        # (스케일링 로직이 이미 구현되어 있음)
        temp_json_data = {
            "url": url,
            "actions": actions,
            "crosswalks": crosswalks
        }
        
        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False, encoding='utf-8') as tmp_file:
            json.dump(temp_json_data, tmp_file, ensure_ascii=False, indent=2)
            temp_json_path = tmp_file.name
        
        try:
            # convert_json_to_robot_commands()를 사용하여 스케일링 적용된 commands 생성
            from test_naver_map_crawl import convert_json_to_robot_commands
            app.logger.info(f"[도보도로] convert_json_to_robot_commands 호출: target_width={target_width_m}m, target_height={target_height_m}m")
            commands = convert_json_to_robot_commands(
                json_path=temp_json_path,
                target_width_m=target_width_m,
                target_height_m=target_height_m
            )
            app.logger.info(f"[도보도로] convert_json_to_robot_commands 성공: {len(commands)}개 명령 생성")
            
            # 속도 설정 적용 (commands에 linear/duration 업데이트)
            for cmd in commands:
                if cmd.get("distance", 0) > 0.001:
                    cmd["linear"] = current_speed
                    cmd["duration"] = cmd["distance"] / current_speed if current_speed > 0 else 0.0
            
            commands = store_route_commands(commands)
            
            # 실제 적용된 스케일 팩터 계산 (commands의 distance/original_distance 평균)
            actual_scale_factor = 1.0
            if commands:
                scale_factors = []
                for cmd in commands:
                    orig_dist = cmd.get("original_distance", 0)
                    scaled_dist = cmd.get("distance", 0)
                    if orig_dist > 0.001:
                        scale_factors.append(scaled_dist / orig_dist)
                if scale_factors:
                    actual_scale_factor = sum(scale_factors) / len(scale_factors)
            
            app.logger.info(f"[도보도로] 스케일 팩터: {actual_scale_factor:.6f}")
            
        except Exception as e:
            app.logger.error(f"[도보도로] convert_json_to_robot_commands 실패: {e}, fallback 로직 사용")
            import traceback
            traceback.print_exc()
            
            # 실패 시 기본 commands 생성 (스케일링 없이) - 차선도로 직선 로직 참고
            # 이렇게 하면 최소한 다운로드는 가능하고, 스케일은 디폴트로 적용됨
            commands = []
            segment_idx = 0
            
            # 첫 번째 액션 (출발지) 처리
            if actions and actions[0].get("action") in ["기타", "출발"]:
                commands.append({
                    "segment": segment_idx,
                    "start_point": [0.0, 0.0],
                    "end_point": [0.0, 0.0],
                    "distance": 0.0,
                    "original_distance": 0.0,
                    "turn_angle": 0.0,
                    "instruction": "출발",
                    "linear": current_speed,
                    "angular": 0.0,
                    "duration": 0.0,
                    "type": "straight"
                })
                segment_idx += 1
                actions = actions[1:]  # 첫 번째 액션 제거
            
            # 나머지 액션 처리
            for i, action in enumerate(actions):
                action_type = action.get("action", "")
                distance = action.get("distance", 0) or 0
                text = action.get("text", "")
                
                # 첫 번째 action: 회전 제외, 직진만 처리
                if i == 0:
                    turn_angle = 0.0
                else:
                    # 회전 각도 결정
                    turn_angle = 0.0
                    if "횡단보도_좌회전" in action_type:
                        turn_angle = 90.0
                    elif "횡단보도_우회전" in action_type:
                        turn_angle = -90.0
                    elif "횡단보도_직진" in action_type:
                        turn_angle = 0.0
                    elif "좌회전" in action_type or "왼쪽" in action_type or "왼" in action_type:
                        turn_angle = 90.0
                    elif "우회전" in action_type or "오른쪽" in action_type or "우측" in action_type or "우" in action_type:
                        turn_angle = -90.0
                
                # 도착지 처리 (마지막 action)
                if "도착" in text and i == len(actions) - 1:
                    arrival_turn_angle = 0.0
                    if "도로의 오른쪽에 있습니다" in text:
                        arrival_turn_angle = -90.0
                    elif "도로의 왼쪽에 있습니다" in text:
                        arrival_turn_angle = 90.0
                    
                    commands.append({
                        "segment": segment_idx,
                        "start_point": [0.0, 0.0],
                        "end_point": [0.0, 0.0],
                        "distance": 0.0,
                        "original_distance": 0.0,
                        "turn_angle": arrival_turn_angle,
                        "instruction": "도착" + (f" (도로의 {'오른쪽' if arrival_turn_angle < 0 else '왼쪽'}에 있습니다)" if arrival_turn_angle != 0 else ""),
                        "linear": 0.0,
                        "angular": 0.0,
                        "duration": 0.0,
                        "type": "arrival"
                    })
                    break
                
                # 원본 거리를 그대로 사용 (스케일링 없음)
                original_distance = float(distance)
                scaled_distance = original_distance  # fallback이므로 스케일링 없음
                duration = scaled_distance / current_speed if current_speed > 0 else 0.0
                
                # 명령 추가
                instruction = action_type.replace("횡단보도_", "횡단보도 ")
                if distance > 0:
                    instruction = f"{instruction} {distance}m"
                
                commands.append({
                    "segment": segment_idx,
                    "start_point": [0.0, 0.0],
                    "end_point": [0.0, 0.0],
                    "distance": scaled_distance,
                    "original_distance": original_distance,
                    "turn_angle": turn_angle,
                    "instruction": instruction,
                    "linear": current_speed,
                    "angular": 0.0,
                    "duration": duration,
                    "type": "straight" if abs(turn_angle) < 1.0 else "turn"
                })
                segment_idx += 1
            
            commands = store_route_commands(commands)
            app.logger.info(f"[도보도로] fallback 로직으로 {len(commands)}개 명령 생성 (스케일링 없음)")
            
            # fallback이므로 스케일 팩터 1.0
            actual_scale_factor = 1.0
        finally:
            # 임시 파일 삭제
            try:
                if os.path.exists(temp_json_path):
                    os.unlink(temp_json_path)
            except Exception:
                pass
        
        session["last_route_mode"] = "walk"
        session["last_route_actions"] = actions  # actions도 저장 (주행 시작용)
        session["last_route_url"] = url  # URL도 저장
        session["last_route_crosswalks"] = crosswalks  # crosswalks도 저장 (재계산용)
        session["last_route_scale_factor"] = actual_scale_factor

        # 도보도로 경로 이미지 생성
        route_image_url = None
        if url:
            try:
                from pathlib import Path
                route_image_dir = BASE_DIR / "dashboard" / "static" / "routes"
                app.logger.info(f"[도보도로] 경로 이미지 생성 시도: URL={url[:80]}..., actions={len(actions)}개")
                app.logger.info(f"[도보도로] output_dir={route_image_dir}")
                route_image_url = visualize_walking_route_image(
                    url=url,
                    actions=actions,
                    output_dir=route_image_dir,
                )
                app.logger.info(f"[도보도로] visualize_walking_route_image 반환값: {route_image_url}")
                if route_image_url:
                    app.logger.info(f"[도보도로] 경로 이미지 생성 완료: {route_image_url}")
                else:
                    app.logger.warning(f"[도보도로] 경로 이미지 생성 실패: route_image_url이 None입니다.")
            except Exception as img_exc:
                app.logger.exception(f"[도보도로] 경로 이미지 생성 중 예외 발생: {img_exc}")

        return jsonify({
            "status": "ok",
            "actions": actions,
            "route_image": route_image_url,
            "scale_factor": actual_scale_factor,  # 실제 스케일 비율 포함
            "commands": commands,  # commands도 반환 (STEP 04 업데이트용)
        })
    except Exception as exc:
        app.logger.exception(f"[도보도로] 경로 처리 중 예외 발생: {exc}")
        return jsonify({"status": "error", "message": str(exc)}), 500


def generate_move_path_py(commands: list, speed: float = 0.1) -> str:
    """경로 명령을 받아서 move_path.py 파일 내용 생성
    
    Args:
        commands: 로봇 명령 리스트
        speed: 로봇 속도 (m/s), 기본값 0.1
    """
    commands = flatten_commands(commands)
    
    commands_repr = pformat(commands, indent=4, width=100, compact=False)
    if commands_repr.startswith("[") and commands_repr.endswith("]"):
        commands_repr = commands_repr[1:-1].strip()
    commands_block = "\n".join(" " * 12 + line for line in commands_repr.splitlines())
    
    template = dedent(f"""\
        #!/usr/bin/env python3
        \"\"\"
        ROS2 node: execute generated path commands.
        \"\"\"
        import math
        import time

        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist


        class RobotMover(Node):
            def __init__(self):
                super().__init__('robot_mover')
                self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
                self.get_logger().info('Robot Mover node started.')
                self.default_linear_speed = {speed:.1f}  # m/s
                self.default_angular_speed = 0.5  # rad/s
                
                # Publisher 연결 대기 (첫 메시지 누락 방지)
                self.get_logger().info('Waiting for publisher connection...')
                time.sleep(1.0)
                
                # 더미 메시지 발행 (연결 확인)
                dummy_twist = Twist()
                for _ in range(5):
                    self.publisher.publish(dummy_twist)
                    time.sleep(0.1)
                self.get_logger().info('Publisher connected!')

            def move_straight(self, distance_m: float, speed_mps: float = None):
                if distance_m <= 0:
                    return

                speed = speed_mps if speed_mps is not None else self.default_linear_speed
                if speed <= 0:
                    speed = 0.1

                self.get_logger().info(f'Move start: {{distance_m:.3f}} m (speed: {{speed:.2f}}m/s)')
                
                # 명령 시작 전 정지 명령 먼저 발행
                stop_twist = Twist()
                for _ in range(3):
                    self.publisher.publish(stop_twist)
                    time.sleep(0.05)
                
                twist = Twist()
                twist.linear.x = speed

                duration = distance_m / speed
                start_time = time.time()

                while (time.time() - start_time) < duration:
                    self.publisher.publish(twist)
                    time.sleep(0.05)

                # 정지 (여러 번 발행)
                twist.linear.x = 0.0
                for _ in range(5):
                    self.publisher.publish(twist)
                    time.sleep(0.05)
                
                self.get_logger().info(f'Move done: {{distance_m:.3f}} m')

            def rotate(self, angle_deg: float, angular_speed_radps: float = None):
                if abs(angle_deg) < 0.1:
                    return

                angular_speed = angular_speed_radps if angular_speed_radps is not None else self.default_angular_speed

                self.get_logger().info(f'Rotate start: {{angle_deg:.2f}} deg')
                
                # 명령 시작 전 정지 명령 먼저 발행
                stop_twist = Twist()
                for _ in range(3):
                    self.publisher.publish(stop_twist)
                    time.sleep(0.05)
                
                twist = Twist()
                angle_rad = math.radians(angle_deg)
                twist.angular.z = angular_speed if angle_deg > 0 else -angular_speed

                duration = abs(angle_rad) / angular_speed
                start_time = time.time()

                while (time.time() - start_time) < duration:
                    self.publisher.publish(twist)
                    time.sleep(0.05)

                # 정지 (여러 번 발행)
                twist.angular.z = 0.0
                for _ in range(5):
                    self.publisher.publish(twist)
                    time.sleep(0.05)
                
                self.get_logger().info(f'Rotate done: {{angle_deg:.2f}} deg')

            def move_curve(self, distance_m: float, linear_speed: float, angular_speed: float):
                if distance_m <= 0 or linear_speed <= 0:
                    return

                self.get_logger().info(
                    f'Curve start: distance={{distance_m:.3f}}m, linear={{linear_speed:.2f}}m/s, angular={{angular_speed:.2f}}rad/s'
                )
                
                # 명령 시작 전 정지 명령 먼저 발행
                stop_twist = Twist()
                for _ in range(3):
                    self.publisher.publish(stop_twist)
                    time.sleep(0.05)
                
                twist = Twist()
                twist.linear.x = linear_speed
                twist.angular.z = angular_speed

                duration = distance_m / linear_speed
                start_time = time.time()

                while (time.time() - start_time) < duration:
                    self.publisher.publish(twist)
                    time.sleep(0.05)

                # 정지 (여러 번 발행)
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                for _ in range(5):
                    self.publisher.publish(twist)
                    time.sleep(0.05)
                
                self.get_logger().info('Curve done')

            def execute_path(self):
                commands = [
{commands_block}
                ]
                self.get_logger().info(f'Total segments: {{len(commands)}}')
                
                # 실행 전 완전 정지 확인 (토픽 누락 방지)
                self.get_logger().info('Checking stop state before execution...')
                stop_twist = Twist()
                for _ in range(10):
                    self.publisher.publish(stop_twist)
                    time.sleep(0.1)
                self.get_logger().info('Stop state confirmed, starting execution!')

                for i, cmd in enumerate(commands):
                    self.get_logger().info(f\"--- Segment {{i + 1}}/{{len(commands)}}: {{cmd['instruction']}} ---\")

                    # 각 명령 전 짧은 대기
                    time.sleep(0.3)

                    cmd_type = cmd.get(\"type\", \"straight\")
                    linear_speed = cmd.get(\"linear\", self.default_linear_speed)
                    angular_speed = cmd.get(\"angular\", 0.0)
                    distance = cmd.get(\"distance\", 0.0)
                    turn_angle = cmd.get(\"turn_angle\", 0.0)

                    if cmd_type == \"smooth_curve\" and abs(angular_speed) > 1e-3:
                        self.move_curve(distance, linear_speed, angular_speed)
                        time.sleep(0.5)
                        continue

                    # 먼저 회전 (이미지 생성 순서와 일치: 회전 → 직진)
                    if abs(turn_angle) > 0.1:
                        self.rotate(turn_angle)
                        time.sleep(0.5)

                    # 그 다음 직진 (스케일링 적용된 distance 사용)
                    # move_path.py는 스케일링이 이미 적용된 commands를 사용
                    if distance > 0:
                        self.move_straight(distance, linear_speed)
                        time.sleep(0.5)

                self.get_logger().info('All segments completed.')


        def main(args=None):
            rclpy.init(args=args)
            mover = RobotMover()
            try:
                mover.execute_path()
            except KeyboardInterrupt:
                pass
            finally:
                mover.destroy_node()
                rclpy.shutdown()


        if __name__ == '__main__':
            main()
        """)
    return template


@app.get("/api/download-move-path")
def download_move_path():
    """move_path.py 파일 다운로드"""
    commands = get_last_route_commands()
    
    if not commands:
        return jsonify({"status": "error", "message": "경로가 생성되지 않았습니다. 먼저 경로를 생성해주세요."}), 400
    
    # 현재 motion settings 읽기
    motion_settings = load_motion_settings()
    current_mode = motion_settings.get("mode", "scale")
    current_speed = motion_settings.get("scale_speed", 0.1) if current_mode == "scale" else motion_settings.get("real_speed", 0.2)
    
    tmp_path = None
    try:
        # move_path.py 파일 내용 생성 (현재 속도 적용)
        file_content = generate_move_path_py(commands, speed=current_speed)
        
        # 임시 파일 생성
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False, encoding='utf-8') as tmp_file:
            tmp_file.write(file_content)
            tmp_path = Path(tmp_file.name)
        
        return send_file(
            str(tmp_path),
            as_attachment=True,
            download_name='move_path.py',
            mimetype='text/x-python'
        )
    except Exception as exc:
        return jsonify({"status": "error", "message": str(exc)}), 500
    finally:
        # 임시 파일 삭제 (send_file이 완료된 후 실행됨)
        if tmp_path and tmp_path.exists():
            try:
                # 약간의 지연 후 삭제 (파일 전송 완료 대기)
                import threading
                def delayed_delete():
                    import time
                    time.sleep(1)
                    try:
                        tmp_path.unlink()
                    except:
                        pass
                threading.Thread(target=delayed_delete, daemon=True).start()
            except:
                pass


@app.get("/api/download-robot-test")
def download_robot_test():
    """test_move_command.py 파일 다운로드"""
    try:
        file_path = ROOT_SRC_DIR / "test_move_command.py"
        if not file_path.exists():
            return jsonify({"status": "error", "message": "파일을 찾을 수 없습니다."}), 404
        
        return send_file(
            str(file_path),
            as_attachment=True,
            download_name='test_move_command.py',
            mimetype='text/x-python'
        )
    except Exception as exc:
        app.logger.error(f"[다운로드] test_move_command.py 오류: {exc}")
        return jsonify({"status": "error", "message": str(exc)}), 500


@app.get("/api/download-robot-move")
def download_robot_move():
    """move_full_path.py 파일 다운로드"""
    try:
        file_path = ROOT_SRC_DIR / "move_full_path.py"
        if not file_path.exists():
            return jsonify({"status": "error", "message": "파일을 찾을 수 없습니다."}), 404
        
        return send_file(
            str(file_path),
            as_attachment=True,
            download_name='move_full_path.py',
            mimetype='text/x-python'
        )
    except Exception as exc:
        app.logger.error(f"[다운로드] move_full_path.py 오류: {exc}")
        return jsonify({"status": "error", "message": str(exc)}), 500


if __name__ == "__main__":
    # Render.com은 PORT 환경변수를 사용, 없으면 DASHBOARD_PORT, 기본값 8088
    port = int(os.environ.get("PORT", os.environ.get("DASHBOARD_PORT", 8088)))
    # 프로덕션 환경에서는 debug=False
    debug_mode = os.environ.get("FLASK_DEBUG", "False").lower() == "true"
    app.run(host="0.0.0.0", port=port, debug=debug_mode)

