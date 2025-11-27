#!/usr/bin/env python3
"""
도보도로 경로 이미지 생성 모듈
크롤링한 텍스트와 사용자 입력 횡단보도 방향을 기반으로 경로 이미지 생성
"""
import math
import re
from datetime import datetime
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("[WARNING] PIL/Pillow가 설치되지 않았습니다. 이미지 생성 기능이 비활성화됩니다.")

# 공통 경로 설정
BASE_DIR = Path(__file__).resolve().parent
DASHBOARD_STATIC_DIR = BASE_DIR / "dashboard" / "static"
ROUTE_IMAGE_DIR = DASHBOARD_STATIC_DIR / "routes"

# api_to_position.py에서 visualize_path_image 함수 사용
from api_to_position import visualize_path_image


def extract_coordinates_from_url(url: str):
    """
    네이버 지도 길찾기 URL에서 출발지/도착지 좌표 추출
    
    Args:
        url: 네이버 지도 길찾기 URL
            예: https://map.naver.com/p/directions/14153002.4755856,4501066.3018202,경기.../14153313.7248819,4501217.2234268,신한...
    
    Returns:
        (start_lon, start_lat, goal_lon, goal_lat) 또는 None
    """
    try:
        # /directions/ 뒤의 패턴 찾기: 숫자,숫자,텍스트/숫자,숫자,텍스트
        pattern = r'/directions/([0-9.]+),([0-9.]+),.*?/([0-9.]+),([0-9.]+),'
        match = re.search(pattern, url)
        if match:
            start_lon = float(match.group(1))
            start_lat = float(match.group(2))
            goal_lon = float(match.group(3))
            goal_lat = float(match.group(4))
            return start_lon, start_lat, goal_lon, goal_lat
        return None
    except Exception as e:
        print(f"[WARNING] URL에서 좌표 추출 실패: {e}")
        return None


def visualize_walking_route_image(
    url: str,
    actions: list,
    output_dir=None
):
    """
    도보도로 경로를 시각화하여 이미지 파일로 저장
    actions의 거리와 각도(0, 90, -90)를 기반으로 실제 경로 포인트 생성
    
    Args:
        url: 네이버 지도 길찾기 URL (좌표 추출용)
        actions: 경로 액션 리스트 [{"distance": float, "action": str, "turn_angle": float}, ...]
        output_dir: 저장할 디렉토리
    
    Returns:
        저장된 이미지 파일의 웹 경로 (str) 또는 None
    """
    if not PIL_AVAILABLE:
        print("[WARNING] PIL/Pillow가 없어 이미지를 생성할 수 없습니다.")
        return None
    
    # URL에서 좌표 추출
    coords = extract_coordinates_from_url(url)
    if not coords:
        print("[WARNING] URL에서 좌표를 추출할 수 없습니다.")
        return None
    
    start_lon, start_lat, goal_lon, goal_lat = coords
    
    if not actions or len(actions) == 0:
        print("[WARNING] actions가 비어있습니다.")
        return None
    
    # 거리가 있는 첫 번째 action 찾기
    first_distance_m = None
    first_action_idx = None
    for idx, action in enumerate(actions):
        distance_m = float(action.get("distance", 0) or 0)
        if distance_m > 0.001:
            first_distance_m = distance_m
            first_action_idx = idx
            break
    
    if first_distance_m is None or first_distance_m <= 0:
        print("[WARNING] 거리가 있는 action을 찾을 수 없습니다.")
        return None
    
    # 비율: 100px = 첫 번째 거리가 있는 action 거리(m)
    # 예: 100px = 13m → 1m = 100/13 px
    px_per_m = 100.0 / first_distance_m
    
    print(f"[INFO] 첫 번째 거리가 있는 action (idx={first_action_idx}) 거리: {first_distance_m}m → 비율: {px_per_m:.2f} px/m")
    
    # 픽셀 좌표계로 경로 포인트 생성
    # 좌표계: 왼쪽 상단이 (0, 0), 왼쪽 = x 감소, 아래 = y 증가
    # heading: 0=위쪽(북), 90=오른쪽(동), 180=아래쪽(남), 270=왼쪽(서)
    keypoints_px = []
    path_coordinates_px = []
    
    # 시작 포인트: 왼쪽 상단 (0, 0)
    current_x_px = 0.0
    current_y_px = 0.0
    current_heading = 90.0  # 오른쪽 방향 (90도)
    
    keypoints_px.append([current_x_px, current_y_px])
    path_coordinates_px.append([current_x_px, current_y_px])
    
    print(f"[INFO] 시작 위치: ({current_x_px:.1f}, {current_y_px:.1f}) px, 초기 방향: {current_heading:.1f}도 (오른쪽)")
    
    # actions를 순회하며 경로 포인트 생성
    for idx, action in enumerate(actions):
        distance_m = float(action.get("distance", 0) or 0)
        action_str = action.get("action", "")
        text = action.get("text", "")
        
        # 마지막 action이 도착지인 경우 이미지에서 제외
        if "도착" in text and idx == len(actions) - 1:
            print(f"[INFO] Action {idx+1}/{len(actions)}: 도착지 - 이미지 경로에서 제외")
            continue
        
        # 각도 추출 (0, 90, -90)
        turn_angle = 0.0
        if "횡단보도_좌회전" in action_str or ("횡단보도" in action_str and "좌" in action_str):
            turn_angle = 90.0
        elif "횡단보도_우회전" in action_str or ("횡단보도" in action_str and ("우" in action_str or "-90" in str(action))):
            turn_angle = -90.0
        elif "좌회전" in action_str or "왼쪽" in action_str or "왼" in action_str:
            turn_angle = 90.0
        elif "우회전" in action_str or "오른쪽" in action_str or "우측" in action_str or "우" in action_str:
            turn_angle = -90.0
        
        print(f"[INFO] Action {idx+1}/{len(actions)}: '{action_str}' | 회전: {turn_angle}도, 거리: {distance_m}m, 현재방향: {current_heading:.1f}도")
        
        # 첫 번째 action: 회전 제외, 직진만 처리
        if idx == 0:
            if abs(turn_angle) > 0.1:
                print(f"[INFO]   → 첫 번째 action이므로 회전 무시, 직진만 처리")
                turn_angle = 0.0
        else:
            # 회전 (각도가 0이 아닐 때만) - 회전 후 직진 순서
            # 터틀봇: 우회전(-90도) = 시계방향, 좌회전(+90도) = 반시계방향
            # 이미지 좌표계: 0=위, 90=오른쪽, 180=아래, 270=왼쪽 (반시계방향 증가)
            # 터틀봇 기준으로 회전: turn_angle 부호 그대로 적용하면 시계방향/반시계방향 반대로 됨
            if abs(turn_angle) > 0.1:
                # turn_angle의 부호를 반대로: -90(우회전)을 시계방향으로, +90(좌회전)을 반시계방향으로
                current_heading = (current_heading - turn_angle) % 360
                print(f"[INFO]   → 회전 {turn_angle}도 완료, 새 방향: {current_heading:.1f}도")
                # 회전 지점도 keypoint로 추가
                keypoints_px.append([current_x_px, current_y_px])
                path_coordinates_px.append([current_x_px, current_y_px])
        
        # 직진 이동 (거리가 0보다 클 때만)
        if distance_m > 0.001:
            # 거리를 픽셀로 변환
            if idx == first_action_idx:
                # 첫 번째 거리가 있는 action: 무조건 100px
                distance_px = 100.0
            else:
                # 나머지 action: 비율 적용
                distance_px = distance_m * px_per_m
            
            # 현재 방향으로 거리만큼 이동
            heading_rad = math.radians(current_heading)
            
            # 좌표계 변환:
            # heading 0도(위) → dx=0, dy=-distance
            # heading 90도(오른쪽) → dx=distance, dy=0
            # heading 180도(아래) → dx=0, dy=distance
            # heading 270도(왼쪽) → dx=-distance, dy=0
            dx_px = math.sin(heading_rad) * distance_px
            dy_px = -math.cos(heading_rad) * distance_px  # y축 반전 (위가 -)
            
            # 직선 세그먼트 시작점 저장
            segment_start_x = current_x_px
            segment_start_y = current_y_px
            
            # 새로운 위치 계산
            current_x_px += dx_px
            current_y_px += dy_px
            
            # 이동 후 위치를 keypoint에 추가
            keypoints_px.append([current_x_px, current_y_px])
            print(f"[DEBUG]   → 이동 후 위치: ({current_x_px:.1f}, {current_y_px:.1f}) px")
            
            # path_coordinates에는 더 세밀한 점들을 추가 (직선 세그먼트)
            num_points = max(2, int(distance_px / 5.0))  # 5px마다 점 추가
            for i in range(1, num_points):
                ratio = i / num_points
                interp_x = segment_start_x + (current_x_px - segment_start_x) * ratio
                interp_y = segment_start_y + (current_y_px - segment_start_y) * ratio
                path_coordinates_px.append([interp_x, interp_y])
            
            # 마지막 위치 추가
            path_coordinates_px.append([current_x_px, current_y_px])
    
    # 최소 2개 포인트 필요
    if len(keypoints_px) < 2:
        print("[WARNING] 생성된 keypoint가 부족합니다.")
        keypoints_px = [[0, 0], [100, 0]]
        path_coordinates_px = keypoints_px
    
    print(f"[INFO] 도보 경로 포인트 생성: {len(keypoints_px)}개 keypoints, {len(path_coordinates_px)}개 전체 경로 포인트")
    
    # 픽셀 좌표를 위경도 좌표로 변환 (이미지 생성용)
    # 위경도 좌표는 실제 위치와 무관하게 시각화만을 위해 사용
    # 경로의 범위를 계산하여 중앙에 배치
    if len(path_coordinates_px) > 0:
        x_coords = [p[0] for p in path_coordinates_px]
        y_coords = [p[1] for p in path_coordinates_px]
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # 범위를 위경도로 변환 (시각화용, 실제 위치와 무관)
        # 중앙 좌표를 출발지로 설정
        center_lon = start_lon
        center_lat = start_lat
        
        # 1px ≈ 0.00001도 (대략적)
        scale_deg_per_px = 0.00001
        
        keypoints = []
        path_coordinates = []
        
        for px, py in keypoints_px:
            # 중앙 기준으로 변환
            lon = center_lon + (px - (min_x + max_x) / 2) * scale_deg_per_px
            lat = center_lat - (py - (min_y + max_y) / 2) * scale_deg_per_px  # y축 반전
            keypoints.append([lon, lat])
        
        for px, py in path_coordinates_px:
            lon = center_lon + (px - (min_x + max_x) / 2) * scale_deg_per_px
            lat = center_lat - (py - (min_y + max_y) / 2) * scale_deg_per_px
            path_coordinates.append([lon, lat])
    else:
        keypoints = [[start_lon, start_lat], [goal_lon, goal_lat]]
        path_coordinates = keypoints
    
    # 이미지 생성
    print(f"[DEBUG] visualize_walking_route_image: keypoints={len(keypoints)}, path_coordinates={len(path_coordinates)}, output_dir={output_dir}")
    if len(keypoints) < 2:
        print(f"[ERROR] visualize_walking_route_image: keypoints가 2개 미만입니다. (keypoints={keypoints})")
        return None
    if len(path_coordinates) == 0:
        print(f"[ERROR] visualize_walking_route_image: path_coordinates가 비어있습니다.")
        return None
    
    try:
        image_path = visualize_path_image(keypoints, path_coordinates, output_dir)
        print(f"[DEBUG] visualize_walking_route_image: image_path={image_path}")
        
        if image_path:
            # 고정된 파일명 사용
            web_path = "/static/routes/current_route_visualization.png"
            print(f"[DEBUG] visualize_walking_route_image: web_path={web_path}")
            return web_path
        print(f"[WARNING] visualize_walking_route_image: 이미지 생성 실패 (image_path가 None)")
        return None
    except Exception as e:
        print(f"[ERROR] visualize_walking_route_image: 예외 발생: {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

