import requests
import math
from datetime import datetime
from pathlib import Path
from typing import Any, List, Tuple

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("[WARNING] NumPy가 설치되지 않았습니다. 곡선 보간 기능이 제한됩니다.")

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("[WARNING] PIL/Pillow가 설치되지 않았습니다. 이미지 생성 기능이 비활성화됩니다.")

try:
    if NUMPY_AVAILABLE:
        from scipy.interpolate import CubicSpline
        SCIPY_AVAILABLE = True
    else:
        SCIPY_AVAILABLE = False
except (ImportError, ValueError, AttributeError, Exception):
    SCIPY_AVAILABLE = False
    # NumPy/SciPy 버전 호환성 문제로 인한 오류를 조용히 처리
    # (대시보드 실행 시 오류 메시지가 너무 많이 출력되지 않도록)

BASE_DIR = Path(__file__).resolve().parent
DASHBOARD_STATIC_DIR = BASE_DIR / "dashboard" / "static"
ROUTE_IMAGE_DIR = DASHBOARD_STATIC_DIR / "routes"

# -----------------------------
# 1) 네이버 API 키 (환경변수 또는 기본값)
# -----------------------------
import os

# 환경변수에서 API 키 읽기, 없으면 기본값 사용
# 사용자가 제공한 기본 API 키
def clean_api_key(key):
    """API 키에서 공백, 따옴표, 줄바꿈 제거"""
    if not key:
        return ""
    return str(key).strip().strip('"').strip("'").strip()

raw_client_id = os.environ.get("NAVER_CLIENT_ID", "713xl4yz8m")
raw_client_secret = os.environ.get("NAVER_CLIENT_SECRET", "lCVsMexZ5D16RAT9LmHrKXF67AeqCesuId0uREIW")

client_id = clean_api_key(raw_client_id)
client_secret = clean_api_key(raw_client_secret)

# API 키 검증
if not client_id or not client_secret:
    print(f"[WARNING] API 키가 비어있습니다! Client ID: '{client_id}', Secret: {'설정됨' if client_secret else '없음'}")

# -----------------------------
# 2) 주소 -> 위경도 함수
# -----------------------------
def geocode(address):
    url = "https://maps.apigw.ntruss.com/map-geocode/v2/geocode"
    params = {"query": address}
    headers = {
        "X-NCP-APIGW-API-KEY-ID": client_id,
        "X-NCP-APIGW-API-KEY": client_secret,
        "Accept": "application/json"
    }
    
    # 디버깅: API 키 확인 (마스킹)
    print(f"[DEBUG] API 키 사용 중 - Client ID: '{client_id}' (길이: {len(client_id)})")
    print(f"[DEBUG] Client Secret 길이: {len(client_secret)}")
    print(f"[DEBUG] 요청 URL: {url}")
    print(f"[DEBUG] 요청 주소: {address}")
    print(f"[DEBUG] 요청 헤더 Client ID: '{headers['X-NCP-APIGW-API-KEY-ID']}'")
    print(f"[DEBUG] 요청 헤더 Secret (처음 10자): '{headers['X-NCP-APIGW-API-KEY'][:10]}...'")
    
    try:
        res = requests.get(url, params=params, headers=headers, timeout=10)
        
        # 응답 상태 코드 확인
        print(f"[DEBUG] 응답 상태 코드: {res.status_code}")
        
        # 401 또는 403 에러인 경우 상세 정보 출력
        if res.status_code in [401, 403]:
            print(f"[DEBUG] {res.status_code} {'Unauthorized' if res.status_code == 401 else 'Forbidden'} - API 인증/권한 실패")
            print(f"[DEBUG] 응답 헤더: {dict(res.headers)}")
            error_text = res.text
            print(f"[DEBUG] 응답 본문 (전체): {error_text}")
            try:
                error_data = res.json()
                print(f"[DEBUG] 에러 응답 (JSON): {error_data}")
                error_info = error_data.get("error", {})
                error_msg = error_info.get("message", "알 수 없는 오류")
                error_code = error_info.get("errorCode", "UNKNOWN")
                error_details = error_info.get("details", "")
                print(f"[DEBUG] 에러 코드: {error_code}, 메시지: {error_msg}")
                if error_details:
                    print(f"[DEBUG] 상세: {error_details}")
                
                # 에러 코드별 해결 방법
                if error_code == "210":
                    print("\n" + "="*60)
                    print("에러 코드 210: Permission Denied")
                    print("="*60)
                    print("원인: API 서비스 구독이 필요합니다.")
                    print("\n해결 방법:")
                    print("1. 네이버 클라우드 플랫폼 Console 접속")
                    print("   https://console.ncloud.com/")
                    print("2. AI·NAVER API > Application 메뉴로 이동")
                    print("3. Application 'zcdl9o8h7g' 선택")
                    print("4. 서비스 탭으로 이동")
                    print("5. Geocoding API 서비스 찾기")
                    print("6. '신청' 또는 '활성화' 버튼 클릭")
                    print("7. 서비스 활성화 완료까지 대기 (몇 분 소요)")
                    print("="*60)
                    raise Exception(f"API 서비스 구독 필요 (210): Geocoding API 서비스가 활성화되지 않았습니다.\n네이버 클라우드 플랫폼 Console에서 Geocoding API 서비스를 활성화하세요.")
                elif res.status_code == 401:
                    print("\n" + "="*60)
                    print("401 에러 해결 방법:")
                    print("="*60)
                    print("1. 네이버 클라우드 플랫폼 Console 접속")
                    print("   https://console.ncloud.com/")
                    print("2. AI·NAVER API > Application 메뉴로 이동")
                    print("3. Application ID와 Secret이 일치하는지 확인")
                    print("4. Geocoding API 서비스가 활성화되어 있는지 확인")
                    print("5. API 사용량 제한에 걸리지 않았는지 확인")
                    print("="*60)
                    raise Exception(f"API 인증 실패 (401): 클라이언트 ID '{client_id}' 또는 SECRET 키가 잘못되었습니다.\n네이버 클라우드 플랫폼 Console에서 API 키와 서비스 활성화 상태를 확인하세요.")
            except:
                print(f"[DEBUG] JSON 파싱 실패, 텍스트 응답: {error_text[:500]}")
                raise Exception(f"API 호출 실패 ({res.status_code}): {error_text[:200]}")
        
        res.raise_for_status()  # HTTP 에러 체크 (4xx, 5xx)
        
        data = res.json()
        
        # API 에러 응답 확인
        if "error" in data:
            error_msg = data.get("error", {}).get("message", "알 수 없는 오류")
            error_code = data.get("error", {}).get("code", "UNKNOWN")
            print(f"[DEBUG] API 에러 코드: {error_code}, 메시지: {error_msg}")
            raise Exception(f"네이버 API 오류 ({error_code}): {error_msg}")
        
        # 주소가 없는 경우
        if "addresses" not in data or len(data["addresses"]) == 0:
            # API 응답 내용 로깅
            print(f"[DEBUG] API 응답: {data}")
            error_msg = data.get("errorMessage", "")
            if error_msg:
                raise Exception(f"주소를 찾을 수 없습니다: {address}\nAPI 오류: {error_msg}")
            raise Exception(f"주소를 찾을 수 없습니다: {address}\n\n💡 팁:\n- 정확한 도로명 주소를 입력하세요 (예: '경기 성남시 수정구 수정로 167')\n- 장소명만 입력할 경우 지역명을 포함하세요 (예: '성남시 수정구청')\n- 여러 결과가 나올 수 있으므로 가능한 한 구체적으로 입력하세요")
        
        info = data["addresses"][0]
        lon = float(info["x"])
        lat = float(info["y"])
        print(f"[DEBUG] ✅ 주소 변환 성공: ({lat}, {lon})")
        return lon, lat
        
    except requests.exceptions.RequestException as e:
        print(f"[DEBUG] 요청 예외 발생: {type(e).__name__}: {str(e)}")
        raise Exception(f"네트워크 오류 또는 API 호출 실패: {str(e)}")
    except KeyError as e:
        print(f"[DEBUG] 키 에러: {str(e)}")
        raise Exception(f"API 응답 형식 오류: {str(e)}")
    except Exception as e:
        print(f"[DEBUG] 기타 예외: {type(e).__name__}: {str(e)}")
        raise

# -----------------------------
# 3) Directions 15 API 호출 (자동차 경로)
# -----------------------------
def get_directions(start_lon, start_lat, goal_lon, goal_lat, option=None):
    """
    Directions 15 API - 자동차 경로 조회 (보행자 경로 엔드포인트는 제공하지 않음)
    option: 경로 조회 옵션 (기본값: None - API 기본 옵션 사용)
    - trafast: 실시간 빠른 길 (가장빠른)
    - tracomfort: 실시간 편한 길
    - traoptimal: 실시간 최적 (기본값)
    - traavoidtoll: 무료 우선
    - traavoidcaronly: 자동차 전용 도로 회피 우선 (도보에 필수)
    """
    start = f"{start_lon},{start_lat}"
    goal = f"{goal_lon},{goal_lat}"
    
    # Directions 15 API - 자동차 경로 사용 (보행자 경로는 제공하지 않음)
    # 옵션이 제공되면 사용, 없으면 기본값 사용 (옵션 파라미터 없음)
    url = f"https://maps.apigw.ntruss.com/map-direction-15/v1/driving?start={start}&goal={goal}"
    if option:
        url += f"&option={option}"
    
    print(f"[DEBUG] Directions 15 API 요청 URL: {url}")
    
    headers = {
        "X-NCP-APIGW-API-KEY-ID": client_id,
        "X-NCP-APIGW-API-KEY": client_secret
    }
    
    try:
        res = requests.get(url, headers=headers)
        
        # 응답 상태 확인
        print(f"[DEBUG] Directions 15 API 응답 상태: {res.status_code}")
        if res.status_code != 200:
            print(f"[ERROR] API 응답 오류: {res.status_code}")
            print(f"[ERROR] 응답 내용: {res.text[:1000]}")
            res.raise_for_status()
        
        data = res.json()
        
        # 응답 구조 디버깅
        print(f"[DEBUG] API 응답 키: {list(data.keys())}")
        
        # 에러 응답 확인
        if "error" in data:
            error_msg = data.get("error", {}).get("message", "알 수 없는 오류")
            error_code = data.get("error", {}).get("errorCode", "UNKNOWN")
            print(f"[ERROR] API 에러 코드: {error_code}, 메시지: {error_msg}")
            print(f"[ERROR] 전체 응답: {data}")
            raise Exception(f"Directions 15 API 오류 ({error_code}): {error_msg}")
        
        # 응답 구조 확인 및 경로 추출
        if "route" not in data:
            print(f"[ERROR] 응답에 'route' 키가 없습니다. 응답 구조: {list(data.keys())}")
            print(f"[ERROR] 전체 응답: {data}")
            raise Exception(f"Directions 15 API 응답 구조 오류: 'route' 키가 없습니다.")
        
        route_data = data["route"]
        if route_data is None:
            print(f"[ERROR] route 값이 None입니다. 응답 code: {data.get('code')}, message: {data.get('message')}")
            print(f"[ERROR] 전체 응답: {data}")
            raise Exception(f"Directions 15 API 응답 오류: route가 None입니다. (code: {data.get('code')}, message: {data.get('message')})")
        
        if not isinstance(route_data, dict):
            print(f"[ERROR] route 타입이 dict가 아닙니다. 타입: {type(route_data)}, 값: {route_data}")
            print(f"[ERROR] 전체 응답: {data}")
            raise Exception(f"Directions 15 API 응답 구조 오류: route가 dict가 아닙니다.")
        
        # Directions 15 API 응답 구조 확인
        print(f"[DEBUG] route 데이터 키: {list(route_data.keys())}")
        print(f"[DEBUG] route 데이터 구조: {type(route_data)}")
        
        # Directions 15 API는 traoptimal 대신 다른 키를 사용할 수 있음
        # 가능한 키: traoptimal, trafast, tracomfort 등
        if "traoptimal" not in route_data:
            # 다른 옵션 키 확인
            possible_keys = ["trafast", "tracomfort", "traoptimal"]
            traoptimal_data = None
            used_key = None
            
            for key in possible_keys:
                if key in route_data and route_data[key]:
                    traoptimal_data = route_data[key]
                    used_key = key
                    print(f"[DEBUG] '{key}' 키를 사용합니다.")
                    break
            
            if traoptimal_data is None:
                # route가 리스트일 수도 있음
                if isinstance(route_data, list) and len(route_data) > 0:
                    if isinstance(route_data[0], dict) and "path" in route_data[0]:
                        traoptimal_data = route_data
                        used_key = "route[0]"
                        print(f"[DEBUG] route가 리스트 형태입니다. 첫 번째 요소 사용.")
                    else:
                        print(f"[ERROR] 응답에 'traoptimal' 키가 없고, route 구조가 예상과 다릅니다.")
                        print(f"[ERROR] route 키: {list(route_data.keys()) if isinstance(route_data, dict) else '리스트'}")
                        print(f"[ERROR] route 데이터: {route_data}")
                        print(f"[ERROR] 전체 응답: {data}")
                        raise Exception(f"Directions 15 API 응답 구조 오류: 'traoptimal' 키가 없고 route 구조가 예상과 다릅니다.")
                else:
                    print(f"[ERROR] 응답에 'traoptimal' 키가 없습니다. route 키: {list(route_data.keys())}")
                    print(f"[ERROR] route 데이터: {route_data}")
                    print(f"[ERROR] 전체 응답: {data}")
                    raise Exception(f"Directions 15 API 응답 구조 오류: 'traoptimal' 키가 없습니다.")
        else:
            traoptimal_data = route_data["traoptimal"]
            used_key = "traoptimal"
        if traoptimal_data is None:
            print(f"[ERROR] traoptimal 값이 None입니다.")
            print(f"[ERROR] 전체 응답: {data}")
            raise Exception(f"Directions 15 API 응답 오류: traoptimal이 None입니다.")
        
        print(f"[DEBUG] traoptimal_data 타입: {type(traoptimal_data)}, 길이: {len(traoptimal_data) if isinstance(traoptimal_data, list) else 'N/A'}")
        
        if not isinstance(traoptimal_data, list) or len(traoptimal_data) == 0:
            print(f"[ERROR] traoptimal 배열이 비어있거나 리스트가 아닙니다. 타입: {type(traoptimal_data)}, 값: {traoptimal_data}")
            print(f"[ERROR] 전체 응답: {data}")
            raise Exception(f"Directions 15 API 응답 오류: 경로 결과가 없습니다.")
        
        print(f"[DEBUG] traoptimal_data[0] 키: {list(traoptimal_data[0].keys()) if isinstance(traoptimal_data[0], dict) else 'dict 아님'}")
        
        if not isinstance(traoptimal_data[0], dict):
            print(f"[ERROR] traoptimal_data[0]이 dict가 아닙니다. 타입: {type(traoptimal_data[0])}, 값: {traoptimal_data[0]}")
            print(f"[ERROR] 전체 응답: {data}")
            raise Exception(f"Directions 15 API 응답 구조 오류: traoptimal_data[0]이 dict가 아닙니다.")
        
        if "path" not in traoptimal_data[0]:
            print(f"[ERROR] traoptimal_data[0]에 'path' 키가 없습니다. 키: {list(traoptimal_data[0].keys())}")
            print(f"[ERROR] traoptimal_data[0] 내용: {traoptimal_data[0]}")
            print(f"[ERROR] 전체 응답: {data}")
            raise Exception(f"Directions 15 API 응답 구조 오류: 'path' 키가 없습니다.")
        
        path = traoptimal_data[0]["path"]
        print(f"[DEBUG] ✅ 경로 포인트 수: {len(path)}")
        return path  # [[lon, lat], [lon, lat], ...]
        
    except requests.exceptions.RequestException as e:
        print(f"[ERROR] Directions 15 API 요청 실패: {type(e).__name__}: {str(e)}")
        raise Exception(f"Directions 15 API 네트워크 오류: {str(e)}")
    except KeyError as e:
        print(f"[ERROR] Directions 15 API 응답 파싱 오류: {str(e)}")
        print(f"[ERROR] 전체 응답: {data if 'data' in locals() else 'N/A'}")
        raise Exception(f"Directions 15 API 응답 형식 오류: {str(e)}")
    except Exception as e:
        print(f"[ERROR] Directions 15 API 처리 중 오류: {type(e).__name__}: {str(e)}")
        raise

# -----------------------------
# 4) 곡선 최적화 경로 처리 함수들
# -----------------------------

def scale_path(points, target_width_m, target_height_m):
    """
    경로를 목표 공간에 맞게 스케일링
    fit_path_to_area()와 동일한 기능이지만 곡선 처리용 별도 함수
    """
    return fit_path_to_area(points, target_width_m, target_height_m)


def compute_angles(points):
    """
    세 점(P(i-1), P(i), P(i+1))의 회전각 계산
    
    Args:
        points: [[x, y], ...] 좌표 리스트
    
    Returns:
        angles: [None, angle1, angle2, ..., None] 각 점에서의 회전각 (도)
    """
    if len(points) < 3:
        return [None] * len(points)
    
    angles = [None]  # 첫 번째 점은 각도 없음
    
    for i in range(1, len(points) - 1):
        p_prev = points[i - 1]
        p_curr = points[i]
        p_next = points[i + 1]
        
        # 벡터 계산
        v1 = [p_curr[0] - p_prev[0], p_curr[1] - p_prev[1]]
        v2 = [p_next[0] - p_curr[0], p_next[1] - p_curr[1]]
        
        # 각도 계산
        angle1 = math.atan2(v1[1], v1[0])
        angle2 = math.atan2(v2[1], v2[0])
        
        # 각도 차이 (-180~180도)
        angle_diff = angle2 - angle1
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        angles.append(math.degrees(abs(angle_diff)))
    
    angles.append(None)  # 마지막 점은 각도 없음
    return angles


def compute_line_deviation(points, start_idx, end_idx):
    """
    구간의 직선과 실제 경로 간 거리 편차 계산
    
    Args:
        points: [[x, y], ...] 전체 좌표 리스트
        start_idx: 구간 시작 인덱스
        end_idx: 구간 끝 인덱스 (포함)
    
    Returns:
        max_deviation: 최대 편차 (m)
        avg_deviation: 평균 편차 (m)
    """
    if end_idx <= start_idx or start_idx < 0 or end_idx >= len(points):
        return 0.0, 0.0
    
    p_start = points[start_idx]
    p_end = points[end_idx]
    
    # 직선 방정식: ax + by + c = 0
    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]
    length = math.sqrt(dx**2 + dy**2)
    
    if length < 1e-6:
        return 0.0, 0.0
    
    # 직선의 법선 벡터 (단위 벡터)
    nx = -dy / length
    ny = dx / length
    
    # 각 점에서 직선까지의 거리 계산
    deviations = []
    for i in range(start_idx + 1, end_idx):
        p = points[i]
        # 점에서 시작점으로의 벡터
        vx = p[0] - p_start[0]
        vy = p[1] - p_start[1]
        # 직선까지의 거리 (절대값)
        dist = abs(vx * nx + vy * ny)
        deviations.append(dist)
    
    if not deviations:
        return 0.0, 0.0
    
    max_deviation = max(deviations)
    avg_deviation = sum(deviations) / len(deviations)
    
    return max_deviation, avg_deviation


def classify_segments(points, angles):
    """
    직선/곡선/급회전 구간 분류
    
    Args:
        points: [[x, y], ...] 좌표 리스트
        angles: [None, angle1, ...] 각도 리스트
    
    Returns:
        segments: [{"type": "straight"/"curve"/"sharp_turn", "start": idx, "end": idx}, ...]
    """
    if len(points) < 2:
        return []
    
    segments = []
    current_start = 0
    current_type = None
    
    ANGLE_STRAIGHT_MAX = 15.0
    ANGLE_SMOOTH_MAX = 75.0
    DEVIATION_STRAIGHT_CM = 2.5
    DEVIATION_SMOOTH_CM = 15.0

    for i in range(1, len(points)):
        angle = angles[i] if i < len(angles) else None
        max_dev_cm = 0.0
        
        # 각도 기반 판단 (A)
        angle_type = None
        if angle is not None:
            if angle < ANGLE_STRAIGHT_MAX:
                angle_type = "straight"
            elif angle < ANGLE_SMOOTH_MAX:
                angle_type = "curve"
            else:
                angle_type = "sharp_turn"
        
        # 거리 편차 기반 판단 (B) - 구간 단위로 계산
        if i > current_start + 1:
            max_dev, avg_dev = compute_line_deviation(points, current_start, i)
            # 미터를 센티미터로 변환
            max_dev_cm = max_dev * 100
            avg_dev_cm = avg_dev * 100
            
            deviation_type = None
            if max_dev_cm < DEVIATION_STRAIGHT_CM:
                deviation_type = "straight"
            elif max_dev_cm < DEVIATION_SMOOTH_CM:
                deviation_type = "curve"
            else:
                deviation_type = "sharp_turn"
            
            # 두 기준 모두 고려하여 최종 결정
            final_type = "straight"
            if angle_type == "sharp_turn" or deviation_type == "sharp_turn":
                # 각도가 그리 크지 않고 거리 편차가 작으면 부드러운 곡선으로 처리
                if (angle is not None and angle < 100) and max_dev_cm < 40:
                    final_type = "sharp_curve" if angle and angle >= 40 else "smooth_curve"
                else:
                    final_type = "sharp_turn"
            elif angle_type == "curve" or deviation_type == "curve":
                if angle is not None and angle >= 45:
                    final_type = "sharp_curve"
                else:
                    final_type = "smooth_curve"
            else:
                final_type = "straight"
        else:
            # 구간이 너무 짧으면 각도만 사용
            final_type = angle_type if angle_type else "straight"
        
        # 구간 타입이 변경되면 이전 구간 저장
        if current_type is not None and final_type != current_type:
            segments.append({
                "type": current_type,
                "start": current_start,
                "end": i - 1
            })
            current_start = i - 1
            current_type = final_type
        else:
            if current_type is None:
                current_type = final_type
    
    # 마지막 구간 추가
    if current_type is not None:
        segments.append({
            "type": current_type,
            "start": current_start,
            "end": len(points) - 1
        })
    
    return segments


def find_furthest_point_from_line(points, start_idx, end_idx):
    """
    두 포인트 사이 직선으로부터 가장 멀리 떨어진 스케일 좌표를 찾는다.
    
    Args:
        points: [[x, y], ...] 스케일된 좌표 리스트
        start_idx: 구간 시작 인덱스
        end_idx: 구간 끝 인덱스 (포함)
    
    Returns:
        (furthest_point, furthest_idx, max_distance)
    """
    if (
        end_idx <= start_idx
        or start_idx < 0
        or end_idx >= len(points)
        or len(points) < 2
    ):
        return None, None, 0.0
    
    p_start = points[start_idx]
    p_end = points[end_idx]
    dx = p_end[0] - p_start[0]
    dy = p_end[1] - p_start[1]
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return None, None, 0.0
    
    # 단위 법선 벡터
    nx = -dy / length
    ny = dx / length
    
    max_dist = 0.0
    furthest_point = None
    furthest_idx = None
    
    for idx in range(start_idx + 1, end_idx):
        px, py = points[idx]
        vx = px - p_start[0]
        vy = py - p_start[1]
        dist = abs(vx * nx + vy * ny)
        if dist > max_dist:
            max_dist = dist
            furthest_point = [px, py]
            furthest_idx = idx
    
    return furthest_point, furthest_idx, max_dist


def simplify_curve_path(
    scaled_path,
    original_path=None,
    segments=None,
    max_points=20,
    min_distance_m=0.1,
    curve_deviation_threshold=0.03,
):
    """
    곡선 경로를 단순화하여 최대 20개 포인트로 제한하고, 각 포인트 간 최소 거리 보장
    
    Args:
        scaled_path: [[x, y], ...] 스케일된 좌표 리스트
        original_path: [[lon, lat], ...] 원본 경로 좌표
        segments: [{"type": str, "start": idx, "end": idx}, ...] 구간 정보
        max_points: 최대 포인트 수 (기본값: 20)
        min_distance_m: 최소 포인트 간 거리 (m, 기본값: 0.1m = 10cm)
    
    Returns:
        simplified_points: [[x, y], ...] 단순화된 포인트 리스트
        simplified_segments: [{"type": str, "start": idx, "end": idx}, ...] 단순화된 구간 정보
    """
    if segments is None:
        segments = []
    
    if len(scaled_path) <= max_points:
        return scaled_path, segments if segments else classify_segments(scaled_path, compute_angles(scaled_path))
    
    simplified = []
    simplified_segments = []
    
    # 직선 포인트는 모두 유지
    straight_points = []
    for seg in segments:
        if seg["type"] == "straight":
            for i in range(seg["start"], seg["end"] + 1):
                if i < len(scaled_path):
                    straight_points.append((i, scaled_path[i]))
    
    # 곡선 구간에서 최외곽 점 1개씩 추가
    curve_points = []
    for seg in segments:
        if seg["type"] in ["curve", "smooth_curve", "sharp_curve"]:
            furthest, furthest_idx, max_dev = find_furthest_point_from_line(
                scaled_path, seg["start"], seg["end"]
            )
            if (
                furthest
                and furthest_idx is not None
                and max_dev >= curve_deviation_threshold
            ):
                curve_points.append((furthest_idx, furthest))
    
    # 모든 포인트를 인덱스 순으로 정렬
    all_points = sorted(straight_points + curve_points, key=lambda x: x[0])
    
    # 중복 제거 및 최소 거리 보장
    simplified = [all_points[0][1]]  # 첫 번째 점
    for i in range(1, len(all_points)):
        if all_points[i][0] != all_points[i-1][0]:
            curr_pt = all_points[i][1]
            # 이전 점과의 거리 계산
            prev_pt = simplified[-1]
            dx = curr_pt[0] - prev_pt[0]
            dy = curr_pt[1] - prev_pt[1]
            dist = math.sqrt(dx**2 + dy**2)
            
            # 최소 거리 이상이면 추가
            if dist >= min_distance_m:
                simplified.append(curr_pt)
    
    # 마지막 점 추가 (거리 확인)
    if scaled_path and simplified[-1] != scaled_path[-1]:
        last_pt = scaled_path[-1]
        prev_pt = simplified[-1]
        dx = last_pt[0] - prev_pt[0]
        dy = last_pt[1] - prev_pt[1]
        dist = math.sqrt(dx**2 + dy**2)
        if dist >= min_distance_m or len(simplified) < 2:
            simplified.append(last_pt)
        else:
            # 마지막 점이 너무 가까우면 이전 점을 마지막 점으로 교체
            simplified[-1] = last_pt
    
    # 포인트 수가 여전히 많으면 추가 다운샘플링 (최소 거리 유지)
    if len(simplified) > max_points:
        # 최소 거리를 보장하면서 다운샘플링
        filtered = [simplified[0]]  # 첫 번째 점
        step = len(simplified) / max_points
        
        for i in range(1, max_points - 1):
            idx = int(i * step)
            if idx < len(simplified):
                curr_pt = simplified[idx]
                prev_pt = filtered[-1]
                dx = curr_pt[0] - prev_pt[0]
                dy = curr_pt[1] - prev_pt[1]
                dist = math.sqrt(dx**2 + dy**2)
                
                # 최소 거리 이상이면 추가
                if dist >= min_distance_m:
                    filtered.append(curr_pt)
                else:
                    # 최소 거리 미만이면 다음 포인트 시도
                    for j in range(idx + 1, min(idx + 5, len(simplified))):
                        next_pt = simplified[j]
                        dx = next_pt[0] - prev_pt[0]
                        dy = next_pt[1] - prev_pt[1]
                        dist = math.sqrt(dx**2 + dy**2)
                        if dist >= min_distance_m:
                            filtered.append(next_pt)
                            break
        
        # 마지막 점 보장
        if simplified[-1] != filtered[-1]:
            filtered.append(simplified[-1])
        
        simplified = filtered
    
    # 단순화된 구간 정보 재계산
    angles = compute_angles(simplified)
    simplified_segments = classify_segments(simplified, angles)
    
    print(f"단순화 완료: {len(simplified)}개 포인트 (최소 거리: {min_distance_m}m 보장)")
    
    return simplified, simplified_segments


def generate_spline_for_curves(points, segment):
    """
    곡선 구간에 대해 Spline 보간 수행
    
    Args:
        points: [[x, y], ...] 전체 좌표 리스트
        segment: {"type": "curve"/"smooth_curve", "start": idx, "end": idx}
    
    Returns:
        interpolated_points: [[x, y], ...] 보간된 좌표 리스트 (약 20개)
    """
    if segment["type"] not in ["curve", "smooth_curve"]:
        # 곡선이 아니면 원본 반환
        return points[segment["start"]:segment["end"] + 1]
    
    start_idx = segment["start"]
    end_idx = segment["end"]
    
    if end_idx <= start_idx:
        return [points[start_idx]]
    
    # 구간의 원본 포인트 추출
    segment_points = points[start_idx:end_idx + 1]
    
    if len(segment_points) < 2:
        return segment_points
    
    if not SCIPY_AVAILABLE:
        # SciPy가 없으면 선형 보간으로 최소 5개 포인트 생성
        print("[WARNING] SciPy가 없어 선형 보간을 사용합니다.")
        interpolated = [segment_points[0]]
        target_samples = max(5, len(segment_points))
        for idx in range(1, target_samples - 1):
            t = idx / (target_samples - 1)
            # 보간 위치 계산
            dist = t * (len(segment_points) - 1)
            base = int(math.floor(dist))
            frac = dist - base
            base = min(base, len(segment_points) - 2)
            p1 = segment_points[base]
            p2 = segment_points[base + 1]
            x = p1[0] + (p2[0] - p1[0]) * frac
            y = p1[1] + (p2[1] - p1[1]) * frac
            interpolated.append([x, y])
        interpolated.append(segment_points[-1])
        return interpolated
    
    try:
        # 매개변수 t 계산 (누적 거리 기반)
        t_values = [0.0]
        for i in range(1, len(segment_points)):
            dx = segment_points[i][0] - segment_points[i-1][0]
            dy = segment_points[i][1] - segment_points[i-1][1]
            dist = math.sqrt(dx**2 + dy**2)
            t_values.append(t_values[-1] + dist)
        
        # 정규화 (0~1)
        if t_values[-1] > 0:
            t_values = [t / t_values[-1] for t in t_values]
        
        # 좌표 분리
        x_coords = [p[0] for p in segment_points]
        y_coords = [p[1] for p in segment_points]
        
        # CubicSpline 보간
        cs_x = CubicSpline(t_values, x_coords)
        cs_y = CubicSpline(t_values, y_coords)
        
        # 약 20개 포인트로 샘플링
        num_samples = min(20, max(5, len(segment_points) * 2))
        if NUMPY_AVAILABLE:
            t_new = np.linspace(0, 1, num_samples)
        else:
            # NumPy 없으면 수동으로 생성
            t_new = [i / (num_samples - 1) for i in range(num_samples)]
        
        x_new = cs_x(t_new)
        y_new = cs_y(t_new)
        
        interpolated = [[float(x_new[i]), float(y_new[i])] for i in range(len(t_new))]
        
        return interpolated
    except Exception as e:
        print(f"[WARNING] Spline 보간 실패: {e}, 원본 포인트 사용")
        return segment_points


def generate_cmd_vel_sequence(final_points, segments_info, linear_speed=0.1, original_path=None, scaled_path=None, total_original_length=0.0):
    """
    최종 경로 포인트로부터 Turtlebot 명령 리스트 생성
    
    Args:
        final_points: 최종 스케일된 경로 포인트
        segments_info: 구간 정보
        linear_speed: 선속도
        original_path: 원본 경로 좌표 (위경도, optional)
        scaled_path: 스케일된 원본 경로 (optional)
        total_original_length: 전체 원본 경로 길이 (m)
    """
    commands: List[dict[str, Any]] = []
    segment_idx = 0
    prev_heading: float | None = None
    
    def heading_between(p_a, p_b) -> float:
        """두 점 사이의 방향각 (도) - y축 반전 적용 (직선 로직과 동일)"""
        dx = p_b[0] - p_a[0]
        dy = -(p_b[1] - p_a[1])  # y축 반전 적용 (직선 로직과 동일)
        heading_rad = math.atan2(dy, dx)
        heading = math.degrees(heading_rad)
        # 0~360 정규화
        heading = heading % 360
        if heading < 0:
            heading += 360
        return heading
    
    def normalize_angle(angle: float) -> float:
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def find_closest_scaled_index(point):
        """final_points의 한 점에 대해 가장 가까운 scaled_path 인덱스 찾기"""
        if not scaled_path:
            return None
        min_dist = float('inf')
        closest_idx = 0
        for idx, sp in enumerate(scaled_path):
            dist = math.dist(point, sp)
            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
        return closest_idx
    
    def get_original_distance(start_pt, end_pt) -> float:
        """직선 로직처럼 haversine으로 원본 거리 계산"""
        if original_path and scaled_path:
            start_idx = find_closest_scaled_index(start_pt)
            end_idx = find_closest_scaled_index(end_pt)
            if start_idx is not None and end_idx is not None:
                if 0 <= start_idx < len(original_path) and 0 <= end_idx < len(original_path):
                    return haversine_distance(original_path[start_idx], original_path[end_idx])
        # fallback: 비율 적용
        scaled_dist = math.dist(start_pt, end_pt)
        total_scaled_length = sum(math.dist(final_points[i], final_points[i + 1]) for i in range(len(final_points) - 1))
        distance_ratio = total_original_length / total_scaled_length if total_scaled_length > 1e-6 else 1.0
        return scaled_dist * distance_ratio
    
    for seg_info in segments_info:
        start_idx = seg_info["start"]
        end_idx = seg_info["end"]
        seg_type = seg_info["type"]
        
        if end_idx >= len(final_points):
            end_idx = len(final_points) - 1
        if start_idx >= end_idx:
            continue
        
        if seg_type == "straight":
            segment_points = final_points[start_idx : end_idx + 1]
            if len(segment_points) < 2:
                continue
            # 직선 로직처럼 각 포인트 쌍별로 개별 명령 생성
            for i in range(len(segment_points) - 1):
                curr_pt = segment_points[i]
                next_pt = segment_points[i + 1]
                distance = math.dist(curr_pt, next_pt)
                if distance < 1e-4:
                    continue
                heading = heading_between(curr_pt, next_pt)
                turn_angle = 0.0 if prev_heading is None else normalize_angle(heading - prev_heading)
                prev_heading = heading
                duration = distance / linear_speed if linear_speed > 0 else 0.0
                actual_start_idx = start_idx + i
                actual_end_idx = start_idx + i + 1
                orig_dist = get_original_distance(curr_pt, next_pt)
                commands.append({
                    "segment": segment_idx,
                    "type": "straight",
                    "linear": linear_speed,
                    "angular": 0.0,
                    "distance": distance,
                    "duration": duration,
                    "turn_angle": turn_angle,
                    "start_idx": actual_start_idx,
                    "end_idx": actual_end_idx,
                    "original_distance": orig_dist,
                    "instruction": f"직진 {distance:.3f}m (속도: {linear_speed}m/s, 시간: {duration:.1f}초)",
                })
                segment_idx += 1
        
        elif seg_type in ["curve", "smooth_curve", "sharp_curve"]:
            segment_points = final_points[start_idx : end_idx + 1]
            if len(segment_points) < 3:
                continue
            segment_length = sum(
                math.dist(segment_points[i], segment_points[i + 1])
                for i in range(len(segment_points) - 1)
            )
            if segment_length < 1e-3:
                continue
            start_heading = heading_between(segment_points[0], segment_points[1])
            end_heading = heading_between(segment_points[-2], segment_points[-1])
            turn_angle = 0.0 if prev_heading is None else normalize_angle(end_heading - prev_heading)
            prev_heading = end_heading
            duration = segment_length / linear_speed if linear_speed > 0 else 0.0
            angular_vel = 0.0
            if duration > 0 and abs(turn_angle) > 1e-2:
                angular_vel = math.radians(turn_angle) / duration
            curve_type = "smooth_curve" if seg_type != "sharp_curve" else "sharp_curve"
            orig_dist = get_original_distance(segment_points[0], segment_points[-1])
            
            # 회전 방향 판단
            turn_info = ""
            if abs(turn_angle) >= 1.0:
                if turn_angle > 0:
                    turn_info = f", 좌회전 {abs(turn_angle):.1f}°"
                else:
                    turn_info = f", 우회전 {abs(turn_angle):.1f}°"
            
            commands.append({
                "segment": segment_idx,
                "type": curve_type,
                "linear": linear_speed,
                "angular": angular_vel,
                "distance": segment_length,
                "duration": duration,
                "turn_angle": turn_angle,
                "start_idx": start_idx,
                "end_idx": end_idx,
                "original_distance": orig_dist,
                "instruction": f"곡선 {segment_length:.3f}m{turn_info} (속도: {linear_speed}m/s, 각속도: {math.degrees(angular_vel):.1f}°/s, 시간: {duration:.1f}초)",
            })
            segment_idx += 1
        
        elif seg_type == "sharp_turn":
            # 급회전 구간: 정지 → 회전 → 직진
            commands.append({
                "segment": segment_idx,
                "type": "stop",
                "linear": 0.0,
                "angular": 0.0,
                "distance": 0.0,
                "duration": 0.5,
                "turn_angle": 0.0,
                "start_idx": start_idx,
                "end_idx": start_idx,
                "instruction": "정지",
            })
            segment_idx += 1
            p1 = final_points[start_idx]
            p2 = final_points[min(start_idx + 1, len(final_points) - 1)]
            heading = heading_between(p1, p2)
            turn_angle = 0.0 if prev_heading is None else normalize_angle(heading - prev_heading)
            prev_heading = heading
            angular_speed = 0.5 if turn_angle >= 0 else -0.5
            rotation_duration = abs(math.radians(turn_angle)) / abs(angular_speed) if abs(angular_speed) > 1e-6 else 0.0
            
            # 회전 방향 판단
            turn_direction = "좌회전" if turn_angle > 0 else "우회전"
            
            commands.append({
                "segment": segment_idx,
                "type": "rotate",
                "linear": 0.0,
                "angular": angular_speed,
                "distance": 0.0,
                "duration": rotation_duration,
                "turn_angle": turn_angle,
                "start_idx": start_idx,
                "end_idx": start_idx,
                "instruction": f"{turn_direction} {abs(turn_angle):.1f}° (시간: {rotation_duration:.1f}초)",
            })
            segment_idx += 1
            segment_points = final_points[start_idx + 1 : end_idx + 1]
            if len(segment_points) >= 2:
                distance = sum(
                    math.dist(segment_points[i], segment_points[i + 1])
                    for i in range(len(segment_points) - 1)
                )
                if distance >= 1e-4:
                    heading = heading_between(segment_points[0], segment_points[-1])
                    turn_angle = normalize_angle(heading - prev_heading) if prev_heading is not None else 0.0
                    prev_heading = heading
                    duration = distance / linear_speed if linear_speed > 0 else 0.0
                    orig_dist = get_original_distance(segment_points[0], segment_points[-1])
                    
                    # 회전 방향 판단
                    turn_info = ""
                    if abs(turn_angle) >= 1.0:
                        if turn_angle > 0:
                            turn_info = f", 좌회전 {abs(turn_angle):.1f}°"
                        else:
                            turn_info = f", 우회전 {abs(turn_angle):.1f}°"
                    
                    commands.append({
                        "segment": segment_idx,
                        "type": "straight",
                        "linear": linear_speed,
                        "angular": 0.0,
                        "distance": distance,
                        "duration": duration,
                        "turn_angle": turn_angle,
                        "start_idx": start_idx + 1,
                        "end_idx": end_idx,
                        "original_distance": orig_dist,
                        "instruction": f"직진 {distance:.3f}m{turn_info} (속도: {linear_speed}m/s, 시간: {duration:.1f}초)",
                    })
                    segment_idx += 1
    
    commands.append({
        "segment": segment_idx,
        "type": "arrival",
        "linear": 0.0,
        "angular": 0.0,
        "distance": 0.0,
        "duration": 0.0,
        "turn_angle": 0.0,
        "start_idx": len(final_points) - 1,
        "end_idx": len(final_points) - 1,
        "instruction": "도착",
    })
    
    return commands


# -----------------------------
# 4-1) 기존 경로 코너 추출 함수 (호환성 유지)
# -----------------------------
def extract_keypoints(path, angle_threshold=20, return_indices=False):
    """
    path: [[lon, lat], ...]
    angle_threshold: deg 이상 회전하면 코너로 간주
    return_indices: True면 (keypoints, indices) 튜플 반환
    """
    if len(path) < 3:
        if return_indices:
            return path, list(range(len(path)))
        return path

    keypoints = [path[0]]  # 시작점 항상 포함
    keypoint_indices = [0]  # 시작점 인덱스
    corner_count = 0  # 각도 변화가 있는 코너 개수

    prev_angle = calculate_bearing(path[0], path[1])

    for i in range(1, len(path)-1):
        curr_angle = calculate_bearing(path[i], path[i+1])
        # 각도 차이 계산 (0~360도 범위 고려)
        angle_diff = abs(curr_angle - prev_angle)
        # 180도를 넘어가면 반대 방향으로 계산
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        if angle_diff >= angle_threshold:
            keypoints.append(path[i])
            keypoint_indices.append(i)
            corner_count += 1
            prev_angle = curr_angle

    keypoints.append(path[-1])  # 종료점 항상 포함
    keypoint_indices.append(len(path) - 1)  # 종료점 인덱스
    
    print(f"\n=== 코너 추출 결과 ===")
    print(f"전체 경로 포인트 수: {len(path)}")
    print(f"각도 변화 임계값: {angle_threshold}°")
    print(f"추출된 코너 개수: {corner_count}개")
    print(f"최종 keypoint 개수: {len(keypoints)}개 (시작점 + 코너 {corner_count}개 + 도착점)")
    
    if return_indices:
        return keypoints, keypoint_indices
    return keypoints


def remove_start_end_points(points):
    """
    시작점과 도착점을 제거하고, 두 번째 점을 시작점으로, 마지막 두 번째 점을 도착점으로 사용
    """
    if len(points) <= 2:
        return points
    
    if len(points) == 3:
        # 점이 3개면 중간 점만 반환
        return [points[1]]
    
    # 두 번째 점부터 마지막 두 번째 점까지 반환
    result = points[1:-1]
    
    print(f"\n=== 시작/끝 점 제거 ===")
    print(f"원본 keypoint 개수: {len(points)}개")
    print(f"시작점과 도착점 제거 후: {len(result)}개")
    
    return result

# -----------------------------
# 5) 좌표 스케일 변환 함수 (테스트 공간 크기에 맞게)
# -----------------------------
def fit_path_to_area(points, target_width_m, target_height_m):
    """
    points: [[lon, lat], ...]
    target_width_m / target_height_m: 테스트 공간 가용 크기 (m)
    returns: (scaled_points, scale_factor, bbox_size)
    - scaled_points: [[x, y], ...] 형태 (0,0) 기준, 가로/세로 범위 내 배치
    - scale_factor: 지도 좌표 -> 테스트 공간 변환에 사용된 배율
    - bbox_size: {"width": w, "height": h} (지도 좌표 기준)
    """
    if not points:
        return [], 1.0, {"width": 0.0, "height": 0.0}

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    bbox_width = max_x - min_x
    bbox_height = max_y - min_y

    # bbox가 한 축으로만 존재하는 경우 대비
    if bbox_width == 0 and bbox_height == 0:
        return [[0.0, 0.0] for _ in points], 1.0, {"width": 0.0, "height": 0.0}

    # 목표 영역에 맞추기 위해 축소 배율 계산
    scale_x = target_width_m / bbox_width if bbox_width != 0 else float("inf")
    scale_y = target_height_m / bbox_height if bbox_height != 0 else float("inf")
    scale_factor = min(scale_x, scale_y)

    scaled = []
    for lon, lat in points:
        x = (lon - min_x) * scale_factor
        y = (lat - min_y) * scale_factor
        scaled.append([x, y])

    return scaled, scale_factor, {"width": bbox_width, "height": bbox_height}

# -----------------------------
# 6) 각 세그먼트별 거리와 회전 각도 계산 함수
# -----------------------------
def haversine_distance(point_a, point_b):
    """두 위경도 좌표 간의 거리(m)를 반환"""
    R = 6371000  # 지구 반지름 (m)
    lon1, lat1 = map(math.radians, point_a)
    lon2, lat2 = map(math.radians, point_b)
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def calculate_bearing(point_a, point_b):
    """두 위경도 포인트 간 방위각(0~360도)을 반환"""
    lat1 = math.radians(point_a[1])
    lat2 = math.radians(point_b[1])
    dlon = math.radians(point_b[0] - point_a[0])
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    angle = math.degrees(math.atan2(x, y))
    angle = (angle + 360) % 360
    return angle


def calculate_robot_commands(points, scale_factor=1.0, original_points=None):
    """
    points: [[x, y], ...] - 축소된 좌표 리스트
    returns: [{"segment": int, "start_point": [x, y], "end_point": [x, y], 
               "distance": m, "original_distance": m, "turn_angle": deg, "instruction": str}, ...]
    - y축 반전 고려 (지도 좌표계 → 로봇 좌표계)
    - 좌회전: 양수, 우회전: 음수
    """
    if len(points) < 2:
        return [{
            "segment": 0,
            "start_point": points[0],
            "end_point": points[0],
            "distance": 0,
            "turn_angle": 0,
            "original_distance": 0,
            "instruction": "출발"
        }]
    
    segments = []
    prev_heading = None
    
    for i in range(len(points) - 1):
        curr = points[i]
        next_p = points[i + 1]
        
        # 거리 계산 (유클리드 거리, m) - y축 반전 고려
        dx = next_p[0] - curr[0]
        dy = -(next_p[1] - curr[1])  # y축 반전
        distance = math.sqrt(dx**2 + dy**2)

        if original_points and len(original_points) == len(points):
            original_distance = haversine_distance(original_points[i], original_points[i + 1])
        else:
            original_distance = distance / scale_factor if scale_factor > 0 else distance
        
        # heading 계산 (0~360도) - y축 반전 고려
        heading_rad = math.atan2(dy, dx)
        heading = math.degrees(heading_rad)
        # 0~360 범위로 정규화
        heading = heading % 360
        if heading < 0:
            heading += 360
        
        # turn_angle 계산 (좌회전: 양수, 우회전: 음수, -180~180도)
        if prev_heading is None:
            turn_angle = 0  # 첫 번째 세그먼트는 회전 없음
            instruction = "출발"
        else:
            angle_diff = heading - prev_heading
            # -180~180 범위로 정규화
            while angle_diff > 180:
                angle_diff -= 360
            while angle_diff < -180:
                angle_diff += 360
            # 좌회전: 양수, 우회전: 음수 (부호 유지)
            turn_angle = angle_diff
            
            # 디버깅: 각도 계산 상세 정보
            print(f"  세그먼트 {i}: prev_heading={prev_heading:.1f}° → heading={heading:.1f}°, angle_diff={angle_diff:.1f}°, turn_angle={turn_angle:.1f}°")
            
            if abs(turn_angle) < 1.0:  # 거의 직진
                instruction = "직진"
            elif turn_angle > 0:
                instruction = f"좌회전 {abs(turn_angle):.1f}°"
            else:
                instruction = f"우회전 {abs(turn_angle):.1f}°"
        
        segments.append({
            "segment": i,
            "start_point": curr,
            "end_point": next_p,
            "distance": distance,  # 스케일된 이동 거리 (실제 로봇 명령용)
            "original_distance": original_distance,
            "turn_angle": turn_angle,  # 그 다음 회전
            "instruction": instruction
        })
        
        prev_heading = heading
    
    # 마지막 세그먼트 추가 (도착)
    segments.append({
        "segment": len(segments),
        "start_point": points[-1],
        "end_point": points[-1],
        "distance": 0,
        "original_distance": 0,
        "turn_angle": 0,
        "instruction": "도착"
    })
    
    return segments

# -----------------------------
# 6-1) 경로 시각화 이미지 생성 함수
# -----------------------------
# 6) URL에서 좌표 추출
# -----------------------------
# -----------------------------
# 7) 경로 시각화 이미지 생성
# -----------------------------
def visualize_path_image(keypoints, path_coordinates, output_dir=None):
    """
    경로를 시각화하여 이미지 파일로 저장
    - 100m당 100px 변환
    - keypoints와 전체 경로 표시
    
    Args:
        keypoints: [[lon, lat], ...] - 추출된 keypoint 좌표
        path_coordinates: [[lon, lat], ...] - 전체 경로 좌표
        output_dir: 저장할 디렉토리 (기본값: 현재 파일의 src 디렉토리)
    
    Returns:
        저장된 이미지 파일 경로 (str) 또는 None (PIL 없을 경우)
    """
    if not PIL_AVAILABLE:
        print("[WARNING] PIL/Pillow가 없어 이미지를 생성할 수 없습니다.")
        return None
    
    if not keypoints or len(keypoints) < 2:
        print("[WARNING] keypoint가 부족하여 이미지를 생성할 수 없습니다.")
        return None
    
    try:
        # 좌표 범위 계산
        all_points = path_coordinates if path_coordinates else keypoints
        lons = [p[0] for p in all_points]
        lats = [p[1] for p in all_points]
        min_lon, max_lon = min(lons), max(lons)
        min_lat, max_lat = min(lats), max(lats)
        
        # Haversine 거리로 실제 거리 계산 (m)
        center_lat = (min_lat + max_lat) / 2
        lat_to_m = 111320  # 위도 1도 ≈ 111320m
        lon_to_m = 111320 * math.cos(math.radians(center_lat))  # 경도 1도 ≈ 111320 * cos(위도) m
        
        width_m = (max_lon - min_lon) * lon_to_m
        height_m = (max_lat - min_lat) * lat_to_m
        
        # 이미지 크기 제한 (너무 큰 이미지 방지)
        # 100m당 100px 변환 (1m = 1px)
        scale_px_per_m = 100.0 / 100.0  # 1m = 1px
        padding = 100  # 여백 100px
        
        img_width = int(width_m * scale_px_per_m) + padding * 2
        img_height = int(height_m * scale_px_per_m) + padding * 2
        
        # 이미지 크기 제한 (최대 10000x10000px)
        MAX_IMAGE_SIZE = 10000
        if img_width > MAX_IMAGE_SIZE or img_height > MAX_IMAGE_SIZE:
            # 스케일 조정
            scale_factor = min(MAX_IMAGE_SIZE / img_width, MAX_IMAGE_SIZE / img_height)
            scale_px_per_m *= scale_factor
            img_width = int(width_m * scale_px_per_m) + padding * 2
            img_height = int(height_m * scale_px_per_m) + padding * 2
            print(f"[INFO] 이미지 크기 제한 적용: 스케일 조정 {scale_factor:.2f}배")
        
        # 이미지 생성
        img = Image.new('RGB', (img_width, img_height), color='white')
        draw = ImageDraw.Draw(img)
        
        # 좌표 변환 함수 (위경도 -> 픽셀 좌표)
        def to_pixel(lon, lat):
            x = int((lon - min_lon) * lon_to_m * scale_px_per_m) + padding
            y = int((max_lat - lat) * lat_to_m * scale_px_per_m) + padding  # y축 반전 (위도 증가 = 위로)
            return x, y
        
        # 전체 경로 그리기 (회색 선)
        if path_coordinates and len(path_coordinates) > 1:
            path_pixels = [to_pixel(p[0], p[1]) for p in path_coordinates]
            for i in range(len(path_pixels) - 1):
                draw.line([path_pixels[i], path_pixels[i+1]], fill='gray', width=2)
        
        # Keypoint 선 그리기 (파란색 선)
        keypoint_pixels = [to_pixel(p[0], p[1]) for p in keypoints]
        for i in range(len(keypoint_pixels) - 1):
            draw.line([keypoint_pixels[i], keypoint_pixels[i+1]], fill='blue', width=3)
        
        # Keypoint 점 그리기
        for i, (x, y) in enumerate(keypoint_pixels):
            # 시작점 (두 번째 점)
            if i == 0:
                color = 'green'
                radius = 8
            # 도착점 (마지막 두 번째 점)
            elif i == len(keypoint_pixels) - 1:
                color = 'red'
                radius = 8
            # 중간 점
            else:
                color = 'blue'
                radius = 6
            
            draw.ellipse([x - radius, y - radius, x + radius, y + radius], 
                        fill=color, outline='black', width=1)
            
            # 번호 표시
            draw.text((x + radius + 2, y - 8), str(i + 1), fill='black')
        
        # 저장 경로 설정
        if output_dir is None:
            output_dir = ROUTE_IMAGE_DIR
        else:
            output_dir = Path(output_dir)
        
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # 기존 이미지 파일 삭제 (한 장만 유지)
        if output_dir.exists():
            # route_visualization_*.png 패턴 파일 삭제
            for old_file in output_dir.glob("route_visualization_*.png"):
                try:
                    old_file.unlink()
                    print(f"[INFO] 기존 이미지 삭제: {old_file.name}")
                except Exception as e:
                    print(f"[WARNING] 기존 이미지 삭제 실패: {old_file.name}, {e}")
            # current_route_visualization.png도 삭제 (새 경로 생성 전)
            current_file = output_dir / "current_route_visualization.png"
            if current_file.exists():
                try:
                    current_file.unlink()
                    print(f"[INFO] 기존 이미지 삭제: {current_file.name}")
                except Exception as e:
                    print(f"[WARNING] 기존 이미지 삭제 실패: {current_file.name}, {e}")
        
        # 고정된 파일명 사용 (한 장만 유지)
        output_path = output_dir / "current_route_visualization.png"
        
        img.save(output_path)
        print(f"[INFO] 경로 시각화 이미지 저장: {output_path}")
        
        return str(output_path)
        
    except Exception as e:
        print(f"[ERROR] 이미지 생성 실패: {type(e).__name__}: {str(e)}")
        return None

# -----------------------------
# 7) 전체 경로 계산 함수 (외부에서 사용)
# -----------------------------
def get_robot_path_from_coordinates(
    path,
    target_width_m=2.0,
    target_height_m=3.0,
    angle_threshold=20,
    linear_speed=0.1,
    start_coords=None,
    goal_coords=None,
):
    """
    곡선 최적화 경로 생성 (CURVE_PATH_LOGIC.md 기준)
    
    1. Keypoint 추출 (직선 로직과 동일)
    2. Keypoint 간 직선/곡선 판단
    3. 곡선 구간의 최외곽 점 찾기 및 3점 곡선 생성
    4. 로봇 명령 생성 (직선/곡선 구분)
    """
    # api_to_position_curve.py에서 자체 정의한 함수들을 사용
    # extract_keypoints, calculate_bearing, fit_path_to_area, calculate_robot_commands 모두 로컬에 정의됨
    
    if not path or len(path) < 2:
        raise ValueError("경로 좌표가 부족합니다.")
    
    # 출발지/도착지 좌표 설정
    if start_coords is None:
        start_lon, start_lat = path[0][0], path[0][1]
    else:
        start_lon, start_lat = start_coords[0], start_coords[1]
    
    if goal_coords is None:
        goal_lon, goal_lat = path[-1][0], path[-1][1]
    else:
        goal_lon, goal_lat = goal_coords[0], goal_coords[1]
    
    print(f"\n=== 곡선 최적화 경로 분석 시작 ===")
    print(f"전체 경로 포인트 수: {len(path)}")
    
    # 1단계: Keypoint 추출 (직선 로직과 동일, 인덱스도 함께 반환)
    keypoints, keypoint_indices = extract_keypoints(path, angle_threshold=angle_threshold, return_indices=True)
    print(f"Keypoint 추출 완료: {len(keypoints)}개")
    
    # 2단계: 스케일링 (직선 로직과 동일)
    scaled_keypoints, scale_factor, bbox_size = fit_path_to_area(
        keypoints,
        target_width_m=target_width_m,
        target_height_m=target_height_m,
    )
    print(f"스케일링 완료: 스케일 팩터 = {scale_factor:.6f}")
    
    # 전체 원본 경로 길이 계산
    total_original_length = 0.0
    if len(path) >= 2:
        for idx in range(len(path) - 1):
            total_original_length += haversine_distance(path[idx], path[idx + 1])
    print(f"전체 원본 거리: {total_original_length:.2f}m")
    
    # 3단계: Keypoint 간 직선/곡선 판단 및 명령 생성
    robot_commands = generate_curve_commands(
        keypoints,
        scaled_keypoints,
        path,
        scale_factor,
        linear_speed,
        angle_threshold,
        keypoint_indices=keypoint_indices
    )
    
    # 지도 표시용 정보
    path_coordinates = [[p[0], p[1]] for p in path]
    start_coords_result = [start_lon, start_lat]
    goal_coords_result = [goal_lon, goal_lat]
    
    # BBox 정보
    min_lon = min(kp[0] for kp in keypoints)
    max_lon = max(kp[0] for kp in keypoints)
    min_lat = min(kp[1] for kp in keypoints)
    max_lat = max(kp[1] for kp in keypoints)
    
    # 경로 시각화 이미지 생성
    route_image_url = None
    try:
        image_path = visualize_path_image(
            keypoints,
            path_coordinates,
            output_dir=ROUTE_IMAGE_DIR,
        )
        if image_path:
            print(f"[INFO] 곡선 경로 시각화 이미지 저장 완료: {image_path}")
            route_image_url = "/static/routes/current_route_visualization.png"
    except Exception as e:
        print(f"[WARNING] 경로 시각화 이미지 생성 실패: {type(e).__name__}: {str(e)}")
    
    return {
        "commands": robot_commands,
        "scale_factor": scale_factor,
        "bbox": bbox_size,
        "target_area": {"width": target_width_m, "height": target_height_m},
        "path_coordinates": path_coordinates,
        "start_coords": start_coords_result,
        "goal_coords": goal_coords_result,
        "map_bbox": {
            "min_lon": min_lon,
            "max_lon": max_lon,
            "min_lat": min_lat,
            "max_lat": max_lat,
        },
        "route_image": route_image_url,
    }


def generate_curve_commands(keypoints, scaled_keypoints, original_path, scale_factor, linear_speed, angle_threshold, keypoint_indices=None):
    """
    Keypoint 간 직선/곡선을 판단하고 로봇 명령 생성
    
    MD 파일 로직:
    - 직선: keypoint A → B 단순 직진
    - 곡선: keypoint A → B 사이 최외곽 점 C 찾아서 A, C, B 3점 곡선
    
    Args:
        keypoint_indices: keypoint의 원본 경로 인덱스 리스트 (None이면 자동 계산)
    """
    commands = []
    segment_idx = 0
    prev_heading = None
    prev_segment_type = None  # 이전 세그먼트 타입 추적 (곡선→곡선 판단용)
    
    # 첫 명령으로 "출발" 더미 추가 (토픽 누락 방지)
    if len(scaled_keypoints) > 0:
        commands.append({
            "segment": segment_idx,
            "start_point": scaled_keypoints[0],
            "end_point": scaled_keypoints[0],
            "distance": 0.0,
            "original_distance": 0.0,
            "turn_angle": 0.0,
            "instruction": "출발",
            "linear": linear_speed,
            "angular": 0.0,
            "duration": 0.0,
            "type": "departure"
        })
        segment_idx += 1
    
    # 헬퍼 함수
    def normalize_angle(angle):
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def heading_between(p1, p2):
        """두 점 사이의 방향각 (도) - y축 반전 적용 (직선 로직과 동일)"""
        dx = p2[0] - p1[0]
        dy = -(p2[1] - p1[1])  # y축 반전 적용 (직선 로직과 동일)
        heading_rad = math.atan2(dy, dx)
        heading = math.degrees(heading_rad)
        # 0~360 범위로 정규화
        heading = heading % 360
        if heading < 0:
            heading += 360
        return heading
    
    def is_straight_segment(kp_a_idx, kp_b_idx):
        """두 keypoint 사이가 직선인지 판단"""
        # keypoint_indices가 있으면 직접 사용, 없으면 거리 기반으로 찾기
        if keypoint_indices and len(keypoint_indices) > max(kp_a_idx, kp_b_idx):
            a_idx = keypoint_indices[kp_a_idx]
            b_idx = keypoint_indices[kp_b_idx]
        else:
            # fallback: 거리 기반으로 찾기
            kp_a = keypoints[kp_a_idx]
            kp_b = keypoints[kp_b_idx]
            
            def find_closest_index(target_point):
                min_dist = float('inf')
                closest_idx = None
                for i, p in enumerate(original_path):
                    dist = haversine_distance(target_point, p)
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = i
                # 1m 이내에 있으면 같은 점으로 간주
                return closest_idx if min_dist < 1.0 else None
            
            a_idx = find_closest_index(kp_a)
            b_idx = find_closest_index(kp_b)
        
        if a_idx is None or b_idx is None or a_idx >= b_idx:
            return True  # 기본값: 직선
        
        # A-B 사이 포인트가 2개 미만이면 직선
        if b_idx - a_idx <= 1:
            return True
        
        # A-B 사이 포인트들의 최대 편차 계산
        # 원본 경로의 좌표를 사용 (keypoint가 아닌)
        line_start = original_path[a_idx]
        line_end = original_path[b_idx]
        max_deviation = 0.0
        for i in range(a_idx + 1, b_idx):
            deviation = point_to_line_distance(original_path[i], line_start, line_end)
            max_deviation = max(max_deviation, deviation)
        
        # 최대 편차가 2.75m 미만이면 직선으로 판단
        # 차로 폭 기준: 이면도로(2.75m), 도시부 도로(3.0~3.5m), 고속도로(3.5~3.6m)
        # 2.75m는 이면도로 차로 폭과 동일하여 완만한 도로를 직선으로 처리
        is_straight = max_deviation < 2.75
        # 디버깅 로그 (안전하게)
        try:
            print(f"  [직선/곡선] kp{kp_a_idx}→{kp_b_idx}: 편차={max_deviation:.2f}m → {'직선' if is_straight else '곡선'}")
        except:
            pass
        return is_straight
    
    def point_to_line_distance(point, line_start, line_end):
        """점에서 직선까지의 최단 거리 (m)"""
        # Haversine 거리 사용
        d_total = haversine_distance(line_start, line_end)
        if d_total < 1e-6:
            return haversine_distance(point, line_start)
        
        d_start = haversine_distance(line_start, point)
        d_end = haversine_distance(point, line_end)
        
        # 헤론 공식으로 삼각형 넓이 구하고 높이 계산
        s = (d_total + d_start + d_end) / 2
        if s <= d_total or s <= d_start or s <= d_end:
            return min(d_start, d_end)
        
        area_sq = s * (s - d_total) * (s - d_start) * (s - d_end)
        if area_sq <= 0:
            return min(d_start, d_end)
        
        area = math.sqrt(area_sq)
        height = 2 * area / d_total
        return height
    
    def find_furthest_point(kp_a_idx, kp_b_idx):
        """두 keypoint 사이의 최외곽 점 찾기"""
        # keypoint_indices가 있으면 직접 사용, 없으면 거리 기반으로 찾기
        if keypoint_indices and len(keypoint_indices) > max(kp_a_idx, kp_b_idx):
            a_idx = keypoint_indices[kp_a_idx]
            b_idx = keypoint_indices[kp_b_idx]
        else:
            # fallback: 거리 기반으로 찾기
            kp_a = keypoints[kp_a_idx]
            kp_b = keypoints[kp_b_idx]
            
            def find_closest_index(target_point):
                min_dist = float('inf')
                closest_idx = None
                for i, p in enumerate(original_path):
                    dist = haversine_distance(target_point, p)
                    if dist < min_dist:
                        min_dist = dist
                        closest_idx = i
                return closest_idx if min_dist < 1.0 else None
            
            a_idx = find_closest_index(kp_a)
            b_idx = find_closest_index(kp_b)
        
        if a_idx is None or b_idx is None or a_idx >= b_idx - 1:
            return None
        
        # 원본 경로의 좌표를 사용 (keypoint가 아닌)
        line_start = original_path[a_idx]
        line_end = original_path[b_idx]
        max_dist = 0.0
        furthest_pt = None
        for i in range(a_idx + 1, b_idx):
            dist = point_to_line_distance(original_path[i], line_start, line_end)
            if dist > max_dist:
                max_dist = dist
                furthest_pt = original_path[i]
        
        if max_dist > 2.75:  # 2.75m 이상이어야 의미있는 곡선 (직선 임계값과 동일)
            try:
                print(f"  [최외곽 점] kp{kp_a_idx}→{kp_b_idx}: 편차={max_dist:.2f}m, 최외곽 점 사용")
            except:
                pass
            return furthest_pt
        return None
    
    # Keypoint 간 명령 생성
    for i in range(len(scaled_keypoints) - 1):
        curr_kp = scaled_keypoints[i]
        next_kp = scaled_keypoints[i + 1]
        
        # 거리 계산 (직선 로직과 동일하게 y축 반전)
        dx = next_kp[0] - curr_kp[0]
        dy = -(next_kp[1] - curr_kp[1])  # y축 반전
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 1e-4:
            continue
        
        # 방향각 및 회전각 계산
        heading = heading_between(curr_kp, next_kp)
        # turn_angle: 양수 = 좌회전, 음수 = 우회전 (api_to_position.py와 동일)
        turn_angle = 0.0 if prev_heading is None else normalize_angle(prev_heading - heading)
        prev_heading = heading
        
        # 원본 거리 계산
        original_distance = haversine_distance(keypoints[i], keypoints[i + 1])
        
        # 회전 방향 판단 (공통)
        # turn_angle: 양수 = 좌회전, 음수 = 우회전 (api_to_position.py와 동일)
        turn_info = ""
        if abs(turn_angle) >= 1.0:
            if turn_angle > 0:
                turn_info = f", 좌회전 {abs(turn_angle):.1f}°"
            else:
                turn_info = f", 우회전 {abs(turn_angle):.1f}°"
        
        # 직선/곡선 판단
        current_segment_type = "straight" if is_straight_segment(i, i + 1) else "curve"
        
        if current_segment_type == "straight":
            # 직선 구간 - 항상 angular = 0
            duration = distance / linear_speed if linear_speed > 0 else 0.0
            
            commands.append({
                "segment": segment_idx,
                "type": "straight",
                "linear": linear_speed,
                "angular": 0.0,  # 직선은 항상 angular = 0
                "distance": distance,
                "duration": duration,
                "turn_angle": turn_angle,
                "start_point": curr_kp,
                "end_point": next_kp,
                "original_distance": original_distance,
                "instruction": f"직진 {distance:.3f}m{turn_info} (속도: {linear_speed}m/s, 시간: {duration:.1f}초)",
            })
            segment_idx += 1
            prev_segment_type = "straight"
        else:
            # 곡선 구간 (경로 형태)
            duration = distance / linear_speed if linear_speed > 0 else 0.0
            
            # 규칙: 곡선→곡선만 smooth_curve (angular_vel 적용), 직선→곡선은 곡선이지만 angular_vel = 0
            if prev_segment_type == "curve":
                # 곡선→곡선: smooth_curve (스무스 회전)
                segment_type = "smooth_curve"
                angular_vel = math.radians(turn_angle) / duration if duration > 0 and abs(turn_angle) > 1e-2 else 0.0
                angular_deg_per_sec = abs(math.degrees(angular_vel))
                instruction_text = f"곡선 {distance:.3f}m{turn_info} (속도: {linear_speed}m/s, 각속도: {angular_deg_per_sec:.1f}°/s, 시간: {duration:.1f}초)"
            else:
                # 직선→곡선 또는 첫 세그먼트: 곡선이지만 angular_vel = 0 (정지→회전→직진)
                segment_type = "curve"  # 곡선으로 표시
                angular_vel = 0.0
                instruction_text = f"곡선 {distance:.3f}m{turn_info} (속도: {linear_speed}m/s, 시간: {duration:.1f}초)"
            
            commands.append({
                "segment": segment_idx,
                "type": segment_type,
                "linear": linear_speed,
                "angular": angular_vel,
                "distance": distance,
                "duration": duration,
                "turn_angle": turn_angle,
                "start_point": curr_kp,
                "end_point": next_kp,
                "original_distance": original_distance,
                "instruction": instruction_text,
            })
            segment_idx += 1
            prev_segment_type = "curve"
    
    # 도착 명령
    commands.append({
        "segment": segment_idx,
        "type": "arrival",
        "linear": 0.0,
        "angular": 0.0,
        "distance": 0.0,
        "duration": 0.0,
        "turn_angle": 0.0,
        "start_point": scaled_keypoints[-1],
        "end_point": scaled_keypoints[-1],
        "original_distance": 0.0,
        "instruction": "도착",
    })
    
    print(f"\n=== 명령 생성 완료 ===")
    print(f"총 {len(commands)}개 명령 (도착 포함)")
    straight_count = sum(1 for cmd in commands if cmd["type"] == "straight")
    curve_count = sum(1 for cmd in commands if cmd["type"] == "smooth_curve")
    print(f"직선: {straight_count}개, 곡선: {curve_count}개")
    
    return commands


def get_robot_path_from_addresses(
    start_address,
    goal_address,
    target_width_m=2.0,
    target_height_m=3.0,
    angle_threshold=20,
    route_option=None,
    linear_speed=0.1,
):
    """
    주소를 입력받아 곡선 최적화 경로를 계산합니다.
    """
    start_lon, start_lat = geocode(start_address)
    goal_lon, goal_lat = geocode(goal_address)
    path = get_directions(start_lon, start_lat, goal_lon, goal_lat, option=route_option)
    return get_robot_path_from_coordinates(
        path=path,
        target_width_m=target_width_m,
        target_height_m=target_height_m,
        angle_threshold=angle_threshold,
        linear_speed=linear_speed,
        start_coords=[start_lon, start_lat],
        goal_coords=[goal_lon, goal_lat],
    )

# -----------------------------
# 8) 실행
# -----------------------------
if __name__ == "__main__":
    # 입력 주소
    start_address = "경기 성남시 수정구 수정로 167"
    goal_address  = "경기 성남시 수정구 신흥동 5455"
    target_width_m = 2.0
    target_height_m = 3.0

    # 테스트용: API 호출 포함 함수 사용
    from api_to_position import get_robot_path_from_addresses as get_robot_path_from_addresses_straight
    result = get_robot_path_from_addresses_straight(
        start_address,
        goal_address,
        target_width_m=target_width_m,
        target_height_m=target_height_m,
        angle_threshold=20,
    )

    robot_commands = result["commands"]

    print("\n=== 지도 경로 정보 ===")
    print(f"지도 좌표 BBox width: {result['bbox']['width']:.6f}, height: {result['bbox']['height']:.6f}")
    print(f"테스트 공간: {result['target_area']['width']}m x {result['target_area']['height']}m")
    print(f"적용된 스케일 배율: {result['scale_factor']:.6f}")

    print("\n=== 로봇 명령 데이터 ===")
    print(f"추출된 keypoint 개수: {len(result.get('path_coordinates', []))}개")
    print(f"생성된 세그먼트 수: {len(robot_commands)}개 (keypoint 개수에 따라 동적으로 결정됨)")
    print("\n각 세그먼트별 상세 정보:")
    for cmd in robot_commands:
        print(f"  [세그먼트 {cmd['segment']}] {cmd['instruction']}")
        print(f"    시작점: [{cmd['start_point'][0]:.4f}, {cmd['start_point'][1]:.4f}]")
        print(f"    종료점: [{cmd['end_point'][0]:.4f}, {cmd['end_point'][1]:.4f}]")
        print(f"    직진: {cmd['original_distance']:.4f}m (scale: {cmd['distance']:.4f}m) → 회전: {cmd['turn_angle']:.2f}°")
    
    print("\n=== ROS2 명령용 데이터 (Python 리스트) ===")
    print(robot_commands)

    
