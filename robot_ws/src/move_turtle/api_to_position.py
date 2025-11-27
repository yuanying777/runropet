import requests
import math
from datetime import datetime
from pathlib import Path

try:
    from PIL import Image, ImageDraw, ImageFont
    PIL_AVAILABLE = True
except ImportError:
    PIL_AVAILABLE = False
    print("[WARNING] PIL/Pillow가 설치되지 않았습니다. 이미지 생성 기능이 비활성화됩니다.")

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
# 4) 경로 코너 추출 함수
# -----------------------------
def extract_keypoints(path, angle_threshold=20):
    """
    path: [[lon, lat], ...]
    angle_threshold: deg 이상 회전하면 코너로 간주
    """
    if len(path) < 3:
        return path

    keypoints = [path[0]]  # 시작점 항상 포함
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
            corner_count += 1
            prev_angle = curr_angle

    keypoints.append(path[-1])  # 종료점 항상 포함
    
    print(f"\n=== 코너 추출 결과 ===")
    print(f"전체 경로 포인트 수: {len(path)}")
    print(f"각도 변화 임계값: {angle_threshold}°")
    print(f"추출된 코너 개수: {corner_count}개")
    print(f"최종 keypoint 개수: {len(keypoints)}개 (시작점 + 코너 {corner_count}개 + 도착점)")
    
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


def calculate_robot_commands(points, scale_factor=1.0, original_points=None, linear_speed=0.1):
    """
    points: [[x, y], ...] - 축소된 좌표 리스트
    linear_speed: 선속도 (m/s), 기본값 0.1
    returns: [{"segment": int, "start_point": [x, y], "end_point": [x, y], 
               "distance": m, "original_distance": m, "turn_angle": deg, "instruction": str,
               "linear": float, "angular": float, "duration": float}, ...]
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
        
        # duration 계산 (초)
        duration = distance / linear_speed if linear_speed > 0 else 0.0
        
        segments.append({
            "segment": i,
            "start_point": curr,
            "end_point": next_p,
            "distance": distance,  # 스케일된 이동 거리 (실제 로봇 명령용)
            "original_distance": original_distance,
            "turn_angle": turn_angle,  # 그 다음 회전
            "instruction": instruction,
            "linear": linear_speed,
            "angular": 0.0,
            "duration": duration,
            "type": "straight" if abs(turn_angle) < 1.0 else "turn"
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
        "instruction": "도착",
        "linear": 0.0,
        "angular": 0.0,
        "duration": 0.0,
        "type": "arrival"
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
    원본 경로 좌표로부터 로봇 명령 경로까지 계산 (API 호출 없음)
    
    Args:
        path: [[lon, lat], ...] 원본 경로 좌표 리스트
        target_width_m: 목표 가로 크기 (m)
        target_height_m: 목표 세로 크기 (m)
        angle_threshold: 각도 임계값 (도)
        linear_speed: 선속도 (m/s)
        start_coords: [lon, lat] 출발지 좌표 (선택적)
        goal_coords: [lon, lat] 도착지 좌표 (선택적)
    
    Returns:
        {
        "commands": [...],
        "scale_factor": float,
        "bbox": {"width": float, "height": float},
        "target_area": {"width": float, "height": float},
        "path_coordinates": [[lon, lat], ...],
        "start_coords": [lon, lat],
        "goal_coords": [lon, lat],
        "map_bbox": {...},
        "route_image": str or None
    }
    """
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
    
    # 코너 기반 포인트 추출
    print(f"\n=== 경로 분석 시작 (좌표 재사용) ===")
    print(f"전체 경로 포인트 수: {len(path)}")
    keypoints = extract_keypoints(path, angle_threshold=angle_threshold)
    
    # 테스트 공간 크기에 맞춰 스케일링
    scaled_keypoints, scale_factor, bbox_size = fit_path_to_area(
        keypoints,
        target_width_m=target_width_m,
        target_height_m=target_height_m,
    )
    
    # 로봇 명령 계산
    robot_commands = calculate_robot_commands(
        scaled_keypoints,
        scale_factor=scale_factor,
        original_points=keypoints,
        linear_speed=linear_speed
    )
    
    # 지도 표시용 정보 추가
    # 경로의 실제 위경도 좌표 (전체 경로 path 사용 - 경로선 표시용)
    path_coordinates = [[p[0], p[1]] for p in path]  # [lon, lat] 형식 - 전체 경로
    
    # 출발지/도착지 좌표
    start_coords_result = [start_lon, start_lat]
    goal_coords_result = [goal_lon, goal_lat]
    
    # BBox 정보 (위경도 좌표계)
    min_lon = min(kp[0] for kp in keypoints)
    max_lon = max(kp[0] for kp in keypoints)
    min_lat = min(kp[1] for kp in keypoints)
    max_lat = max(kp[1] for kp in keypoints)
    
    # 경로 시각화 이미지 생성 (src 폴더에 저장)
    route_image_url = None
    try:
        image_path = visualize_path_image(
            keypoints,
            path_coordinates,
            output_dir=ROUTE_IMAGE_DIR,
        )
        if image_path:
            print(f"[INFO] 경로 시각화 이미지 저장 완료: {image_path}")
            # 고정된 파일명 사용
            route_image_url = "/static/routes/current_route_visualization.png"
    except Exception as e:
        print(f"[WARNING] 경로 시각화 이미지 생성 실패: {type(e).__name__}: {str(e)}")
    
    return {
        "commands": robot_commands,
        "scale_factor": scale_factor,
        "bbox": bbox_size,
        "target_area": {"width": target_width_m, "height": target_height_m},
        # 지도 표시용 추가 정보
        "path_coordinates": path_coordinates,  # [[lon, lat], ...]
        "start_coords": start_coords_result,  # [lon, lat]
        "goal_coords": goal_coords_result,  # [lon, lat]
        "map_bbox": {
            "min_lon": min_lon,
            "max_lon": max_lon,
            "min_lat": min_lat,
            "max_lat": max_lat,
        },
        "route_image": route_image_url,
        "path": path,  # 원본 경로 좌표 저장 (재계산용)
    }


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
    주소에서 로봇 명령 경로까지 전체 계산
    route_option: 경로 조회 옵션 (기본값: None - API 기본 옵션 사용)
    - 기본값: None (API 기본 옵션 사용)
    - trafast, tracomfort, traoptimal 등 (driving 엔드포인트에서 사용)
    returns: {
        "commands": [...],
        "scale_factor": float,
        "bbox": {"width": float, "height": float},
        "target_area": {"width": float, "height": float}
    }
    """
    # 주소 -> 위경도
    start_lon, start_lat = geocode(start_address)
    goal_lon, goal_lat = geocode(goal_address)
    
    # Directions 15 → 경로 (자동차 경로)
    path = get_directions(start_lon, start_lat, goal_lon, goal_lat, option=route_option)
    
    # 코너 기반 포인트 추출
    print(f"\n=== 경로 분석 시작 ===")
    print(f"전체 경로 포인트 수: {len(path)}")
    keypoints = extract_keypoints(path, angle_threshold=angle_threshold)
    # 첫 번째 점을 시작점, 마지막 점을 도착점으로 사용
    
    # 테스트 공간 크기에 맞춰 스케일링
    scaled_keypoints, scale_factor, bbox_size = fit_path_to_area(
        keypoints,
        target_width_m=target_width_m,
        target_height_m=target_height_m,
    )
    
    # 로봇 명령 계산
    robot_commands = calculate_robot_commands(
        scaled_keypoints,
        scale_factor=scale_factor,
        original_points=keypoints,
        linear_speed=linear_speed
    )
    
    # 지도 표시용 정보 추가
    # 경로의 실제 위경도 좌표 (전체 경로 path 사용 - 경로선 표시용)
    path_coordinates = [[p[0], p[1]] for p in path]  # [lon, lat] 형식 - 전체 경로
    
    # 출발지/도착지 좌표
    start_coords = [start_lon, start_lat]
    goal_coords = [goal_lon, goal_lat]
    
    # BBox 정보 (위경도 좌표계)
    min_lon = min(kp[0] for kp in keypoints)
    max_lon = max(kp[0] for kp in keypoints)
    min_lat = min(kp[1] for kp in keypoints)
    max_lat = max(kp[1] for kp in keypoints)
    
    # 경로 시각화 이미지 생성 (src 폴더에 저장)
    route_image_url = None
    try:
        image_path = visualize_path_image(
            keypoints,
            path_coordinates,
            output_dir=ROUTE_IMAGE_DIR,
        )
        if image_path:
            print(f"[INFO] 경로 시각화 이미지 저장 완료: {image_path}")
            # 고정된 파일명 사용
            route_image_url = "/static/routes/current_route_visualization.png"
    except Exception as e:
        print(f"[WARNING] 경로 시각화 이미지 생성 실패: {type(e).__name__}: {str(e)}")
    
    return {
        "commands": robot_commands,
        "scale_factor": scale_factor,
        "bbox": bbox_size,
        "target_area": {"width": target_width_m, "height": target_height_m},
        # 지도 표시용 추가 정보
        "path_coordinates": path_coordinates,  # [[lon, lat], ...]
        "start_coords": start_coords,  # [lon, lat]
        "goal_coords": goal_coords,  # [lon, lat]
        "map_bbox": {
            "min_lon": min_lon,
            "max_lon": max_lon,
            "min_lat": min_lat,
            "max_lat": max_lat,
        },
        "route_image": route_image_url,
    }

# -----------------------------
# 8) 실행
# -----------------------------
if __name__ == "__main__":
    # 입력 주소
    start_address = "경기 성남시 수정구 수정로 167"
    goal_address  = "경기 성남시 수정구 신흥동 5455"
    target_width_m = 2.0
    target_height_m = 3.0

    result = get_robot_path_from_addresses(
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

    
