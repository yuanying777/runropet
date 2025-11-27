#!/usr/bin/env python3
"""
도보 경로 이미지 생성 테스트 스크립트
"""
import sys
from pathlib import Path

# 프로젝트 모듈 경로 추가
BASE_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE_DIR))

from api_to_position import visualize_walking_route_image

# 테스트용 경로 텍스트
route_texts = [
    "경기 성남시 수정구 수정로 167",
    "하나은행 성남중앙지점까지 직진 13m 이동",
    "컴포즈커피 성남의료원점 방면으로 횡단보도 건너기 26m 이동",
    "왼쪽 방향 16m 이동",
    "파리바게뜨 성남신흥점에서 오른쪽 방향 37m 이동",
    "농민장터에서 왼쪽 방향 66m 이동",
    "블랙국밥 성남신흥점상세정보 도착지는 도로의 오른쪽에 있습니다."
]

# 출발지/도착지 좌표 (네이버 지도에서 추출한 실제 좌표 사용)
# URL 형식: https://map.naver.com/p/directions/14153002.4755856,4501066.3018202,.../...
test_url = "https://map.naver.com/p/directions/14153002.4755856,4501066.3018202,경기%20성남시%20수정구%20수정로%20167,02131102,ADDRESS_POI/14153141.7585325,4501003.8951645,블랙국밥%20성남신흥점,1374806476,PLACE_POI/-/walk/0?c=18.00,0,0,0,dh"

# actions 생성 (parse_route_actions와 동일한 형식)
import re
actions = []
crosswalks = []

# 첫 번째는 출발지, 마지막은 도착지이므로 제외
for i, text in enumerate(route_texts):
    if i == 0 or i == len(route_texts) - 1:
        continue  # 출발지/도착지 제외
    
    # 거리 추출
    distance_match = re.search(r'(\d+)m\s*이동', text)
    distance = int(distance_match.group(1)) if distance_match else None
    
    if distance is None:
        continue  # 거리가 없으면 건너뛰기
    
    # 횡단보도 체크
    if "횡단보도" in text:
        crosswalks.append({
            "index": i,
            "text": text,
            "distance": distance,
            "direction": -90  # 사용자 지정: -90도
        })
        action_type = "횡단보도_우회전"
    elif "직진" in text:
        action_type = "직진"
    elif "왼쪽" in text or "좌회전" in text:
        action_type = "좌회전"
    elif "오른쪽" in text or "우회전" in text:
        action_type = "우회전"
    else:
        action_type = "직진"  # 기본값은 직진
    
    actions.append({
        "index": i,
        "text": text,
        "action": action_type,
        "distance": distance
    })

print("=" * 60)
print("도보 경로 이미지 생성 테스트")
print("=" * 60)
print(f"출발지: {route_texts[0]}")
print(f"도착지: {route_texts[-1]}")
print(f"\nActions ({len(actions)}개):")
for i, action in enumerate(actions):
    print(f"  {i+1}. {action['text'][:50]}...")
    print(f"     - action: {action['action']}, distance: {action['distance']}m")
print(f"\n횡단보도: {len(crosswalks)}개")
for cw in crosswalks:
    print(f"  - {cw['direction']}도: {cw['text'][:50]}...")

print("\n" + "=" * 60)
print("이미지 생성 중...")
print("=" * 60)

# 이미지 생성
output_dir = BASE_DIR / "dashboard" / "static" / "routes"
output_dir.mkdir(parents=True, exist_ok=True)

image_url = visualize_walking_route_image(
    url=test_url,
    actions=actions,
    output_dir=output_dir,
)

if image_url:
    print(f"\n✅ 이미지 생성 완료!")
    print(f"   파일 경로: {output_dir / Path(image_url).name}")
    print(f"   웹 경로: {image_url}")
else:
    print("\n❌ 이미지 생성 실패")
    import traceback
    traceback.print_exc()

