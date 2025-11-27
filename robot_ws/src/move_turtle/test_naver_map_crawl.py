"""
네이버 지도 웹사이트 크롤링 및 경로 텍스트 추출
"""
import argparse
import sys
import requests
from bs4 import BeautifulSoup
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
import time
import json
import re
from pathlib import Path
from datetime import datetime

def test_requests_crawl(url):
    """requests + BeautifulSoup으로 크롤링 시도"""
    print("=" * 60)
    print("방법 1: requests + BeautifulSoup 테스트")
    print("=" * 60)
    
    headers = {
        'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36'
    }
    
    try:
        response = requests.get(url, headers=headers, timeout=10)
        print(f"응답 상태 코드: {response.status_code}")
        print(f"응답 헤더 Content-Type: {response.headers.get('Content-Type', 'N/A')}")
        
        if response.status_code == 200:
            soup = BeautifulSoup(response.text, 'html.parser')
            
            # direction_detail_desc_area 클래스 찾기
            desc_areas = soup.find_all('div', class_='direction_detail_desc_area')
            print(f"\n찾은 direction_detail_desc_area 개수: {len(desc_areas)}")
            
            if desc_areas:
                print("\n첫 번째 요소 텍스트:")
                print(desc_areas[0].get_text(strip=True))
                return True
            else:
                print("\n⚠️ direction_detail_desc_area를 찾을 수 없습니다.")
                print("JavaScript로 동적 렌더링되는 것 같습니다.")
                return False
        else:
            print(f"❌ 요청 실패: {response.status_code}")
            return False
            
    except Exception as e:
        print(f"❌ 오류 발생: {type(e).__name__}: {str(e)}")
        return False


def find_elements_in_all_frames(driver, by, value, timeout=5):
    """모든 iframe을 탐색하여 요소 찾기"""
    elements = []
    
    # 1. 메인 프레임에서 먼저 시도
    try:
        main_elements = driver.find_elements(by, value)
        if main_elements:
            print(f"  [메인 프레임] {len(main_elements)}개 요소 발견")
            elements.extend(main_elements)
    except:
        pass
    
    # 2. 모든 iframe 탐색
    try:
        iframes = driver.find_elements(By.TAG_NAME, "iframe")
        print(f"  [iframe 탐색] {len(iframes)}개 iframe 발견")
        
        for idx, iframe in enumerate(iframes):
            try:
                driver.switch_to.frame(iframe)
                print(f"  [iframe {idx+1}] 탐색 중...")
                
                # iframe 내부에서 요소 찾기
                frame_elements = driver.find_elements(by, value)
                if frame_elements:
                    print(f"  [iframe {idx+1}] {len(frame_elements)}개 요소 발견!")
                    elements.extend(frame_elements)
                
                # iframe 내부의 iframe도 재귀적으로 탐색 (최대 2단계)
                nested_iframes = driver.find_elements(By.TAG_NAME, "iframe")
                if nested_iframes:
                    for nested_idx, nested_iframe in enumerate(nested_iframes):
                        try:
                            driver.switch_to.frame(nested_iframe)
                            nested_elements = driver.find_elements(by, value)
                            if nested_elements:
                                print(f"  [iframe {idx+1}-{nested_idx+1}] {len(nested_elements)}개 요소 발견!")
                                elements.extend(nested_elements)
                            driver.switch_to.parent_frame()
                        except:
                            driver.switch_to.parent_frame()
                
                driver.switch_to.default_content()  # 메인 프레임으로 복귀
            except Exception as e:
                print(f"  [iframe {idx+1}] 탐색 실패: {e}")
                try:
                    driver.switch_to.default_content()
                except:
                    pass
    except Exception as e:
        print(f"  [iframe 탐색] 오류: {e}")
        try:
            driver.switch_to.default_content()
        except:
            pass
    
    return elements


def test_selenium_crawl(url, headless=True, wait_time=10, timeout=30, max_retries=3):
    """Selenium으로 크롤링 시도 (재시도 로직 포함)
    
    Args:
        url (str): 네이버 지도 URL
        headless (bool): 헤드리스 모드 여부
        wait_time (int): 초기 추가 대기 시간(초)
        timeout (int): 요소 탐색 타임아웃(초)
        max_retries (int): 최대 재시도 횟수
    """
    print("\n" + "=" * 60)
    print("방법 2: Selenium 테스트")
    print("=" * 60)
    
    for attempt in range(1, max_retries + 1):
        print(f"\n[시도 {attempt}/{max_retries}]")
        
        chrome_options = Options()
        if headless:
            chrome_options.add_argument('--headless=new')  # 최신 headless 모드
        chrome_options.add_argument('--no-sandbox')
        chrome_options.add_argument('--disable-dev-shm-usage')
        chrome_options.add_argument('--disable-gpu')
        chrome_options.add_argument('--window-size=1920,1080')
        chrome_options.add_argument('user-agent=Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36')
        # 봇 감지 완화
        chrome_options.add_argument('--disable-blink-features=AutomationControlled')
        chrome_options.add_experimental_option("excludeSwitches", ["enable-automation"])
        chrome_options.add_experimental_option('useAutomationExtension', False)
        
        driver = None
        try:
            print("Chrome 드라이버 초기화 중...")
            driver = webdriver.Chrome(options=chrome_options)
            
            # 봇 감지 우회 스크립트
            driver.execute_cdp_cmd('Page.addScriptToEvaluateOnNewDocument', {
                'source': '''
                    Object.defineProperty(navigator, 'webdriver', {
                        get: () => undefined
                    })
                '''
            })
            
            print(f"페이지 로드 중: {url}")
            driver.get(url)
            
            # 페이지 로드 대기 (JavaScript 렌더링 완료 대기)
            print("페이지 로드 대기 중...")
            
            # JavaScript 실행 완료 대기
            WebDriverWait(driver, max(20, wait_time)).until(
                lambda d: d.execute_script("return document.readyState") == "complete"
            )
            
            # 추가 대기 (동적 콘텐츠 로드) - 재시도할수록 더 오래 대기
            adjusted_wait_time = wait_time + (attempt - 1) * 5  # 시도마다 5초씩 증가
            print(f"동적 콘텐츠 로드 대기 중... ({adjusted_wait_time}초)")
            time.sleep(adjusted_wait_time)
            
            # 페이지 스크롤하여 요소가 로드되도록 유도
            # 네이버 지도는 좌측 패널에 경로 정보가 있으므로, 스크롤이 필요할 수 있음
            print("페이지 스크롤하여 동적 콘텐츠 로드 유도...")
            for scroll_y in [300, 600, 900, 0]:  # 여러 위치로 스크롤
                driver.execute_script(f"window.scrollTo(0, {scroll_y});")
                time.sleep(1)
            
            # 좌측 패널 영역으로 포커스 이동 시도
            try:
                # 좌측 패널이 있는 경우 클릭하여 활성화
                panel_selectors = [
                    "div._panel",
                    "div.direction_panel",
                    "[class*='panel']",
                ]
                for selector in panel_selectors:
                    try:
                        panel = driver.find_element(By.CSS_SELECTOR, selector)
                        driver.execute_script("arguments[0].scrollIntoView({behavior: 'smooth', block: 'center'});", panel)
                        time.sleep(1)
                        break
                    except:
                        continue
            except:
                pass
            
            # direction_detail_desc_area 요소 찾기 (더 긴 타임아웃)
            wait = WebDriverWait(driver, timeout)
            try:
                # 먼저 요소가 나타날 때까지 대기
                print("경로 텍스트 요소 찾는 중...")
                
                # 1단계: 메인 프레임에서 요소 찾기 시도
                try:
                    elements = wait.until(
                        EC.presence_of_all_elements_located((By.CLASS_NAME, "direction_detail_desc_area"))
                    )
                    print(f"  [1단계] 메인 프레임에서 {len(elements)}개 요소 발견")
                except:
                    # 메인 프레임에서 못 찾으면 iframe 탐색
                    print("  [1단계] 메인 프레임에서 요소를 찾지 못함, iframe 탐색 시작...")
                    elements = find_elements_in_all_frames(driver, By.CLASS_NAME, "direction_detail_desc_area", timeout)
                    if not elements:
                        raise Exception("모든 프레임에서 요소를 찾지 못했습니다.")
                    print(f"  [1단계] iframe 탐색 결과: {len(elements)}개 요소 발견")
                
                # 2단계: 요소가 실제로 보일 때까지 대기
                try:
                    wait.until(
                        EC.visibility_of_any_elements_located((By.CLASS_NAME, "direction_detail_desc_area"))
                    )
                    print(f"  [2단계] 요소가 화면에 표시됨")
                except:
                    # visibility 체크 실패해도 계속 진행 (iframe 내부일 수 있음)
                    print(f"  [2단계] visibility 체크 건너뜀 (iframe 내부일 수 있음)")
                
                # 3단계: 텍스트가 채워질 때까지 추가 대기 (재시도할수록 더 오래 대기)
                print("  [3단계] 텍스트 로드 대기 중...")
                max_text_wait = 10 + (attempt - 1) * 5  # 시도마다 5초씩 증가
                text_wait_start = time.time()
                while time.time() - text_wait_start < max_text_wait:
                    # iframe 탐색을 다시 해서 최신 요소 가져오기
                    current_elements = find_elements_in_all_frames(driver, By.CLASS_NAME, "direction_detail_desc_area", timeout=2)
                    if current_elements:
                        # 텍스트가 있는 요소가 하나라도 있으면 성공
                        texts = [elem.text.strip() for elem in current_elements]
                        valid_texts = [t for t in texts if t]
                        if valid_texts and len(valid_texts) >= 2:  # 최소 2개 이상의 텍스트가 있어야 함
                            print(f"  [3단계] 텍스트 로드 완료 ({len(valid_texts)}개 요소에 텍스트 있음)")
                            elements = current_elements
                            break
                    time.sleep(0.5)
                else:
                    print("  [경고] 일부 요소에 텍스트가 없을 수 있습니다.")
                    # 메인 프레임으로 복귀
                    try:
                        driver.switch_to.default_content()
                    except:
                        pass
                
                print(f"\n✅ 찾은 direction_detail_desc_area 개수: {len(elements)}")
                
                if elements:
                    # 브라우저 종료 전에 텍스트 미리 추출
                    route_texts = []
                    for idx, elem in enumerate(elements):
                        try:
                            # 요소가 보이는지 확인
                            if not elem.is_displayed():
                                print(f"  [요소 {idx+1}] 화면에 보이지 않음, 건너뜀")
                                continue
                            
                            # direction_detail_desc_area의 텍스트 추출
                            text = elem.text.strip()
                            
                            # 텍스트가 비어있으면 추가 대기
                            if not text:
                                print(f"  [요소 {idx+1}] 텍스트가 비어있음, 추가 대기...")
                                time.sleep(1)
                                text = elem.text.strip()  # 재시도
                            
                            # 같은 부모 요소에서 direction_distance 찾기
                            try:
                                # 부모 요소 찾기
                                parent = elem.find_element(By.XPATH, "./..")
                                # direction_distance 요소 찾기
                                distance_elem = parent.find_element(By.CLASS_NAME, "direction_distance")
                                distance_text = distance_elem.text.strip()  # 예: "11m 이동"
                                
                                # 텍스트에 거리 정보 추가
                                if distance_text and text:
                                    text = f"{text} {distance_text}"
                            except Exception as dist_exc:
                                # direction_distance를 찾지 못하면 원본 텍스트만 사용
                                print(f"  [요소 {idx+1}] direction_distance 찾기 실패: {dist_exc}")
                                pass
                            
                            if text:
                                route_texts.append(text)
                                print(f"  [요소 {idx+1}] 텍스트 추출 성공: {text[:50]}...")
                            else:
                                print(f"  [요소 {idx+1}] 텍스트가 여전히 비어있음")
                        except Exception as elem_exc:
                            print(f"  [요소 {idx+1}] 처리 중 오류: {elem_exc}")
                            continue
                    
                    print("\n추출된 텍스트 (처음 5개):")
                    for i, text in enumerate(route_texts[:5]):
                        print(f"\n[{i+1}] {text}")
                    
                    if len(route_texts) > 5:
                        print(f"\n... 외 {len(route_texts) - 5}개")
                    
                    # 성공 시 브라우저 종료하고 결과 반환
                    driver.quit()
                    print("\n✅ 크롤링 성공!")
                    return True, route_texts
                else:
                    print("\n⚠️ direction_detail_desc_area를 찾을 수 없습니다.")
                    # 재시도 필요
                    if attempt < max_retries:
                        print(f"  [재시도 예정] {attempt + 1}번째 시도에서 더 긴 대기 시간으로 재시도합니다...")
                        if driver:
                            driver.quit()
                        time.sleep(5)  # 재시도 전 5초 대기 (서버 부하 방지)
                        continue
                    else:
                        if driver:
                            driver.quit()
                        return False, None
                        
            except Exception as e:
                print(f"\n⚠️ 요소를 찾는 중 오류: {type(e).__name__}: {str(e)}")
                # 페이지 소스 확인
                if driver:
                    print("\n페이지 제목:", driver.title)
                    print("현재 URL:", driver.current_url)
            
            # 재시도 필요
            if attempt < max_retries:
                print(f"  [재시도 예정] {attempt + 1}번째 시도에서 더 긴 대기 시간으로 재시도합니다...")
                if driver:
                    driver.quit()
                time.sleep(5)  # 재시도 전 5초 대기 (서버 부하 방지)
                continue
            else:
                # 마지막 시도에서도 대체 선택자 시도
                print("\n대체 선택자로 재시도 중...")
                try:
                    driver.switch_to.default_content()  # 메인 프레임으로 복귀
                    # 다른 가능한 선택자들 시도
                    alternative_selectors = [
                        (By.CSS_SELECTOR, "div.direction_detail_desc_area"),
                        (By.CSS_SELECTOR, "[class*='direction_detail']"),
                        (By.XPATH, "//div[contains(@class, 'direction_detail')]"),
                    ]
                    
                    for selector_type, selector_value in alternative_selectors:
                        try:
                            elements = find_elements_in_all_frames(driver, selector_type, selector_value, timeout=5)
                            if elements:
                                print(f"✅ 대체 선택자로 {len(elements)}개 요소 발견")
                                # 원래 로직으로 텍스트 추출
                                route_texts = []
                                for elem in elements:
                                    text = elem.text.strip()
                                    if text:
                                        route_texts.append(text)
                                if route_texts:
                                    driver.quit()
                                    return True, route_texts
                        except:
                            continue
                except:
                    pass
                
                if driver:
                    driver.quit()
                return False, None
                
        except Exception as e:
            print(f"❌ Selenium 오류: {type(e).__name__}: {str(e)}")
            if driver:
                driver.quit()
            # 재시도 가능하면 재시도
            if attempt < max_retries:
                print(f"  [재시도 예정] {attempt + 1}번째 시도에서 재시도합니다...")
                time.sleep(5)  # 재시도 전 5초 대기 (서버 부하 방지)
                continue
            # 마지막 시도 실패
            return False, None
    
    # 모든 시도 실패
    print(f"\n❌ {max_retries}번 시도 모두 실패했습니다.")
    return False, None


def extract_route_texts(elements):
    """추출된 요소들에서 텍스트만 파싱"""
    if not elements:
        return []
    
    route_texts = []
    for elem in elements:
        text = elem.text.strip()
        if text:
            route_texts.append(text)
    
    return route_texts


def find_crosswalks(route_texts):
    """경로 텍스트에서 횡단보도 정보 추출"""
    crosswalks = []
    
    for i, text in enumerate(route_texts):
        # "횡단보도" 키워드가 포함된 텍스트 찾기
        if "횡단보도" in text:
            # 거리 정보 추출 (예: "28m 이동")
            distance_match = re.search(r'(\d+)m\s*이동', text)
            distance = int(distance_match.group(1)) if distance_match else None
            
            crosswalks.append({
                "index": i,
                "text": text,
                "distance": distance,
                "direction": None  # 사용자 입력으로 채워짐
            })
    
    return crosswalks


def get_crosswalk_directions(crosswalks):
    """사용자로부터 횡단보도 방향 입력 받기"""
    print("\n" + "=" * 60)
    print("횡단보도 방향 입력")
    print("=" * 60)
    print("\n각 횡단보도에 대해 방향을 입력해주세요:")
    print("  0도: 직진")
    print("  90도: 좌회전")
    print("  -90도: 우회전")
    print()
    
    for i, crosswalk in enumerate(crosswalks, 1):
        print(f"\n[{i}/{len(crosswalks)}] {crosswalk['text']}")
        print(f"    거리: {crosswalk['distance']}m")
        
        while True:
            try:
                direction_input = input("    각도를 입력하세요 (0/90/-90): ").strip()
                direction = int(direction_input)
                
                if direction in [0, 90, -90]:
                    crosswalk['direction'] = direction
                    break
                else:
                    print("    ⚠️ 0, 90, 또는 -90만 입력 가능합니다.")
            except ValueError:
                print("    ⚠️ 숫자를 입력해주세요.")
    
    return crosswalks


def parse_route_actions(route_texts, crosswalks):
    """경로 텍스트를 액션 리스트로 파싱"""
    actions = []
    
    for i, text in enumerate(route_texts):
        # 거리 정보 추출
        distance_match = re.search(r'(\d+)m\s*이동', text)
        distance = int(distance_match.group(1)) if distance_match else None
        
        # 액션 타입 판단
        action_type = None
        if "횡단보도" in text:
            # 횡단보도 정보 찾기
            crosswalk_info = next((cw for cw in crosswalks if cw['index'] == i), None)
            if crosswalk_info:
                if crosswalk_info['direction'] == 0:
                    action_type = "횡단보도_직진"
                elif crosswalk_info['direction'] == 90:
                    action_type = "횡단보도_좌회전"
                elif crosswalk_info['direction'] == -90:
                    action_type = "횡단보도_우회전"
                else:
                    action_type = "횡단보도"
        elif "직진" in text or "방면으로" in text:
            action_type = "직진"
        elif "왼쪽" in text or "좌회전" in text or "왼" in text:
            action_type = "좌회전"
        elif "오른쪽" in text or "우회전" in text or "우측" in text or "우" in text:
            action_type = "우회전"
        else:
            action_type = "기타"
        
        actions.append({
            "index": i,
            "text": text,
            "action": action_type,
            "distance": distance
        })
    
    return actions


def crawl_route(url: str, headless: bool = True, wait_time: int = 10, timeout: int = 30):
    """URL을 크롤링하여 경로 텍스트/횡단보도/액션 정보를 반환"""
    print(f"[크롤링] URL: {url}")
    route_texts: list[str] = []
    
    # 1) BeautifulSoup 시도
    success = test_requests_crawl(url)
    if success:
        headers = {
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/120.0.0.0 Safari/537.36'
        }
        try:
            response = requests.get(url, headers=headers, timeout=10)
            if response.status_code == 200:
                soup = BeautifulSoup(response.text, 'html.parser')
                desc_areas = soup.find_all('div', class_='direction_detail_desc_area')
                for elem in desc_areas:
                    text = elem.get_text(strip=True)
                    if text:
                        route_texts.append(text)
        except Exception as exc:
            print(f"[크롤링] BeautifulSoup 추가 처리 중 오류: {exc}")
    
    # 2) Selenium 시도
    if not route_texts:
        success, route_texts = test_selenium_crawl(
            url,
            headless=headless,
            wait_time=wait_time,
            timeout=timeout,
        )
        if not success or not route_texts:
            raise RuntimeError("경로 텍스트 추출 실패")
    
    crosswalks = find_crosswalks(route_texts)
    actions = parse_route_actions(route_texts, crosswalks)
    
    result_data = {
        "url": url,
        "extracted_at": datetime.now().isoformat(),
        "route_texts": route_texts,
        "crosswalks": crosswalks,
        "actions": actions,
    }
    return result_data


def save_to_json(data, output_path=None):
    """데이터를 JSON 파일로 저장"""
    if output_path is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = Path(__file__).parent / f"walking_route_{timestamp}.json"
    else:
        output_path = Path(output_path)
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    
    print(f"\n✅ JSON 파일 저장 완료: {output_path}")
    return str(output_path)


def convert_json_to_robot_commands(json_path, target_width_m=2.0, target_height_m=3.0):
    """
    JSON 파일을 로봇 명령 형식으로 변환 (api_to_position.py와 동일한 형식)
    
    Args:
        json_path: walking_route JSON 파일 경로
        target_width_m: 테스트 공간 가로 크기 (m)
        target_height_m: 테스트 공간 세로 크기 (m)
    
    Returns:
        api_to_position.py와 동일한 형식의 commands 리스트
        [{"segment": int, "start_point": [x, y], "end_point": [x, y], 
          "distance": m, "original_distance": m, "turn_angle": deg, "instruction": str}, ...]
    """
    import math
    
    # JSON 파일 읽기
    json_path = Path(json_path)
    if not json_path.exists():
        raise FileNotFoundError(f"JSON 파일을 찾을 수 없습니다: {json_path}")
    
    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    actions = data.get("actions", [])
    if not actions:
        raise ValueError("JSON에 actions 데이터가 없습니다.")
    
    # 거리 정보가 없는 경우 텍스트에서 다시 추출
    for action in actions:
        if action.get("distance") is None:
            text = action.get("text", "")
            distance_match = re.search(r'(\d+)m\s*이동', text)
            if distance_match:
                action["distance"] = int(distance_match.group(1))
    
    # ============================================================
    # 1단계: 실제 경로 시뮬레이션하여 bbox 계산 (m 단위)
    # ============================================================
    current_x = 0.0  # m 단위
    current_y = 0.0  # m 단위
    current_heading = 90.0  # 오른쪽 방향 (90도)
    
    x_coords = [current_x]
    y_coords = [current_y]
    
    # 첫 번째 액션 제외 여부 확인
    start_idx = 0
    if actions and actions[0].get("action") in ["기타", "출발"]:
        start_idx = 1
    
    # 경로 시뮬레이션 (실제 거리 m 단위로)
    for i in range(start_idx, len(actions)):
        action = actions[i]
        action_type = action.get("action", "")
        distance_m = float(action.get("distance", 0) or 0)
        text = action.get("text", "")
        
        # 도착지 처리 (마지막 action)
        if "도착" in text and i == len(actions) - 1:
            break
        
        # 회전 각도 결정
        turn_angle = 0.0
        if "횡단보도_좌회전" in action_type:
            turn_angle = 90.0
        elif "횡단보도_우회전" in action_type:
            turn_angle = -90.0
        elif "좌회전" in action_type or "왼쪽" in action_type or "왼" in action_type:
            turn_angle = 90.0
        elif "우회전" in action_type or "오른쪽" in action_type or "우측" in action_type or "우" in action_type:
            turn_angle = -90.0
        
        # 첫 번째 action: 회전 제외
        if i == start_idx:
            turn_angle = 0.0
        else:
            # 회전 적용
            if abs(turn_angle) > 0.1:
                current_heading = (current_heading - turn_angle) % 360
        
        # 직진 이동
        if distance_m > 0.001:
            heading_rad = math.radians(current_heading)
            dx = math.sin(heading_rad) * distance_m
            dy = -math.cos(heading_rad) * distance_m  # y축 반전
            
            current_x += dx
            current_y += dy
            x_coords.append(current_x)
            y_coords.append(current_y)
    
    # bbox 계산
    if len(x_coords) > 1:
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        bbox_width = max_x - min_x
        bbox_height = max_y - min_y
    else:
        bbox_width = 0.0
        bbox_height = 0.0
    
    # ============================================================
    # 2단계: 스케일 팩터 계산 (차선도로와 동일한 로직)
    # ============================================================
    if bbox_width == 0 and bbox_height == 0:
        scale_factor = 1.0
    else:
        scale_x = target_width_m / bbox_width if bbox_width != 0 else float("inf")
        scale_y = target_height_m / bbox_height if bbox_height != 0 else float("inf")
        scale_factor = min(scale_x, scale_y)
    
    print(f"[도보도로 스케일링] 실제 경로 bbox: {bbox_width:.2f}m × {bbox_height:.2f}m")
    print(f"[도보도로 스케일링] 목표 공간: {target_width_m}m × {target_height_m}m")
    print(f"[도보도로 스케일링] 스케일 팩터: {scale_factor:.6f}")
    
    # ============================================================
    # 3단계: 로봇 명령 생성 (스케일링 적용)
    # ============================================================
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
            "linear": 0.1,
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
            turn_angle = 0.0  # 첫 번째 action에서는 회전 무시
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
        if "도착" in text or i == len(actions) - 1:
            # 도착지에서 "도로의 오른쪽/왼쪽에 있습니다" 확인
            arrival_turn_angle = 0.0
            if "도로의 오른쪽에 있습니다" in text:
                arrival_turn_angle = -90.0  # 우회전
            elif "도로의 왼쪽에 있습니다" in text:
                arrival_turn_angle = 90.0  # 좌회전
            
            commands.append({
                "segment": segment_idx,
                "start_point": [0.0, 0.0],
                "end_point": [0.0, 0.0],
                "distance": 0.0,
                "original_distance": 0.0,
                "turn_angle": arrival_turn_angle,  # 도착지 회전
                "instruction": "도착" + (f" (도로의 {'오른쪽' if arrival_turn_angle < 0 else '왼쪽'}에 있습니다)" if arrival_turn_angle != 0 else ""),
                "linear": 0.0,
                "angular": 0.0,
                "duration": 0.0,
                "type": "arrival"
            })
            break
        
        # 스케일링 적용 (차선도로와 동일한 로직)
        original_distance = float(distance)  # 실제 거리 (m)
        scaled_distance = original_distance * scale_factor  # 스케일링된 거리 (m)
        
        # 명령 추가
        instruction = action_type.replace("횡단보도_", "횡단보도 ")
        if distance > 0:
            instruction = f"{instruction} {distance}m"
        
        # duration 계산 (기본 속도 0.1 m/s 사용)
        default_speed = 0.1
        duration = scaled_distance / default_speed if default_speed > 0 else 0.0
        
        commands.append({
            "segment": segment_idx,
            "start_point": [0.0, 0.0],  # 더미 좌표
            "end_point": [0.0, 0.0],
            "distance": scaled_distance,  # 스케일링된 거리
            "original_distance": original_distance,  # 실제 거리
            "turn_angle": turn_angle,
            "instruction": instruction,
            "linear": default_speed,
            "angular": 0.0,
            "duration": duration,
            "type": "straight" if abs(turn_angle) < 1.0 else "turn"
        })
        segment_idx += 1
    
    return commands


def run_interactive():
    print("=" * 60)
    print("네이버 지도 경로 텍스트 추출")
    print("=" * 60)
    
    # 네이버 지도 URL 입력 받기
    default_url = "https://map.naver.com/p/directions/14153002.4755856,4501066.3018202,%EA%B2%BD%EA%B8%B0%20%EC%84%B1%EB%82%A8%EC%8B%9C%20%EC%88%98%EC%A0%95%EA%B5%AC%20%EC%88%98%EC%A0%95%EB%A1%9C%20167,02131102,ADDRESS_POI/14154073.591726,4500876.0825572,%EB%8F%99%EC%9E%90%EB%A7%A4%20%EC%96%91%EA%BC%AC%EC%B9%98%26%EB%A7%88%EB%9D%BC%ED%83%95,34320876,PLACE_POI/-/walk/0?c=18.78,0,0,3,dh"
    
    url_input = input(f"\n네이버 지도 URL을 입력하세요 (엔터: 기본값 사용):\n").strip()
    
    if url_input:
        url = url_input
    else:
        url = default_url
        print(f"\n기본 URL 사용: {url}")
    
    print(f"\n사용할 URL: {url}\n")
    
    # 텍스트 추출 시도 (Selenium 사용 - JavaScript 렌더링 필요)
    print("경로 텍스트 추출 중...")
    success, route_texts = test_selenium_crawl(url, headless=False)
    
    if not success or not route_texts:
        print("\n❌ 텍스트 추출 실패")
        sys.exit(1)
    
    print("\n" + "=" * 60)
    print("추출된 경로 텍스트")
    print("=" * 60)
    for i, text in enumerate(route_texts, 1):
        print(f"\n[{i}] {text}")
    
    # 횡단보도 찾기
    crosswalks = find_crosswalks(route_texts)
    
    if crosswalks:
        # 횡단보도 방향 입력 받기
        crosswalks = get_crosswalk_directions(crosswalks)
    else:
        print("\n⚠️ 횡단보도가 없습니다.")
    
    # 경로 액션 파싱
    actions = parse_route_actions(route_texts, crosswalks)
    
    # JSON 데이터 구성
    result_data = {
        "url": url,
        "extracted_at": datetime.now().isoformat(),
        "route_texts": route_texts,
        "crosswalks": crosswalks,
        "actions": actions
    }
    
    # JSON 파일로 저장
    print("\n" + "=" * 60)
    print("JSON 파일 저장")
    print("=" * 60)
    save_to_json(result_data)
    
    print("\n" + "=" * 60)
    print("완료!")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser(description="네이버 지도 경로 텍스트 추출기")
    parser.add_argument("--url", help="네이버 지도 URL")
    parser.add_argument("--output-json", help="결과를 저장할 JSON 경로")
    parser.add_argument("--headless", action="store_true", help="Selenium을 headless 모드로 실행")
    parser.add_argument("--wait-time", type=int, default=10, help="Selenium 추가 대기 시간(초)")
    parser.add_argument("--timeout", type=int, default=30, help="Selenium 요소 탐색 타임아웃(초)")
    parser.add_argument("--interactive", action="store_true", help="기존 인터랙티브 모드로 실행")
    
    args = parser.parse_args()
    
    if args.interactive or not args.url:
        run_interactive()
        return
    
    try:
        result = crawl_route(
            args.url,
            headless=args.headless,
            wait_time=args.wait_time,
            timeout=args.timeout,
        )
    except Exception as exc:
        print(f"❌ 크롤링 실패: {exc}", file=sys.stderr)
        sys.exit(1)
    
    if args.output_json:
        save_to_json(result, args.output_json)
    else:
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

