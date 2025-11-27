// 네이버 지도 JavaScript API v3 관리
let naverMap = null;
let startMarker = null;
let goalMarker = null;
let pathPolyline = null;

// 지도 초기화
function initMap() {
  const mapDiv = document.getElementById('map');
  if (!mapDiv) {
    console.error('[지도] map div를 찾을 수 없습니다.');
    return;
  }

  // 기본 중심점 (성남시 수정구)
  const defaultCenter = new naver.maps.LatLng(37.4500, 127.1500);
  
  const mapOptions = {
    center: defaultCenter,
    zoom: 15,
    zoomControl: true,
    zoomControlOptions: {
      position: naver.maps.Position.TOP_RIGHT
    }
  };

  naverMap = new naver.maps.Map('map', mapOptions);
  console.log('[지도] 네이버 지도 초기화 완료');
}

// 주소를 좌표로 변환 (Geocoding)
async function geocodeAddress(address) {
  try {
    console.log('[지도] Geocoding 요청:', address);
    const resp = await fetch('/api/geocode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ address: address })
    });
    
    if (!resp.ok) {
      const errorText = await resp.text();
      throw new Error(`서버 오류 (${resp.status}): ${errorText}`);
    }
    
    const data = await resp.json();
    console.log('[지도] Geocoding 응답:', data);
    
    if (data.status === 'ok' && data.coords && data.coords.length >= 2) {
      return new naver.maps.LatLng(data.coords[1], data.coords[0]); // [lon, lat] -> [lat, lon]
    } else {
      throw new Error(data.message || '주소를 찾을 수 없습니다.');
    }
  } catch (error) {
    console.error('[지도] Geocoding 오류:', error);
    throw error;
  }
}

// 출발지 마커 표시
async function showStartMarker(address) {
  try {
    const position = await geocodeAddress(address);
    
    if (startMarker) {
      startMarker.setPosition(position);
    } else {
      startMarker = new naver.maps.Marker({
        position: position,
        map: naverMap,
        icon: {
          content: '<div style="background-color: #00C768; width: 20px; height: 20px; border-radius: 50%; border: 2px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>',
          anchor: new naver.maps.Point(10, 10)
        },
        title: '출발지'
      });
    }
    
    // 지도 중심 이동
    naverMap.setCenter(position);
    return position;
  } catch (error) {
    console.error('[지도] 출발지 마커 표시 오류:', error);
    alert('출발지를 찾을 수 없습니다: ' + error.message);
    return null;
  }
}

// 도착지 마커 표시
async function showGoalMarker(address) {
  try {
    const position = await geocodeAddress(address);
    
    if (goalMarker) {
      goalMarker.setPosition(position);
    } else {
      goalMarker = new naver.maps.Marker({
        position: position,
        map: naverMap,
        icon: {
          content: '<div style="background-color: #FF0000; width: 20px; height: 20px; border-radius: 50%; border: 2px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>',
          anchor: new naver.maps.Point(10, 10)
        },
        title: '도착지'
      });
    }
    
    return position;
  } catch (error) {
    console.error('[지도] 도착지 마커 표시 오류:', error);
    alert('도착지를 찾을 수 없습니다: ' + error.message);
    return null;
  }
}

// 경로선 표시
function showPath(pathCoordinates) {
  // 기존 경로선 제거
  if (pathPolyline) {
    pathPolyline.setMap(null);
  }

  if (!pathCoordinates || pathCoordinates.length < 2) {
    console.warn('[지도] 경로 좌표가 부족합니다.');
    return;
  }

  // 좌표 변환: [lon, lat] -> naver.maps.LatLng
  const pathArray = pathCoordinates.map(coord => 
    new naver.maps.LatLng(coord[1], coord[0]) // [lon, lat] -> [lat, lon]
  );

  pathPolyline = new naver.maps.Polyline({
    map: naverMap,
    path: pathArray,
    strokeColor: '#0475F4',
    strokeWeight: 5,
    strokeOpacity: 0.8
  });

  // 경로가 모두 보이도록 지도 범위 조정
  const bounds = new naver.maps.LatLngBounds();
  pathArray.forEach(latlng => bounds.extend(latlng));
  naverMap.fitBounds(bounds);
  
  console.log('[지도] 경로선 표시 완료:', pathCoordinates.length, '개 좌표');
}

// 출발지와 도착지 모두 표시하고 지도 범위 조정
async function showStartAndGoal(startAddress, goalAddress) {
  const startPos = await showStartMarker(startAddress);
  const goalPos = await showGoalMarker(goalAddress);
  
  if (startPos && goalPos) {
    // 두 마커가 모두 보이도록 지도 범위 조정
    const bounds = new naver.maps.LatLngBounds();
    bounds.extend(startPos);
    bounds.extend(goalPos);
    naverMap.fitBounds(bounds);
  }
}

// 지도 초기화 (페이지 로드 시)
window.addEventListener('load', () => {
  // 네이버 지도 API 로드 확인
  if (typeof naver === 'undefined' || !naver.maps) {
    console.error('[지도] 네이버 지도 API가 로드되지 않았습니다.');
    const mapDiv = document.getElementById('map');
    if (mapDiv) {
      mapDiv.innerHTML = '<div style="padding: 20px; text-align: center; color: #666;">네이버 지도 API 로드 실패. API 키를 확인해주세요.</div>';
    }
    return;
  }
  
  // 지도 초기화
  try {
    initMap();
  } catch (error) {
    console.error('[지도] 지도 초기화 오류:', error);
    const mapDiv = document.getElementById('map');
    if (mapDiv) {
      mapDiv.innerHTML = `<div style="padding: 20px; text-align: center; color: #f00;">지도 초기화 실패: ${error.message}</div>`;
    }
  }
});

