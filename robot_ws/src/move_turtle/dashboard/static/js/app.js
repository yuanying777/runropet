const scaleModal = document.getElementById("scaleModal");
const scaleModeButtons = document.querySelectorAll("#scaleModal .toggle button");
const scaleInputs = document.getElementById("scaleInputs");
const realInputs = document.getElementById("realInputs");
const modalWidthInput = document.getElementById("modalWidth");
const modalHeightInput = document.getElementById("modalHeight");
const modalScaleSpeedInput = document.getElementById("modalScaleSpeed");
const modalRealSpeedInput = document.getElementById("modalRealSpeed");
const scaleModeTag = document.getElementById("scaleModeTag");
const speedTag = document.getElementById("speedTag");
const scaleTag = document.getElementById("scaleTag");
const scaleSaveBtn = document.getElementById("scaleSaveBtn");
const testModal = document.getElementById("testModal");
const testModalText = document.getElementById("testModalText");

let timelinePlaybackEntries = [];
let timelinePlaybackController = null;
let animationStartTimeout = null; // 애니메이션 시작 타이머 ID 저장

function createTimelineArrows() {
  const arrowsWrap = document.createElement("span");
  arrowsWrap.className = "timeline-arrows";
  for (let i = 0; i < 3; i += 1) {
    const arrow = document.createElement("span");
    arrow.className = "timeline-arrow";
    arrowsWrap.appendChild(arrow);
  }
  return arrowsWrap;
}

function registerTimelineEntries(entries = []) {
  stopTimelinePlayback();
  timelinePlaybackEntries = entries;
  setActiveTimelineIndex(-1);
  console.log(`[타임라인 등록] ${entries.length}개 엔트리 등록 완료`);
}

function setActiveTimelineIndex(idx) {
  if (idx < 0) {
    // 모든 엔트리 비활성화
    timelinePlaybackEntries.forEach((entry) => {
      if (entry.element) {
        entry.element.classList.remove("is-active");
      }
    });
    return;
  }
  
  let activatedCount = 0;
  timelinePlaybackEntries.forEach((entry, entryIdx) => {
    if (!entry.element) {
      console.warn(`[타임라인 활성화] 엔트리 ${entryIdx}의 element가 없습니다.`);
      return;
    }
    if (idx === entryIdx) {
      entry.element.classList.add("is-active");
      activatedCount++;
    } else {
      entry.element.classList.remove("is-active");
    }
  });
  
  if (activatedCount === 0 && idx >= 0) {
    console.warn(`[타임라인 활성화] 인덱스 ${idx}를 활성화했지만 실제로 활성화된 엔트리가 없습니다. (총 ${timelinePlaybackEntries.length}개)`);
  }
}

function stopTimelinePlayback() {
  if (timelinePlaybackController) {
    timelinePlaybackController.cancelled = true;
    timelinePlaybackController = null;
  }
  setActiveTimelineIndex(-1);
}

function waitFor(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

async function startTimelinePlayback() {
  if (!timelinePlaybackEntries.length) {
    console.warn("[타임라인 애니메이션] 엔트리가 없어 애니메이션을 시작할 수 없습니다.");
    return;
  }
  
  console.log(`[타임라인 애니메이션] 시작 - 총 ${timelinePlaybackEntries.length}개 엔트리`);
  
  // 기존 애니메이션 중지
  if (timelinePlaybackController) {
    timelinePlaybackController.cancelled = true;
  }
  
  // 모든 엔트리 비활성화
  setActiveTimelineIndex(-1);
  
  // 새 컨트롤러 생성
  const controller = { cancelled: false };
  timelinePlaybackController = controller;
  
  try {
    // 첫 번째 엔트리를 즉시 활성화 (사용자가 바로 볼 수 있도록)
    if (timelinePlaybackEntries.length > 0) {
      setActiveTimelineIndex(0);
      console.log(`[타임라인 애니메이션] 1/${timelinePlaybackEntries.length} 즉시 활성화`);
    }
    
    // 나머지 엔트리들을 순차적으로 활성화
    for (let i = 0; i < timelinePlaybackEntries.length; i += 1) {
      if (controller.cancelled) {
        console.log(`[타임라인 애니메이션] 취소됨 (인덱스: ${i})`);
        return;
      }
      
      // 첫 번째는 이미 활성화했으므로 건너뛰기
      if (i > 0) {
        setActiveTimelineIndex(i);
        console.log(`[타임라인 애니메이션] ${i + 1}/${timelinePlaybackEntries.length} 활성화`);
      }
      
      const durationMs = Math.max(800, timelinePlaybackEntries[i].durationMs || 0);
      console.log(`[타임라인 애니메이션] ${i + 1}번 엔트리 대기 중 (${durationMs}ms)`);
      
      // 다음 엔트리로 넘어가기 전 대기
      await waitFor(durationMs);
      
      if (controller.cancelled) {
        console.log(`[타임라인 애니메이션] 취소됨 (인덱스: ${i})`);
        return;
      }
    }
    
    // 모든 엔트리 재생 완료
    timelinePlaybackController = null;
    setActiveTimelineIndex(-1);
    console.log("[타임라인 애니메이션] 완료");
  } catch (error) {
    console.error("[타임라인 애니메이션] 오류 발생:", error);
    console.error("[타임라인 애니메이션] 스택 트레이스:", error.stack);
    timelinePlaybackController = null;
    setActiveTimelineIndex(-1);
  }
}

const timelineStartColor = { r: 0, g: 199, b: 104 };
const timelineEndColor = { r: 4, g: 117, b: 244 };

function mixTimelineColor(ratio) {
  const clamped = Math.max(0, Math.min(1, ratio || 0));
  const r = Math.round(timelineStartColor.r + (timelineEndColor.r - timelineStartColor.r) * clamped);
  const g = Math.round(timelineStartColor.g + (timelineEndColor.g - timelineStartColor.g) * clamped);
  const b = Math.round(timelineStartColor.b + (timelineEndColor.b - timelineStartColor.b) * clamped);
  return `rgb(${r}, ${g}, ${b})`;
}

function renderTimelineFromLines(lineEntries = []) {
  if (!executionCommandsList) {
    console.warn("[타임라인 렌더링] executionCommandsList 요소를 찾을 수 없습니다.");
    return;
  }
  console.log(`[타임라인 렌더링] 시작 - ${lineEntries.length}개 엔트리`);
  executionCommandsList.innerHTML = "";
  executionCommandsList.classList.add("timeline-list");

  const total = lineEntries.length || 1;
  const playbackEntries = [];

  lineEntries.forEach((entry, idx) => {
    const current = typeof entry === "string" ? { text: entry } : entry || {};
    const text = current.text || "";
    const durationMs = current.durationMs || 1200;

    const div = document.createElement("div");
    div.className = "timeline-item";
    div.dataset.durationMs = durationMs.toString(); // duration 저장

    const number = document.createElement("div");
    number.className = "timeline-number";
    number.textContent = `${idx + 1}/${total}`;
    const track = document.createElement("div");
    track.className = "timeline-track";
    const dot = document.createElement("span");
    dot.className = "timeline-dot";
    const color = mixTimelineColor(total > 1 ? idx / (total - 1) : 0);
    dot.style.background = color;
    dot.style.boxShadow = `0 0 0 4px ${color.replace("rgb", "rgba").replace(")", ", 0.15)")}`;
    number.style.color = color;
    track.appendChild(dot);

    const body = document.createElement("div");
    body.className = "timeline-text";
    const textSpan = document.createElement("span");
    textSpan.textContent = text;
    body.appendChild(textSpan);
    body.appendChild(createTimelineArrows());

    div.appendChild(number);
    div.appendChild(track);
    div.appendChild(body);
    executionCommandsList.appendChild(div);

    playbackEntries.push({ element: div, durationMs });
  });

  setTimeout(() => {
    adjustTimelineLine(executionCommandsList);
  }, 0);
  console.log(`[타임라인 렌더링] 완료 - ${playbackEntries.length}개 엔트리 생성`);
  registerTimelineEntries(playbackEntries);
}

function buildCommandTimeline(commands = [], options = {}) {
  const { showScaleDetails = true } = options;
  if (!commands || !commands.length) {
    renderTimelineFromLines([]);
    return;
  }

  const formatDistance = (cmd) => {
    const original = cmd.original_distance ?? cmd.distance;
    if (original < 0.01 && original > 0) {
      return showScaleDetails
        ? `${original.toFixed(3)} m (scale: ${cmd.distance.toFixed(2)} m)`
        : `${original.toFixed(3)} m`;
    }
    return showScaleDetails
      ? `${original.toFixed(2)} m (scale: ${cmd.distance.toFixed(2)} m)`
      : `${original.toFixed(2)} m`;
  };

  const lines = [];

  commands.forEach((cmd, idx) => {
    const cmdType = cmd.type || "";
    const hasDistance = cmd.distance > 0.001 || (cmd.original_distance ?? 0) > 0.001;
    const duration =
      cmd.duration || (cmd.distance && cmd.linear ? cmd.distance / Math.max(cmd.linear, 0.01) : 0);
    const durationMs = Math.max(800, (duration || 0) * 1000);
    const linear = cmd.linear || 0;
    const angular = cmd.angular || 0;
    const angularDeg =
      Math.abs(angular) > 1e-3 ? ((Math.abs(angular) * 180) / Math.PI).toFixed(1) : null;

    if (idx === 0) {
      // 출발 단계는 2.5초 고정
      if (hasDistance) {
        if (cmdType === "smooth_curve" || cmdType === "curve") {
          const angularText =
            cmdType === "smooth_curve" && angularDeg ? `, 각속도: ${angularDeg}°/s` : "";
          lines.push({
            text: `출발: 곡선 ${formatDistance(cmd)} (속도: ${linear.toFixed(2)}m/s${angularText}, 시간: ${(
              duration || 0
            ).toFixed(1)}초)`,
            durationMs: 2500, // 출발은 2.5초 고정
          });
        } else {
          lines.push({
            text: `출발: 직진 ${formatDistance(cmd)} (속도: ${linear.toFixed(2)}m/s, 시간: ${(
              duration || 0
            ).toFixed(1)}초)`,
            durationMs: 2500, // 출발은 2.5초 고정
          });
        }
      } else {
        lines.push({ text: "출발", durationMs: 2500 }); // 출발은 2.5초 고정
      }
      return;
    }

    if (cmdType === "stop") {
      lines.push({ text: `정지 (시간: ${(duration || 0).toFixed(1)}초)`, durationMs });
      return;
    }

    const turn = cmd.turn_angle || 0;
    if (cmdType === "rotate" || Math.abs(turn) >= 1.0) {
      // 회전 단계는 2.5초 고정
      const direction = turn > 0 ? "좌회전" : "우회전";
      if (cmdType === "rotate") {
        lines.push({
          text: `${direction} ${Math.abs(turn).toFixed(1)}° (각속도: ${angularDeg || 0}°/s, 시간: ${(duration || 0).toFixed(1)}초)`,
          durationMs: 2500, // 회전은 2.5초 고정
        });
      } else {
        lines.push({ text: `${direction} ${Math.abs(turn).toFixed(0)}°`, durationMs: 2500 }); // 회전은 2.5초 고정
      }
    }

    if (hasDistance) {
      if (cmdType === "smooth_curve" || cmdType === "curve") {
        const angularText =
          cmdType === "smooth_curve" && angularDeg ? `, 각속도: ${angularDeg}°/s` : "";
        lines.push({
          text: `곡선 ${formatDistance(cmd)} (속도: ${linear.toFixed(2)}m/s${angularText}, 시간: ${(
            duration || 0
          ).toFixed(1)}초)`,
          durationMs,
        });
      } else if (cmdType !== "sharp_curve") {
        // 직선 거리는 대시보드에 나타나는 초수에 +0.5초 추가
        const straightDurationMs = durationMs + 100;
        lines.push({
          text: `직진 ${formatDistance(cmd)} (속도: ${linear.toFixed(2)}m/s, 시간: ${(
            duration || 0
          ).toFixed(1)}초)`,
          durationMs: straightDurationMs,
        });
      }
    }

    if (idx === commands.length - 1) {
      lines.push({ text: "도착", durationMs: 1200 });
    }
  });

  renderTimelineFromLines(lines);
}

const routeForm = document.getElementById("routeForm");
const commandsList = document.getElementById("commandsList"); // STEP 02 요약
const executionCommandsList = document.getElementById("executionCommandsList"); // STEP 04용 (실행용)
const startAddressInput = document.querySelector('input[name="start_address"]');
const goalAddressInput = document.querySelector('input[name="goal_address"]');
const hiddenScaleWidthInput = document.getElementById("scaleWidth");
const hiddenScaleHeightInput = document.getElementById("scaleHeight");
const movementModeInput = document.getElementById("movementMode");
const movementSpeedInput = document.getElementById("movementSpeed");
const connectForm = document.getElementById("connectForm");
const connectBtn = document.getElementById("connectBtn");
const testBtn = document.getElementById("testBtn");
const startDriveBtn = document.getElementById("startDriveBtn");
const downloadBtn = document.getElementById("downloadBtn");
const routePreviewImage = document.getElementById("routePreviewImage");
const routePreviewPlaceholder = document.getElementById("routePreviewPlaceholder");
const step3StatusEl = document.getElementById("step3Status");
// 모드 전환 관련
const carModeBtn = document.getElementById("carModeBtn");
const walkModeBtn = document.getElementById("walkModeBtn");
const routeFormDiv = document.getElementById("routeForm");
const walkingModeDiv = document.getElementById("walkingMode");

// 도보도로 관련
const walkingUrlInput = document.getElementById("walkingUrlInput");
const crawlBtn = document.getElementById("crawlBtn");
const crosswalkSection = document.getElementById("crosswalkSection");
const crosswalkText = document.getElementById("crosswalkText");
const crosswalkAngleInput = document.getElementById("crosswalkAngleInput");
const crosswalkNextBtn = document.getElementById("crosswalkNextBtn");
const crawlProgressFill = document.getElementById("crawlProgressFill");
const crawlProgressText = document.getElementById("crawlProgressText");

// 링크 복사 버튼
const copyLinkBtn = document.getElementById("copyLinkBtn");

// 현재 모드 (기본값: car)
let currentMode = "car";
// 경로 타입은 곡선으로 고정
const currentPathType = "curve";
let currentCrosswalks = [];
let currentCrosswalkIndex = 0;
let crawlingData = null;
let crawlProgressValue = 0;
let testStatusTimeout = null;

function setCrawlProgress(value) {
  crawlProgressValue = Math.max(0, Math.min(100, Math.round(value)));
  if (crawlProgressFill) {
    crawlProgressFill.style.width = `${crawlProgressValue}%`;
  }
  if (crawlProgressText) {
    crawlProgressText.textContent = `${crawlProgressValue}%`;
  }
}

function setStep3ButtonState(button, isActive) {
  if (!button) return;
  button.classList.toggle("is-active", isActive);
  button.disabled = isActive;
}

function setStep3Status(text) {
  if (step3StatusEl) {
    step3StatusEl.textContent = text;
  }
}

function updateRoutePreview(imageUrl) {
  console.log('[이미지 업데이트] updateRoutePreview 호출:', imageUrl);
  if (!routePreviewImage || !routePreviewPlaceholder) {
    console.warn('[이미지 업데이트] routePreviewImage 또는 routePreviewPlaceholder가 없습니다.');
    return;
  }
  if (imageUrl) {
    const fullUrl = `${imageUrl}?v=${Date.now()}`;
    console.log('[이미지 업데이트] 이미지 URL 설정:', fullUrl);
    routePreviewImage.src = fullUrl;
    routePreviewImage.style.display = "block";
    routePreviewPlaceholder.style.display = "none";
    // 이미지 로드 성공/실패 확인
    routePreviewImage.onload = () => {
      console.log('[이미지 업데이트] 이미지 로드 성공:', fullUrl);
    };
    routePreviewImage.onerror = (e) => {
      console.error('[이미지 업데이트] 이미지 로드 실패:', fullUrl, e);
    };
  } else {
    console.log('[이미지 업데이트] imageUrl이 없어서 placeholder 표시');
    routePreviewImage.style.display = "none";
    routePreviewPlaceholder.style.display = "flex";
  }
}

function updateSpeed(speed) {
  if (!speedTag) return;
  let speedValue;
  if (speed != null && speed > 0) {
    speedValue = speed.toFixed(1);
  } else {
    // 디폴트 속도
    const defaultSpeed = movementSpeedInput ? parseFloat(movementSpeedInput.value) || 0.1 : 0.1;
    speedValue = defaultSpeed.toFixed(1);
  }
  // 숫자만 볼드, 단위는 일반 폰트
  speedTag.innerHTML = `<strong>${speedValue}</strong>m/s`;
}

function updateScaleFactor(scaleFactor) {
  if (!scaleTag || !scaleModeTag) return;
  
  // 현재 모드 확인
  const currentMode = movementModeInput ? movementModeInput.value : "scale";
  
  if (currentMode === "scale") {
    // 스케일 비율이 있으면 표시
    if (scaleFactor != null && scaleFactor > 0) {
      // 스케일 팩터를 퍼센트로 변환 (소수점 아래 3자리까지)
      const percentage = (scaleFactor * 100).toFixed(3);
      // 숫자만 볼드, 단위는 일반 폰트
      scaleTag.innerHTML = `<strong>${percentage}</strong>%`;
      scaleTag.style.display = "inline-block";
      scaleModeTag.textContent = "DOWN";
      // 세션에 저장
      sessionStorage.setItem("lastScaleFactor", scaleFactor.toString());
    } else {
      // 스케일 비율이 없으면 "--" 표시 (경로 생성 전)
      // sessionStorage는 경로 생성 후에만 사용하므로 여기서는 무시
      scaleTag.textContent = "--";
      scaleTag.style.display = "inline-block";
      scaleModeTag.textContent = "DOWN";
    }
  } else {
    // REAL 모드일 때 스케일 박스 숨기기
    scaleTag.style.display = "none";
    scaleModeTag.textContent = "REAL";
  }
}

// 페이지 로드 시 초기 속도 및 스케일 비율 표시
// 경로가 생성되지 않았으므로 sessionStorage 초기화
if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", () => {
    // 경로 생성 전이므로 스케일 팩터 초기화
    sessionStorage.removeItem("lastScaleFactor");
    updateSpeed(null);
    updateScaleFactor(null);
  });
} else {
  // 경로 생성 전이므로 스케일 팩터 초기화
  sessionStorage.removeItem("lastScaleFactor");
  updateSpeed(null);
  updateScaleFactor(null);
}

const scaleBtn = document.getElementById("scaleBtn");
let scaleConfigMode = movementModeInput ? movementModeInput.value || "scale" : "scale";

if (scaleBtn) {
  scaleBtn.addEventListener("click", () => {
    const savedMode = movementModeInput ? movementModeInput.value || "scale" : "scale";
    if (hiddenScaleWidthInput && modalWidthInput) {
      modalWidthInput.value = hiddenScaleWidthInput.value || modalWidthInput.value || "2";
    }
    if (hiddenScaleHeightInput && modalHeightInput) {
      modalHeightInput.value = hiddenScaleHeightInput.value || modalHeightInput.value || "3";
    }
    if (movementModeInput && movementModeInput.value === "scale" && movementSpeedInput && modalScaleSpeedInput) {
      modalScaleSpeedInput.value = movementSpeedInput.value || modalScaleSpeedInput.value || "0.1";
    }
    if (movementModeInput && movementModeInput.value === "real" && movementSpeedInput && modalRealSpeedInput) {
      modalRealSpeedInput.value = movementSpeedInput.value || modalRealSpeedInput.value || "0.2";
    }
    setScaleMode(savedMode);
    scaleModal.classList.add("active");
  });
}

document.querySelectorAll(".modal-close").forEach((btn) =>
  btn.addEventListener("click", () => {
    btn.closest(".modal").classList.remove("active");
  })
);

// 스케일 모달 X 버튼
const scaleModalCloseBtn = document.getElementById("scaleModalCloseBtn");
if (scaleModalCloseBtn) {
  scaleModalCloseBtn.addEventListener("click", () => {
    scaleModal.classList.remove("active");
  });
}

// 사용 설명 모달
const helpModal = document.getElementById("helpModal");
const helpBtn = document.getElementById("helpBtn");
const helpModalCloseBtn = document.getElementById("helpModalCloseBtn");

if (helpBtn) {
  helpBtn.addEventListener("click", () => {
    if (helpModal) {
      helpModal.classList.add("active");
    }
  });
}

if (helpModalCloseBtn) {
  helpModalCloseBtn.addEventListener("click", () => {
    if (helpModal) {
      helpModal.classList.remove("active");
    }
  });
}

if (helpModal) {
  helpModal.addEventListener("click", (e) => {
    if (e.target === helpModal) {
      helpModal.classList.remove("active");
    }
  });
}

scaleModeButtons.forEach((btn) =>
  btn.addEventListener("click", () => setScaleMode(btn.dataset.mode))
);

function setScaleMode(mode) {
  scaleConfigMode = mode;
  scaleModeButtons.forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.mode === mode);
  });
  if (scaleInputs) {
    scaleInputs.style.display = mode === "scale" ? "block" : "none";
  }
  if (realInputs) {
    realInputs.style.display = mode === "real" ? "block" : "none";
  }
}

setScaleMode(scaleConfigMode);

if (scaleSaveBtn) {
  scaleSaveBtn.addEventListener("click", async () => {
    const width = parseFloat(modalWidthInput?.value || hiddenScaleWidthInput?.value || "2") || 2;
    const height = parseFloat(modalHeightInput?.value || hiddenScaleHeightInput?.value || "3") || 3;
    const scaleSpeed = parseFloat(modalScaleSpeedInput?.value || "0.1") || 0.1;
    const realSpeed = parseFloat(modalRealSpeedInput?.value || "0.2") || 0.2;

    const payload = {
      mode: scaleConfigMode,
      scale_width: width,
      scale_height: height,
      scale_speed: scaleSpeed,
      real_speed: realSpeed,
    };

    try {
      const resp = await fetch("/api/save-motion-settings", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });
      const data = await resp.json();
      if (data.status !== "ok") {
        throw new Error(data.message || "설정 저장에 실패했습니다.");
      }

      if (scaleConfigMode === "scale") {
        if (hiddenScaleWidthInput) hiddenScaleWidthInput.value = width;
        if (hiddenScaleHeightInput) hiddenScaleHeightInput.value = height;
        if (movementModeInput) movementModeInput.value = "scale";
        if (movementSpeedInput) movementSpeedInput.value = scaleSpeed;
        // 속도 업데이트
        updateSpeed(scaleSpeed);
        // 스케일 팩터 재계산 결과가 있으면 업데이트 (경로가 생성된 경우에만)
        if (data.scale_factor != null && data.scale_factor > 0) {
          updateScaleFactor(data.scale_factor);
        } else {
          updateScaleFactor(null);
        }
        
        // STEP 04 업데이트 (속도 변경 시 duration 재계산된 commands 반영)
        if (data.commands && data.commands.length > 0) {
          buildCommandTimeline(data.commands, { showScaleDetails: true });
        }
        
        if (data.scale_factor == null || data.scale_factor <= 0) {
          // 경로가 생성되지 않았으면 null 전달 (-- 표시)
          updateScaleFactor(null);
        }
        if (modalScaleSpeedInput) modalScaleSpeedInput.value = scaleSpeed;
      } else {
        if (movementModeInput) movementModeInput.value = "real";
        if (movementSpeedInput) movementSpeedInput.value = realSpeed;
        // 속도 업데이트
        updateSpeed(realSpeed);
        // REAL 모드일 때 스케일 팩터 업데이트 (스케일 박스 숨김)
        updateScaleFactor(null);
        if (modalRealSpeedInput) modalRealSpeedInput.value = realSpeed;
        
        // STEP 04 업데이트 (REAL 모드에서도 속도 변경 시 duration 재계산된 commands 반영)
        if (data.commands && data.commands.length > 0) {
          buildCommandTimeline(data.commands, { showScaleDetails: false });
        }
      }

      scaleModal.classList.remove("active");
    } catch (error) {
      alert(error.message || "설정 저장 중 오류가 발생했습니다.");
    }
  });
}

// 네이버 지도 iframe 업데이트
let addressUpdateTimeout = null;

function updateNaverMapIframe() {
  const startAddress = startAddressInput.value.trim();
  const goalAddress = goalAddressInput.value.trim();
  
  if (!startAddress || !goalAddress) {
    // 둘 다 입력되지 않으면 기본 길찾기 페이지 표시
    const iframe = document.getElementById('naverMapFrame');
    if (iframe) {
      iframe.src = 'https://map.naver.com/p/directions';
    }
    return;
  }
  
  // 네이버 지도 길찾기 URL 생성
  const encodedStart = encodeURIComponent(startAddress);
  const encodedGoal = encodeURIComponent(goalAddress);
  
  // 네이버 지도 길찾기 URL 형식: https://map.naver.com/v5/directions/출발지,도착지
  const mapUrl = `https://map.naver.com/v5/directions/${encodedStart},${encodedGoal}?c=15.0,0,0,0,dh`;
  
  const iframe = document.getElementById('naverMapFrame');
  if (iframe) {
    iframe.src = mapUrl;
    console.log('[지도] 네이버 지도 iframe 업데이트:', mapUrl);
  }
}

// iframe URL에서 출발지/도착지 추출 (제한적 - URL 파라미터 기반)
function extractAddressesFromIframeUrl() {
  const iframe = document.getElementById('naverMapFrame');
  if (!iframe) return null;
  
  try {
    // iframe의 현재 URL 가져오기 (같은 도메인이어야 함, 네이버는 불가능)
    // 대신 iframe의 src 속성에서 추출
    const iframeSrc = iframe.src;
    
    // 네이버 지도 URL 패턴: /directions/출발지,도착지
    const directionsMatch = iframeSrc.match(/\/directions\/([^,]+),([^?]+)/);
    if (directionsMatch) {
      const start = decodeURIComponent(directionsMatch[1]);
      const goal = decodeURIComponent(directionsMatch[2]);
      return { start, goal };
    }
  } catch (error) {
    console.error('[지도] iframe URL 파싱 오류:', error);
  }
  
  return null;
}

// 입력 필드 변경 시 iframe 자동 업데이트 비활성화
// 사용자가 iframe 내부에서 입력한 값을 대시보드에 복사할 때 iframe이 새로고침되지 않도록 함
// 주석 처리: 자동 업데이트 기능 제거
// startAddressInput.addEventListener('input', () => {
//   clearTimeout(addressUpdateTimeout);
//   updateNaverMapIframe();
// });

// goalAddressInput.addEventListener('input', () => {
//   clearTimeout(addressUpdateTimeout);
//   updateNaverMapIframe();
// });

// Enter 키 입력 시에도 iframe 업데이트 안 함
// startAddressInput.addEventListener('keypress', (e) => {
//   if (e.key === 'Enter') {
//     e.preventDefault();
//     clearTimeout(addressUpdateTimeout);
//     updateNaverMapIframe();
//   }
// });

// goalAddressInput.addEventListener('keypress', (e) => {
//   if (e.key === 'Enter') {
//     e.preventDefault();
//     clearTimeout(addressUpdateTimeout);
//     updateNaverMapIframe();
//   }
// });

// 페이지 로드 시 iframe 초기화도 제거 (빈 값이므로)
// window.addEventListener('load', () => {
//   if (startAddressInput && goalAddressInput) {
//     updateNaverMapIframe();
//     console.log('[지도] 페이지 로드 시 iframe 초기화 완료');
//   }
// });

function renderCarRouteSummary(routeData) {
  if (!commandsList || !routeData) return;

  const keypointCount = routeData.path_coordinates ? routeData.path_coordinates.length : routeData.commands.length;
  const segmentCount = routeData.commands.length;
  const totalDistance = routeData.commands.reduce(
    (sum, cmd) => sum + (cmd.original_distance ?? cmd.distance ?? 0),
    0
  );
  const turnCount = routeData.commands.filter((cmd) => Math.abs(cmd.turn_angle || 0) >= 1).length;
  
  // 곡선 로직 통계 (터미널 출력 기준)
  const isCurveMode = currentPathType === "curve";
  let curveStats = "";
  
  if (isCurveMode) {
    // 직선: type === "straight"
    const straightCount = routeData.commands.filter((cmd) => cmd.type === "straight").length;
    
    // 곡선: type이 "curve", "smooth_curve", "sharp_curve" 등 curve가 포함된 모든 타입
    const curveCount = routeData.commands.filter((cmd) => 
      cmd.type && (cmd.type.includes("curve") || cmd.type === "curve")
    ).length;
    
    // 스무스 커브: type === "smooth_curve"
    const smoothCurveCount = routeData.commands.filter((cmd) => cmd.type === "smooth_curve").length;
    
    curveStats = `
      <div class="summary-card">
        <div class="summary-label">곡선 개수</div>
        <strong><span style="color: #0475f4;">${curveCount}</span>개</strong>
        <span class="summary-hint">직선 <span style="color: #0475f4;">${straightCount}</span>개</span>
      </div>
      <div class="summary-card">
        <div class="summary-label">회전 타입</div>
        <strong><span style="color: #0475f4;">${smoothCurveCount}</span> 스무스</strong>
      </div>
    `;
  }

  commandsList.innerHTML = `
    <div class="summary-grid">
      <div class="summary-card">
        <div class="summary-label">Keypoint</div>
        <strong><span style="color: #0475f4;">${keypointCount}</span>개</strong>
        <span class="summary-hint">세그먼트 <span style="color: #0475f4;">${segmentCount}</span>개</span>
      </div>
      <div class="summary-card">
        <div class="summary-label">총 직진 거리</div>
        <strong><span style="color: #0475f4;">${(totalDistance / 1000).toFixed(2)}</span> km</strong>
        <span class="summary-hint">축소 전 기준</span>
      </div>
      <div class="summary-card">
        <div class="summary-label">회전 지점</div>
        <strong><span style="color: #0475f4;">${turnCount}</span>회</strong>
        <span class="summary-hint">좌/우회전 전체</span>
      </div>
      ${curveStats}
    </div>
  `;
  updateRoutePreview(routeData.route_image || null);
}

function renderWalkRouteSummary(actions = [], crosswalks = [], routeTexts = [], routeImageUrl = null) {
  if (!commandsList) return;

  const totalDistance = actions.reduce(
    (sum, action) => sum + (action.distance || 0),
    0
  );
  const crosswalkCount = crosswalks.length;
  const angleMap = { 90: "좌회전 90°", "-90": "우회전 -90°", 0: "직진 0°" };
  const angleSummary = crosswalks.length
    ? crosswalks
        .map((cw, idx) => `#${idx + 1} ${angleMap[cw.direction] || "미입력"}`)
        .join(" · ")
    : "횡단보도 없음";


  commandsList.innerHTML = `
    <div class="summary-grid" style="grid-template-columns: 1fr !important;">
      <div class="summary-card">
        <div class="summary-label">총 단계</div>
        <strong>${actions.length}단계</strong>
        <span class="summary-hint">지도 텍스트 기반</span>
      </div>
      <div class="summary-card">
        <div class="summary-label">총 직진 거리</div>
        <strong>${totalDistance} m</strong>
        <span class="summary-hint">텍스트 추출값 합산</span>
      </div>
      <div class="summary-card">
        <div class="summary-label">횡단보도</div>
        <strong>${crosswalkCount}회</strong>
        <span class="summary-hint">사용자 입력 각도</span>
      </div>
    </div>
    <div class="summary-list">
      <div class="summary-title">횡단보도 각도 요약</div>
      <p class="summary-text">${angleSummary}</p>
    </div>
  `;
  // 도보도로 경로 이미지 표시
  console.log('[도보도로] renderWalkRouteSummary 완료, routeImageUrl:', routeImageUrl);
  updateRoutePreview(routeImageUrl);
}

routeForm.addEventListener("submit", async (e) => {
  e.preventDefault();
  const formData = new FormData(routeForm);
  
  // 대시보드 입력 필드에서 값 가져오기 (기본)
  let startAddress = formData.get("start_address");
  let goalAddress = formData.get("goal_address");
  
  // 추가: iframe URL에서도 시도 (제한적 - URL이 변경되었을 때만 가능)
  // 주의: iframe 내부에서 사용자가 직접 변경한 값은 가져올 수 없음
  const iframeAddresses = extractAddressesFromIframeUrl();
  if (iframeAddresses && (!startAddress || !goalAddress)) {
    // 입력 필드가 비어있을 때만 iframe URL에서 가져오기
    startAddress = startAddress || iframeAddresses.start;
    goalAddress = goalAddress || iframeAddresses.goal;
    console.log('[경로] iframe URL에서 주소 추출:', iframeAddresses);
  }
  
  const payload = {
    start_address: startAddress,
    goal_address: goalAddress,
    width: hiddenScaleWidthInput ? hiddenScaleWidthInput.value : 2,
    height: hiddenScaleHeightInput ? hiddenScaleHeightInput.value : 3,
    angle_threshold: formData.get("angle_threshold") || 20,
    path_type: currentPathType  // 직선 또는 곡선
  };

  const resp = await fetch("/api/plan-route", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const data = await resp.json();

  if (data.status !== "ok") {
    alert(data.message || "경로 생성 중 오류가 발생했습니다.");
    return;
  }

  // iframe은 이미 네이버 지도에서 경로를 표시하므로 별도 처리 불필요
  console.log('[경로] 경로 생성 완료:', data.commands.length, '개 세그먼트');

  // STEP 02: 간단한 정보만 표시
  renderCarRouteSummary(data);
  
  // 스케일 비율 업데이트
  updateScaleFactor(data.scale_factor);
  
  buildCommandTimeline(data.commands, { showScaleDetails: true });
});

// move_path.py 다운로드 버튼
if (downloadBtn) {
  downloadBtn.addEventListener("click", async () => {
    try {
      const response = await fetch("/api/download-move-path");
      
      if (!response.ok) {
        const data = await response.json();
        alert(data.message || "다운로드 실패");
        return;
      }
      
      // 파일 다운로드
      const blob = await response.blob();
      const url = window.URL.createObjectURL(blob);
      const a = document.createElement("a");
      a.href = url;
      a.download = "move_path.py";
      document.body.appendChild(a);
      a.click();
      window.URL.revokeObjectURL(url);
      document.body.removeChild(a);
    } catch (error) {
      alert("다운로드 중 오류가 발생했습니다: " + error.message);
    }
  });
}

// 타임라인 라인 조정 함수
function adjustTimelineLine(timelineList) {
  const items = timelineList.querySelectorAll('.timeline-item');
  if (items.length === 0) return;
  
  const firstItem = items[0];
  const lastItem = items[items.length - 1];
  
  const firstDot = firstItem.querySelector('.timeline-dot');
  const lastDot = lastItem.querySelector('.timeline-dot');
  
  if (!firstDot || !lastDot) return;
  
  // timeline-list의 위치 기준
  const timelineListRect = timelineList.getBoundingClientRect();
  
  // 첫 번째 dot의 세로축 중앙 위치 (timeline-list 기준)
  const firstDotRect = firstDot.getBoundingClientRect();
  const firstDotCenterY = firstDotRect.top + firstDotRect.height / 2 - timelineListRect.top;
  
  // 마지막 dot의 세로축 중앙 위치 (timeline-list 기준)
  const lastDotRect = lastDot.getBoundingClientRect();
  const lastDotCenterY = lastDotRect.top + lastDotRect.height / 2 - timelineListRect.top;
  
  // timeline-list의 전체 높이
  const timelineListHeight = timelineListRect.height;
  
  // top: 첫 번째 dot의 중앙까지의 거리
  // bottom: 마지막 dot의 중앙부터 끝까지의 거리
  const topOffset = firstDotCenterY;
  const bottomOffset = timelineListHeight - lastDotCenterY;
  
  timelineList.style.setProperty('--timeline-line-top', `${topOffset}px`);
  timelineList.style.setProperty('--timeline-line-bottom', `${bottomOffset}px`);
}

// 브링업 상태 확인 및 버튼 텍스트 업데이트
async function checkBringupStatus() {
  const robot_ip = document.getElementById("robot_ip").value;
  const username = document.getElementById("robot_id").value;
  const password = document.getElementById("robot_pw").value;
  
  if (!robot_ip || !username || !password) {
    // 입력 필드가 비어있으면 기본 상태로 설정
    if (connectBtn) {
      connectBtn.querySelector("span").textContent = "로봇 연동";
      connectBtn.disabled = false;
    }
    return;
  }
  
  try {
    const resp = await fetch("/api/check-status", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ robot_ip, username, password })
    });
    const data = await resp.json();
    
    if (data.status === "ok" && data.is_running) {
      // 브링업 실행 중
      if (connectBtn) {
        connectBtn.querySelector("span").textContent = "로봇 연동됨";
        connectBtn.disabled = true;
      }
    } else {
      // 브링업 미실행
      if (connectBtn) {
        connectBtn.querySelector("span").textContent = "로봇 연동";
        connectBtn.disabled = false;
      }
    }
  } catch (error) {
    console.error("상태 확인 중 오류:", error);
  }
}

// 페이지 로드 시 상태 확인
document.addEventListener("DOMContentLoaded", () => {
  setTimeout(() => {
    checkBringupStatus();
  }, 500);
});

// 입력 필드 변경 시 상태 확인 (debounce)
let checkTimeout;
["robot_ip", "robot_id", "robot_pw"].forEach(id => {
  const input = document.getElementById(id);
  if (input) {
    input.addEventListener("input", () => {
      clearTimeout(checkTimeout);
      checkTimeout = setTimeout(checkBringupStatus, 500);
    });
  }
});

connectForm.addEventListener("submit", async (e) => {
  e.preventDefault();
  const formData = new FormData(connectForm);
  const payload = Object.fromEntries(formData.entries());

  try {
    setStep3Status("연동중...");
    setStep3ButtonState(connectBtn, true);
    const resp = await fetch("/api/connect", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await resp.json();

    if (data.status === "ok") {
      setStep3Status("로봇이 연동되었습니다.");
      // 연결 성공 시 버튼 텍스트 변경
      if (connectBtn) {
        connectBtn.querySelector("span").textContent = "로봇 연동됨";
        connectBtn.disabled = true;
      }
      alert("로봇 브링업이 시작되었습니다.");
    } else {
      setStep3Status(data.message || "연동에 실패했습니다.");
      alert(data.message || "연동 중 오류가 발생했습니다.");
    }
  } catch (error) {
    setStep3Status("연동 중 오류가 발생했습니다.");
    alert("연동 중 오류가 발생했습니다: " + error.message);
  } finally {
    setStep3ButtonState(connectBtn, false);
  }
});

testBtn.addEventListener("click", async () => {
  const payload = getRobotCredentials();
  if (!payload) {
    alert("STEP 03 정보가 필요합니다.");
    return;
  }
  try {
    if (testStatusTimeout) {
      clearTimeout(testStatusTimeout);
      testStatusTimeout = null;
    }
    setStep3Status("테스트 시작");
    setStep3ButtonState(testBtn, true);
    const resp = await fetch("/api/test-move", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await resp.json();

    testModalText.textContent =
      data.status === "ok"
        ? "전진 30cm 토픽이 발행되었습니다. 움직이는지 확인하세요!"
        : data.message || "테스트 명령 전송 실패";
    testModal.classList.add("active");

    if (data.status === "ok") {
      setStep3Status("30cm 전진하는 토픽을 발행하고 있습니다...");
      testStatusTimeout = setTimeout(() => {
        setStep3Status("로봇이 움직이는지 확인하세요.");
      }, 2500);
    } else {
      setStep3Status(data.message || "테스트 명령 전송 실패");
    }
  } catch (error) {
    setStep3Status("테스트 중 오류가 발생했습니다.");
    alert("테스트 명령 중 오류: " + error.message);
  } finally {
    setStep3ButtonState(testBtn, false);
  }
});

if (startDriveBtn) {
  startDriveBtn.addEventListener("click", async () => {
    const payload = getRobotCredentials();
    if (!payload) {
      alert("STEP 03 정보가 필요합니다.");
      return;
    }

    try {
      setStep3Status("주행이 시작되었습니다.");
      setStep3ButtonState(startDriveBtn, true);
      
      // 타임라인 애니메이션 준비 (타임라인이 이미 렌더링되어 있어야 함)
      console.log(`[주행 시작] 타임라인 상태 확인 - 등록된 엔트리: ${timelinePlaybackEntries.length}개`);
      
      // 타임라인이 등록되지 않았다면 현재 렌더링된 타임라인에서 다시 등록
      if (timelinePlaybackEntries.length === 0 && executionCommandsList) {
        console.warn("[주행 시작] 타임라인 엔트리가 비어있어 DOM에서 재등록 시도");
        const timelineItems = executionCommandsList.querySelectorAll(".timeline-item");
        console.log(`[주행 시작] DOM에서 발견된 타임라인 아이템: ${timelineItems.length}개`);
        
        if (timelineItems.length > 0) {
          const entries = Array.from(timelineItems).map((item, idx) => {
            const durationMs = parseInt(item.dataset.durationMs) || 1200;
            return { element: item, durationMs };
          });
          registerTimelineEntries(entries);
          console.log(`[주행 시작] 타임라인 엔트리 ${entries.length}개 재등록 완료`);
        } else {
          console.error("[주행 시작] DOM에도 타임라인 아이템이 없습니다. 경로를 먼저 생성해주세요.");
        }
      } else if (timelinePlaybackEntries.length > 0) {
        console.log(`[주행 시작] 타임라인 엔트리가 이미 등록되어 있음 (${timelinePlaybackEntries.length}개)`);
      }
      
      // 기존 타이머가 있으면 취소 (중복 실행 방지)
      if (animationStartTimeout !== null) {
        console.log("[주행 시작] 기존 애니메이션 타이머 취소");
        clearTimeout(animationStartTimeout);
        animationStartTimeout = null;
      }
      
      // 40초 후 애니메이션 시작 (로봇이 실제로 움직이기까지 약 40초 소요)
      if (timelinePlaybackEntries.length === 0) {
        console.error("[주행 시작] 타임라인 엔트리가 여전히 비어있습니다. 애니메이션을 시작할 수 없습니다.");
      } else {
        const startTime = Date.now();
        console.log(`[주행 시작] 40초 후 애니메이션 시작 예정 - ${timelinePlaybackEntries.length}개 엔트리 (시작 시간: ${new Date(startTime).toLocaleTimeString()})`);
        animationStartTimeout = setTimeout(() => {
          const elapsed = Date.now() - startTime;
          console.log(`[주행 시작] 애니메이션 시작 (경과 시간: ${elapsed}ms, 예상: 40000ms)`);
          animationStartTimeout = null; // 타이머 ID 초기화
          startTimelinePlayback();
        }, 18000);
      }
      
      // 서버 요청은 백그라운드로 전송 (응답 대기하지 않음, 에러가 발생해도 애니메이션은 계속 진행)
      fetch("/api/start-drive", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      })
        .then(async (resp) => {
          if (!resp.ok) {
            console.warn(`[주행 시작] 서버 응답 오류: HTTP ${resp.status}`);
            return; // 에러가 있어도 애니메이션은 계속 진행
          }
          const data = await resp.json();
          if (data.status !== "ok") {
            console.warn(`[주행 시작] 서버 응답: ${data.message || "주행 시작 명령 실패"}`);
            // 에러가 있어도 애니메이션은 계속 진행 (로봇이 이미 주행 중일 수 있음)
          } else {
            console.log("[주행 시작] 서버 응답 성공");
          }
        })
        .catch((error) => {
          // 네트워크 에러나 타임아웃이 발생해도 애니메이션은 계속 진행
          console.warn(`[주행 시작] 서버 요청 중 오류 (무시됨): ${error.message}`);
          // 에러 메시지나 alert를 표시하지 않음 (로봇이 이미 주행 중일 수 있음)
        })
        .finally(() => {
          setStep3ButtonState(startDriveBtn, false);
        });
    } catch (error) {
      // 초기화 단계에서만 에러가 발생한 경우 (타임라인 준비 등)
      console.error("[주행 시작] 초기화 중 오류:", error);
      // 에러가 발생해도 이미 시작된 애니메이션은 계속 진행
      // alert를 표시하지 않음 (로봇이 이미 주행 중일 수 있음)
      setStep3ButtonState(startDriveBtn, false);
    }
  });
}


function getRobotCredentials() {
  const robot_ip = document.getElementById("robot_ip").value;
  const username = document.getElementById("robot_id").value;
  const password = document.getElementById("robot_pw").value;
  const domainInput = document.getElementById("domain_id");
  const domain_id = domainInput ? domainInput.value : "";
  if (!robot_ip || !username || !password) {
    return null;
  }
  return { robot_ip, username, password, domain_id };
}

// 모드 전환
carModeBtn.addEventListener("click", () => {
  currentMode = "car";
  carModeBtn.classList.add("active");
  walkModeBtn.classList.remove("active");
  routeFormDiv.classList.add("active");
  walkingModeDiv.classList.remove("active");
});

walkModeBtn.addEventListener("click", () => {
  currentMode = "walk";
  walkModeBtn.classList.add("active");
  carModeBtn.classList.remove("active");
  routeFormDiv.classList.remove("active");
  walkingModeDiv.classList.add("active");
});

// 크롤링 버튼
crawlBtn.addEventListener("click", async () => {
  const url = walkingUrlInput.value.trim();
  if (!url) {
    alert("네이버 지도 링크를 입력해주세요.");
    return;
  }

  setCrawlProgress(5);
  crawlBtn.disabled = true;
  crawlBtn.textContent = "경로 생성 중...";
  setCrawlProgress(20);

  try {
    const resp = await fetch("/api/crawl-walking-route", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ url }),
    });

    setCrawlProgress(45);
    const data = await resp.json();

    if (data.status !== "ok") {
      alert(data.message || "크롤링 실패");
      setCrawlProgress(0);
      return;
    }

    crawlingData = data;
    currentCrosswalks = data.crosswalks || [];
    currentCrosswalkIndex = 0;
    setCrawlProgress(70);

    // 횡단보도가 있으면 첫 번째 횡단보도 입력 시작
    if (currentCrosswalks.length > 0) {
      showCrosswalkInput(0);
    } else {
      // 횡단보도가 없으면 바로 완료
      setCrawlProgress(90);
      await finishWalkingRoute();
    }
  } catch (error) {
    alert("크롤링 중 오류: " + error.message);
    setCrawlProgress(0);
  } finally {
    crawlBtn.disabled = false;
    crawlBtn.textContent = "경로 생성";
  }
});

function showCrosswalkInput(index) {
  if (index >= currentCrosswalks.length) {
    finishWalkingRoute();
    return;
  }

  if (currentCrosswalks.length > 0) {
    const progressBase = 75;
    const step = (index / currentCrosswalks.length) * 15;
    setCrawlProgress(progressBase + step);
  }

  const crosswalk = currentCrosswalks[index];
  crosswalkText.textContent = crosswalk.text;
  crosswalkAngleInput.value = "";
  crosswalkSection.style.display = "block";
  crosswalkNextBtn.textContent = index === currentCrosswalks.length - 1 ? "완료" : "다음";
  currentCrosswalkIndex = index;
}

crosswalkNextBtn.addEventListener("click", () => {
  const angle = parseInt(crosswalkAngleInput.value);
  if (isNaN(angle) || ![0, 90, -90].includes(angle)) {
    alert("0, 90, 또는 -90을 입력해주세요.");
    return;
  }

  // 횡단보도 방향 저장
  currentCrosswalks[currentCrosswalkIndex].direction = angle;

  // 다음 횡단보도로 이동
  showCrosswalkInput(currentCrosswalkIndex + 1);
});

async function finishWalkingRoute() {
  crosswalkSection.style.display = "none";

  // 횡단보도 방향을 포함하여 서버에 전송
  try {
    const url = walkingUrlInput ? walkingUrlInput.value.trim() : "";
    const resp = await fetch("/api/process-walking-route", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        route_texts: crawlingData.route_texts,
        crosswalks: currentCrosswalks,
        url: url,
      }),
    });

    const data = await resp.json();

    if (data.status !== "ok") {
      alert(data.message || "경로 처리 실패");
      setCrawlProgress(0);
      return;
    }

    // STEP 02/04 업데이트
    console.log('[도보도로] 경로 처리 완료:', {
      actions: data.actions?.length,
      route_image: data.route_image,
      scale_factor: data.scale_factor
    });
    renderWalkRouteSummary(data.actions, currentCrosswalks, crawlingData?.route_texts, data.route_image);
    
    // 스케일 비율 업데이트 (도보도로는 대략적인 추정값)
    // 서버에서 계산된 스케일 비율이 있다면 사용, 없으면 1.0
    const walkScaleFactor = data.scale_factor || 1.0;
    console.log('[도보도로] 스케일 비율:', walkScaleFactor);
    updateScaleFactor(walkScaleFactor);
    displayWalkingCommands(data.actions);
    setCrawlProgress(100);
  } catch (error) {
    alert("경로 처리 중 오류: " + error.message);
    setCrawlProgress(0);
  }
}

function displayWalkingCommands(actions) {
  if (!actions || !actions.length) {
    renderTimelineFromLines([]);
    return;
  }

  const lineEntries = actions.map((action, idx) => {
    let sanitizedText = action.text || "";
    sanitizedText = sanitizedText
      .replace(/<span[^>]*>/gi, "")
      .replace(/<\/span>/gi, "")
      .replace(/도착지는 도로의 왼쪽에 있습니다\./g, "")
      .trim();
    if (!sanitizedText) {
      sanitizedText = `단계 ${idx + 1}`;
    }
    const distance = parseFloat(action.distance || action.original_distance || 0) || 0;
    const estimatedDuration = distance > 0 ? distance / 0.1 : 2; // 0.1m/s 기준
    return {
      text: sanitizedText,
      durationMs: Math.max(800, estimatedDuration * 500),
    };
  });

  renderTimelineFromLines(lineEntries);
}

// 새 브라우저에서 열기 버튼
copyLinkBtn.addEventListener("click", () => {
  const iframe = document.getElementById("naverMapFrame");
  if (!iframe) return;

  const currentUrl = iframe.src || "https://map.naver.com/p/directions";
  console.log('[링크 복사] 사용할 URL:', currentUrl);
  
  // 새 탭에서 열기
  const newWindow = window.open(currentUrl, '_blank');
  
  if (newWindow) {
    // 새 탭이 열리면 URL 복사 시도
    navigator.clipboard.writeText(currentUrl).then(() => {
      const originalText = copyLinkBtn.textContent;
      copyLinkBtn.textContent = "열림!";
      setTimeout(() => {
        copyLinkBtn.textContent = originalText;
      }, 2000);
    }).catch(() => {
      // 클립보드 API 실패 시 fallback
      const textarea = document.createElement("textarea");
      textarea.value = currentUrl;
      textarea.style.position = "fixed";
      textarea.style.opacity = "0";
      document.body.appendChild(textarea);
      textarea.select();
      
      try {
        document.execCommand("copy");
        const originalText = copyLinkBtn.textContent;
        copyLinkBtn.textContent = "열림!";
        setTimeout(() => {
          copyLinkBtn.textContent = originalText;
        }, 2000);
      } catch (err) {
        // 복사 실패해도 새 탭은 열렸으므로 경고만 표시
        console.warn("링크 복사 실패:", err);
      }
      
      document.body.removeChild(textarea);
    });
  } else {
    // 팝업 차단된 경우
    alert("팝업이 차단되었습니다. 브라우저 설정에서 팝업을 허용해주세요.\n\n또는 아래 URL을 수동으로 복사하세요:\n" + currentUrl);
  }
});

// 경로 타입은 곡선으로 고정 (직선/곡선 선택 UI 제거됨)

// 카메라 토글 기능
const cameraToggle = document.getElementById("cameraToggle");
const cameraFeed = document.getElementById("cameraFeed");
const cameraToggleLabel = document.getElementById("cameraToggleLabel");

if (cameraToggle && cameraFeed && cameraToggleLabel) {
  // 초기 상태: OFF
  cameraToggle.checked = false;
  cameraFeed.style.display = "none";
  cameraToggleLabel.textContent = "OFF";
  cameraToggleLabel.classList.add("off");
  
  cameraToggle.addEventListener("change", (e) => {
    if (e.target.checked) {
      // ON: 동영상 재생
      cameraToggleLabel.textContent = "ON";
      cameraToggleLabel.classList.remove("off");
      cameraFeed.style.display = "block";
      cameraFeed.classList.add("playing");
      cameraFeed.currentTime = 0; // 처음부터 재생
      cameraFeed.play().catch(err => {
        console.error("동영상 재생 실패:", err);
      });
    } else {
      // OFF: 동영상 정지 및 숨김
      cameraToggleLabel.textContent = "OFF";
      cameraToggleLabel.classList.add("off");
      cameraFeed.pause();
      cameraFeed.currentTime = 0;
      cameraFeed.style.display = "none";
      cameraFeed.classList.remove("playing");
    }
  });
  
  // 동영상이 끝나면 다시 처음부터 재생 (무한반복)
  cameraFeed.addEventListener("ended", () => {
    if (cameraToggle.checked) {
      cameraFeed.currentTime = 0;
      cameraFeed.play().catch(err => {
        console.error("동영상 재생 실패:", err);
      });
    }
  });
}

