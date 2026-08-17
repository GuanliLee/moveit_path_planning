const CAMERA_IDS = ["cam_high", "cam_left", "cam_right"];
const CAMERA_STREAM_FPS = 12;
const CAMERA_STATUS_POLL_MS = 2000;
const CAMERA_REQUEST_TIMEOUT_MS = 3000;
const CAMERA_RETRY_MS = 1200;

const cameraGrid = document.getElementById("cameraGrid");
const cameraBridgeStatus = document.getElementById("cameraBridgeStatus");
const refreshCamerasBtn = document.getElementById("refreshCamerasBtn");
const cameraBridgeUrl = resolveCameraBridgeUrl();
const cameraCards = new Map();

let cameraSnapshot = new Map();
let cameraPollTimer = null;
let cameraRequestController = null;
let cameraBridgeWasUnavailable = false;
let cameraPageActive = true;

if (cameraGrid && cameraBridgeStatus && refreshCamerasBtn) {
  initCameraMonitor();
}

function initCameraMonitor() {
  cameraGrid.querySelectorAll("[data-camera-id]").forEach((card) => {
    const cameraId = card.dataset.cameraId;
    const image = card.querySelector("[data-camera-image]");
    const refs = {
      cameraId,
      card,
      image,
      source: card.querySelector("[data-camera-source]"),
      status: card.querySelector("[data-camera-state]"),
      placeholder: card.querySelector("[data-camera-placeholder]"),
      fps: card.querySelector("[data-camera-fps]"),
      age: card.querySelector("[data-camera-age]"),
      seq: card.querySelector("[data-camera-seq]"),
      retryTimer: null,
    };

    image.addEventListener("load", () => handleCameraStreamLoad(refs));
    image.addEventListener("error", () => handleCameraStreamError(refs));
    cameraCards.set(cameraId, refs);
  });

  refreshCamerasBtn.addEventListener("click", () => {
    reconnectCameraStreams();
    void refreshCameraStatus({ reconnect: true });
  });

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      pauseCameraMonitor("页面已隐藏，画面暂停");
    } else {
      startCameraMonitor(true);
    }
  });

  document.addEventListener("drink-page-change", (event) => {
    cameraPageActive = event.detail?.page !== "playback";
    if (cameraPageActive && !document.hidden) {
      startCameraMonitor(true);
    } else {
      pauseCameraMonitor("实时画面已暂停");
    }
  });

  window.addEventListener("offline", () => {
    pauseCameraMonitor("浏览器网络离线");
    setBridgeStatus("浏览器网络离线 · " + cameraBridgeUrl, "offline");
  });

  window.addEventListener("online", () => startCameraMonitor(true));
  window.addEventListener("beforeunload", () => pauseCameraMonitor("页面正在关闭"));

  if (navigator.onLine === false) {
    setBridgeStatus("浏览器网络离线 · " + cameraBridgeUrl, "offline");
    return;
  }
  startCameraMonitor(true);
}

function startCameraMonitor(reconnect = false) {
  if (!cameraPageActive || document.hidden) {
    return;
  }
  if (cameraPollTimer === null) {
    cameraPollTimer = window.setInterval(() => {
      void refreshCameraStatus();
    }, CAMERA_STATUS_POLL_MS);
  }
  void refreshCameraStatus({ reconnect });
}

function pauseCameraMonitor(message) {
  if (cameraPollTimer !== null) {
    window.clearInterval(cameraPollTimer);
    cameraPollTimer = null;
  }
  if (cameraRequestController) {
    cameraRequestController.abort();
    cameraRequestController = null;
  }
  cameraCards.forEach((refs) => stopCameraStream(refs, message));
  setBridgeStatus(message + " · " + cameraBridgeUrl, "waiting");
}

async function refreshCameraStatus(options = {}) {
  if (!cameraPageActive || document.hidden || cameraRequestController) {
    return;
  }

  const reconnect = Boolean(options.reconnect);
  const controller = new AbortController();
  cameraRequestController = controller;
  refreshCamerasBtn.disabled = true;
  const timeout = window.setTimeout(() => controller.abort(), CAMERA_REQUEST_TIMEOUT_MS);

  try {
    const response = await fetch(cameraBridgeUrl + "/api/cameras", {
      cache: "no-store",
      signal: controller.signal,
    });
    if (!response.ok) {
      throw new Error(response.status + " " + response.statusText);
    }
    const payload = await response.json();
    const cameras = Array.isArray(payload.cameras) ? payload.cameras : [];
    const reconnectStreams = reconnect || cameraBridgeWasUnavailable;
    cameraBridgeWasUnavailable = false;
    cameraSnapshot = new Map(cameras.map((camera) => [camera.id, camera]));

    let onlineCount = 0;
    CAMERA_IDS.forEach((cameraId) => {
      const camera = cameraSnapshot.get(cameraId);
      if (cameraAvailable(camera)) {
        onlineCount += 1;
      }
      updateCameraCard(cameraId, camera, reconnectStreams);
    });

    const bridgeClass = onlineCount === CAMERA_IDS.length ? "online" : "waiting";
    setBridgeStatus(
      onlineCount + "/" + CAMERA_IDS.length + " 在线 · " + cameraBridgeUrl
        + " · 预览 " + CAMERA_STREAM_FPS + " FPS",
      bridgeClass,
    );
  } catch (error) {
    if (error.name !== "AbortError" || !document.hidden) {
      cameraBridgeWasUnavailable = true;
      const detail = error.name === "AbortError" ? "连接超时" : error.message;
      setBridgeStatus("图像桥不可用 · " + cameraBridgeUrl, "offline", detail);
      markBridgeUnavailable();
    }
  } finally {
    window.clearTimeout(timeout);
    if (cameraRequestController === controller) {
      cameraRequestController = null;
    }
    refreshCamerasBtn.disabled = false;
  }
}

function updateCameraCard(cameraId, camera, reconnect) {
  const refs = cameraCards.get(cameraId);
  if (!refs) {
    return;
  }

  const enabled = camera?.enabled !== false;
  const subscribed = Boolean(camera?.subscribed);
  const online = Boolean(camera?.online);
  const available = Boolean(camera) && enabled && subscribed && online;

  refs.source.textContent = camera?.name
    ? cameraId + " · " + camera.name
    : cameraId;
  refs.fps.textContent = "源 FPS: " + formatNumber(camera?.fps, 1);
  refs.age.textContent = "帧龄: " + formatInteger(camera?.last_frame_age_ms, " ms");
  refs.seq.textContent = "Seq: " + formatInteger(camera?.seq);

  refs.card.classList.toggle("is-online", available);
  refs.card.classList.toggle("is-waiting", Boolean(camera) && !available);
  refs.card.classList.toggle("is-offline", !camera || !enabled);

  if (!camera) {
    setCameraState(refs, "未配置", "offline");
    stopCameraStream(refs, "图像桥未返回该相机");
    return;
  }
  if (!enabled) {
    setCameraState(refs, "已关闭", "offline");
    stopCameraStream(refs, "相机已关闭");
    return;
  }
  if (!subscribed) {
    setCameraState(refs, "未订阅", "waiting");
    stopCameraStream(refs, "等待 ROS 图像订阅");
    return;
  }
  if (!online) {
    setCameraState(refs, "等待画面", "waiting");
    stopCameraStream(refs, camera.last_error || "等待相机新帧");
    return;
  }

  setCameraState(refs, "在线", "online");
  if (!document.hidden) {
    ensureCameraStream(refs, camera, reconnect);
  }
}

function ensureCameraStream(refs, camera, force = false) {
  const streamPath = camera.stream_mjpeg || "/stream/" + refs.cameraId + ".mjpeg";
  const streamKey = new URL(streamPath, cameraBridgeUrl + "/").toString();
  if (
    !force
    && refs.image.dataset.streamKey === streamKey
    && refs.image.hasAttribute("src")
  ) {
    return;
  }

  clearCameraRetry(refs);
  refs.image.dataset.streamKey = streamKey;
  refs.placeholder.hidden = true;
  refs.card.classList.add("is-streaming");
  refs.image.src = streamUrlWithRetry(streamKey);
}

function handleCameraStreamLoad(refs) {
  if (!refs.image.dataset.streamKey) {
    return;
  }
  refs.card.classList.add("is-streaming");
  refs.placeholder.hidden = true;
  setCameraState(refs, "在线", "online");
}

function handleCameraStreamError(refs) {
  if (!refs.image.dataset.streamKey) {
    return;
  }

  delete refs.image.dataset.streamKey;
  refs.image.removeAttribute("src");
  refs.card.classList.remove("is-streaming");
  refs.placeholder.hidden = false;
  refs.placeholder.textContent = "画面连接中断，正在重连";
  setCameraState(refs, "重连中", "waiting");
  clearCameraRetry(refs);
  refs.retryTimer = window.setTimeout(() => {
    refs.retryTimer = null;
    const camera = cameraSnapshot.get(refs.cameraId);
    if (!document.hidden && cameraAvailable(camera)) {
      ensureCameraStream(refs, camera, true);
    } else if (!document.hidden) {
      void refreshCameraStatus();
    }
  }, CAMERA_RETRY_MS);
}

function stopCameraStream(refs, message) {
  clearCameraRetry(refs);
  delete refs.image.dataset.streamKey;
  refs.image.removeAttribute("src");
  refs.card.classList.remove("is-streaming");
  refs.placeholder.hidden = false;
  refs.placeholder.textContent = message;
}

function reconnectCameraStreams() {
  cameraCards.forEach((refs, cameraId) => {
    const camera = cameraSnapshot.get(cameraId);
    if (cameraAvailable(camera)) {
      ensureCameraStream(refs, camera, true);
    }
  });
}

function clearCameraRetry(refs) {
  if (refs.retryTimer !== null) {
    window.clearTimeout(refs.retryTimer);
    refs.retryTimer = null;
  }
}

function cameraAvailable(camera) {
  return Boolean(
    camera
    && camera.enabled !== false
    && camera.subscribed
    && camera.online,
  );
}

function setCameraState(refs, text, className) {
  refs.status.textContent = text;
  refs.status.className = "camera-live-state " + className;
}

function markBridgeUnavailable() {
  cameraCards.forEach((refs) => {
    if (!refs.image.hasAttribute("src")) {
      setCameraState(refs, "桥接离线", "offline");
      refs.placeholder.hidden = false;
      refs.placeholder.textContent = "无法连接 8080 图像桥";
    }
  });
}

function setBridgeStatus(text, className, title = "") {
  cameraBridgeStatus.textContent = text;
  cameraBridgeStatus.className = "camera-bridge-status " + className;
  cameraBridgeStatus.title = title;
}

function streamUrlWithRetry(streamKey) {
  const url = new URL(streamKey);
  url.searchParams.set("fps", String(CAMERA_STREAM_FPS));
  url.searchParams.set("_retry", String(Date.now()));
  return url.toString();
}

function resolveCameraBridgeUrl() {
  const params = new URLSearchParams(window.location.search);
  const override = String(params.get("camera_bridge") || "").trim();
  if (override) {
    const withScheme = /^[a-z][a-z0-9+.-]*:\/\//i.test(override)
      ? override
      : (window.location.protocol === "https:" ? "https://" : "http://") + override;
    try {
      return new URL(withScheme).origin;
    } catch {
      // Fall through to the same host on port 8080.
    }
  }

  const protocol = window.location.protocol === "https:" ? "https:" : "http:";
  let host = window.location.hostname || "127.0.0.1";
  if (host.includes(":") && !host.startsWith("[")) {
    host = "[" + host + "]";
  }
  return protocol + "//" + host + ":8080";
}

function formatNumber(value, digits) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "-";
}

function formatInteger(value, suffix = "") {
  const number = Number(value);
  return Number.isFinite(number) ? Math.round(number).toLocaleString() + suffix : "-";
}
