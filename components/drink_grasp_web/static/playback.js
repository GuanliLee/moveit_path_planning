const CAMERA_IDS = ["cam_high", "cam_left", "cam_right"];
const JOINT_COLORS = [
  "#e45756",
  "#4c78a8",
  "#54a24b",
  "#f2cf5b",
  "#b279a2",
  "#ff9da6",
  "#9d755d",
];
const POSE_SERIES = [
  { index: 1, label: "X", color: "#e45756" },
  { index: 2, label: "Y", color: "#4c78a8" },
  { index: 3, label: "Z", color: "#54a24b" },
];

const playbackPage = document.getElementById("playbackPage");
const operationSections = Array.from(
  document.querySelectorAll(".operation-only"),
);
const pageTabs = Array.from(document.querySelectorAll("[data-page-tab]"));
const refreshPlaybackBtn = document.getElementById("refreshPlaybackBtn");
const playbackSearchInput = document.getElementById("playbackSearchInput");
const episodeCatalogList = document.getElementById("episodeCatalogList");
const episodeCatalogSummary = document.getElementById(
  "episodeCatalogSummary",
);
const playbackLoadStatus = document.getElementById("playbackLoadStatus");
const playbackEmpty = document.getElementById("playbackEmpty");
const playbackViewer = document.getElementById("playbackViewer");
const playbackEpisodeName = document.getElementById(
  "playbackEpisodeName",
);
const playbackEpisodeNote = document.getElementById(
  "playbackEpisodeNote",
);
const playbackDurationMetric = document.getElementById(
  "playbackDurationMetric",
);
const playbackMessagesMetric = document.getElementById(
  "playbackMessagesMetric",
);
const playbackSizeMetric = document.getElementById(
  "playbackSizeMetric",
);
const playbackPlayBtn = document.getElementById("playbackPlayBtn");
const playbackRewindBtn = document.getElementById("playbackRewindBtn");
const playbackForwardBtn = document.getElementById(
  "playbackForwardBtn",
);
const playbackClock = document.getElementById("playbackClock");
const playbackTimeline = document.getElementById("playbackTimeline");
const playbackSpeedSelect = document.getElementById(
  "playbackSpeedSelect",
);
const playbackArmSelect = document.getElementById("playbackArmSelect");
const poseTrajectoryCanvas = document.getElementById(
  "poseTrajectoryCanvas",
);
const jointTrajectoryCanvas = document.getElementById(
  "jointTrajectoryCanvas",
);
const poseTopicLabel = document.getElementById("poseTopicLabel");
const jointTopicLabel = document.getElementById("jointTopicLabel");
const jointLegend = document.getElementById("jointLegend");
const poseCurrentValues = document.getElementById(
  "poseCurrentValues",
);
const jointCurrentValues = document.getElementById(
  "jointCurrentValues",
);
const playbackWarnings = document.getElementById("playbackWarnings");

const cameraElements = new Map(
  CAMERA_IDS.map((cameraId) => {
    const card = document.querySelector(
      `[data-playback-camera="${cameraId}"]`,
    );
    return [
      cameraId,
      {
        card,
        image: card?.querySelector("[data-playback-image]"),
        placeholder: card?.querySelector(
          "[data-playback-placeholder]",
        ),
        meta: card?.querySelector("[data-playback-frame-meta]"),
      },
    ];
  }),
);

const playbackState = {
  page: "operation",
  catalog: [],
  selectedId: "",
  manifest: null,
  currentTime: 0,
  duration: 0,
  playing: false,
  speed: 1,
  lastTick: 0,
  lastDrawAt: 0,
  animationFrame: null,
  frameIndexes: new Map(),
  catalogLoaded: false,
  loadToken: 0,
};

if (playbackPage) {
  initPlayback();
}

function initPlayback() {
  bindPlaybackEvents();
  const initialPage = window.location.hash === "#playback"
    ? "playback"
    : "operation";
  setPage(initialPage, false);
}

function bindPlaybackEvents() {
  pageTabs.forEach((button) => {
    button.addEventListener("click", () => {
      setPage(button.dataset.pageTab || "operation");
    });
  });

  refreshPlaybackBtn?.addEventListener("click", () => {
    void refreshCatalog(true);
  });
  playbackSearchInput?.addEventListener("input", renderCatalog);
  episodeCatalogList?.addEventListener("click", (event) => {
    const item = event.target.closest("[data-episode-id]");
    if (!item || item.disabled) return;
    void loadEpisode(item.dataset.episodeId);
  });

  playbackPlayBtn?.addEventListener("click", togglePlayback);
  playbackRewindBtn?.addEventListener("click", () => {
    seek(playbackState.currentTime - 1);
  });
  playbackForwardBtn?.addEventListener("click", () => {
    seek(playbackState.currentTime + 1);
  });
  playbackTimeline?.addEventListener("input", () => {
    seek(Number(playbackTimeline.value));
  });
  playbackSpeedSelect?.addEventListener("change", () => {
    playbackState.speed = Number(playbackSpeedSelect.value) || 1;
    playbackState.lastTick = performance.now();
  });
  playbackArmSelect?.addEventListener("change", () => {
    renderTrajectoryLabels();
    drawTrajectories();
  });

  window.addEventListener("resize", scheduleTrajectoryDraw);
  window.addEventListener("hashchange", () => {
    setPage(
      window.location.hash === "#playback" ? "playback" : "operation",
      false,
    );
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) pausePlayback();
  });
}

function setPage(page, updateHash = true) {
  const normalized = page === "playback" ? "playback" : "operation";
  playbackState.page = normalized;
  const showingPlayback = normalized === "playback";
  operationSections.forEach((section) => {
    section.classList.toggle("hidden", showingPlayback);
  });
  playbackPage.classList.toggle("hidden", !showingPlayback);
  pageTabs.forEach((button) => {
    button.classList.toggle(
      "active",
      button.dataset.pageTab === normalized,
    );
  });
  if (!showingPlayback) {
    pausePlayback();
  } else {
    void refreshCatalog(false);
    scheduleTrajectoryDraw();
  }
  if (updateHash) {
    history.replaceState(
      null,
      "",
      showingPlayback ? "#playback" : window.location.pathname,
    );
  }
  document.dispatchEvent(
    new CustomEvent("drink-page-change", {
      detail: { page: normalized },
    }),
  );
}

async function refreshCatalog(force) {
  if (
    playbackState.catalogLoaded
    && !force
    && playbackState.catalog.length
  ) {
    renderCatalog();
    return;
  }
  refreshPlaybackBtn.disabled = true;
  episodeCatalogSummary.textContent = "正在读取录制目录...";
  try {
    const payload = await apiGet("/api/playback/episodes");
    playbackState.catalog = Array.isArray(payload.episodes)
      ? payload.episodes
      : [];
    playbackState.catalogLoaded = true;
    episodeCatalogSummary.textContent =
      `${playbackState.catalog.length} 个 Episode`;
    renderCatalog();
  } catch (error) {
    episodeCatalogSummary.textContent = "读取失败";
    episodeCatalogList.innerHTML =
      `<div class="episode-list-empty">无法读取录制数据：${escapeHtml(error.message)}</div>`;
    setLoadStatus("目录读取失败", "error");
  } finally {
    refreshPlaybackBtn.disabled = false;
  }
}

function renderCatalog() {
  const query = String(playbackSearchInput?.value || "")
    .trim()
    .toLowerCase();
  const visible = playbackState.catalog.filter((episode) => {
    if (!query) return true;
    return [
      episode.id,
      episode.group,
      episode.name,
      episode.task_name,
      episode.note,
    ].some((value) =>
      String(value || "").toLowerCase().includes(query),
    );
  });

  episodeCatalogSummary.textContent =
    query
      ? `显示 ${visible.length} / ${playbackState.catalog.length}`
      : `${playbackState.catalog.length} 个 Episode`;

  if (!visible.length) {
    episodeCatalogList.innerHTML =
      '<div class="episode-list-empty">没有匹配的录制数据</div>';
    return;
  }

  episodeCatalogList.innerHTML = visible
    .map((episode) => {
      const active = episode.id === playbackState.selectedId;
      const error = episode.status === "error";
      const cameraCount = Array.isArray(episode.cameras)
        ? episode.cameras.length
        : 0;
      return `
        <button
          type="button"
          class="episode-item ${active ? "active" : ""} ${error ? "error" : ""}"
          data-episode-id="${escapeHtml(episode.id)}"
          ${error ? "disabled" : ""}
        >
          <span class="episode-item-title">
            <strong>${escapeHtml(episode.name || "-")}</strong>
            <span>${escapeHtml(formatDate(episode.started_at))}</span>
          </span>
          <span class="episode-item-note">${escapeHtml(episode.note || episode.task_name || episode.group || "-")}</span>
          <span class="episode-item-meta">
            <span>${formatTime(episode.duration_sec || 0)}</span>
            <span>${cameraCount} 路视频</span>
            <span>${formatBytes(episode.mcap_bytes || 0)}</span>
          </span>
        </button>
      `;
    })
    .join("");
}

async function loadEpisode(episodeId) {
  if (!episodeId) return;
  pausePlayback();
  const loadToken = ++playbackState.loadToken;
  playbackState.selectedId = episodeId;
  playbackState.manifest = null;
  playbackState.frameIndexes.clear();
  renderCatalog();
  setLoadStatus("正在解码 MCAP...", "loading");
  playbackViewer.classList.add("hidden");
  playbackEmpty.classList.remove("hidden");
  playbackEmpty.innerHTML = `
    <strong>正在准备 ${escapeHtml(episodeId)}</strong>
    <span>正在解码三路图像与轨迹，首次加载通常需要几秒。</span>
  `;

  try {
    const payload = await apiGet(
      `/api/playback/manifest?episode=${encodeURIComponent(episodeId)}`,
    );
    if (loadToken !== playbackState.loadToken) return;
    playbackState.manifest = payload;
    playbackState.duration = Math.max(
      0,
      Number(payload.duration_sec) || 0,
    );
    playbackState.currentTime = 0;
    playbackTimeline.max = String(playbackState.duration || 1);
    playbackTimeline.value = "0";
    chooseInitialArm(payload);
    renderManifest();
    seek(0, true);
    playbackEmpty.classList.add("hidden");
    playbackViewer.classList.remove("hidden");
    setLoadStatus("已就绪", "ready");
    scheduleTrajectoryDraw();
  } catch (error) {
    if (loadToken !== playbackState.loadToken) return;
    playbackEmpty.classList.remove("hidden");
    playbackEmpty.innerHTML = `
      <strong>该 Episode 无法加载</strong>
      <span>${escapeHtml(error.message)}</span>
    `;
    setLoadStatus("加载失败", "error");
  }
}

function chooseInitialArm(manifest) {
  const episode = manifest.episode || {};
  const hint = `${episode.task_name || ""} ${episode.note || ""}`
    .toLowerCase();
  const poses = manifest.trajectories?.poses || {};
  const joints = manifest.trajectories?.joints || {};
  let side = hint.includes("左臂") || hint.includes("_left")
    ? "left"
    : "right";
  if (!poses[side] && !joints[side]) {
    side = poses.right || joints.right ? "right" : "left";
  }
  playbackArmSelect.value = side;
}

function renderManifest() {
  const manifest = playbackState.manifest;
  const episode = manifest?.episode || {};
  playbackEpisodeName.textContent = episode.id || "-";
  playbackEpisodeNote.textContent =
    episode.note || episode.task_name || "-";
  playbackDurationMetric.textContent =
    `时长 ${formatTime(playbackState.duration)}`;
  playbackMessagesMetric.textContent =
    `消息 ${formatInteger(episode.message_count)}`;
  playbackSizeMetric.textContent =
    `大小 ${formatBytes(episode.mcap_bytes || 0)}`;

  CAMERA_IDS.forEach((cameraId) => {
    const refs = cameraElements.get(cameraId);
    const camera = manifest.cameras?.[cameraId];
    refs?.image?.removeAttribute("src");
    if (refs?.meta) {
      refs.meta.textContent = camera
        ? `${camera.frame_count} 帧 · ${camera.width}×${camera.height}`
        : "未录制";
    }
    if (refs?.placeholder) {
      refs.placeholder.textContent = camera ? "正在读取首帧" : "未录制";
    }
  });

  const warnings = Array.isArray(manifest.warnings)
    ? manifest.warnings
    : [];
  playbackWarnings.classList.toggle("hidden", !warnings.length);
  playbackWarnings.textContent = warnings.length
    ? `部分数据未能解码：\n${warnings.join("\n")}`
    : "";
  renderTrajectoryLabels();
}

function renderTrajectoryLabels() {
  const manifest = playbackState.manifest;
  if (!manifest) return;
  const side = playbackArmSelect.value || "right";
  const pose = manifest.trajectories?.poses?.[side];
  const joint = manifest.trajectories?.joints?.[side];
  poseTopicLabel.textContent = pose?.topic
    ? `${pose.topic} · ${formatInteger(pose.sample_count)} 点`
    : "该侧无末端轨迹";
  jointTopicLabel.textContent = joint?.topic
    ? `${joint.topic} · ${formatInteger(joint.sample_count)} 点`
    : "该侧无关节轨迹";

  const names = joint?.names?.length
    ? joint.names
    : Array.from(
        { length: Math.max(0, (joint?.samples?.[0]?.length || 1) - 1) },
        (_, index) => `J${index + 1}`,
      );
  jointLegend.innerHTML = names
    .slice(0, JOINT_COLORS.length)
    .map(
      (name, index) =>
        `<span style="--series-color:${JOINT_COLORS[index]}">${escapeHtml(shortJointName(name, index))}</span>`,
    )
    .join("");
}

function togglePlayback() {
  if (!playbackState.manifest) return;
  if (playbackState.playing) {
    pausePlayback();
    return;
  }
  if (playbackState.currentTime >= playbackState.duration) {
    seek(0);
  }
  playbackState.playing = true;
  playbackState.lastTick = performance.now();
  playbackPlayBtn.textContent = "暂停";
  playbackState.animationFrame = requestAnimationFrame(playbackTick);
}

function pausePlayback() {
  playbackState.playing = false;
  playbackState.lastTick = 0;
  if (playbackState.animationFrame !== null) {
    cancelAnimationFrame(playbackState.animationFrame);
    playbackState.animationFrame = null;
  }
  if (playbackPlayBtn) playbackPlayBtn.textContent = "播放";
}

function playbackTick(now) {
  if (!playbackState.playing) return;
  const elapsed = Math.max(0, (now - playbackState.lastTick) / 1000);
  playbackState.lastTick = now;
  playbackState.currentTime = Math.min(
    playbackState.duration,
    playbackState.currentTime + elapsed * playbackState.speed,
  );
  updatePlaybackPresentation(now);
  if (playbackState.currentTime >= playbackState.duration) {
    pausePlayback();
    return;
  }
  playbackState.animationFrame = requestAnimationFrame(playbackTick);
}

function seek(value, force = false) {
  playbackState.currentTime = clamp(
    Number(value) || 0,
    0,
    playbackState.duration,
  );
  playbackState.lastTick = performance.now();
  updatePlaybackPresentation(performance.now(), force);
}

function updatePlaybackPresentation(now, force = false) {
  playbackTimeline.value = String(playbackState.currentTime);
  playbackClock.textContent =
    `${formatTime(playbackState.currentTime, true)} / ${formatTime(playbackState.duration, true)}`;
  updateCameraFrames(force);
  if (force || now - playbackState.lastDrawAt >= 32) {
    playbackState.lastDrawAt = now;
    drawTrajectories();
  }
}

function updateCameraFrames(force) {
  const manifest = playbackState.manifest;
  if (!manifest) return;
  CAMERA_IDS.forEach((cameraId) => {
    const camera = manifest.cameras?.[cameraId];
    const refs = cameraElements.get(cameraId);
    if (!camera || !camera.times?.length || !refs?.image) {
      refs?.image?.removeAttribute("src");
      if (refs?.meta) refs.meta.textContent = "未录制";
      return;
    }
    const index = findSampleIndex(camera.times, playbackState.currentTime);
    if (
      !force
      && playbackState.frameIndexes.get(cameraId) === index
    ) {
      return;
    }
    playbackState.frameIndexes.set(cameraId, index);
    refs.image.src =
      `/api/playback/frame?episode=${encodeURIComponent(playbackState.selectedId)}`
      + `&camera=${encodeURIComponent(cameraId)}&index=${index}`;
    refs.meta.textContent =
      `${index + 1}/${camera.frame_count} · ${formatTime(camera.times[index], true)}`;
  });
}

function drawTrajectories() {
  const manifest = playbackState.manifest;
  if (!manifest || playbackViewer.classList.contains("hidden")) return;
  const side = playbackArmSelect.value || "right";
  const pose = manifest.trajectories?.poses?.[side];
  const joint = manifest.trajectories?.joints?.[side];

  drawTimeSeries(
    poseTrajectoryCanvas,
    pose?.samples || [],
    POSE_SERIES,
    playbackState.currentTime,
    playbackState.duration,
    "m",
  );
  const poseRow = nearestRow(
    pose?.samples || [],
    playbackState.currentTime,
  );
  poseCurrentValues.textContent = poseRow
    ? `t ${formatTime(poseRow[0], true)} · X ${formatNumber(poseRow[1], 4)} m · Y ${formatNumber(poseRow[2], 4)} m · Z ${formatNumber(poseRow[3], 4)} m`
    : "无末端轨迹";

  const jointNames = joint?.names?.length
    ? joint.names
    : Array.from(
        { length: Math.max(0, (joint?.samples?.[0]?.length || 1) - 1) },
        (_, index) => `J${index + 1}`,
      );
  const jointSeries = jointNames
    .slice(0, JOINT_COLORS.length)
    .map((name, index) => ({
      index: index + 1,
      label: shortJointName(name, index),
      color: JOINT_COLORS[index],
    }));
  drawTimeSeries(
    jointTrajectoryCanvas,
    joint?.samples || [],
    jointSeries,
    playbackState.currentTime,
    playbackState.duration,
    "rad",
  );
  const jointRow = nearestRow(
    joint?.samples || [],
    playbackState.currentTime,
  );
  jointCurrentValues.textContent = jointRow
    ? jointSeries
        .map(
          (series) =>
            `${series.label} ${formatNumber(jointRow[series.index], 3)}`,
        )
        .join(" · ")
    : "无关节轨迹";
}

function drawTimeSeries(
  canvas,
  samples,
  series,
  currentTime,
  duration,
  unit,
) {
  if (!canvas) return;
  const width = Math.max(300, canvas.clientWidth || 300);
  const height = Math.max(180, canvas.clientHeight || 260);
  const ratio = Math.max(1, window.devicePixelRatio || 1);
  const pixelWidth = Math.round(width * ratio);
  const pixelHeight = Math.round(height * ratio);
  if (
    canvas.width !== pixelWidth
    || canvas.height !== pixelHeight
  ) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const context = canvas.getContext("2d");
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  context.clearRect(0, 0, width, height);

  const plot = {
    left: 52,
    top: 14,
    right: width - 14,
    bottom: height - 30,
  };
  const plotWidth = Math.max(1, plot.right - plot.left);
  const plotHeight = Math.max(1, plot.bottom - plot.top);

  context.fillStyle = "#fbfcfd";
  context.fillRect(0, 0, width, height);
  context.font = '10px -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif';
  context.lineWidth = 1;

  if (!samples.length || !series.length) {
    context.fillStyle = "#667085";
    context.textAlign = "center";
    context.fillText("无轨迹数据", width / 2, height / 2);
    return;
  }

  const values = [];
  samples.forEach((row) => {
    series.forEach((item) => {
      const value = Number(row[item.index]);
      if (Number.isFinite(value)) values.push(value);
    });
  });
  if (!values.length) return;
  let minimum = Math.min(...values);
  let maximum = Math.max(...values);
  if (maximum === minimum) {
    maximum += 0.5;
    minimum -= 0.5;
  } else {
    const padding = (maximum - minimum) * 0.08;
    minimum -= padding;
    maximum += padding;
  }
  const timeSpan = Math.max(
    0.001,
    duration || Number(samples.at(-1)?.[0]) || 0.001,
  );
  const xFor = (time) =>
    plot.left + clamp(Number(time) / timeSpan, 0, 1) * plotWidth;
  const yFor = (value) =>
    plot.bottom
    - ((Number(value) - minimum) / (maximum - minimum)) * plotHeight;

  context.strokeStyle = "#e3e8ee";
  context.fillStyle = "#7a8793";
  for (let index = 0; index <= 4; index += 1) {
    const ratioY = index / 4;
    const y = plot.top + ratioY * plotHeight;
    context.beginPath();
    context.moveTo(plot.left, y);
    context.lineTo(plot.right, y);
    context.stroke();
    const value = maximum - ratioY * (maximum - minimum);
    context.textAlign = "right";
    context.fillText(
      `${formatNumber(value, 2)}`,
      plot.left - 6,
      y + 3,
    );
  }
  for (let index = 0; index <= 4; index += 1) {
    const ratioX = index / 4;
    const x = plot.left + ratioX * plotWidth;
    context.beginPath();
    context.moveTo(x, plot.top);
    context.lineTo(x, plot.bottom);
    context.stroke();
    context.textAlign = index === 0
      ? "left"
      : index === 4
        ? "right"
        : "center";
    context.fillText(
      formatTime(timeSpan * ratioX),
      x,
      plot.bottom + 17,
    );
  }
  context.textAlign = "left";
  context.fillText(unit, 6, 11);

  series.forEach((item) => {
    context.strokeStyle = item.color;
    context.lineWidth = 1.4;
    context.beginPath();
    let started = false;
    samples.forEach((row) => {
      const value = Number(row[item.index]);
      if (!Number.isFinite(value)) return;
      const x = xFor(row[0]);
      const y = yFor(value);
      if (!started) {
        context.moveTo(x, y);
        started = true;
      } else {
        context.lineTo(x, y);
      }
    });
    context.stroke();
  });

  const cursorX = xFor(currentTime);
  context.strokeStyle = "#172026";
  context.lineWidth = 1;
  context.setLineDash([4, 3]);
  context.beginPath();
  context.moveTo(cursorX, plot.top);
  context.lineTo(cursorX, plot.bottom);
  context.stroke();
  context.setLineDash([]);
}

function findSampleIndex(times, currentTime) {
  if (!times.length) return -1;
  if (currentTime <= times[0]) return 0;
  let low = 0;
  let high = times.length - 1;
  while (low <= high) {
    const middle = (low + high) >> 1;
    if (times[middle] <= currentTime) {
      low = middle + 1;
    } else {
      high = middle - 1;
    }
  }
  return Math.max(0, Math.min(times.length - 1, high));
}

function nearestRow(rows, currentTime) {
  if (!rows.length) return null;
  const times = rows.map((row) => Number(row[0]) || 0);
  return rows[findSampleIndex(times, currentTime)] || rows[0];
}

function scheduleTrajectoryDraw() {
  requestAnimationFrame(drawTrajectories);
}

function setLoadStatus(text, className = "") {
  playbackLoadStatus.textContent = text;
  playbackLoadStatus.className =
    `playback-status ${className}`.trim();
}

async function apiGet(path) {
  const response = await fetch(path, { cache: "no-store" });
  const text = await response.text();
  let payload = {};
  if (text) {
    try {
      payload = JSON.parse(text);
    } catch {
      payload = { raw: text };
    }
  }
  if (!response.ok) {
    throw new Error(
      payload.message
      || payload.error
      || `${response.status} ${response.statusText}`,
    );
  }
  return payload;
}

function formatTime(seconds, precise = false) {
  const value = Math.max(0, Number(seconds) || 0);
  const minutes = Math.floor(value / 60);
  const rest = value - minutes * 60;
  return precise
    ? `${minutes}:${rest.toFixed(1).padStart(4, "0")}`
    : `${minutes}:${String(Math.floor(rest)).padStart(2, "0")}`;
}

function formatBytes(bytes) {
  const value = Math.max(0, Number(bytes) || 0);
  if (value >= 1024 ** 3) return `${(value / 1024 ** 3).toFixed(1)} GB`;
  if (value >= 1024 ** 2) return `${(value / 1024 ** 2).toFixed(1)} MB`;
  if (value >= 1024) return `${(value / 1024).toFixed(1)} KB`;
  return `${Math.round(value)} B`;
}

function formatDate(timestamp) {
  const value = Number(timestamp);
  if (!Number.isFinite(value) || value <= 0) return "-";
  return new Date(value * 1000).toLocaleString("zh-CN", {
    month: "2-digit",
    day: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
  });
}

function formatNumber(value, digits) {
  const number = Number(value);
  return Number.isFinite(number) ? number.toFixed(digits) : "-";
}

function formatInteger(value) {
  const number = Number(value);
  return Number.isFinite(number)
    ? Math.round(number).toLocaleString("zh-CN")
    : "-";
}

function shortJointName(name, index) {
  const matched = String(name || "").match(/joint[_ ]?(\d+)/i);
  return matched ? `J${matched[1]}` : String(name || `J${index + 1}`);
}

function clamp(value, minimum, maximum) {
  return Math.min(maximum, Math.max(minimum, value));
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}
