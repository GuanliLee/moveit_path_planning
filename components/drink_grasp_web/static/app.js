const fallbackOptions = {
  task_modes: [
    { id: "restock", name_zh: "上架" },
    { id: "pick", name_zh: "拣货" },
  ],
  drinks: [
    ["coca_cola", "可口可乐", "Coca-Cola", "Coca-Cola", "#d92d20"],
    ["ad_calcium_milk", "AD钙奶", "AD Calcium Milk", "AD Calcium Milk", "#f2b705"],
    ["daily_c_orange", "味全每日C橙汁", "Daily C Orange Juice", "Daily C Orange Juice", "#f97316"],
    ["daily_c_grape", "味全每日C葡萄汁", "Daily C Grape Juice", "Daily C Grape Juice", "#7c3aed"],
    ["wanglaoji", "王老吉", "wanglaoji", "wanglaoji", "#b42318"],
    ["dahongpao_milk_tea", "果子熟了大红袍乌龙轻乳茶", "Dahongpao Milk Tea", "Dahongpao Milk Tea", "#9a3412"],
    ["sprite", "雪碧", "Sprite", "Sprite", "#16a34a"],
    ["hk_orange_fanta", "港版芬达橙子味", "HK Orange Fanta", "HK Orange Fanta", "#fb923c"],
    ["aojiru", "伊藤园青汁", "Aojiru", "Aojiru", "#15803d"],
  ].map(([id, name_zh, brand_zh, prompt, tone]) => ({ id, name_zh, brand_zh, prompt, tone })),
  arms: [
    { id: "right", name_zh: "右臂", camera_frame: "cam_right_color_optical_frame" },
    { id: "left", name_zh: "左臂", camera_frame: "cam_left_color_optical_frame" },
  ],
  shelf: {
    layers: ["1", "2", "3"].map((id) => ({ id, name_zh: `第${["一", "二", "三"][Number(id) - 1]}层` })),
    positions: ["1", "2", "3", "4", "5", "6"].map((id) => ({ id, name_zh: `位置 ${id}` })),
  },
  defaults: { scene_id: 1, gripper_width_m: 0.08 },
  place_pose_config: { configured_slots: [] },
  pick_place_pose_config: { configured_arms: [] },
  recording: { enabled: true, target_dir: "", storage: "mcap", topics: [] },
};

const state = {
  options: fallbackOptions,
  selectedTaskMode: "restock",
  selectedDrinkId: "dahongpao_milk_tea",
  selectedLayer: "1",
  selectedPosition: "1",
  lastRecording: null,
  polling: null,
};

const drinkGrid = document.getElementById("drinkGrid");
const shelfGrid = document.getElementById("shelfGrid");
const taskModeSwitch = document.getElementById("taskModeSwitch");
const promptPreview = document.getElementById("promptPreview");
const slotStatus = document.getElementById("slotStatus");
const runSummary = document.getElementById("runSummary");
const statusOutput = document.getElementById("statusOutput");
const executeBtn = document.getElementById("executeBtn");
const stopExecuteBtn = document.getElementById("stopExecuteBtn");
const refreshBtn = document.getElementById("refreshBtn");
const recordStartBtn = document.getElementById("recordStartBtn");
const recordStopBtn = document.getElementById("recordStopBtn");
const recordSaveBtn = document.getElementById("recordSaveBtn");
const recordDiscardBtn = document.getElementById("recordDiscardBtn");
const recordStatusText = document.getElementById("recordStatusText");
const recordDetail = document.getElementById("recordDetail");
const rosStatus = document.getElementById("rosStatus");
const armSelect = document.getElementById("armSelect");
const sceneInput = document.getElementById("sceneInput");
const gripperInput = document.getElementById("gripperInput");
const placeSectionTitle = document.getElementById("placeSectionTitle");

init();

async function init() {
  renderOptions();
  bindEvents();
  try {
    const options = await apiGet("/api/options");
    state.options = { ...fallbackOptions, ...options };
    sceneInput.value = String(options.defaults?.scene_id ?? 1);
    gripperInput.value = String(options.defaults?.gripper_width_m ?? 0.08);
    renderOptions();
  } catch (error) {
    setRunSummary(`选项读取失败，使用本地默认：${error.message}`, true);
  }
  await refreshStatus();
}

function bindEvents() {
  taskModeSwitch.addEventListener("click", (event) => {
    const button = event.target.closest("[data-task-mode]");
    if (!button) return;
    state.selectedTaskMode = button.dataset.taskMode;
    renderTaskModeSwitch();
    updatePromptPreview();
    updateSlotStatus();
  });
  drinkGrid.addEventListener("click", (event) => {
    const button = event.target.closest("[data-drink-id]");
    if (!button) return;
    state.selectedDrinkId = button.dataset.drinkId;
    renderDrinkGrid();
    updatePromptPreview();
  });
  shelfGrid.addEventListener("click", (event) => {
    const button = event.target.closest("[data-slot]");
    if (!button) return;
    state.selectedLayer = button.dataset.layer;
    state.selectedPosition = button.dataset.position;
    renderShelfGrid();
    updateSlotStatus();
  });
  armSelect.addEventListener("change", updateSlotStatus);
  executeBtn.addEventListener("click", executeTask);
  stopExecuteBtn.addEventListener("click", stopExecution);
  refreshBtn.addEventListener("click", refreshStatus);
  recordStartBtn.addEventListener("click", startRecording);
  recordStopBtn.addEventListener("click", () => recordingAction("stop"));
  recordSaveBtn.addEventListener("click", () => recordingAction("save"));
  recordDiscardBtn.addEventListener("click", () => recordingAction("discard"));
}

function renderOptions() {
  renderTaskModeSwitch();
  renderArmSelect();
  renderDrinkGrid();
  renderShelfGrid();
  updatePromptPreview();
  updateSlotStatus();
}

function renderTaskModeSwitch() {
  const modes = state.options.task_modes || fallbackOptions.task_modes;
  taskModeSwitch.innerHTML = modes.map((mode) => {
    const active = mode.id === state.selectedTaskMode;
    return `
      <button class="${active ? "active" : ""}" type="button" data-task-mode="${escapeHtml(mode.id)}">
        ${escapeHtml(mode.name_zh)}
      </button>
    `;
  }).join("");
}

function renderArmSelect() {
  const current = armSelect.value || "right";
  armSelect.innerHTML = state.options.arms
    .map((arm) => `<option value="${escapeHtml(arm.id)}">${escapeHtml(arm.name_zh)}</option>`)
    .join("");
  armSelect.value = state.options.arms.some((arm) => arm.id === current) ? current : "right";
}

function renderDrinkGrid() {
  drinkGrid.innerHTML = state.options.drinks.map((drink) => {
    const active = drink.id === state.selectedDrinkId;
    return `
      <button class="drink-card ${active ? "active" : ""}" type="button" data-drink-id="${escapeHtml(drink.id)}">
        <span class="drink-swatch" style="background:${escapeHtml(drink.tone || "#667085")}"></span>
        <strong>${escapeHtml(drink.name_zh)}</strong>
        <span>${escapeHtml(drink.brand_zh)}</span>
      </button>
    `;
  }).join("");
}

function renderShelfGrid() {
  const layers = state.options.shelf?.layers || fallbackOptions.shelf.layers;
  const positions = state.options.shelf?.positions || fallbackOptions.shelf.positions;
  shelfGrid.innerHTML = layers.map((layer) => `
    <div class="shelf-row">
      <div class="shelf-layer">${escapeHtml(layer.name_zh)}</div>
      <div class="slot-row">
        ${positions.map((position) => {
          const active = layer.id === state.selectedLayer && position.id === state.selectedPosition;
          const configured = slotConfigured(armSelect.value, layer.id, position.id);
          return `
            <button
              class="slot-button ${active ? "active" : ""} ${configured ? "configured" : ""}"
              type="button"
              data-slot
              data-layer="${escapeHtml(layer.id)}"
              data-position="${escapeHtml(position.id)}"
            >
              ${escapeHtml(position.id)}
            </button>
          `;
        }).join("")}
      </div>
    </div>
  `).join("");
}

function updatePromptPreview() {
  const drink = selectedDrink();
  promptPreview.textContent = `${taskModeLabel(state.selectedTaskMode)}发给 121: ${drink?.prompt || "-"}`;
}

function updateSlotStatus() {
  renderShelfGrid();
  if (state.selectedTaskMode === "pick") {
    const configured = pickPlaceConfigured(armSelect.value);
    placeSectionTitle.textContent = "拣货放置";
    shelfGrid.classList.add("hidden");
    slotStatus.textContent = configured
      ? `${armLabel(armSelect.value)} 拣货放置位姿已配置`
      : `${armLabel(armSelect.value)} 拣货放置位姿未配置，使用状态机 pick_place 默认位姿`;
    executeBtn.textContent = "执行拣货";
    return;
  }
  placeSectionTitle.textContent = "货架位置";
  shelfGrid.classList.remove("hidden");
  const configured = slotConfigured(armSelect.value, state.selectedLayer, state.selectedPosition);
  slotStatus.textContent = configured
    ? `${armLabel(armSelect.value)} 第${state.selectedLayer}层 位置${state.selectedPosition} 已配置放置位姿`
    : `${armLabel(armSelect.value)} 第${state.selectedLayer}层 位置${state.selectedPosition} 未配置位姿，使用状态机默认放置点`;
  executeBtn.textContent = "执行上架";
}

function selectedDrink() {
  return state.options.drinks.find((drink) => drink.id === state.selectedDrinkId) || state.options.drinks[0];
}

function slotConfigured(arm, layer, position) {
  const keys = state.options.place_pose_config?.configured_slots || [];
  return keys.includes(`${arm}:${layer}:${position}`);
}

function pickPlaceConfigured(arm) {
  const arms = state.options.pick_place_pose_config?.configured_arms || [];
  return arms.includes(arm);
}

function armLabel(armId) {
  return state.options.arms.find((arm) => arm.id === armId)?.name_zh || armId;
}

function taskModeLabel(modeId) {
  return (state.options.task_modes || fallbackOptions.task_modes)
    .find((mode) => mode.id === modeId)?.name_zh || modeId;
}

function placeSummary(job) {
  if ((job.task_mode || "restock") === "pick") {
    return "拣货放置点";
  }
  return `第${job.shelf_layer || "-"}层位置${job.shelf_position || "-"}`;
}

async function executeTask() {
  const drink = selectedDrink();
  if (!drink) {
    setRunSummary("请选择饮料", true);
    return;
  }
  executeBtn.disabled = true;
  const placeText = state.selectedTaskMode === "pick"
    ? "拣货放置点"
    : `第${state.selectedLayer}层位置${state.selectedPosition}`;
  setRunSummary(`已提交${taskModeLabel(state.selectedTaskMode)}：${drink.name_zh} / ${armLabel(armSelect.value)} / ${placeText}`, false);
  try {
    const payload = await apiPost("/api/execute", {
      task_mode: state.selectedTaskMode,
      drink_id: drink.id,
      arm: armSelect.value,
      shelf_layer: state.selectedLayer,
      shelf_position: state.selectedPosition,
      scene_id: Number(sceneInput.value || 1),
      gripper_width_m: Number(gripperInput.value || 0.08),
    });
    renderStatus(payload);
    startPolling();
  } catch (error) {
    setRunSummary(`启动失败：${error.message}`, true);
    if (error.payload) {
      renderStatus(error.payload);
    }
  } finally {
    await refreshStatus();
  }
}

async function stopExecution() {
  stopExecuteBtn.disabled = true;
  stopExecuteBtn.textContent = "停止中...";
  setRunSummary("正在停止当前执行...", false);
  try {
    const payload = await apiPost("/api/stop", {});
    setRunSummary(payload.message || "停止请求已发送", false);
    startPolling();
  } catch (error) {
    setRunSummary(`停止失败：${error.message}`, true);
    if (error.payload) renderStatus(error.payload);
  } finally {
    await refreshStatus();
  }
}

async function startRecording() {
  const drink = selectedDrink();
  const taskMode = state.selectedTaskMode;
  const placeText = taskMode === "pick"
    ? "pick_place"
    : `layer_${state.selectedLayer}_position_${state.selectedPosition}`;
  const taskName = `${taskMode}_${drink?.id || "drink"}_${armSelect.value}`;
  const note = `${taskModeLabel(taskMode)} / ${drink?.name_zh || "-"} / ${armLabel(armSelect.value)} / ${placeText}`;
  await runRecordingRequest("start", {
    task_mode: taskMode,
    drink_id: drink?.id || "",
    arm: armSelect.value,
    task_name: taskName,
    note,
  });
}

async function recordingAction(action) {
  await runRecordingRequest(action, {});
}

async function runRecordingRequest(action, body) {
  setRecordingBusy(true);
  try {
    await apiPost(`/api/recording/${action}`, body);
    await refreshStatus();
  } catch (error) {
    setRunSummary(`录制${recordingActionLabel(action)}失败：${error.message}`, true);
    if (error.payload?.recording) {
      renderRecording(error.payload.recording);
    }
  } finally {
    setRecordingBusy(false);
  }
}

function startPolling() {
  if (state.polling) return;
  state.polling = window.setInterval(refreshStatus, 1000);
}

function stopPolling() {
  if (!state.polling) return;
  window.clearInterval(state.polling);
  state.polling = null;
}

async function refreshStatus() {
  try {
    const payload = await apiGet("/api/status");
    renderStatus(payload);
  } catch (error) {
    setRunSummary(`状态读取失败：${error.message}`, true);
  }
}

function renderStatus(payload) {
  const status = payload || {};
  const ros = status.ros || {};
  const recording = status.recording || {};
  rosStatus.textContent = ros.available
    ? `ROS ${ros.domain_id || "-"} · ${ros.service || "-"}`
    : `ROS 不可用`;
  rosStatus.className = `status-chip ${ros.available ? "online" : "offline"}`;
  renderRecording(recording);

  const busy = Boolean(status.busy);
  executeBtn.disabled = busy;
  if (busy) {
    const job = status.current_job || status.job || {};
    const stopping = job.status === "stopping";
    stopExecuteBtn.disabled = stopping;
    stopExecuteBtn.textContent = stopping ? "停止中..." : "停止当前执行";
    setRunSummary(stopping
      ? `正在停止${taskModeLabel(job.task_mode || "restock")}：${job.drink_name_zh || job.prompt || "-"}`
      : `执行中${taskModeLabel(job.task_mode || "restock")}：${job.drink_name_zh || job.prompt || "-"} / ${armLabel(job.arm)} / ${placeSummary(job)}`, false);
    startPolling();
  } else {
    stopExecuteBtn.disabled = true;
    stopExecuteBtn.textContent = "停止当前执行";
    if (recordingActive(recording)) {
      startPolling();
    } else {
      stopPolling();
    }
    const last = status.last_result;
    if (last) {
      const ok = Boolean(last.ok || last.success);
      const prefix = ok ? "完成" : "失败";
      setRunSummary(`${prefix}${taskModeLabel(last.task_mode || "restock")}：${last.drink_name_zh || last.prompt || "-"} · ${last.message || ""}`, !ok);
    } else if (!statusOutput.textContent || statusOutput.textContent === "-") {
      setRunSummary("等待任务", false);
    }
  }
  statusOutput.textContent = JSON.stringify(status, null, 2);
}

function renderRecording(recording) {
  state.lastRecording = recording || {};
  const enabled = recording.enabled !== false;
  const status = recording.status || "idle";
  const session = recording.session || null;
  recordStatusText.textContent = enabled
    ? recordingStatusLabel(status, session)
    : "录制未启用";
  recordStatusText.className = status === "error" ? "error" : "";

  const idle = status === "idle";
  const recordingNow = status === "recording";
  const waitingDecision = status === "stopped" || status === "error";
  recordStartBtn.disabled = !enabled || !idle;
  recordStopBtn.disabled = !enabled || !recordingNow;
  recordSaveBtn.disabled = !enabled || !waitingDecision;
  recordDiscardBtn.disabled = !enabled || !waitingDecision;

  const lines = [];
  if (session?.episode_id) lines.push(`episode: ${session.episode_id}`);
  if (session?.elapsed_sec !== undefined) lines.push(`时长: ${formatDuration(session.elapsed_sec)}`);
  if (session?.pending_path && waitingDecision) lines.push(`待处理: ${session.pending_path}`);
  if (session?.final_path && status === "saved") lines.push(`保存: ${session.final_path}`);
  if (!session && recording.last_saved?.path) lines.push(`最近保存: ${recording.last_saved.path}`);
  if (!session && recording.last_discarded?.episode_id) lines.push(`最近丢弃: ${recording.last_discarded.episode_id}`);
  if (recording.last_error) lines.push(`错误: ${recording.last_error}`);
  if (!lines.length) {
    lines.push(`目录: ${recording.target_dir || state.options.recording?.target_dir || "-"}`);
    lines.push(`格式: ${recording.storage || state.options.recording?.storage || "mcap"}`);
  }
  recordDetail.textContent = lines.join("\n");
}

function setRecordingBusy(isBusy) {
  if (!isBusy) {
    renderRecording(state.lastRecording || {});
    return;
  }
  [recordStartBtn, recordStopBtn, recordSaveBtn, recordDiscardBtn].forEach((button) => {
    button.disabled = true;
  });
}

function recordingActive(recording) {
  return ["recording"].includes(recording?.status || "");
}

function recordingStatusLabel(status, session) {
  if (status === "recording") return `录制中 · ${formatDuration(session?.elapsed_sec || 0)}`;
  if (status === "stopped") return "已停止，等待保存";
  if (status === "error") return "录制异常，等待处理";
  return "未录制";
}

function recordingActionLabel(action) {
  return {
    start: "开始",
    stop: "停止",
    save: "保存",
    discard: "丢弃",
  }[action] || action;
}

function formatDuration(seconds) {
  const total = Math.max(0, Math.floor(Number(seconds) || 0));
  const minutes = Math.floor(total / 60);
  const rest = total % 60;
  return `${minutes}:${String(rest).padStart(2, "0")}`;
}

function setRunSummary(message, isError = false) {
  runSummary.textContent = message;
  runSummary.className = isError ? "error" : "";
}

async function apiGet(path) {
  const response = await fetch(path, { cache: "no-store" });
  return parseResponse(response);
}

async function apiPost(path, body) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
    cache: "no-store",
  });
  return parseResponse(response);
}

async function parseResponse(response) {
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
    const error = new Error(payload.error || payload.message || `${response.status} ${response.statusText}`);
    error.payload = payload;
    throw error;
  }
  return payload;
}

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}
