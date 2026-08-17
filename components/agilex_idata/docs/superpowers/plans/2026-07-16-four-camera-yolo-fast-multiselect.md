# Four-Camera YOLO Fast Multi-Select Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a deployed-class multi-select dropdown, 0.2-second inference and page refresh, and temporal stream requests to the four-camera YOLO console.

**Architecture:** Keep the existing standard-library HTTP server and latest-frame scheduler. Extend only the outbound YOLO request contract, timing bounds/defaults, and the single-file HTML control layer; retain the existing one-in-flight-per-camera guard as backpressure.

**Tech Stack:** Python 3.10, ROS 2 Humble, standard-library HTTP/threading, HTML/CSS/vanilla JavaScript, pytest, Node syntax checking.

## Global Constraints

- Targets must come only from deployed classes returned by `/api/classes`.
- Multiple classes can be selected simultaneously.
- Inference interval is configurable from 0.2 through 60 seconds and defaults to 0.2 seconds.
- Browser status polling runs every 200 milliseconds.
- Every camera has a stable `stream_id` and at most one request in flight.
- Temporal state resets on the first request and whenever target names change.
- Do not add dependencies, WebSockets, camera changes, or model changes.

---

### Task 1: Temporal YOLO Request Contract

**Files:**
- Modify: `scripts/inference/four_camera_yolo_core.py:184-230`
- Test: `tests/test_four_camera_yolo_http.py:139-170`

**Interfaces:**
- Consumes: `YoloHttpClient.predict(camera: str, frame: FrameSnapshot, targets: tuple[str, ...])`.
- Produces: `/predict` requests containing stable `stream_id`, `temporal`, and target-sensitive `reset_temporal` fields while preserving unique `request_id` values.

- [ ] **Step 1: Write failing temporal contract tests**

Extend the direct API test with:

```python
assert request["stream_id"] == "four-camera-left"
assert request["temporal"] is True
assert request["reset_temporal"] is True
```

Add:

```python
def test_predict_keeps_stream_id_and_resets_temporal_only_for_new_targets(fake_yolo, core):
    client = core.YoloHttpClient(fake_yolo.url, timeout=1.0)
    frame = core.FrameSnapshot(fake_jpeg(640, 480), 4, 9, 1.0)
    client.predict("left", frame, ("Wanglaoji",))
    client.predict("left", frame, ("Wanglaoji",))
    client.predict("left", frame, ("Sprite",))
    requests = fake_yolo.requests[-3:]
    assert [item["stream_id"] for item in requests] == [
        "four-camera-left",
        "four-camera-left",
        "four-camera-left",
    ]
    assert [item["temporal"] for item in requests] == [True, True, True]
    assert [item["reset_temporal"] for item in requests] == [True, False, True]
```

- [ ] **Step 2: Run the focused tests and confirm RED**

Run:

```bash
pytest -q tests/test_four_camera_yolo_http.py -k 'direct_json or stream_id'
```

Expected: failures for missing `stream_id`, `temporal`, or `reset_temporal`.

- [ ] **Step 3: Implement the request fields**

Initialize per-camera target state in `YoloHttpClient.__init__`:

```python
self._stream_targets: dict[str, tuple[str, ...]] = {}
```

Build the request and update state only after a successful YOLO response:

```python
reset_temporal = self._stream_targets.get(camera) != targets
request_payload = {
    "request_id": f"four-camera-{camera}-{uuid.uuid4()}",
    "stream_id": f"four-camera-{camera}",
    "temporal": True,
    "reset_temporal": reset_temporal,
    "image_base64": base64.b64encode(frame.jpeg).decode("ascii"),
    "targets": list(targets),
    "return_overlay": True,
}
payload = self._require_ok(self._json("/predict", request_payload), "predict")
self._stream_targets[camera] = targets
```

- [ ] **Step 4: Run HTTP and worker tests and confirm GREEN**

Run:

```bash
pytest -q tests/test_four_camera_yolo_http.py tests/test_four_camera_yolo_core.py
```

Expected: all tests pass.

- [ ] **Step 5: Commit the temporal request change**

```bash
git add scripts/inference/four_camera_yolo_core.py tests/test_four_camera_yolo_http.py
git commit -m "fix: enable temporal four-camera yolo inference"
```

### Task 2: 0.2-Second Backend Timing

**Files:**
- Modify: `scripts/inference/four_camera_yolo_core.py:35-39`
- Modify: `scripts/inference/four_camera_yolo_web.py:194-212`
- Modify: `scripts/inference/run_four_camera_yolo_web.sh:5-11`
- Test: `tests/test_four_camera_yolo_core.py:34-42`
- Test: `tests/test_four_camera_yolo_web.py:170-190,306-365`

**Interfaces:**
- Consumes: `validate_interval_sec(value: object) -> float` and CLI/environment defaults.
- Produces: a shared 0.2–60 second contract with a 0.2-second live default.

- [ ] **Step 1: Write failing timing tests**

Change the accepted and rejected core cases:

```python
@pytest.mark.parametrize(
    "value, expected", [(0.2, 0.2), ("0.3", 0.3), (1, 1.0), (60, 60.0)]
)
def test_validate_interval_sec_accepts_closed_range(core, value, expected):
    assert core.validate_interval_sec(value) == expected

@pytest.mark.parametrize("value", [None, True, 0.19, 60.01, "nan", "bad"])
def test_validate_interval_sec_rejects_invalid_values(core, value):
    with pytest.raises(ValueError):
        core.validate_interval_sec(value)
```

Update Web/launcher expectations to require `0.2`, including the CLI default,
launcher `DEFAULT_INTERVAL_SEC`, and the API error message `between 0.2 and 60.0`.

- [ ] **Step 2: Run the focused timing tests and confirm RED**

Run:

```bash
pytest -q tests/test_four_camera_yolo_core.py -k interval
pytest -q tests/test_four_camera_yolo_web.py -k 'defaults or invalid_network or invalid_payload or launcher'
```

Expected: 0.2 is rejected and defaults remain 3.0.

- [ ] **Step 3: Implement shared timing values**

Set:

```python
MIN_INTERVAL_SEC = 0.2
```

Change the CLI parser default and help:

```python
parser.add_argument(
    "--default-interval-sec",
    type=validate_interval_sec,
    default=0.2,
    help="global inference interval, 0.2 through 60 seconds",
)
```

Change the launcher default:

```bash
DEFAULT_INTERVAL_SEC="${DEFAULT_INTERVAL_SEC:-0.2}"
```

- [ ] **Step 4: Run timing and scheduler tests and confirm GREEN**

Run:

```bash
pytest -q tests/test_four_camera_yolo_core.py tests/test_four_camera_yolo_web.py
```

Expected: all tests pass and the existing busy-round test still verifies no queue.

- [ ] **Step 5: Commit timing support**

```bash
git add scripts/inference/four_camera_yolo_core.py scripts/inference/four_camera_yolo_web.py scripts/inference/run_four_camera_yolo_web.sh tests/test_four_camera_yolo_core.py tests/test_four_camera_yolo_web.py
git commit -m "feat: support 0.2 second yolo rounds"
```

### Task 3: Searchable Multi-Select Dropdown and 200 ms Rendering

**Files:**
- Modify: `scripts/inference/four_camera_yolo_web.html:130-245,435-465,565-939`
- Test: `tests/test_four_camera_yolo_web.py:238-290`

**Interfaces:**
- Consumes: `/api/classes`, `/api/status`, `/api/config`, and `selectedTargets`.
- Produces: `#target-picker-toggle`, `#target-menu`, `#target-search`, `#target-options`, `#clear-targets`, selected chips, and 200 ms polling.

- [ ] **Step 1: Write failing HTML contract tests**

Require the new controls and remove the free-form contract:

```python
assert 'id="target-picker-toggle"' in html
assert 'id="target-menu"' in html
assert 'id="target-search"' in html
assert 'id="target-options"' in html
assert 'id="clear-targets"' in html
assert 'aria-multiselectable="true"' in html
assert 'id="target-input"' not in html
assert 'min="0.2"' in html
assert 'step="0.1"' in html
assert 'value="0.2"' in html
assert "setInterval(refreshStatus, 200)" in html
assert "renderTargetOptions" in html
assert "setTargetMenuOpen" in html
```

Keep assertions for safe `textContent`, sequence-based images, responsiveness,
and Node syntax validation.

- [ ] **Step 2: Run HTML tests and confirm RED**

Run:

```bash
pytest -q tests/test_four_camera_yolo_web.py -k 'home_page or javascript'
```

Expected: failures because the page still has a free-form input and 500 ms polling.

- [ ] **Step 3: Replace the target editor markup**

Use this structure:

```html
<div id="target-picker" class="target-picker">
  <button id="target-picker-toggle" class="target-picker-toggle" type="button"
    aria-expanded="false" aria-controls="target-menu">
    <span id="target-picker-summary">请选择物品</span>
    <span aria-hidden="true">⌄</span>
  </button>
  <div id="target-menu" class="target-menu" hidden>
    <div class="target-menu-toolbar">
      <input id="target-search" type="search" autocomplete="off" placeholder="搜索物品">
      <button id="clear-targets" type="button">清空</button>
    </div>
    <div id="target-options" class="target-options" role="listbox"
      aria-multiselectable="true" aria-label="可检测物品"></div>
    <p id="target-empty" class="target-empty" hidden>没有匹配的已部署物品</p>
  </div>
</div>
<div id="target-tags" class="tag-list" aria-live="polite"></div>
```

Change the interval input to:

```html
<input id="interval-sec" type="number" min="0.2" max="60" step="0.1" value="0.2">
```

- [ ] **Step 4: Implement dropdown state and styling**

Add the anchored, scrollable menu styles:

```css
.target-picker { position: relative; }
.target-picker-toggle {
  display: flex;
  width: 100%;
  min-height: 42px;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  padding: 9px 12px;
  border: 1px solid var(--line);
  border-radius: 10px;
  color: var(--text);
  background: rgba(9, 18, 27, .86);
  text-align: left;
}
.target-menu {
  position: absolute;
  z-index: 30;
  top: calc(100% + 6px);
  left: 0;
  width: min(520px, 90vw);
  padding: 10px;
  border: 1px solid var(--line);
  border-radius: 12px;
  background: #0d1924;
  box-shadow: 0 16px 38px rgba(0, 0, 0, .38);
}
.target-menu[hidden] { display: none; }
.target-menu-toolbar { display: flex; gap: 8px; }
#target-search { flex: 1; min-width: 0; }
.target-options { max-height: 280px; margin-top: 8px; overflow-y: auto; }
.target-option {
  display: flex;
  align-items: center;
  gap: 9px;
  padding: 8px 9px;
  border-radius: 8px;
  cursor: pointer;
}
.target-option:hover { background: rgba(79, 140, 255, .12); }
.target-empty { margin: 10px 4px 2px; color: var(--muted); }
```

Replace free-form functions with:

```javascript
function setTargetMenuOpen(open) {
  byId("target-menu").hidden = !open;
  byId("target-picker-toggle").setAttribute("aria-expanded", String(open));
  if (open) {
    renderTargetOptions();
    byId("target-search").focus();
  }
}

function toggleTarget(name, checked) {
  const index = selectedTargets.indexOf(name);
  if (checked && index < 0) selectedTargets.push(name);
  if (!checked && index >= 0) selectedTargets.splice(index, 1);
  draftDirty = true;
  renderTargetSummary();
  renderTargetTags();
  renderTargetOptions();
}

function renderTargetSummary() {
  byId("target-picker-summary").textContent = selectedTargets.length
    ? `已选择 ${selectedTargets.length} 项`
    : "请选择物品";
}

function renderTargetOptions() {
  const query = byId("target-search").value.trim().toLocaleLowerCase();
  const matches = deployedClasses.filter(item =>
    !query || item.name.toLocaleLowerCase().includes(query)
  );
  byId("target-options").replaceChildren(...matches.map(item => {
    const label = document.createElement("label");
    label.className = "target-option";
    label.setAttribute("role", "option");
    const checked = selectedTargets.includes(item.name);
    label.setAttribute("aria-selected", String(checked));
    const input = document.createElement("input");
    input.type = "checkbox";
    input.checked = checked;
    input.addEventListener("change", () => toggleTarget(item.name, input.checked));
    const text = document.createElement("span");
    text.textContent = item.name;
    label.append(input, text);
    return label;
  }));
  byId("target-empty").hidden = matches.length > 0;
}

function renderTargetTags() {
  const host = byId("target-tags");
  host.replaceChildren(...selectedTargets.map(name => {
    const tag = document.createElement("span");
    tag.className = "target-tag";
    tag.append(document.createTextNode(name));
    const remove = document.createElement("button");
    remove.type = "button";
    remove.textContent = "×";
    remove.setAttribute("aria-label", `删除目标 ${name}`);
    remove.addEventListener("click", () => toggleTarget(name, false));
    tag.append(remove);
    return tag;
  }));
}
```

Wire toggle click, search input, clear-all, Escape, and outside-click events:

```javascript
byId("target-picker-toggle").addEventListener("click", () => {
  setTargetMenuOpen(byId("target-menu").hidden);
});
byId("target-search").addEventListener("input", renderTargetOptions);
byId("clear-targets").addEventListener("click", () => {
  selectedTargets.splice(0, selectedTargets.length);
  draftDirty = true;
  renderTargetSummary();
  renderTargetTags();
  renderTargetOptions();
});
document.addEventListener("keydown", event => {
  if (event.key === "Escape") setTargetMenuOpen(false);
});
document.addEventListener("click", event => {
  if (!byId("target-picker").contains(event.target)) setTargetMenuOpen(false);
});
```

After `loadClasses()` populates `deployedClasses`, call
`renderTargetSummary()`, `renderTargetTags()`, and `renderTargetOptions()`.
After `syncDraftFromStatus()` replaces `selectedTargets`, call the same three
functions. The existing `draftDirty` guard continues to prevent status polling
from overwriting an un-applied selection.

Change browser validation to `interval < 0.2` with the message
`请求周期必须在 0.2–60 秒之间`, and poll with:

```javascript
setInterval(refreshStatus, 200);
```

- [ ] **Step 5: Run Web tests and confirm GREEN**

Run:

```bash
pytest -q tests/test_four_camera_yolo_web.py
```

Expected: all Web/API/HTML/Node tests pass.

- [ ] **Step 6: Commit the dropdown UI**

```bash
git add scripts/inference/four_camera_yolo_web.html tests/test_four_camera_yolo_web.py
git commit -m "feat: add fast multi-select yolo controls"
```

### Task 4: Documentation, Full Verification, and Live Restart

**Files:**
- Modify: `scripts/inference/README.md:23-84`
- Test: `tests/test_four_camera_yolo_core.py`
- Test: `tests/test_four_camera_yolo_http.py`
- Test: `tests/test_four_camera_yolo_web.py`

**Interfaces:**
- Consumes: completed temporal, timing, and UI behavior.
- Produces: operator documentation and a verified live console on port 7788.

- [ ] **Step 1: Update operator documentation**

Replace the operating paragraphs with text that states:

```markdown
页面从 `/classes` 加载已部署类别。点击“选择物品”打开可搜索的复选下拉菜单，
可同时选择多个类别；选好后点击“应用并开始”。默认每 0.2 秒发起一轮四路
推理，网页每 200 ms 拉取一次最新状态。每路相机使用独立且稳定的
`stream_id` 启用时序高召回阈值；同一路最多保留一个在途请求，服务变慢时会
跳过中间轮次，不会积压图片。

推理周期可在 0.2–60 秒之间调整。默认启动命令：

    cd /home/caizj/agilex_idata/.worktrees/four-camera-yolo-web
    bash scripts/inference/run_four_camera_yolo_web.sh
```

- [ ] **Step 2: Run static and complete automated verification**

Run:

```bash
bash -n scripts/inference/run_four_camera_yolo_web.sh
python3 -m py_compile scripts/inference/four_camera_yolo_core.py scripts/inference/four_camera_yolo_web.py
pytest -q tests/test_four_camera_yolo_core.py tests/test_four_camera_yolo_http.py tests/test_four_camera_yolo_web.py
git diff --check
```

Expected: shell/Python checks succeed, every test passes, and no whitespace errors.

- [ ] **Step 3: Commit documentation**

```bash
git add scripts/inference/README.md
git commit -m "docs: describe fast four-camera yolo controls"
```

- [ ] **Step 4: Restart and verify the live service**

Stop only the process whose command line points to this worktree, then relaunch
the same script detached with logs under `/tmp/four-camera-yolo-web.log`.

Verify:

```bash
curl --noproxy '*' -fsS http://127.0.0.1:7788/api/status
curl --noproxy '*' -fsS http://127.0.0.1:7788/api/classes
```

Apply Yakult at 0.2 seconds and verify status reports fresh nonzero temporal
results for at least one camera without request errors.

- [ ] **Step 5: Record final repository state**

Run:

```bash
git status --short --branch
git log -5 --oneline --decorate
```

Expected: clean `feature/four-camera-yolo-web` worktree with the feature commits at HEAD.
