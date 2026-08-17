# Data Collection Save Success Topic Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish a retained ROS 2 Boolean `/data_collection/save_success` state that becomes true only after episode data and A/B/F review metadata are saved and the episode advances.

**Architecture:** Extend the existing embedded `StateMachineBridge` with a second reliable, transient-local Boolean publisher. Derive success from the collection HTTP status (`capture_running`, `quality_review_pending`, `last_saved_episode`, and `episode`) so both automatic and web-UI saves are observed, while an explicit capture-cycle arm suppresses stale startup status and still permits reused episode numbers.

**Tech Stack:** Bash, embedded Python 3, ROS 2 Humble `rclpy`, `std_msgs/msg/Bool`, pytest.

## Global Constraints

- Default topic: `/data_collection/save_success`.
- Topic type: `std_msgs/msg/Bool`.
- `true` means raw episode data and A/B/F review metadata succeeded and the controller advanced to the next episode.
- HDF5 conversion, QC, and LeRobot conversion are outside the success boundary.
- Review completion and episode advancement are written to status before conversion/QC scheduling.
- The publisher uses reliable, transient-local QoS with depth 1.
- Startup, a new capture cycle, saving/review, discard, error, bridge disable, and shutdown retain `false`.
- Existing `/data_collection/start` behavior must remain unchanged.

---

### Task 1: Add the configurable retained ROS topic interface

**Files:**
- Modify: `tests/test_collection_failure_ui.py`
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh:47-79`
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh:258-275`
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh:1451-1768`
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh:2294-2316`

**Interfaces:**
- Consumes: `DATA_COLLECTION_SAVE_SUCCESS_TOPIC` environment variable.
- Produces: `StateMachineBridge.publish_save_success(value: bool, repeats: int | None = None) -> None` and a reliable, transient-local `std_msgs/msg/Bool` publisher.

- [ ] **Step 1: Write the failing interface and QoS tests**

Append these focused tests to `tests/test_collection_failure_ui.py`:

```python
def test_staged_bridge_exposes_configurable_save_success_topic():
    text = STAGED_SCRIPT.read_text(encoding="utf-8")

    assert 'DATA_COLLECTION_SAVE_SUCCESS_TOPIC="${DATA_COLLECTION_SAVE_SUCCESS_TOPIC:-/data_collection/save_success}"' in text
    assert "DATA_COLLECTION_SAVE_SUCCESS_TOPIC=/data_collection/save_success" in text
    assert 'echo "  DATA_COLLECTION_SAVE_SUCCESS_TOPIC=${DATA_COLLECTION_SAVE_SUCCESS_TOPIC:-/data_collection/save_success}"' in text
    assert '"save_success_topic": save_success_topic' in text


def test_staged_bridge_creates_retained_save_success_bool_publisher():
    text = STAGED_SCRIPT.read_text(encoding="utf-8")
    bridge_block = text.split("class StateMachineBridge(Node):", 1)[1].split(
        "def wait_for_status",
        1,
    )[0]

    assert "save_success_qos = QoSProfile(depth=1)" in bridge_block
    assert "save_success_qos.reliability = ReliabilityPolicy.RELIABLE" in bridge_block
    assert "save_success_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL" in bridge_block
    assert "self.save_success_pub = self.create_publisher(Bool, save_success_topic, save_success_qos)" in bridge_block
    assert "def publish_save_success(self, value: bool, repeats: int | None = None) -> None:" in bridge_block
    assert "self.save_success_pub.publish(msg)" in bridge_block
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
pytest -q tests/test_collection_failure_ui.py -k save_success
```

Expected: both new tests fail because the environment variable, control metadata, publisher, and helper do not exist.

- [ ] **Step 3: Add the topic configuration and publisher**

In `collect_mobile_pipeline_web_staged.sh`, add the help/default/output entries:

```bash
  DATA_COLLECTION_SAVE_SUCCESS_TOPIC=/data_collection/save_success
```

```bash
DATA_COLLECTION_SAVE_SUCCESS_TOPIC="${DATA_COLLECTION_SAVE_SUCCESS_TOPIC:-/data_collection/save_success}"
```

```bash
echo "  DATA_COLLECTION_SAVE_SUCCESS_TOPIC=${DATA_COLLECTION_SAVE_SUCCESS_TOPIC:-/data_collection/save_success}"
```

Inside `start_state_machine_bridge`, resolve `save_success_topic`, pass it to both embedded Python programs, parse it as `save_success_topic`, and include it in the initial/control JSON:

```python
"save_success_topic": save_success_topic,
```

Create the second publisher and helper without changing `publish_ready`:

```python
save_success_qos = QoSProfile(depth=1)
save_success_qos.reliability = ReliabilityPolicy.RELIABLE
save_success_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
self.save_success_pub = self.create_publisher(Bool, save_success_topic, save_success_qos)
self.save_success_state: bool | None = None
```

```python
def publish_save_success(self, value: bool, repeats: int | None = None) -> None:
    value = bool(value)
    if self.save_success_state == value:
        return
    self.save_success_state = value
    msg = Bool()
    msg.data = value
    count = publish_repeats if repeats is None else max(1, int(repeats))
    for _ in range(count):
        self.save_success_pub.publish(msg)
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except ExternalShutdownException:
            return
        time.sleep(0.05)
```

Publish initial `false` after creating the bridge and publish `false` in the `finally` block before destroying the node.

- [ ] **Step 4: Run the focused tests and shell parser**

Run:

```bash
pytest -q tests/test_collection_failure_ui.py -k save_success
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
```

Expected: focused tests pass and `bash -n` exits 0 without output.

- [ ] **Step 5: Commit the interface**

```bash
git add tests/test_collection_failure_ui.py scripts/collection/collect_mobile_pipeline_web_staged.sh
git commit -m "feat: add save success ROS publisher"
```

### Task 2: Drive save-success state from reviewed episode status

**Files:**
- Modify: `tests/test_collection_failure_ui.py`
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh:1789-2254`

**Interfaces:**
- Consumes: collection `/status` fields `capture_running: bool`, `quality_review_pending: bool`, `last_saved_episode: str`, and `episode: int`.
- Produces: `reviewed_save_success_episode(status: dict) -> str`; returns the saved episode number only when review is complete and the controller has advanced, otherwise `""`.
- Produces: `save_success_transition(status: dict, *, baseline_initialized: bool, armed: bool) -> tuple[bool, bool, bool | None]`; returns updated baseline/arm state and an optional Boolean publication.

**Code-review amendment:** Cycle arming supersedes episode-only deduplication in the original steps below. The first available status establishes a false baseline; capture/review arms one success; discard disarms false; a reviewed-and-advanced status consumes the arm and publishes true. `collect_mobile_episode_web.sh` must write its advanced review status before calling `start_saved_episode_processing` so conversion/QC cannot delay this signal.

- [ ] **Step 1: Write failing success-boundary and transition tests**

Append:

```python
def test_save_success_requires_review_completion_and_episode_advance():
    text = STAGED_SCRIPT.read_text(encoding="utf-8")
    helper_block = text.split("def reviewed_save_success_episode(status: dict) -> str:", 1)[1].split(
        "def collection_config_payload",
        1,
    )[0]

    assert 'status.get("capture_running")' in helper_block
    assert 'status.get("quality_review_pending")' in helper_block
    assert 'status.get("last_saved_episode", "")' in helper_block
    assert 'status.get("episode", "")' in helper_block
    assert "int(current_episode) <= int(saved_episode)" in helper_block
    assert "return saved_episode" in helper_block


def test_save_success_is_reset_for_new_cycles_and_only_set_for_new_reviewed_episode():
    text = STAGED_SCRIPT.read_text(encoding="utf-8")
    main_block = text.split("def main() -> int:", 1)[1].split(
        'if __name__ == "__main__":',
        1,
    )[0]

    assert "save_success_baseline_initialized" in main_block
    assert "save_success_armed" in main_block
    assert "def observe_collection_status(status: dict) -> None:" in main_block
    assert "node.publish_save_success(False)" in main_block
    assert "node.publish_save_success(publish_value)" in main_block
    assert "observe_collection_status(http.status())" in main_block
    assert "observe_collection_status(saved_status)" in main_block
```

- [ ] **Step 2: Run the focused transition tests and verify RED**

Run:

```bash
pytest -q tests/test_collection_failure_ui.py -k 'save_success_requires or save_success_is_reset'
```

Expected: both tests fail because the status predicate and observer do not exist.

- [ ] **Step 3: Implement the reviewed-save predicate**

Add beside `save_review_finished`:

```python
def reviewed_save_success_episode(status: dict) -> str:
    if bool(status.get("capture_running")) or bool(status.get("quality_review_pending")):
        return ""
    saved_episode = str(status.get("last_saved_episode", "")).strip()
    current_episode = str(status.get("episode", "")).strip()
    if not saved_episode.isdigit() or not current_episode.isdigit():
        return ""
    if int(current_episode) <= int(saved_episode):
        return ""
    return saved_episode
```

Add the cycle transition beside it:

```python
def save_success_transition(
    status: dict,
    *,
    baseline_initialized: bool,
    armed: bool,
) -> tuple[bool, bool, bool | None]:
    active_cycle = bool(status.get("capture_running")) or bool(status.get("quality_review_pending"))
    if not baseline_initialized:
        return True, active_cycle, False
    if active_cycle:
        return True, True, False
    if armed and reviewed_save_success_episode(status):
        return True, False, True
    if not str(status.get("last_saved_episode", "")).strip():
        return True, False, False
    return True, armed, None
```

- [ ] **Step 4: Implement state observation and transitions**

In `main`, initialize the stale-success baseline and observer:

```python
try:
    initial_collection_status = http.status()
except Exception as exc:  # noqa: BLE001
    log(f"initial collection status unavailable: {exc}")
    initial_collection_status = None
save_success_baseline_initialized = initial_collection_status is not None
save_success_armed = bool(
    initial_collection_status
    and (
        initial_collection_status.get("capture_running")
        or initial_collection_status.get("quality_review_pending")
    )
)
last_status_poll_at = 0.0

def observe_collection_status(status: dict) -> None:
    nonlocal save_success_baseline_initialized, save_success_armed
    save_success_baseline_initialized, save_success_armed, publish_value = save_success_transition(
        status,
        baseline_initialized=save_success_baseline_initialized,
        armed=save_success_armed,
    )
    if publish_value is not None:
        node.publish_save_success(publish_value)
```

Reset to `false` at the beginning of `start_recording` and `save_current_episode`, when automation is disabled, and in every start/save error path. In the main loop, poll local collection status at `poll_interval` so web-UI saves are observed:

```python
now = time.monotonic()
if now - last_status_poll_at >= max(0.05, poll_interval):
    last_status_poll_at = now
    try:
        observe_collection_status(http.status())
    except Exception as exc:  # noqa: BLE001
        log(f"collection status poll failed: {exc}")
        node.publish_save_success(False)
```

After automatic saving returns, call:

```python
observe_collection_status(saved_status)
```

A discard has no episode advance and therefore remains false. A pre-existing success establishes the baseline and is not republished; a new capture cycle can publish success even when its episode number was reused.

- [ ] **Step 5: Run focused and full verification**

Run:

```bash
pytest -q tests/test_collection_failure_ui.py
bash -n scripts/collection/collect_mobile_pipeline_web_staged.sh
pytest -q
git diff --check
```

Expected: all tests pass, shell syntax exits 0, and `git diff --check` produces no output.

- [ ] **Step 6: Commit the state behavior**

```bash
git add tests/test_collection_failure_ui.py scripts/collection/collect_mobile_pipeline_web_staged.sh
git commit -m "feat: publish reviewed episode save success"
```
