# YOLO No-Mask Response Compatibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore live four-camera results using the previous box-based output while ignoring optional mask information.

**Architecture:** `YoloHttpClient.predict` remains the only production-code boundary that changes. The existing request continues to omit `mask`; response validation uses finite, positive-area `bbox_xyxy` geometry and ignores optional mask fields. Downstream state, class counts, overlay rendering, and HTTP routes remain unchanged.

**Tech Stack:** Python 3.10, standard-library HTTP/JSON, pytest, ROS 2 Humble, shell-managed production process on port 7788.

## Global Constraints

- Preserve existing `targets`, temporal stream state, and `return_overlay: true` behavior.
- Do not add a `mask` request field.
- Accept detections when `bbox_xyxy` is a finite four-number positive-area box.
- Ignore optional mask fields and continue rejecting invalid bbox geometry.
- Do not change the production YOLO service on `192.168.4.121:7881`.

---

### Task 1: Add protocol regression coverage

**Files:**
- Modify: `tests/test_four_camera_yolo_http.py:60-235`

**Interfaces:**
- Consumes: `YoloHttpClient.predict(camera, frame, targets) -> InferenceResult`
- Produces: regression fixtures for default bbox-only responses

- [x] **Step 1: Make the fake default response bbox-only**

Omit mask keys, include valid `bbox_xyxy`, and add missing/invalid bbox modes plus a malformed optional-mask mode.

```python
"bbox_xyxy": [1.0, 1.0, 20.0, 20.0]
```

- [x] **Step 2: Assert no-mask requests and bbox-only compatibility**

Add the request assertion to `test_health_classes_and_predict_use_direct_json_api`:

```python
assert "mask" not in request
```

Add the focused regression test:

```python
def test_predict_accepts_default_bbox_only_response(fake_yolo, core):
    result = core.YoloHttpClient(fake_yolo.url, 1.0).predict(
        "left",
        core.FrameSnapshot(fake_jpeg(640, 480), 1, 1, 1.0),
        ("Wanglaoji",),
    )
    assert result.count == 1
```

Add missing and invalid bbox cases to the malformed-response parameter table, and verify malformed optional mask geometry is ignored.

- [x] **Step 3: Run the focused tests and verify RED**

Run:

```bash
pytest -q \
  tests/test_four_camera_yolo_http.py::test_health_classes_and_predict_use_direct_json_api \
  tests/test_four_camera_yolo_http.py::test_predict_accepts_default_bbox_only_response
```

Expected: bbox-only responses fail against production code with `mask_polygon_xy is missing or invalid`.

---

### Task 2: Implement the minimal compatibility change

**Files:**
- Modify: `scripts/inference/four_camera_yolo_core.py:217-288`
- Test: `tests/test_four_camera_yolo_http.py`

**Interfaces:**
- Consumes: YOLO detection dictionaries containing `bbox_xyxy` and optional mask fields
- Produces: validated `InferenceResult` while preserving the remote overlay and counts

- [x] **Step 1: Keep the default no-mask request**

Do not add `mask` to `request_payload`; preserve all existing request keys.

- [x] **Step 2: Validate bbox geometry and ignore optional masks**

Replace polygon validation with:

```python
bbox = detection.get("bbox_xyxy")
bbox_is_valid = (
    isinstance(bbox, (list, tuple))
    and len(bbox) == 4
    and all(
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
        for value in bbox
    )
    and bbox[2] > bbox[0]
    and bbox[3] > bbox[1]
)
```

Reject when `bbox_is_valid` is false. Do not read or validate mask fields.

- [x] **Step 3: Run the focused tests and verify GREEN**

Run:

```bash
pytest -q tests/test_four_camera_yolo_http.py
```

Expected: all tests pass, including missing/invalid bbox rejection and optional-mask ignoring.

- [x] **Step 4: Run the related regression suites**

Run:

```bash
pytest -q \
  tests/test_four_camera_yolo_http.py \
  tests/test_four_camera_yolo_core.py \
  tests/test_four_camera_yolo_web.py
```

Expected: all tests pass with no errors or warnings.

- [x] **Step 5: Commit the code and tests**

```bash
git add scripts/inference/four_camera_yolo_core.py tests/test_four_camera_yolo_http.py
git commit -m "fix: accept default bbox-only yolo responses"
```

---

### Task 3: Restart and verify the production gateway

**Files:**
- Runtime log: `/tmp/four-camera-yolo-web.log`
- Entrypoint: `scripts/inference/run_four_camera_yolo_web.sh`

**Interfaces:**
- Consumes: production YOLO `http://192.168.4.121:7881` and ROS domain 99 camera topics
- Produces: HTTP service `0.0.0.0:7788`

- [x] **Step 1: Snapshot the live gateway configuration**

Read `/api/status` and retain `targets`, `interval_sec`, `paused`, and per-camera enabled flags so restart does not change the user's selection.

- [x] **Step 2: Gracefully replace the current listener**

Send `SIGTERM` to the PID listening on 7788, wait until the port is free, then start:

```bash
nohup bash scripts/inference/run_four_camera_yolo_web.sh \
  >>/tmp/four-camera-yolo-web.log 2>&1 </dev/null &
```

Wait until `/api/status` returns HTTP 200, then POST the saved configuration to `/api/config`.

- [x] **Step 3: Verify live results, not only process health**

Poll `/api/status` for at least 10 seconds and require for every enabled camera:

```text
topic_online == true
result_stale == false
error == ""
result_sequence increases between samples
```

Also require `/api/cameras/{left,front,right,head}/overlay.jpg` to return HTTP 200 with non-zero JPEG bytes.

- [x] **Step 4: Confirm upstream and local process health**

Verify YOLO `/health` returns HTTP 200, 7788 has exactly one listening PID, the new process remains alive, and `/tmp/four-camera-yolo-web.log` contains no traceback or protocol error after restart.
