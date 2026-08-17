# Four-Camera QC Autostart Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Exchange the generic and four-camera QC launchers' default ports, then make the existing enabled user service start the four-camera launcher on port `8001` at boot.

**Architecture:** Keep one existing user-level systemd service and change only its executable entry point. Keep camera selection inside the specialized wrapper, while the shared generic launcher remains three-camera compatible and moves to default port `8012`.

**Tech Stack:** Bash, pytest, systemd user services, Markdown documentation.

## Global Constraints

- The four-camera launcher and autostart service use port `8001`.
- The generic `collect_mobile_pipeline_qc_web.sh` launcher defaults to port `8012`.
- Explicit `WEB_PORT` and `--port` values continue to override launcher defaults.
- Do not create a second QC service or alter application-level camera behavior.
- Preserve all pre-existing deleted test files in the working tree; do not restore, stage, or commit them.

---

### Task 1: Exchange Launcher Defaults

**Files:**
- Modify: `tests/test_selectable_camera_qc.py:129-139`
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web.sh:14`
- Modify: `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh:15`
- Modify: `README.md:34-51`
- Modify: `scripts/embodied_data_pipeline-main/README.md:114-154`
- Modify: `scripts/collection/collect_mobile_pipeline_web_staged.sh:13-14,104`
- Modify: `scripts/collection/collect_mobile_pipeline_web_inference.sh:13-14,100`

**Interfaces:**
- Consumes: the existing launchers' `WEB_PORT="${WEB_PORT:-...}"` contract.
- Produces: generic default port `8012`, specialized default port `8001`, and unchanged four-camera environment defaults.

- [ ] **Step 1: Change the launcher characterization test first**

Replace the current default-port test with:

```python
def test_launcher_default_ports_are_swapped_and_four_cameras_remain_default():
    generic_text = GENERIC_ENTRY.read_text(encoding="utf-8")
    selectable_text = SELECTABLE_ENTRY.read_text(encoding="utf-8")
    generic_result = subprocess.run(
        ["bash", str(GENERIC_ENTRY), "--help"],
        check=False,
        capture_output=True,
        text=True,
        env=os.environ.copy(),
    )
    selectable_result = run_launcher("--help")

    assert generic_result.returncode == 0
    assert selectable_result.returncode == 0
    assert "Default: 8012" in generic_result.stdout
    assert 'WEB_PORT="${WEB_PORT:-8012}"' in generic_text
    assert "Default: 8001" in selectable_result.stdout
    assert 'WEB_PORT="${WEB_PORT:-8001}"' in selectable_text
    assert 'PIPELINE_CAMERA_VARIANT_SELECTABLE="${PIPELINE_CAMERA_VARIANT_SELECTABLE:-1}"' in selectable_text
    assert 'PIPELINE_CAMERA_COUNT="${PIPELINE_CAMERA_COUNT:-4}"' in selectable_text
    assert 'PIPELINE_HEAD_CAMERA_SOURCE="${PIPELINE_HEAD_CAMERA_SOURCE:-front}"' in selectable_text
    assert "PIPELINE_OUTPUT_NAMESPACE" not in selectable_text
```

- [ ] **Step 2: Run the test and confirm the red state**

Run:

```bash
python3 -m pytest -q tests/test_selectable_camera_qc.py -k launcher_default_ports
```

Expected: FAIL because the generic help reports `8001` and the selectable launcher reports `8012`.

- [ ] **Step 3: Exchange the two Bash defaults**

Set the generic launcher line to:

```bash
WEB_PORT="${WEB_PORT:-8012}"
```

Set the specialized launcher line to:

```bash
export WEB_PORT="${WEB_PORT:-8001}"
```

- [ ] **Step 4: Update active documentation and launcher comments**

Document that `collect_mobile_pipeline_qc_web_four_camera.sh` is the default four-camera service on `8001`, and that `collect_mobile_pipeline_qc_web.sh` is the generic three-camera-compatible entry on `8012`. Change the occupied-port examples for the specialized launcher from `8012` to `8001` while retaining the alternate example port `18012`.

In both unified launcher comments and help text, use:

```text
standalone four-camera QC page: collect_mobile_pipeline_qc_web_four_camera.sh on port 8001
```

- [ ] **Step 5: Verify tests, syntax, docs, and explicit overrides**

Run:

```bash
python3 -m pytest -q tests/test_selectable_camera_qc.py
bash -n scripts/collection/collect_mobile_pipeline_qc_web.sh scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh scripts/collection/collect_mobile_pipeline_web_staged.sh scripts/collection/collect_mobile_pipeline_web_inference.sh
WEB_PORT=18012 bash scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh --help | rg 'Default: 18012'
bash scripts/collection/collect_mobile_pipeline_qc_web.sh --port 18013 --help | rg 'Default: 18013'
rg -n '8001|8012' README.md scripts/embodied_data_pipeline-main/README.md scripts/collection/collect_mobile_pipeline_qc_web.sh scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh
git diff --check
```

Expected: pytest passes, all shell syntax checks exit `0`, both override checks match once, active documentation describes the exchanged defaults, and `git diff --check` prints nothing.

- [ ] **Step 6: Commit the repository changes without staging deleted tests**

```bash
git add README.md scripts/embodied_data_pipeline-main/README.md \
  scripts/collection/collect_mobile_pipeline_qc_web.sh \
  scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh \
  scripts/collection/collect_mobile_pipeline_web_staged.sh \
  scripts/collection/collect_mobile_pipeline_web_inference.sh \
  tests/test_selectable_camera_qc.py
git commit -m "fix: make four-camera QC the default web service"
```

Expected: the commit contains only the seven listed repository files.

### Task 2: Switch and Activate the Existing User Service

**Files:**
- Modify: `/home/caizj/.config/systemd/user/collect-mobile-pipeline-qc-web.service:10`

**Interfaces:**
- Consumes: `collect_mobile_pipeline_qc_web_four_camera.sh`, which exports the four-camera environment and delegates to the generic launcher.
- Produces: enabled user service `collect-mobile-pipeline-qc-web.service` running the four-camera web console on `0.0.0.0:8001`.

- [ ] **Step 1: Prove the current unit still points to the old launcher**

Run:

```bash
systemctl --user show collect-mobile-pipeline-qc-web.service --property=ExecStart --value | rg 'collect_mobile_pipeline_qc_web_four_camera\.sh'
```

Expected: FAIL with no match because the loaded unit still references `collect_mobile_pipeline_qc_web.sh`.

- [ ] **Step 2: Change the service entry point**

Set the unit line to:

```ini
ExecStart=/home/caizj/agilex_idata/scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh
```

Retain `Environment=WEB_PORT=8001`, `Restart=on-failure`, and `WantedBy=default.target` unchanged.

- [ ] **Step 3: Validate, reload, and restart the service**

Run:

```bash
systemd-analyze --user verify /home/caizj/.config/systemd/user/collect-mobile-pipeline-qc-web.service
systemctl --user daemon-reload
systemctl --user restart collect-mobile-pipeline-qc-web.service
```

Expected: unit verification and reload exit `0`; restart returns without an error.

- [ ] **Step 4: Wait for the web endpoint and verify service state**

Run:

```bash
for attempt in $(seq 1 30); do
  curl -fsS http://127.0.0.1:8001/ >/dev/null && break
  sleep 1
done
curl -fsS http://127.0.0.1:8001/ >/dev/null
systemctl --user is-enabled collect-mobile-pipeline-qc-web.service
systemctl --user is-active collect-mobile-pipeline-qc-web.service
systemctl --user show collect-mobile-pipeline-qc-web.service --property=ExecStart --value | rg 'collect_mobile_pipeline_qc_web_four_camera\.sh'
ss -ltnp '( sport = :8001 )'
```

Expected: endpoint succeeds, service reports `enabled` and `active`, `ExecStart` matches the specialized launcher, and one listener exists on `0.0.0.0:8001`.

- [ ] **Step 5: Verify the four-camera environment and boot persistence**

Run:

```bash
main_pid=$(systemctl --user show collect-mobile-pipeline-qc-web.service --property=MainPID --value)
tr '\0' '\n' < "/proc/${main_pid}/environ" | rg '^PIPELINE_(CAMERA_LAYOUT=four_camera|CAMERA_COUNT=4|CAMERA_VARIANT_SELECTABLE=1|DEFAULT_PROFILE=.*/aloha_four_camera\.yaml|ALOHA_YAML=.*/aloha_four_camera_data_params\.yaml)$'
loginctl show-user caizj -p Linger
systemctl --user status collect-mobile-pipeline-qc-web.service --no-pager -l
```

Expected: all five four-camera environment entries match, `Linger=yes`, and status shows the service running the four-camera launcher on port `8001`.
