# Four-Camera QC Autostart Design

## Goal

Start the selectable four-camera QC web launcher automatically for user
`caizj` on port `8001`. Swap the two launchers' defaults so the generic
three-camera-compatible launcher moves to `8012` and the specialized
four-camera launcher becomes the default service on `8001`.

## Current State

The enabled user service
`~/.config/systemd/user/collect-mobile-pipeline-qc-web.service` starts
`scripts/collection/collect_mobile_pipeline_qc_web.sh` and explicitly sets
`WEB_PORT=8001`. The generic launcher currently defaults to `8001`, while the
specialized four-camera launcher defaults to `8012`. User lingering is enabled,
so the user service manager starts at boot without requiring an interactive
login.

## Launcher Defaults

Change `collect_mobile_pipeline_qc_web.sh` to default to port `8012`, and change
`collect_mobile_pipeline_qc_web_four_camera.sh` to default to port `8001`.
Explicit `WEB_PORT` values and `--port` arguments remain authoritative, so
existing callers can still select another port.

Update the current user-facing README files and active launcher comments to
describe the exchanged defaults. Historical plans and specifications remain
unchanged because they document earlier decisions.

Add or update launcher tests before changing the scripts. The tests must cover
both sides of the swap and retain the four-camera launcher's default camera
count of `4`.

## Autostart Service

Change only the service's `ExecStart` value to:

```text
/home/caizj/agilex_idata/scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh
```

Keep `Environment=WEB_PORT=8001`. The four-camera launcher will therefore use
the existing URL while supplying its four-camera profile, topic configuration,
camera layout, and selectable-camera defaults to the generic launcher.

Do not create a second service or change application-level camera behavior.
This avoids duplicate processes and limits repository behavior changes to the
requested launcher port defaults.

## Activation and Failure Handling

After editing the unit, run `systemctl --user daemon-reload` and restart the
existing service. The current `Restart=on-failure` behavior remains unchanged.
If startup fails, inspect the user service status and journal; do not fall back
silently to the three-camera launcher.

## Verification

Verification must confirm all of the following:

1. The unit remains enabled and active.
2. The loaded `ExecStart` points to the four-camera launcher.
3. Port `8001` is listening after restart.
4. Launcher tests and shell syntax checks pass for both scripts.
5. The running web process environment contains the four-camera layout,
   four-camera profile, four-camera topic configuration, and camera count `4`.
6. The generic launcher reports default port `8012`, while the four-camera
   launcher reports default port `8001`.
7. No second QC service is introduced.
