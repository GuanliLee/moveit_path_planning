# YOLO Service Shell Aliases Design

## Goal

Provide four short Bash commands for managing the existing port-7788 YOLO Web
gateway without starting a second Python process or duplicating its runtime
arguments.

## Design

Create `/home/caizj/.bash_aliases` with these aliases:

- `yolo-start` starts `four-camera-yolo-web.service` through the user systemd
  manager.
- `yolo-stop` stops that service.
- `yolo-status` prints its status without opening a pager.
- `yolo-log` follows its user-journal log until interrupted with `Ctrl+C`.

The existing `/home/caizj/.bashrc` already sources `~/.bash_aliases`, so no
`.bashrc` change is required. Existing alias content, if any, must be preserved.

## Safety and Verification

All commands target only `four-camera-yolo-web.service`. Installation must not
stop or restart the running service. Verify the alias file with `bash -n`, load
it in a clean interactive Bash process, confirm all four aliases resolve to the
expected systemd commands, and use the status alias to confirm the service
remains active on port 7788.
