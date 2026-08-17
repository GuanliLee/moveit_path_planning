# YOLO Service Shell Aliases Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add four short Bash aliases that manage the existing port-7788 YOLO Web gateway systemd user service.

**Architecture:** Keep systemd as the only service lifecycle manager and add a user-local Bash alias file as a thin command layer. Do not duplicate Python launch arguments, change the service unit, or restart the running service during installation.

**Tech Stack:** Bash aliases, systemd user services, journald

## Global Constraints

- All aliases target only `four-camera-yolo-web.service`.
- Preserve any pre-existing `/home/caizj/.bash_aliases` content.
- Do not stop or restart the running service while installing aliases.
- `/home/caizj/.bashrc` remains unchanged because it already sources `~/.bash_aliases`.

---

### Task 1: Install and Verify YOLO Service Aliases

**Files:**
- Create or modify: `/home/caizj/.bash_aliases`
- Verify: `/home/caizj/.bashrc`

**Interfaces:**
- Consumes: systemd user unit `four-camera-yolo-web.service`
- Produces: interactive Bash commands `yolo-start`, `yolo-stop`, `yolo-status`, and `yolo-log`

- [ ] **Step 1: Record the active service PID and verify the aliases are absent**

Run:

```bash
systemctl --user show four-camera-yolo-web.service -p ActiveState -p MainPID
bash --noprofile --norc -ic 'source /home/caizj/.bash_aliases 2>/dev/null || true; alias yolo-start'
```

Expected: the service reports `ActiveState=active` and a non-zero `MainPID`; the alias lookup exits non-zero with `alias: yolo-start: not found`.

- [ ] **Step 2: Add the exact alias definitions**

Create `/home/caizj/.bash_aliases` with:

```bash
# Four-camera YOLO Web service (port 7788)
alias yolo-start='systemctl --user start four-camera-yolo-web.service'
alias yolo-stop='systemctl --user stop four-camera-yolo-web.service'
alias yolo-status='systemctl --user status --no-pager four-camera-yolo-web.service'
alias yolo-log='journalctl --user -u four-camera-yolo-web.service -f'
```

If the file already exists, append this managed block without modifying unrelated aliases.

- [ ] **Step 3: Check Bash syntax and alias expansion**

Run:

```bash
bash -n /home/caizj/.bash_aliases
bash --noprofile --norc -ic 'source /home/caizj/.bash_aliases; alias yolo-start; alias yolo-stop; alias yolo-status; alias yolo-log'
```

Expected: syntax checking exits zero and all four alias definitions print their exact systemd or journal command.

- [ ] **Step 4: Verify status through the new alias without changing the service**

Run:

```bash
bash --noprofile --norc -ic 'source /home/caizj/.bash_aliases; eval yolo-status'
systemctl --user show four-camera-yolo-web.service -p ActiveState -p MainPID
ss -ltnp 'sport = :7788'
```

Expected: `yolo-status` reports `active (running)`, `ActiveState=active`, the `MainPID` matches Step 1, and the same PID listens on `0.0.0.0:7788`.

- [ ] **Step 5: Document immediate activation for the current terminal**

Run in the user's existing interactive terminal:

```bash
source ~/.bash_aliases
```

Expected: future commands in that terminal recognize all four aliases; newly opened Bash terminals load them through the existing `.bashrc` hook.
