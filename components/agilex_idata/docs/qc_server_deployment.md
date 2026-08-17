# 质检系统通过 GitHub 部署到 H200

本文只部署独立质检/转换网页，不启动机器人采集、状态机或真机回放。

## 部署边界

- GitHub 保存代码，不保存 `data/`、MCAP、HDF5、视频、模型权重或 `.env`；这些内容已被 `.gitignore` 排除。
- 跨平台 LeRobot 质检、可视化回放和纯 HDF5 质检只需安装 Python 依赖，不要求 ROS2。
- H200 默认扫描 `/srv/data/datasets/public` 的直属子目录；网页左栏机器选择 `H200` 后使用该扫描规则。
- MCAP 转 HDF5 还需要 ROS2 `rosbag2_py`，或服务器上可用的 `data-tools-ros2:jazzy` Docker 镜像。
- 当前仓库没有构建 `data-tools-ros2:jazzy` 的 Dockerfile，也没有可验证的镜像仓库地址。若目标服务器需要 MCAP 转换，应先把该镜像发布到内部 Registry，或从已有机器执行 `docker save` / `docker load` 迁移。
- 网页没有用户登录和权限控制，并且可以删除 episode、写入数据目录及启动转换子进程。生产环境应只监听 `127.0.0.1`，通过 VPN、反向代理鉴权或 SSH 隧道访问，不能直接暴露到公网。

## 1. 为私有仓库配置只读访问

推荐为目标服务器创建单独的只读 Deploy Key：

```bash
ssh-keygen -t ed25519 -f ~/.ssh/agilex_idata_deploy -C "agilex-idata-deploy"
```

把 `~/.ssh/agilex_idata_deploy.pub` 添加到 GitHub 仓库的 `Settings -> Deploy keys`，不要勾选写权限。随后配置 SSH 使用该密钥，或在克隆时临时指定：

```bash
GIT_SSH_COMMAND='ssh -i ~/.ssh/agilex_idata_deploy -o IdentitiesOnly=yes' \
git clone --branch agent/qc-server-backup-20260807 --single-branch \
  git@github.com:Edgardcai/agilex_idata.git /srv/agilex_idata
```

也可以使用有仓库读取权限的机器用户，但不要把个人 Token 写入仓库、脚本或 systemd unit。

## 2. 安装 Python 环境

以下示例使用 Python 3.10 或更高版本：

```bash
cd /srv/agilex_idata
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip setuptools wheel
.venv/bin/python -m pip install \
  -r scripts/embodied_data_pipeline-main/quality_pipeline/requirements.txt
```

确认 H200 数据目录存在。分析和回放只需要读取权限；生成“审核后数据集”还需要对输出父目录的写权限：

```bash
sudo install -d /srv/data/datasets/public
sudo chown -R "$USER":"$USER" /srv/data/datasets/public
```

如果生产数据不能整体更改属主，请改用 ACL 只授权实际运行服务的账号，不要执行上面的 `chown -R`。

## 3. 首次启动验证

跨平台 LeRobot 质检、回放和 HDF5 质检可直接启动；没有 `rosbag2_py` 时启动器只会提示 MCAP 转换不可用：

```bash
cd /srv/agilex_idata
PIPELINE_DATA_SCAN_ROOT=/srv/data/datasets/public \
PIPELINE_H200_DATA_SCAN_ROOT=/srv/data/datasets/public \
bash scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh \
  --host 127.0.0.1 \
  --port 8001 \
  --python /srv/agilex_idata/.venv/bin/python \
  --data-root /srv/data/datasets/public
```

启动后打开 `http://127.0.0.1:8001/`，切换到“跨平台 LeRobot 质检与回放”，左栏机器选择 `H200`。健康检查：

```bash
curl -f http://127.0.0.1:8001/
curl -f http://127.0.0.1:8001/cross-platform/
```

若要在页面中用 Docker 转换 MCAP，先确认目标机能看到镜像：

```bash
docker image inspect data-tools-ros2:jazzy
```

然后在启动环境中增加：

```bash
PIPELINE_DEFAULT_USE_DOCKER=1
```

服务只监听本机时，可从工作电脑建立隧道：

```bash
ssh -L 8001:127.0.0.1:8001 <服务器用户>@<服务器IP>
```

浏览器打开 `http://127.0.0.1:8001/`。

若只在可信局域网内直接访问，可把 `--host 127.0.0.1` 改为 `--host 0.0.0.0`，然后访问 `http://<H200局域网IP>:8001/`。此网页没有登录鉴权，防火墙必须把 8001 端口限制在可信网段，不能暴露到公网。

## 4. 配置 systemd

将下面内容保存为 `/etc/systemd/system/agilex-qc.service`，并把 `User`、`Group` 按目标服务器修改：

```ini
[Unit]
Description=AgileX data quality-control web service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=qc
Group=qc
WorkingDirectory=/srv/agilex_idata
Environment=PIPELINE_DATA_SCAN_ROOT=/srv/data/datasets/public
Environment=PIPELINE_H200_DATA_SCAN_ROOT=/srv/data/datasets/public
Environment=PIPELINE_DEFAULT_USE_DOCKER=0
ExecStart=/usr/bin/bash /srv/agilex_idata/scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh --host 127.0.0.1 --port 8001 --python /srv/agilex_idata/.venv/bin/python --data-root /srv/data/datasets/public
Restart=on-failure
RestartSec=5
TimeoutStopSec=30

[Install]
WantedBy=multi-user.target
```

如果启用 Docker 转换，把 `PIPELINE_DEFAULT_USE_DOCKER` 改为 `1`，并确保服务用户能访问 Docker daemon。Docker 组通常等同于主机 root 权限，需按服务器安全策略审批。

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now agilex-qc
sudo systemctl status agilex-qc
journalctl -u agilex-qc -f
```

## 5. 从 GitHub 升级与回滚

记录当前版本，然后只做快进更新：

```bash
cd /srv/agilex_idata
git rev-parse HEAD
git pull --ff-only origin agent/qc-server-backup-20260807
.venv/bin/python -m pip install \
  -r scripts/embodied_data_pipeline-main/quality_pipeline/requirements.txt
sudo systemctl restart agilex-qc
```

部署前记录的 commit ID 就是回滚点。若新版本失败，切回该 commit 并重启：

```bash
cd /srv/agilex_idata
git switch --detach <部署前的commit-id>
sudo systemctl restart agilex-qc
```

长期使用时，建议把验证通过的版本打 Git tag，并让生产服务器部署 tag 或固定 commit，而不是自动跟随持续变化的开发分支。
