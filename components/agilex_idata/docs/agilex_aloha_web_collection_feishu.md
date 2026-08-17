# AgileX ALOHA 网页数据采集教程

适用入口：

```bash
/home/caizj/agilex_idata/scripts/collection/collect_mobile_pipeline_web_staged.sh
```

该网页包含三个模块：

| 页面 | 用途 |
|---|---|
| 数据采集 | 采集 MCAP，支持双臂、底盘、升降柱、分阶段、自检、急停 |
| 质量检查与转换 | MCAP 转 HDF5，QC，修复，删除，生成 LeRobot |
| 数据回放与真机回放 | HDF5/LeRobot 可视化回放，真机轨迹回放，停止回放 |

---

## 1. 启动网页

进入代码目录：

```bash
cd /home/caizj/agilex_idata
```

推荐按数据集路径启动：

```bash
DATA_ROS_WS=/home/caizj/agilex_idata/ros2_ws \
bash scripts/collection/collect_mobile_pipeline_web_staged.sh /home/agilex/data/market_5
```

如果要从指定 episode 开始：

```bash
DATA_ROS_WS=/home/caizj/agilex_idata/ros2_ws \
bash scripts/collection/collect_mobile_pipeline_web_staged.sh /home/agilex/data/market_5 3
```

| 参数 | 说明 |
|---|---|
| `/home/agilex/data/market_5` | 本次数据集目录，网页刷新后默认也使用这个目录 |
| `3` | 起始 episode index，不填默认从 `0` 开始 |
| `DATA_ROS_WS` | 数据采集 ROS2 workspace |

启动成功后，终端会打印访问地址：

```text
http://<robot-ip>:8000/
```

---

## 2. 采集前准备

采集前确认以下节点已经启动：

1. 三路相机。
2. 双 Piper 主从臂。
3. 底盘。
4. 升降柱。
5. ROS2 网络正常。

进入网页后，先进入 `数据采集` 页，点击 `启动自检`。

自检会检查相机、主臂、从臂、底盘、升降柱和 action topic。自检通过后再开始采集。

---

## 3. 数据采集页面配置

主要字段：

| 字段 | 说明 |
|---|---|
| 数据目录 | 当前数据集目录，例如 `/home/agilex/data/market_5` |
| 起始/当前 episode | 当前采集编号 |
| 阶段 preset | 分阶段采集使用的 preset json |
| LeRobot 数据集名 | 生成 LeRobot 时使用的数据集名 |
| 瓶子 A | 任务目标 A |
| 瓶子 B | 任务目标 B |

常用选项：

| 选项 | 建议 | 说明 |
|---|---|---|
| 带底盘 | 移动底盘任务勾选 | 勾选后使用 mobile 配置，采集双臂、相机、底盘、升降柱 |
| 分阶段 | 需要阶段边界时勾选 | 每个 episode 内记录阶段切换点 |
| 阶段完成自动保存 | 按 preset 自动结束时勾选 | 阶段 preset 完成后自动保存 |
| 转 HDF5 | 建议勾选 | 保存 MCAP 后自动转 HDF5 |
| QC | 建议勾选 | HDF5 后自动质量检查 |
| 转 LeRobot | 需要训练数据时勾选 | 自动生成或更新 LeRobot 数据集 |
| 后台处理 | 连续采集时建议勾选 | 保存后转换、QC、LeRobot 在后台跑，可继续采下一条 |

---

## 4. 普通单段采集流程

适合不需要阶段边界的任务。

1. 进入 `数据采集` 页。
2. 确认 `数据目录` 正确。
3. 不勾选 `分阶段`。
4. 根据任务决定是否勾选 `带底盘`。
5. 勾选 `转 HDF5`、`QC`。
6. 如果需要直接产出训练数据，勾选 `转 LeRobot`。
7. 点击 `启动自检`。
8. 自检通过后点击 `开始采集`。
9. 操作机器人完成任务。
10. 点击 `结束采集并保存`。

保存后会生成：

```text
/home/agilex/data/market_5/episodeN/episodeN_0.mcap
/home/agilex/data/market_5/aloha/episodeN/episodeN.hdf5
/home/agilex/data/market_5/aloha/qc_reports/
/home/agilex/data/market_5/lerobot/
```

其中 `N` 是 episode 编号。

---

## 5. 分阶段采集流程

适合货架取物、放入购物车这类多阶段任务。

推荐阶段示例：

| 阶段 | 含义 |
|---|---|
| 阶段 1 | 靠近货架或目标区域 |
| 阶段 2 | 抓取目标物体 |
| 阶段 3 | 靠近购物车 |
| 阶段 4 | 放置物体 |
| 阶段 5 | 等待最终结束保存 |

操作步骤：

1. 进入 `数据采集` 页。
2. 勾选 `分阶段`。
3. 根据任务决定是否勾选 `带底盘`。
4. 勾选 `转 HDF5`、`QC`，按需勾选 `转 LeRobot`。
5. 点击 `启动自检`。
6. 点击 `开始采集`。
7. 完成当前阶段后，点击 `阶段切换`。
8. 重复阶段切换，直到任务完成。
9. 点击 `结束采集并保存`。

分阶段采集会额外记录阶段边界信息，供后续数据处理或训练使用。

---

## 6. 状态机自动监听采集

网页顶部有 `启用自动监听` 和 `分阶段`。

如果接了外部状态机，可以启用自动监听。

默认监听 topic：

```text
/state_machine/start
/state_machine/end
/state_machine/approaching_container/end
/state_machine/grasping/end
/state_machine/approaching_shopping_cart/end
/state_maching/put_back/end
/state_machine/put_back/end
```

行为：

| 模式 | 行为 |
|---|---|
| 不分阶段 | `/state_machine/start=true` 开始采集，`/state_machine/end=true` 保存 |
| 分阶段 | start 开始采集，阶段 topic 依次触发 `阶段切换`，end 保存 |

状态机开始后，网页会发布：

```text
/data_collection/start
```

用于通知外部流程当前数据采集已经准备好。

---

## 7. 放弃、急停、复位

| 按钮 | 作用 |
|---|---|
| 放弃 | 丢弃当前 episode，编号不增加，可重采 |
| 急停中断 | 中断当前采集或回放，丢弃当前 episode，并发送停止/保持命令 |
| 机械臂复位 | 调用 Piper 缓慢回零脚本 |
| 退出 | 退出网页服务 |

注意：

```text
急停中断是软件急停，不能替代实体急停。
真机异常运动时优先使用实体急停。
```

---

## 8. 数据转换与处理 Pipeline

进入网页第二个页签：`质量检查与转换`。

该页面用于：

1. MCAP 转 HDF5。
2. HDF5 质量检查。
3. 修复问题 episode。
4. 删除坏 episode。
5. 生成 LeRobot 数据集。
6. HDF5/LeRobot 可视化回放。

主要字段：

| 字段 | 说明 |
|---|---|
| MCAP 数据集路径 | 原始 MCAP 数据集目录，例如 `/home/agilex/data/market_5` |
| 数据集名 | 留空时通常从 MCAP 路径自动推导 |
| HDF5 路径 | 已有 HDF5 时填写；只有 MCAP 时可留空 |
| 质检报告根目录 | 留空时自动使用默认 qc_reports |
| LeRobot 输出根目录 | 留空时自动使用默认 lerobot 输出 |
| 机器人 | 选择 `aloha` |
| 并行转换/质检数 | 建议 `2` |
| 任务文本 | 写入 LeRobot 的 task，例如 `pick two bottles and place into cart` |
| Profile | 选择机器人后自动生成，通常不需要手改 |
| GPU 编号 | 有 GPU 可填 `0`，没有则留空 |
| Docker 转换/修复 | 根据现场环境选择，默认自动 |
| 覆盖 HDF5 | 需要重转时勾选 |
| ALOHA 包含升降/底盘 state-action | 移动底盘数据必须勾选 |

---

## 9. MCAP 转 HDF5 + QC

如果采集页已经勾选了 `转 HDF5` 和 `QC`，通常不需要手动再转。

如果需要手动处理：

1. 进入 `质量检查与转换`。
2. `MCAP 数据集路径` 填 `/home/agilex/data/market_5`。
3. `机器人` 选择 `aloha`。
4. 确认勾选 `ALOHA 包含升降/底盘 state-action（21维/18维）`。
5. 点击 `转换并质检`。

处理完成后检查右侧概览：

| 指标 | 说明 |
|---|---|
| Episode | 检测到的 episode 数 |
| 通过 | QC 通过数量 |
| 修复 | 需要修复数量 |
| 删除 | 标记删除数量 |
| LeRobot 编号 | 生成 LeRobot 时的编号映射 |

---

## 10. 仅批量质检

已有 HDF5，只想重新 QC：

1. `HDF5 路径` 填 HDF5 根目录。
2. `机器人` 选择 `aloha`。
3. 点击 `仅批量质检`。

适合以下场景：

1. 已经转换过 HDF5。
2. 只想重新检查质量。
3. 没有原始 MCAP 或不想重新转换。

---

## 11. 修复或删除 Episode

修复：

1. 在 `质检报告` 里选择需要修复的 episode。
2. 点击 `选择episode进行修复`。
3. 等待修复和复检完成。

删除：

1. 在 `质检报告` 里选择需要删除的 episode。
2. 点击 `选择episode删除`。
3. 删除后会重新 QC。

注意：

```text
删除是数据处理动作，确认 episode 确实不可用后再操作。
```

---

## 12. 生成 LeRobot

如果采集页已经勾选了 `转 LeRobot`，保存 episode 后会自动生成或更新 LeRobot。

如果需要手动生成：

1. 进入 `质量检查与转换`。
2. 填好 `MCAP 数据集路径` 或 `HDF5 路径`。
3. 填好 `LeRobot 输出根目录`。
4. `机器人` 选择 `aloha`。
5. 填写 `任务文本`。
6. 确认移动数据勾选 `ALOHA 包含升降/底盘 state-action（21维/18维）`。
7. 点击 `生成编号计划`。
8. 确认编号映射没问题。
9. 点击 `生成 LeRobot`。

移动 LeRobot 默认数据布局：

```text
observation.state = 21D
[left arm 7, right arm 7, lift, base_x, base_y, base_yaw, base_vx, base_vy, base_wz]

action = 18D
[left master 7, right master 7, lift_target, base_vx_cmd, base_vy_cmd, base_wz_cmd]
```

---

## 13. 可视化回放

可视化回放不会给真机发动作，建议每条数据保存后先看可视化。

入口 A：质量检查与转换页

1. 点击 `打开/刷新 HDF5 回放`。
2. 或点击 `打开/刷新 LeRobot 回放`。
3. 检查图像、state/action 曲线、episode 内容是否正常。

入口 B：数据回放与真机回放页

左侧是 HDF5 / LeRobot 可视化回放。

也可以在 `数据采集` 页底部的 `回放` 区域配置：

| 字段 | 建议 |
|---|---|
| 格式 | `HDF5` 或 `LeRobot` |
| 模式 | 可视化时不影响真机安全 |
| 数据路径 | HDF5 文件/目录或 LeRobot 数据集目录 |
| LeRobot episode | 要看的 episode 编号 |
| 可视化 FPS | 通常 `30` |
| 可视化最大帧数 | `0` 表示不限制 |

然后点击 `可视化回放`。

可视化视频会输出到日志目录的 replay 子目录。

---

## 14. 真机回放前检查

真机回放会实际控制机器人，必须先确认：

1. 现场无人、无障碍物。
2. 实体急停可用，且有人看守。
3. 双臂、底盘、升降柱节点正常。
4. 数据已经可视化检查过。
5. 首次回放建议低倍速。
6. 首次验证建议先只回放双臂，不动底盘和升降柱。

---

## 15. HDF5 真机回放

进入 `数据回放与真机回放` 页，右侧是真机回放控制。

配置建议：

| 字段 | 推荐值 |
|---|---|
| 格式 | `HDF5` |
| 模式 | 首次选 `仅双臂`，确认后再选 `底盘 + 升降柱 + 双臂` |
| 数据路径 | `/home/agilex/data/market_5/aloha/episodeN/episodeN.hdf5` |
| 真机倍速 | 首次建议 `0.3` |
| 轨迹源 | `HDF5 master / LeRobot action` |
| 底盘源 | `auto` |
| 最大线速度 | 首次建议 `0.2` 或 `0.3` |
| 最大角速度 | 首次建议 `0.5` 或 `0.8` |

操作流程：

1. 先选择 `仅双臂`。
2. 点击 `真机回放`。
3. 浏览器会弹确认框，确认现场安全后再开始。
4. 观察双臂轨迹是否安全。
5. 如需停止，点击 `停止回放` 或使用实体急停。
6. 双臂确认安全后，再切换到 `底盘 + 升降柱 + 双臂`。
7. 保持低倍速回放完整移动轨迹。

---

## 16. LeRobot 真机回放

配置建议：

| 字段 | 推荐值 |
|---|---|
| 格式 | `LeRobot` |
| 模式 | 首次选 `仅双臂`，确认后再选 `底盘 + 升降柱 + 双臂` |
| 数据路径 | LeRobot 数据集根目录，例如 `/home/agilex/data/market_5/lerobot/<dataset_name>` |
| LeRobot episode | 要回放的 episode 编号 |
| 真机倍速 | 首次建议 `0.3` |
| 轨迹源 | `HDF5 master / LeRobot action` 或 `LeRobot state` |
| 最大线速度 | 首次建议 `0.2` 或 `0.3` |
| 最大角速度 | 首次建议 `0.5` 或 `0.8` |

操作流程：

1. 先在 pipeline 页面打开 LeRobot 可视化回放。
2. 确认 episode 图像和动作正常。
3. 在真机回放区选择 `格式=LeRobot`。
4. 填写 LeRobot 数据集目录。
5. 填写 `LeRobot episode`。
6. 首次选择 `仅双臂`。
7. 点击 `真机回放`。
8. 确认安全后再尝试完整模式。

---

## 17. 停止回放和急停

真机回放时有两个停止手段：

| 操作 | 作用 |
|---|---|
| 停止回放 | 停止当前网页启动的回放任务 |
| 急停中断 | 停止回放或采集，并发送机器人保持/停止命令 |

异常情况处理顺序：

1. 优先实体急停。
2. 点击网页 `急停中断`。
3. 点击 `停止回放`。
4. 检查终端日志和机器人状态。

---

## 18. 数据路径速查

以 `/home/agilex/data/market_5` 为例：

```text
原始 MCAP:
  /home/agilex/data/market_5/episodeN/episodeN_0.mcap

ALOHA/HDF5:
  /home/agilex/data/market_5/aloha/episodeN/episodeN.hdf5

QC 报告:
  /home/agilex/data/market_5/aloha/qc_reports/

LeRobot:
  /home/agilex/data/market_5/lerobot/

网页日志:
  /home/agilex/data/market_5/logs/mobile_pipeline_web_staged/

采集子页面日志:
  /home/agilex/data/market_5/logs/collect_mobile_episode_web/

回放日志:
  /home/agilex/data/market_5/logs/collect_mobile_episode_web/replay/
```

---

## 19. 推荐现场流程

1. 启动硬件节点。
2. 启动网页。

```bash
cd /home/caizj/agilex_idata
DATA_ROS_WS=/home/caizj/agilex_idata/ros2_ws \
bash scripts/collection/collect_mobile_pipeline_web_staged.sh /home/agilex/data/market_5
```

3. 打开网页。
4. `数据采集` 页点击 `启动自检`。
5. 配置采集选项。
6. 点击 `开始采集`。
7. 执行任务。
8. 分阶段任务按需要点击 `阶段切换`。
9. 点击 `结束采集并保存`。
10. 等待后台转换、QC、LeRobot。
11. 去 `质量检查与转换` 页检查 QC。
12. 先做 `可视化回放`。
13. 再做低倍速 `仅双臂` 真机回放。
14. 最后做低倍速完整真机回放。

---

## 20. 注意事项

1. 采集时不要随便刷新或关闭终端。
2. `数据目录` 一定要确认正确，避免写到旧数据集。
3. episode 已存在时，重新采集可能覆盖旧数据。
4. 真机回放前必须先可视化。
5. 移动底盘任务必须勾选 `带底盘`。
6. 移动数据生成 LeRobot 时必须保留 21D state / 18D action。
7. 现场异常优先实体急停，网页急停只能作为软件辅助。
