# Scene 10–16 `three_camera_global` 头部相机重同步设计

日期：2026-08-03

状态：待确认

范围：`/home/agilex/data/stage2_new/scene10` 至 `scene16`，仅处理 `three_camera_global`

## 1. 目标

在不覆盖现有数据的前提下，修复 scene10–scene16 中广角头部相机与左右腕部相机的时间错位，并从修复后的 HDF5 重新生成 LeRobot 数据。

本设计同时区分两类问题：

1. **时间戳错误但图像仍在 MCAP 中**：可以通过重建时间轴修复。
2. **采集期间图像本身已经丢失**：无法凭空恢复，只能降级或剔除，不能用重复帧伪装成正常数据。

## 2. 已确认的本质原因

问题最早出现在 `/camera_h` 发布到 ROS 的消息头时间戳，不是 HDF5 或 LeRobot 新制造出来的固定延迟。

当前广角相机由 ROS 2 `usb_cam` 0.8.1 发布。已安装版本的时间换算代码为：

```cpp
epoch_us = epoch_time.tv_sec * 1000000 + epoch_time.tv_usec / 1000.0;
```

`tv_usec` 本来已经是微秒，这里又除以 1000，导致节点启动时计算出的“设备单调时钟到 Unix 时间”的偏移量错误。该错误会给同一次节点生命周期中的 `/camera_h` 消息头增加一个近似固定、但每次重启可能不同的偏差。

现场证据：

- 同一时刻左右腕部与前视相机的消息头只落后当前时间约 3 ms；广角头部相机落后约 543 ms。
- 广角节点重启前测得约 260 ms，重启后变为约 543 ms，说明偏差跟节点启动时刻有关，而不是稳定的 USB 传输延迟。
- 历史 MCAP 中，按 MCAP 记录时间比较相机内容，抽样结果基本在 `-1～+1` 帧；按消息头时间比较则可达到 `-28、-26、-5、-1` 帧。
- 原始 MCAP 已经保存了错误的 `/camera_h` 消息头，因此错误来源早于 HDF5。
- 当前 HDF5 转换脚本按 `msg.header.stamp` 命名并对齐相机帧，于是把错误的标签真正转换成了可见的画面错位。
- LeRobot 从 HDF5 生成，只是继承了错位，没有再次制造新的相机延迟。

错位方向可以这样理解：若实际时刻 `t` 的头部图像被错误标成 `t - 0.93 s`，转换器要给腕部时刻 `t` 找“同时间”的头部图像时，会找到实际时刻约 `t + 0.93 s` 的图像。因此最终看到的是头部画面领先约 28 帧，而不是头部相机真的晚传输了 28 帧。

## 3. 数据盘点与修复边界

以现有 HDF5 元数据中的 `source_mcap_files` 为准建立白名单，而不是把 raw 目录中的所有 MCAP 都重新纳入数据集。这样不会把以前已经人工淘汰的采集重新混入。

当前盘点结果：

- scene10–scene16 的现有 HDF5 episode：1113 个。
- 1113 个均能映射到原始 MCAP。
- raw 目录中另有不属于最终 HDF5 集合的文件，不在本次修复范围内。
- 一份已知损坏且未进入最终 HDF5 的 raw episode 不在白名单中。

按“头部相机原始帧数 / 期望帧数”的初步覆盖率分层：

| 层级 | 覆盖率 | 当前估计数量 | 处理方式 |
|---|---:|---:|---|
| 可发布 | `>= 95%` | 1002 | 自动验证通过后生成正式 LeRobot 数据 |
| 降级待审 | `>= 80%` 且 `< 95%` | 96 | 生成修复 HDF5 和报告，但单独隔离，不进入正式 LeRobot |
| 不可恢复 | `< 80%` | 15 | 保留诊断报告，不生成可训练 LeRobot episode |

这些数量是修复前盘点值，最终数量以重转后的逐 episode 验证为准。覆盖率低表示真实图像没有被录进 MCAP；改变时间戳只能纠正对齐，不能恢复不存在的帧。

## 4. 方案选择

### 4.1 采用的方案：仅对 `/camera_h` 使用 MCAP 记录时间

重转时显式指定：

- `/camera_h/color/image_raw`：使用 `reader.read_next()` 返回的 MCAP record/log time。
- 左右腕部相机、机器人状态、动作及其他消息：继续使用原消息头时间戳。

选择此方案的原因：

- MCAP recorder 当前没有使用 topic stamp 作为记录时间，仍保存了正常的接收顺序和独立记录时间。
- raw 内容抽样已经证明 record time 能恢复三相机到约一帧内。
- 只替换已知损坏 topic 的时间源，避免改变机器人状态/动作以及正常相机的既有时间语义。
- 规则是显式配置并写入元数据，不依赖容易误判的自动猜测。

### 4.2 不采用的方案

- **直接把现有 HDF5 头部视频整体平移固定帧数**：不同 scene、episode 和节点生命周期的偏差不同，且无法正确处理开头/结尾和真实缺帧。
- **直接修改 LeRobot 视频**：LeRobot 没有制造问题；单独改视频会破坏与 HDF5、状态和动作的可追溯关系。
- **按 HDF5 内容相关性自动估计每段偏移**：运动少时相关性不可靠，也不能修复转换阶段已经发生的边界填充。
- **用插值或重复帧填满严重缺帧 episode**：会制造看似同步但信息重复的训练样本，风险高于隔离数据。

## 5. 非覆盖式输出结构

原目录保持只读，不删除、不原地改名、不覆盖：

```text
/home/agilex/data/stage2_new/sceneN/
├── three_camera_global/                 # 现有数据，保持不变
└── three_camera_global_resynced/
    ├── hdf5_episodes/
    ├── lerobot/
    ├── quarantine_hdf5/
    └── repair_reports/<run_id>/
```

批处理对每个 episode 先写临时目录，验证成功后再原子重命名到目标位置。任务可中断续跑；已有且校验通过的输出跳过，失败输出保留日志但不冒充成功结果。

## 6. 转换工具改动

### 6.1 显式时间源配置

在现有调用链中透传一个可重复的 topic 配置，例如：

```text
convert_mcap_dataset.py
  -> mcap_to_icra_episode.py
  -> mcap_to_hdf5.py / mcap_to_aloha_data.py
```

新增语义等价于：

```text
--record-time-topic /camera_h/color/image_raw
```

转换器读取目标 topic 时，用 MCAP record time 生成帧时间文件；其余 topic 维持旧行为。输出元数据必须记录每个 topic 的时间源、header 与 record time 的中位差、软件版本和修复运行 ID。

### 6.2 白名单和批处理清单

新增可恢复执行的 repair manifest，每条至少记录：

- scene、episode 和原质量等级；
- 原 HDF5、source MCAP、修复 HDF5 和 LeRobot 路径；
- 原始估计错位帧数；
- `/camera_h` header/record time 偏差；
- raw 帧覆盖率；
- 修复后的同步指标；
- `publish / quarantine / reject / failed` 状态与原因。

### 6.3 保留业务元数据

从原 HDF5 复制经过人工确认的业务元数据，包括 task/instruction、grade/quality、item/scene、subtask segments 和人工标记。

新生成的同步统计、时间源 provenance、帧选择统计不得被旧元数据覆盖。

### 6.4 重做静止裁剪

修复转换完成后，按原流程重新执行 stationary trim：目标 30 FPS、保留 15 个静止边界帧。不能只替换旧 HDF5 的头部视频，因为状态、动作、master timestamps 和所有相机选择必须由同一条时间轴共同生成。

在 master 状态时间轴未改变且原始数据完整的 episode 中，最终帧数应与旧 HDF5 一致；不一致时进入人工审查而不是静默接受。

### 6.5 从修复 HDF5 重建 LeRobot

LeRobot 必须从修复后的 HDF5 重新生成，并保持原 quality grade 和 episode 映射。正式 LeRobot 只接收 `publish` 层；`quarantine` 数据单独保留 HDF5，除非人工复核后显式晋级。

## 7. 验证策略

### 7.1 先跑金丝雀

先选覆盖不同偏差和缺帧情况的 episode：

- scene10 episode40：历史约 28 帧错位，raw 覆盖正常；
- scene12 episode40：历史约 5 帧错位；
- scene15 episode70：历史约 1 帧错位；
- scene16 episode80：历史约 26 帧错位；
- scene10 episode45：严重 raw 缺帧，用于验证隔离规则。

金丝雀通过并人工抽看三相机拼接视频后，才扩展到 1113 个 episode。

### 7.2 单 episode 验收

`publish` 至少同时满足：

1. source MCAP、原 HDF5 和修复输出的映射唯一且可追溯；
2. 头部相机 raw 覆盖率 `>= 95%`；
3. 修复后头部与腕部的有效内容错位不超过 2 帧；
4. 健康 episode 不出现长段 timestamp-derived 重复填充；
5. HDF5 中 state、action、master timestamp 和三个视频帧数一致；
6. 最终帧数与旧 episode 一致，或差异已有明确、可审计的原因；
7. LeRobot 帧数、episode index、质量等级和 HDF5 一一对应。

`>=80%` 且 `<95%` 自动进入 `quarantine`；`<80%` 自动进入 `reject`。任何解析错误、输出不完整或指标无法计算均为 `failed`，不能默认通过。

### 7.3 全批次验收

- 白名单中的 1113 个 episode 必须全部落入 `publish/quarantine/reject/failed` 之一，无静默遗漏。
- 汇总每个 scene 的旧错位、新错位、覆盖率、填帧数和最终状态。
- 对每个 scene 抽看至少一个原严重错位但修复成功的拼接视频。
- 保存转换命令、代码 commit、配置、manifest 和校验报告。
- 对原 `three_camera_global` 路径做修复前后清单/摘要校验，确认未被改写。

## 8. 采集端永久解决方案

历史数据修复不能代替采集端修复。后续采集需要同时做三层防护：

### 8.1 修复广角相机发布节点

在受控 ROS overlay 中基于当前使用的 `usb_cam` 0.8.1 打补丁，把错误的：

```cpp
epoch_time.tv_usec / 1000.0
```

改为：

```cpp
epoch_time.tv_usec
```

固定源码 commit 和构建产物版本，并让 `cameras.launch.py` 明确加载该 overlay。暂不直接追随 `usb_cam` 最新主分支，因为主分支还包含相机 buffer/publish 行为变化，混合升级会扩大验证范围。

### 8.2 采集前硬性健康检查

在以下采集入口启动录制前检查四相机 header age 和频率：

- `scripts/collection/collect_mobile_pipeline_web_staged.sh`
- `scripts/collection/collect_mobile_pipeline_qc_web_four_camera.sh`

建议门限：

- 任一相机 `abs(now - header.stamp) > 50 ms`：禁止开始 episode；
- 任意两相机 header age 差 `> 30 ms`：禁止开始 episode；
- 实测频率或短窗口帧覆盖率不足：禁止开始 episode；
- 报错必须显示具体 topic、header age、频率和建议重启组件。

现有只按 `25 Hz × duration × 0.8` 判断总帧数的健康检查过松，约 20 Hz 的数据也可能通过，需要改为滑动窗口频率、最长间隙和总覆盖率共同判断。

### 8.3 转换端保留防御与审计

即便发布节点修好，转换器仍保留显式 timestamp-source 和 provenance 能力，但正常新采集默认继续使用 message header。若 header/record 偏差异常，转换应失败并要求人工选择，不能静默自动切换。

## 9. 实施顺序

1. 为转换器时间源策略补测试，再实现显式 `/camera_h` record-time 支持。
2. 跑 5 个金丝雀，生成三相机对比视频和逐帧指标。
3. 人工确认金丝雀后，批量修复 scene10–scene16 的 1113 个白名单 episode。
4. 只从 `publish` HDF5 生成正式 LeRobot；隔离其他层级。
5. 输出逐 episode 和逐 scene 报告，原数据保持不变。
6. 单独实现并部署 `usb_cam` overlay 补丁与采集前硬性检查。
7. 用一次受控新采集验证 MCAP、HDF5 和 LeRobot 三层均不再产生错位。

## 10. 风险与回退

- MCAP record time 是 recorder 接收时间而非硬件曝光时间，但现有 raw 内容实测表明它在本数据集中比损坏的 header 可靠，且误差约在一帧内。
- record time 策略只允许用于明确列出的坏 topic，避免误伤其他传感器。
- 真实缺帧不做“修复式伪造”，一律隔离或剔除。
- 所有输出写入 sibling 目录；回退只需停止使用 `three_camera_global_resynced`，原始 MCAP、HDF5 和 LeRobot 均不受影响。
- `/home` 当前可用空间约 321 GB，足够非覆盖式生成 scene10–16 修复数据，但批处理仍需在每个阶段记录空间并清理可再生中间文件。

## 11. 完成定义

本任务只有在以下条件全部满足时才算完成：

- 1113 个白名单 episode 全部有明确、可审计的最终状态；
- 所有 `publish` episode 的相机内容错位不超过 2 帧；
- 正式 LeRobot 只来自已通过验证的修复 HDF5；
- 原 `three_camera_global` 数据未被修改；
- 根因补丁和采集前检查通过一次真实新采集验证；
- 修复报告能回答每个 episode 原来延迟多少、使用何种时间源、修复后多少、是否缺帧以及为何发布或隔离。
