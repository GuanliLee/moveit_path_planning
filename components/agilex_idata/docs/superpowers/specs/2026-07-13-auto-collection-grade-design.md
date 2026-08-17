# 采集自动质量等级设计

## 目标

为 `collect_mobile_pipeline_web_staged.sh` 增加可选参数 `--grade`。传入等级后，每条成功保存的数据自动记录该等级，不再等待网页人工选择；未传参数时保持现有人工审核流程。

## 命令行接口

- 支持 `--grade a` 和 `--grade=A` 两种写法。
- 输入不区分大小写，内部统一为 `A`、`B` 或 `F`。
- 只允许 `A`、`B`、`F`；缺少参数或传入其他值时，主脚本在启动模块前报错退出。
- 现有位置参数和其他选项保持兼容。
- 未传 `--grade` 时不启用自动等级。

示例：

```bash
/home/caizj/agilex_idata/scripts/collection/collect_mobile_pipeline_web_staged.sh \
  /home/agilex/data/navigation/stage1 0 "Wanglaoji" --grade a
```

## 数据流

1. 主脚本解析并校验 `--grade`，保存为规范化的自动等级配置。
2. 启动 `collect_mobile_episode_web.sh` 时，通过专用环境变量把该配置传入采集子进程。
3. 子脚本正常停止采集、保存 MCAP、写入阶段/episode 元数据。
4. 自动等级未配置时，设置待审核状态并继续现有网页弹窗流程。
5. 自动等级已配置时，设置待审核状态后立即调用现有 `finalize_quality_review_from_json` 路径，以空原因和空备注记录固定等级。
6. 现有审核函数负责写入最终质量元数据、递增 episode、更新状态并启动后续转换/QC。
7. 状态机桥接继续在等级成功落盘且 episode 前进后发布 `/data_collection/save_success=true`。

## 失败处理

- 非法 `--grade` 在启动时失败，不启动任何采集模块。
- MCAP 保存失败沿用现有错误路径，不记录自动等级。
- 自动等级元数据写入失败时，不递增 episode，也不发布保存成功；保留待审核状态，使网页可以人工重试或放弃。
- 自动模式不自动填写 B/F 原因，原因列表与备注均为空。

## 兼容性

- 默认行为不变：没有 `--grade` 时仍由操作者选择 A/B/F 或放弃。
- 不更改 ROS topic 名称、消息类型或 QoS。
- 不增加网页配置开关；自动等级只由启动参数控制，并对本次脚本生命周期内的所有新 episode 生效。

## 验证

- 参数解析覆盖分离值、等号值、大小写归一化、缺失值和非法值。
- 验证自动等级传入采集子进程。
- 验证自动模式复用现有审核函数，人工模式仍写入待审核状态。
- 验证等级落盘失败仍保持待审核状态。
- 运行相关采集测试、两个 Shell 语法检查和嵌入式 Python 语法检查。
