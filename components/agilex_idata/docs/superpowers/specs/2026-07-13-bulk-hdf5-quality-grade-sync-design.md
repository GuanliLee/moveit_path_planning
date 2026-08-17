# 批量写入 HDF5 质量等级设计

## 目标

在网页“质检报告”栏增加“批量写入 HDF5 等级”按钮。用户无需生成 LeRobot，即可将当前网页最终质检等级写入数据集内每个 HDF5 episode 的 `meta/episode_meta.json`，并在同一后台任务中检查所有 HDF5 episode 都有合法且一致的等级。

## 权威数据与写入规则

- 权威来源是 `dataset_status` 返回的网页最终质检行。
- 每个实际存在 `aligned_joints.h5` 的 episode 必须恰好匹配一条网页质检行。
- 最终等级只允许 `A`、`B`、`C`、`F`。
- 写入 `quality_grade` 和 `manual_quality_grade`；两者都等于网页最终等级。
- 写入 `manual_failure`；仅等级 `F` 时为 `true`。
- 使用现有原子 JSON 写入函数，保留 sidecar 内 `task`、来源文件等无关字段。
- JSON 损坏、episode 缺失/重复或网页等级非法时停止任务，不猜测等级。

## 网页交互

按钮位于“质检报告”面板的批量操作区，使用现有 `/api/run` 后台任务机制。任务日志依次显示同步 episode 总数、实际改动数量，以及校验后的 A/B/C/F 数量。任务完成后网页自动刷新；任务失败时现有日志区展示错误。

## 校验

写入后重新从磁盘严格读取每个 `meta/episode_meta.json`，检查：

1. `quality_grade` 存在、合法并等于网页最终等级；
2. `manual_quality_grade` 存在、合法并等于网页最终等级；
3. `manual_failure` 是布尔值，并且恰好等于 `quality_grade == "F"`。

校验只涉及 HDF5 sidecar，不读取或改写 LeRobot 数据。

## 测试与部署

- 单元测试覆盖成功同步、缺失字段、错误等级/失败标志和后台步骤。
- 页面源码测试覆盖按钮位置、文案及 `/api/run` stage。
- 运行现有质量等级回归测试和 Python 语法检查。
- 合并在当前 `main` 上提交，重启 `collect-mobile-pipeline-qc-web.service` 并验证 HTTP 200。
- 对 `/home/agilex/data/stage2/hdf5_episodes` 的现有数据执行一次相同同步和完整性校验。
