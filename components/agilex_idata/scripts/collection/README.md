# 数据采集操作入口

生产操作员只需要启动：

```bash
bash /home/caizj/agilex_idata/scripts/collection/collect_mobile_web.sh
```

默认只监听采集机局域网地址，页面是 `http://192.168.3.101:8000/`。页面可互斥启动四种现有工作流：

- 全阶段采集
- 第二阶段采集
- 抓取 / 放置采集
- 推理触发录制

首页是模式门户：数据目录默认留空、起始编号默认 `episode0`。点击模式后会进入该模式的整页操作台；“返回门户”只切换页面，不会停止正在运行的模式，门户可重新进入操作台或显式停止模式。为避免原 recorder 删除同名目录，门户和每次真正开始采集前都会拒绝已经存在的 `episodeN`，不提供覆盖开关。全阶段入口默认勾选“分阶段”。操作台中的“退出”会关闭整个当前模式及其子进程，不会只留下外层网页假运行。

各模式共用的采集子页面不再显示“转 HDF5 / QC / 转 LeRobot / 后台处理”控件，采集阶段固定只保存原始 MCAP。启动前自检失败会保留详细日志并在页面持续显示红色预警，但不会取消 episode 启动；同名 episode 防覆盖、采集/回放互斥等安全检查仍会阻止对应危险操作。

## 不可变兼容协议

本次重构只改变启动和网页交互。以下现有后端机制保持不变：

- MCAP 服务：`/data_tools_dataCapture/capture_service`
- MCAP 状态：`/data_tools_dataCapture/status`
- 人工状态机：`/state_machine/start`、各阶段结束话题、`/state_machine/end`
- 推理门控：`/inference_record` 与 `/state_machine/start` 同时满足后开始，`/state_machine/end` 保存
- 采集就绪：`/data_collection/start`
- 原有 `episodeN_0_info.json` 字段；非推理模式保留 A/B/F 选择，推理录制保存后自动标记为 A

统一入口不允许配置这些话题。人工与推理模式的控制进程、运行目录和日志相互独立，但因其使用同一个既有 ROS 后端，一次只允许一个采集会话运行。

## 启动环境

所有保留入口统一先执行关闭代理、加载 ROS Humble 和相机工作空间，然后固定：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=99
```

实现位于 `collection_ros_env.sh`。

## 保存契约

新采集在采集阶段不再生成 ALOHA HDF5，也不运行 QC。原始 episode 延续现有目录和文件命名：

```text
episodeN/
├── episodeN_0.mcap
├── metadata.yaml
├── capture_health.json
└── episodeN_0_info.json
```

全阶段模式继续按原逻辑写其已有的阶段 instruction sidecar。日志仍写入数据集已有 `logs/` 目录或统一入口运行目录，不在 episode 中新增 manifest 或新层级。

## 回放

采集页保留：

- 原始 MCAP 真机回放：`scripts/replay/replay_mcap_mobile_to_robot.py`
- `aligned_joints.h5` 真机回放：`scripts/replay/replay_agibot_mobile_to_robot.py`
- LeRobot 真机回放：`scripts/replay/replay_lerobot_mobile_to_robot.py`

MCAP 回放是类型化适配，不使用 `ros2 bag play`，不会广播相机或采集状态机话题。

回放框中的 episode 始终指原始 `episodeN` 编号。MCAP 默认读取当前采集目录，保存完成后自动选中刚保存的 episode；HDF5 和 LeRobot 根据门户填写的原始数据目录动态推导：例如输入 `<scene>/20260729_scene13`，回放根目录就是 `<scene>/three_camera_global`，不绑定任何固定 scene。HDF5 会精确定位 `episodeN/states/aligned_joints.h5`，LeRobot 会通过 `meta/episode_name_mapping.json` 在 A/B 数据集中找到对应 parquet，不把分级后的内部编号当作原始编号。采集与回放互斥：回放运行时不能开始采集，采集运行时也不能启动真机回放。

## 保留的功能脚本

- `collect_mobile_episode_web.sh`：现有 MCAP 采集、人工审核、自检和回放控制器
- `collect_mobile_pipeline_web_staged.sh`：全阶段状态机桥接
- `collect_mobile_pipeline_web_fixed_stage.sh`：第二阶段快捷入口
- `collect_mobile_pipeline_web_grasp_place.sh`：抓取 / 放置快捷入口
- `collect_mobile_pipeline_web_inference.sh`：推理门控状态机桥接
- `collect_mobile_pipeline_qc_web_four_camera.sh`：独立质检入口，不由采集模块自动启动

单臂、左臂快速回零、旧命令行 MCAP/HDF5/LeRobot 串联以及被新入口替代的旧采集壳已删除。

在统一入口终端按 `Ctrl+C` 时，会先触发当前模式原有的保存/清理流程，等待状态机桥、采集控制器、MCAP recorder 和网页进程全部退出，最后才结束统一入口。清理范围只包含本次统一入口启动的进程树。
