# 抓取评级窗口锁存放置启动信号设计

## 背景

现场日志确认 bridge 能收到 `/state_place/start=true`，但状态机在抓取保存后立即发布该信号，而人工质量评级在随后完成。当前 bridge 会丢弃评级期间的所有状态机事件，因此评级通过并进入 `place/idle` 后已经没有新的放置启动信号，第二阶段无法开始。

## 目标

- 抓取保存或评级期间收到一次 `/state_place/start=true` 时，为当前 logical episode 锁存该信号。
- 抓取评级 A/B 并成功切换到放置目录后，一次性消费锁存信号，沿用正常的放置启动路径开始录制。
- 抓取评级 F 时清除锁存信号，保持同一 episode 重新采集抓取，绝不误启动放置。
- 不改变评级完成后新发布 `/state_place/start=true` 的现有行为。
- 不影响旧单阶段模式、放置保存/评级流程或乱序事件保护。

## 方案比较

### 方案 A：要求状态机在评级后重新发布

bridge 无需修改，但状态机并不知道人工评级何时完成，现场已经证明它只在抓取阶段结束时发布一次，因此无法可靠满足当前流程。

### 方案 B：缓存完整 ROS 事件队列

可保留所有事件，但会重新引入跨阶段、跨 episode 的陈旧事件风险，安全边界不清晰。

### 方案 C：仅锁存当前抓取 review 的 place-start（采用）

只保存一个布尔事实：当前抓取 episode 是否已经收到放置启动条件。它不保存任意事件，不跨 episode，并由评级结果决定消费或清除，范围最小且符合现场状态机时序。

## 状态与数据流

新增 bridge 局部状态 `pending_place_start`，默认 `False`。

1. 开始保存某个抓取 episode 时重置为 `False`。
2. bridge 当前处于抓取 `saving/reviewing`，或正在从抓取 review 切换目标时，收到 `place_start`：设置为 `True`；重复消息保持 `True`。
3. 抓取 review 未完成：继续等待，锁存值保持不变。
4. 抓取评级 A/B：先确认放置目录和同编号 episode 配置成功，再切换到 `place/idle`；若已锁存，则清除锁存并向 bridge 自己的事件队列加入一个带新上下文的 `place_start`。下一轮主循环沿用现有 `start_place` 路径启动录制，确认 recording 后发布 `/data_collection/start=true`。
5. 抓取评级 F：清除锁存，切回同编号 `grasp/idle`，等待新的 `/state_machine/start`。
6. 放置 review 或进入下一个 logical episode 时锁存值必须为 `False`。

## 边界与错误处理

- 只有 `pending_review_phase == "grasp"` 的 `place_start` 可以被锁存；其他事件仍按原逻辑忽略。
- 在目标 `/config` 切换的安静窗口中收到的 `place_start` 也视为当前抓取 review 的信号并锁存。
- 必须等抓取 review 通过、目标目录确认成功后才投递合成事件，不能提前开始放置。
- 合成事件使用 `place/idle` 新上下文，因此继续经过现有事件路由、状态检查和错误处理。
- bridge 重启不会恢复内存锁存；状态机或操作员需重新发布。这与现有非持久事件语义一致。

## 测试

- 纯规则测试：只在抓取 review 锁存；重复信号幂等；抓取 accepted 时消费；抓取 discarded 和其他阶段不消费。
- bridge 接线测试：review 队列和目标切换安静窗口都会锁存；accepted 后投递一次新上下文事件；discarded 清除。
- 聚焦回归：现有双阶段、旧单阶段、UI 和 retained publisher 测试全部通过。
- 隔离 ROS2 端到端：`place_start` 只在抓取 review 期间发布一次，评级后不再发布；验证放置录制自动开始。另测抓取 F 后不启动放置并需要重新抓取。
