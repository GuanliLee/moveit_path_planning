# Piper ROS 2 服务依赖子集

该目录保留 `path_planning_service` 实际使用的 Piper ROS 2 内容：

- `piper`：双臂驱动、状态读取工具和 180 Hz 轨迹桥接器；
- `piper_msgs`：驱动消息和使能服务；
- `piper_description`：双臂 URDF 引用的模型与网格；
- `can_activate.sh`、`find_all_can_port.sh`：按接收方实际 USB 端口配置 CAN。

Gazebo、MuJoCo、上游单臂 MoveIt 示例、演示截图和带本机 USB 映射的
脚本不属于当前服务运行链，未纳入该精简副本。Piper 相关许可证保留在
本目录和 `src/piper/` 中。
