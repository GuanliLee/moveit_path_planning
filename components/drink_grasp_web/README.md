# 饮料抓取任务 Web

独立 Web，不修改 `/home/ligl/agilex_xpc/web`。

启动：

```bash
cd /home/ligl/drink_grasp_web
./start.sh
```

打开：

```text
http://127.0.0.1:8090/
```

停止：

```bash
cd /home/ligl/drink_grasp_web
./stop.sh
```

上架按钮会调用 ROS2 服务：

```text
/execute_named_grasp_task
```

拣货按钮会调用 ROS2 服务：

```text
/execute_named_pick_task
```

执行中点击“停止当前执行”会调用 `/cancel_named_grasp_task`，停止状态机后续步骤，
并请求取消当前规划或机械臂/夹爪轨迹。刷新页面或点击“刷新状态”后，按钮仍会按当前任务状态启用。

前端显示中文饮料名，后端发给状态机和 121 的 `target_name` 是英文 prompt。

货架位姿之后补在 `config/place_poses.json`。格式示例：

```json
{
  "arms": {
    "right": {
      "layers": {
        "1": {
          "positions": {
            "1": {
              "frame_id": "world",
              "position": [0.395, -0.132, 0.49],
              "orientation_xyzw": [-0.121, 0.678, 0.168, 0.705]
            }
          }
        }
      }
    }
  }
}
```

未配置的层位会发送空 `place_tcp_pose`，状态机会使用自身 yaml 里的默认放置位。

拣货放置位姿之后补在 `config/pick_place_poses.json`。格式示例：

```json
{
  "arms": {
    "right": {
      "default": {
        "frame_id": "world",
        "position": [0.395, -0.132, 0.49],
        "orientation_xyzw": [-0.121, 0.678, 0.168, 0.705]
      }
    }
  }
}
```

## Episode 数据回放

打开 Web 后点击顶部“数据回放”，左侧会列出 `recordings` 下已经保存的 episode。选择一条数据后可以：

- 同步播放顶部、左腕、右腕三路彩色相机；
- 拖动时间轴、暂停以及按 0.25–2 倍速播放；
- 查看同一时刻的左右臂末端 X/Y/Z 轨迹；
- 查看同一时刻的 J1–J7 关节轨迹和数值。

回放是只读可视化，不会发布 ROS 控制消息，也不会驱动机械臂。当前录制中以点开头的隐藏 MCAP 文件同样能够正常读取。

首次打开某个 episode 时，后端会从 MCAP 解码 JPEG 帧并缓存最近两个 episode。可用环境变量：

```bash
export DRINK_GRASP_PLAYBACK_CACHE_EPISODES=2
export DRINK_GRASP_PLAYBACK_MAX_TRAJECTORY_SAMPLES=1400
export DRINK_GRASP_PLAYBACK_JPEG_QUALITY=76
```

回放接口：

```text
GET /api/playback/episodes
GET /api/playback/manifest?episode=采集组/episode_NNN
GET /api/playback/frame?episode=采集组/episode_NNN&camera=cam_high&index=0
```
