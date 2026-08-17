# cam_left 棋盘格手眼标定原理和代码讲解

本文档解释当前这套 `cam_left` 棋盘格手眼标定是怎么做的、核心代码在哪里、每个关键代码块的作用是什么。

当前使用的棋盘参数：

```text
内角点阵：8 x 11
内角点数量：88
方格大小：20 mm = 0.02 m
```

注意：`8 x 11` 是棋盘“内角点”数量，不是方格数量。OpenCV 棋盘检测和 PnP 都使用内角点。相邻内角点间距就是方格边长，所以这里的尺度来自 `square_size_m = 0.02`。

## 1. 这次手眼标定求的是什么

你现在做的是 `eye_in_hand`，意思是相机固定在左臂末端/左夹爪附近，跟着左臂一起动。

要求的外参是：

```text
T_gripper_camera
```

含义是：把 `cam_left` 相机坐标系下的点，变换到左夹爪末端坐标系。

这份结果已经保存在：

```text
/home/ligl/agilex_xpc/calibration/extrinsics/cam_left_handeye.json
```

当前结果核心数值：

```text
translation = [-0.0767271767, 0.0010660814, -0.0196004373] m
rpy_deg     = [-19.4752, -0.5522, -85.9304] deg
```

## 2. 坐标系和公式

每一帧样本都有两个输入：

```text
T_base_gripper_i
```

来自 ROS topic：

```text
/left/gripper_end_pos
```

它表示第 `i` 个姿态下，左夹爪在机器人 base 坐标系里的位置和方向。

第二个输入是：

```text
T_camera_board_i
```

来自 ROS topic：

```text
/checkerboard/cam_left/pose
```

它表示第 `i` 个姿态下，棋盘格在 `cam_left` 相机坐标系里的位置和方向。

因为棋盘固定在桌面上，所以棋盘在 base 坐标系里的位姿应该不变：

```text
T_base_board_i =
T_base_gripper_i * T_gripper_camera * T_camera_board_i
```

如果 `T_gripper_camera` 是正确的，那么无论机械臂怎么移动，所有样本算出来的 `T_base_board_i` 都应该重合。

手眼标定就是在多组样本中求一个固定的 `T_gripper_camera`，使上面的等式尽可能稳定。

## 3. 整体流程

实际流程是：

```text
1. 启动三路 RealSense。
2. 启动 checkerboard_pose_node.py。
3. checkerboard_pose_node.py 从 cam_left RGB 图里找 8x11 个内角点。
4. 用相机内参 + 20 mm 棋盘模型做 solvePnP。
5. 发布 /checkerboard/cam_left/pose，也就是 T_camera_board。
6. run_handeye_checkerboard.sh 启动 checkerboard_handeye_calibration.py。
7. 每按一次 Enter，保存一组：
   - T_base_gripper
   - T_camera_board
8. 采够 15 组以后按 q。
9. 代码调用 cv2.calibrateHandEye。
10. 得到 T_gripper_camera。
11. 用 validate_handeye_result.py 验证所有样本算出来的 T_base_board 是否稳定。
12. 用 visualize_handeye_application.py 把 RGB、棋盘、点云一起变换到 base 里做应用验证。
```

## 4. 代码位置

核心文件：

```text
/home/ligl/realsense_three_cameras/start_checkerboard_pose.sh
/home/ligl/realsense_three_cameras/checkerboard_pose_node.py
/home/ligl/realsense_three_cameras/run_handeye_checkerboard.sh
/home/ligl/realsense_three_cameras/checkerboard_handeye_calibration.py
/home/ligl/realsense_three_cameras/recompute_handeye_result.py
/home/ligl/realsense_three_cameras/validate_handeye_result.py
/home/ligl/realsense_three_cameras/visualize_handeye_application.py
```

## 5. run_handeye_checkerboard.sh 逐行解释

文件：

```text
/home/ligl/realsense_three_cameras/run_handeye_checkerboard.sh
```

行 1：指定用 bash 执行。

行 2：开启严格模式。任何命令失败、未定义变量、管道失败都会让脚本退出。

行 4-8：注释，写了用法。你当前使用的是：

```bash
./run_handeye_checkerboard.sh eye_in_hand cam_left /left/gripper_end_pos
```

行 10：设置 ROS domain，默认是 `29`。

行 12-14：加载 ROS2 Jazzy 环境。这里用 `set +u` 是避免 ROS setup 脚本里访问未定义变量时报错。

行 16：计算脚本所在目录，确保后面能找到 Python 文件。

行 17：读取第一个参数 `MODE`，没有传就默认 `eye_to_hand`。

行 18：读取第二个参数 `CAMERA`，没有传就默认 `cam_high`。

行 19：读取第三个参数 `ROBOT_POSE_TOPIC`，没有传就默认 `/left/gripper_end_pos`。

行 20：拼出棋盘 pose topic。对 `cam_left` 来说就是 `/checkerboard/cam_left/pose`。

行 21：拼出结果保存目录。你的结果目录是：

```text
/home/ligl/agilex_xpc/calibration/results/cam_left_eye_in_hand_checkerboard
```

行 23-29：真正启动 `checkerboard_handeye_calibration.py`，把 mode、robot pose topic、target pose topic、最少样本数、最大数据年龄和结果目录传进去。

## 6. checkerboard_pose_node.py 逐段解释

文件：

```text
/home/ligl/realsense_three_cameras/checkerboard_pose_node.py
```

这个节点负责一件事：从相机 RGB 图像里识别棋盘，并发布 `T_camera_board`。

### 6.1 头部和依赖

行 1：指定 Python3 执行。

行 2：开启未来版本注解语法，主要是为了类型标注更干净。

行 4-7：导入基础库，`math` 做数学，`os` 取进程号，`time` 控制日志频率，`Optional` 做类型标注。

行 9：导入 OpenCV。棋盘角点检测和 solvePnP 都靠它。

行 10：导入 NumPy。矩阵和数组都靠它。

行 11-16：导入 ROS2、消息类型、QoS。这里订阅的是 `CameraInfo` 和 `CompressedImage`，发布的是 `PoseStamped`。

行 18-21：尝试导入 TF broadcaster。这个功能是可选的，当前主流程主要靠 topic，不强依赖 TF。

### 6.2 图像 QoS

行 24-30：定义图像订阅 QoS。RealSense 图像常用 `BEST_EFFORT`，因为图像是高频数据，丢一帧可以接受，延迟堆积反而不好。

### 6.3 旋转矩阵转四元数

行 33-63：`rotation_matrix_to_quaternion_xyzw()`。

`solvePnP` 输出的是旋转向量 `rvec`，后面会转成旋转矩阵，再转成 ROS pose 需要的四元数 `x,y,z,w`。

行 34：算旋转矩阵 trace。

行 35-59：根据 trace 和主对角线元素选择稳定的转换分支。

行 60-63：归一化四元数，避免数值误差导致四元数长度不是 1。

### 6.4 生成棋盘三维模型

行 66-71：`make_object_points()`。

这段是棋盘尺度最关键的地方。

行 67：创建 `rows * cols` 个三维点。

行 68-70：逐行逐列生成点：

```text
x = col * square_size_m
y = row * square_size_m
z = 0
```

因为你的棋盘是平面，所以所有点的 `z` 都是 0。

对你当前参数来说：

```text
cols = 8
rows = 11
square_size_m = 0.02
```

所以第一个点是 `[0, 0, 0]`，右边相邻点是 `[0.02, 0, 0]`，下一行对应点是 `[0, 0.02, 0]`。

### 6.5 解码图像

行 74-77：把 `/cam_left/color/image_raw/compressed` 的 JPEG/PNG 压缩图像解码成 OpenCV BGR 图像。

### 6.6 ROS 节点初始化

行 80：定义 `CheckerboardPoseNode` 类。

行 81-82：构造函数，节点名带进程号，避免重复启动时节点名冲突。

行 84-93：声明 ROS 参数：

```text
camera_name        默认 cam_high
pattern_cols       默认 8
pattern_rows       默认 11
square_size_m      默认 0.02
publish_tf         是否发布 TF
log_period_sec     日志间隔
```

行 95-99：读取参数值。

行 101-107：确定实际 topic 名字。`cam_left` 时默认是：

```text
image_topic       /cam_left/color/image_raw/compressed
camera_info_topic /cam_left/color/camera_info
pose_topic        /checkerboard/cam_left/pose
```

行 108-112：确定 TF 子 frame 名和是否发布 TF。

行 114：生成棋盘三维模型，也就是 88 个三维内角点。

行 115：设置 OpenCV 棋盘 pattern size，即 `(8, 11)`。

行 116-122：初始化相机内参、畸变参数、统计计数。

行 124：创建 pose 发布器，发布 `/checkerboard/cam_left/pose`。

行 125-129：如果启用 TF，就创建 TF broadcaster。

行 131-142：订阅相机内参和 RGB 压缩图。

行 144-149：打印启动日志，里面会显示相机名、棋盘尺寸、格距、输入输出 topic。

### 6.7 相机内参回调

行 151-156：`_on_camera_info()`。

行 152：把 ROS `CameraInfo.k` 转成 3x3 相机内参矩阵：

```text
K = [[fx, 0,  cx],
     [0,  fy, cy],
     [0,  0,  1 ]]
```

行 153-155：读取畸变参数 `D`。如果没有畸变参数，就设为 `None`。

行 156：记录相机 frame id。

### 6.8 图像回调：检测棋盘并 solvePnP

行 158：`_on_image()`，每来一帧图像就执行。

行 159-161：如果还没有收到 camera_info，就不处理图像。

行 163-166：解码压缩图像，失败就跳过。

行 168：转灰度图，棋盘检测使用灰度。

行 169-170：初始化角点变量。

行 172-177：优先用 `findChessboardCornersSB`。这是 OpenCV 新版的棋盘检测方法，通常比老方法稳。

行 179-184：如果 SB 方法没找到，就退回老的 `findChessboardCorners`。

行 185-187：如果老方法找到了，再用 `cornerSubPix` 做亚像素角点优化。

行 189-192：如果没找到完整棋盘，计数并打印 warning，然后不发布 pose。

行 194：把角点整理成 `N x 2` 的像素坐标数组。

行 195-201：调用 `cv2.solvePnP()`。

这里的输入是：

```text
object_points: 棋盘自身坐标系下的 88 个三维点，单位 m
image_points:  图像里的 88 个二维像素点
camera_matrix: 相机内参 K
dist_coeffs:   畸变参数 D
```

输出是：

```text
rvec, tvec
```

它们表示：

```text
T_camera_board
```

也就是从棋盘坐标系变到相机坐标系的位姿。

行 202-205：如果 PnP 失败，跳过。

行 207：把旋转向量 `rvec` 转成旋转矩阵。

行 208：把旋转矩阵转 ROS 四元数。

行 209-218：创建 `PoseStamped`，填入位置和四元数。

行 212-214：`tvec` 的 x/y/z 就是棋盘原点在相机坐标系里的位置。

行 219：发布 `/checkerboard/cam_left/pose`。

行 220：可选发布 TF。

行 222-224：更新检测计数并打印日志。

### 6.9 TF 和日志

行 226-236：如果启用了 TF，把棋盘 pose 也发布成 TF。

行 238-242：控制 warning 日志频率，避免每帧都刷屏。

行 244-253：控制 pose 日志频率。

行 256-264：主函数，初始化 ROS、创建节点、进入 `spin()` 循环。

行 266-268：处理 Ctrl-C 和 ROS shutdown。

## 7. checkerboard_handeye_calibration.py 逐段解释

文件：

```text
/home/ligl/realsense_three_cameras/checkerboard_handeye_calibration.py
```

这个脚本负责采样并求解 `T_gripper_camera`。

### 7.1 依赖和手眼算法

行 1-22：导入 Python、OpenCV、NumPy、ROS2 和消息类型。

行 24-30：定义可选的 OpenCV 手眼算法：

```text
tsai
park
horaud
andreff
daniilidis
```

当前默认用的是 `tsai`：

```text
cv2.CALIB_HAND_EYE_TSAI
```

### 7.2 数据结构和数学转换

行 33-37：`TimedMessage`。它把收到的 ROS message 和收到时间绑在一起，用来判断数据是不是 stale。

行 39-64：四元数转旋转矩阵。

行 67-94：旋转矩阵转四元数。

行 97-108：旋转矩阵转 RPY，用于结果里显示 roll/pitch/yaw。

行 111-120：`pose_to_matrix()`。把 ROS `Pose` 转成 4x4 齐次变换矩阵：

```text
[ R  t ]
[ 0  1 ]
```

行 123-133：`matrix_payload()`。把 4x4 矩阵同时保存成：

```text
position
orientation_xyzw
rpy_rad
rpy_deg
matrix
```

### 7.3 订阅最新 robot pose 和 checkerboard pose

行 136：定义 `PairSampler`。

行 137-143：创建两个订阅：

```text
robot_pose_topic  -> /left/gripper_end_pos
target_pose_topic -> /checkerboard/cam_left/pose
```

行 144-148：收到消息时，只保存“最新一帧”和收到时间。

### 7.4 数据年龄和后台 ROS spin

行 151-160：计算和格式化消息年龄。你看到的：

```text
robot_age=0.00s target_age=0.02s
```

就是这里算出来的。

行 163-168：启动一个 ROS executor 线程，让 ROS 订阅在后台持续收消息，同时主线程可以等你按键。

### 7.5 参数解析

行 171-183：定义命令行参数。

关键参数：

```text
--mode eye_in_hand
--robot-pose-topic /left/gripper_end_pos
--target-pose-topic /checkerboard/cam_left/pose
--min-samples 15
--max-age-sec 2.0
--method tsai
--result-dir ...
```

### 7.6 主流程开始

行 186-194：解析参数、检查最少样本数、初始化 ROS、创建采样节点、启动后台 executor、创建 `samples` 列表。

行 196-203：打印交互提示。你看到的：

```text
Keys: Enter=capture, d=delete last, q=calculate, c=cancel
```

就是这里。

### 7.7 采样循环

行 205-267：主循环。

行 207-212：每次显示当前样本数和两个 topic 的数据年龄。

行 214：等待你输入。

行 218：如果你直接按 Enter，就尝试采样。

行 219-221：如果机器人 pose 或棋盘 pose 没有收到，就拒绝采样。

行 222-227：如果数据太旧，比如 `target_age > 2.0s`，也拒绝采样。

行 229：把当前 `/left/gripper_end_pos` 转成矩阵：

```text
T_base_gripper
```

行 230-233：根据模式决定传给 OpenCV 的 robot pose。

当前是 `eye_in_hand`，所以：

```text
robot_matrix_for_solver = robot_matrix
```

如果是 `eye_to_hand`，则会反过来取逆。

行 235：把 `/checkerboard/cam_left/pose` 转成矩阵：

```text
T_camera_board
```

行 236-243：把这一组样本保存进 `samples`。

行 244-250：打印这一组的平移信息。你之前看到的：

```text
captured 1: robot=[...], target=[...]
```

就是这里。

行 251-256：输入 `d` 删除最后一组样本。

行 257-261：输入 `q`，如果样本数够，就退出循环并开始计算。

行 262-264：输入 `c` 取消。

行 265-266：其他输入提示 unknown key。

### 7.8 调用 OpenCV 求手眼

行 268-271：从所有样本里拆出 OpenCV 需要的四组数据：

```text
r_gripper2base
t_gripper2base
r_target2cam
t_target2cam
```

对当前 `eye_in_hand` 来说：

```text
r_gripper2base/t_gripper2base 来自 T_base_gripper
r_target2cam/t_target2cam 来自 T_camera_board
```

行 273-279：真正调用：

```python
cv2.calibrateHandEye(...)
```

OpenCV 内部求的是经典 `A X = X B` 问题。这里的未知量 `X` 就是：

```text
T_gripper_camera
```

可以这样理解：

```text
T_base_gripper_i * X * T_camera_board_i = 固定不变的 T_base_board
```

多组姿态之间的相对运动会约束 `X`，所以需要让相机从不同角度、不同位置看同一块固定棋盘。

### 7.9 保存结果

行 280-282：把 OpenCV 返回的旋转和平移组装成 4x4 矩阵。

行 284-303：生成结果 JSON，包括：

```text
mode
method
robot_pose_topic
target_pose_topic
sample_count
result
samples
```

行 305-312：创建结果目录，并保存两个文件：

```text
时间戳_checkerboard_handeye.json
latest_checkerboard_handeye.json
```

行 314-319：把结果打印到终端。

行 320-325：关闭 ROS executor、销毁节点、shutdown。

行 328-335：程序入口和异常处理。

## 8. recompute_handeye_result.py 的作用

文件：

```text
/home/ligl/realsense_three_cameras/recompute_handeye_result.py
```

这个脚本用于剔除坏样本后重新计算。

你之前用它剔除了第 2 组样本。

行 15-21：定义 OpenCV 手眼算法。

行 24-78：把矩阵结果重新转成 position、quaternion、RPY、matrix。

行 81-84：解析 `--exclude` 参数，例如 `--exclude 2`。

行 87-97：解析命令行。

行 100-107：读取旧 JSON，剔除指定样本，并检查剩余样本不少于 5。

行 109-125：按原来的 mode 重新准备 OpenCV 输入。

行 127-133：再次调用 `cv2.calibrateHandEye()`。

行 134-143：组装新结果，记录 `excluded_sample_indexes`。

行 145-150：写入新的结果文件。如果带 `--write-latest`，也会覆盖 latest。

行 152-155：打印重算结果和保存路径。

## 9. validate_handeye_result.py 的作用

文件：

```text
/home/ligl/realsense_three_cameras/validate_handeye_result.py
```

它不重新求外参，只验证结果好不好。

核心验证公式在行 124-128：

```python
robot = load_matrix(sample["robot_matrix"])
camera_target = load_matrix(sample["target_matrix"])
fixed_target_poses.append(robot @ transform @ camera_target)
```

其中：

```text
robot         = T_base_gripper
transform     = T_gripper_camera
camera_target = T_camera_board
```

所以：

```text
robot @ transform @ camera_target
= T_base_gripper * T_gripper_camera * T_camera_board
= T_base_board
```

如果外参正确，所有样本算出来的 `T_base_board` 应该几乎一样。

行 131-133：计算所有棋盘位置的中心和每个样本到中心的距离。

行 135-140：计算所有棋盘旋转的平均值和每个样本的旋转误差。

行 145-181：输出 PASS/BORDERLINE/FAIL、位置误差、旋转误差、运动覆盖范围和最坏样本。

你原始 16 组里第 2 组坏，就是因为它在这个验证里造成了很大的 `T_base_board` 跳变。

## 10. visualize_handeye_application.py 的作用

文件：

```text
/home/ligl/realsense_three_cameras/visualize_handeye_application.py
```

它做应用验证，不重新标定。

行 31-36：和前面一样生成 8x11、20mm 棋盘三维点。

行 39-73：把 ROS pose 转成 4x4 矩阵。

行 76-79：把一组三维点乘上 4x4 变换矩阵。

行 82-87：解码 RGB 压缩图。

行 90-109：解码 depth 图。`16UC1` 深度图乘 `0.001`，从毫米变成米。

行 112-133：再次检测 RGB 图里有没有完整棋盘，用于可视化检查。

行 136-141：把相机坐标系下的三维点投影回像素平面。

行 144-173：`LiveCapture` 同时订阅：

```text
/cam_left/color/image_raw/compressed
/cam_left/color/camera_info
/left/gripper_end_pos
/checkerboard/cam_left/pose
/cam_left/aligned_depth_to_color/image_raw
```

行 364-380：解析参数。这里再次出现：

```text
pattern_cols = 8
pattern_rows = 11
square_size_m = 0.02
```

行 385-388：读取手眼标定结果 JSON，并取出：

```text
T_gripper_camera
```

行 390-394：确定各个 ROS topic 和输出目录。

行 397-410：等待 RGB、camera_info、robot_pose、checkerboard_pose 都收到。

行 412-414：如果有 depth topic，就继续等一帧 depth。这个是后面补的修正，避免太早出图导致 `depth_not_received`。

行 419-425：计算当前空间关系：

```text
t_base_gripper = T_base_gripper
t_camera_board = T_camera_board
t_base_camera  = T_base_gripper * T_gripper_camera
t_base_board   = T_base_camera * T_camera_board
```

行 427-429：生成棋盘模型、重新检测 RGB 角点、画 RGB overlay。

行 431-440：如果收到 depth，就把深度图反投影成点云，再用 `T_base_camera` 变换到 base。

行 442-444：把棋盘点、相机原点、夹爪原点都变到 base。

行 446-465：画 base 俯视图和侧视图。

行 467-496：生成 summary，包括本次的所有变换矩阵、点云状态、角点数量和输出路径。

行 498-516：保存图片和 summary。

## 11. 为什么要采 15 组

单组样本里未知量太多，无法可靠求出相机相对夹爪的固定外参。

多组样本提供的是“相对运动约束”：

```text
机械臂从姿态 i 动到姿态 j，夹爪运动了一段 A
相机看到棋盘的位姿也相应变化了一段 B
由于相机固定在夹爪上，两边的运动必须被同一个 X 连接
```

数学上就是：

```text
A X = X B
```

其中 `X = T_gripper_camera`。

姿态越丰富，约束越强。好的样本应该包含：

```text
不同位置
不同距离
不同 roll/pitch/yaw
棋盘始终完整清晰
没有反光、遮挡、过曝
机器人 pose 和棋盘 pose 都是新鲜数据
```

## 12. 怎么判断坏样本

坏样本通常来自：

```text
棋盘角点顺序错
棋盘被遮挡
图像模糊
反光
机械臂还在动就按 Enter
robot pose 和 checkerboard pose 时间不同步
cam_left/cam_right 序列号映射错
```

验证脚本的判断方式：

```text
对每个样本计算 T_base_board_i
如果某一组让棋盘在 base 里跳很远，它就是可疑样本
```

你之前第 2 组就是这种情况。剔除后结果从 FAIL 变成 PASS。

## 13. 当前结果为什么可以先用

剔除第 2 组后，当前 `latest_checkerboard_handeye.json` 的验证结果是：

```text
sample_count: 15
position error mean: 约 0.008 m
position error max:  约 0.0176 m
rotation error max:  约 1.03 deg
verdict: PASS
```

这说明在采样数据内部，固定棋盘被还原到 base 后比较稳定。

点云应用验证则进一步检查：

```text
RGB 棋盘能否检测到 88 个角点
depth 点云是否成功收到
点云、绿色棋盘网格、base 坐标关系是否合理
```

你最新那次点云验证：

```text
checkerboard found: true
corner_count: 88
pointcloud.status: ok
point_count: 4316
T_base_board z: 约 0.023 m
T_base_camera z: 约 0.286 m
```

这说明点云已经接入，当前静态姿态下变换链路是通的。

