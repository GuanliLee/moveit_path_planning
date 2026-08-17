# Three RealSense Camera Startup

This directory contains wrapper scripts for the three D435i cameras:

- `start_rgb_only.sh`: RGB only, 640x480@30fps. Recommended for stable checkerboard work.
- `start_rgb_depth_low_bandwidth.sh`: RGB + depth, low bandwidth.
- `start_calibration_rgb_depth_aligned.sh`: RGB + aligned depth for calibration capture.
- `capture_checkerboard_sample.sh`: Save one 8x11 checkerboard sample from all three cameras.
- `checkerboard_pose_node.py`: Publish checkerboard pose as `geometry_msgs/PoseStamped`.
- `start_checkerboard_pose.sh`: Start one checkerboard pose publisher.
- `start_checkerboard_pose_all.sh`: Start checkerboard pose publishers for all three cameras.
- `run_handeye_checkerboard.sh`: Collect robot pose + checkerboard pose pairs and run OpenCV hand-eye calibration.
- `validate_handeye_result.sh`: Check whether a hand-eye result is internally consistent.
- `recompute_handeye_result.sh`: Recompute from a saved result after excluding bad sample indexes.
- `visualize_handeye_application.sh`: Apply the hand-eye result to live ROS data and save validation images.
- `capture_touch_point.sh`: Record one checkerboard inner-corner touch point for the fixed-board method.
- `compute_fixed_board_extrinsic.sh`: Compute external camera extrinsic from one checkerboard snapshot plus P0/P1/P2.
- `stop_cameras.sh`: Stop RealSense ROS camera processes.
- `stop_checkerboard_pose.sh`: Stop checkerboard pose publishers.
- `check_cameras.sh`: Check USB devices, ROS publishers, and RGB frame counts.

The underlying ROS launch file is:

```bash
/opt/shared/xiangpc/realsense_ros2_ws/src/realsense_multi_camera_config/launch/three_d435i.launch.py
```

The current serial mapping is:

```text
cam_high  -> 261222077896
cam_left  -> 261222077859
cam_right -> 261222075959
```

## Start RGB Only

Use this mode for calibration image observation and chessboard corner work.

```bash
cd /home/ligl/realsense_three_cameras
./stop_cameras.sh
./start_rgb_only.sh
```

RGB topics:

```text
/cam_high/color/image_raw/compressed
/cam_left/color/image_raw/compressed
/cam_right/color/image_raw/compressed
```

## Start RGB + Depth

Use this only when depth is needed. The profile is intentionally low bandwidth:

```text
RGB:   424x240@6fps
Depth: 480x270@6fps
```

Start:

```bash
cd /home/ligl/realsense_three_cameras
./stop_cameras.sh
./start_rgb_depth_low_bandwidth.sh
```

Depth topics:

```text
/cam_high/depth/image_rect_raw
/cam_left/depth/image_rect_raw
/cam_right/depth/image_rect_raw
```

## Start Calibration Capture Mode

Use this when running `capture_checkerboard_sample.sh`. It publishes aligned depth:

```bash
cd /home/ligl/realsense_three_cameras
./stop_cameras.sh
./start_calibration_rgb_depth_aligned.sh
```

In another terminal, capture one sample:

```bash
cd /home/ligl/realsense_three_cameras
./capture_checkerboard_sample.sh sample_001
```

The sample is saved under:

```text
/home/ligl/agilex_xpc/calibration/snapshots/sample_001
```

## Verify

In another terminal:

```bash
cd /home/ligl/realsense_three_cameras
./check_cameras.sh
```

Expected result for a healthy RGB run:

```text
USB RealSense devices: 3 devices
RGB compressed publishers: publisher count 1 for all three cameras
5-second RGB frame count: non-zero for all three cameras
```

## Checkerboard Pose Topics

The checkerboard is:

```text
inner corners: 8 x 11
square size:   0.02 m
```

Start cameras first:

```bash
cd /home/ligl/realsense_three_cameras
./stop_cameras.sh
./start_rgb_only.sh
```

In another terminal, start checkerboard pose detection for one camera:

```bash
cd /home/ligl/realsense_three_cameras
./start_checkerboard_pose.sh cam_high
```

Or start all three:

```bash
cd /home/ligl/realsense_three_cameras
./start_checkerboard_pose_all.sh
```

Published topics:

```text
/checkerboard/cam_high/pose
/checkerboard/cam_left/pose
/checkerboard/cam_right/pose
```

Check one pose:

```bash
cd /home/ligl/realsense_three_cameras
./check_checkerboard_pose.sh cam_high
```

## Checkerboard Hand-Eye

This replaces the ArUco pose topic in the AgileX hand-eye flow with a checkerboard pose topic.

For an external fixed camera, for example `cam_high`, the checkerboard must be rigidly attached to the gripper or tool. Run:

```bash
cd /home/ligl/realsense_three_cameras
./run_handeye_checkerboard.sh eye_to_hand cam_high /left/gripper_end_pos
```

For a camera fixed on the gripper, for example `cam_left`, keep the checkerboard fixed on the table. Run:

```bash
cd /home/ligl/realsense_three_cameras
./run_handeye_checkerboard.sh eye_in_hand cam_left /left/gripper_end_pos
```

Collect 15-25 samples. Use:

```text
Enter  capture one sample
d      delete last sample
q      calculate after enough samples
c      cancel
```

The result is saved under:

```text
/home/ligl/agilex_xpc/calibration/results
```

Validate the latest result:

```bash
cd /home/ligl/realsense_three_cameras
./validate_handeye_result.sh
```

Recompute after excluding bad samples, for example sample 2:

```bash
cd /home/ligl/realsense_three_cameras
./recompute_handeye_result.sh /home/ligl/agilex_xpc/calibration/results/cam_left_eye_in_hand_checkerboard/latest_checkerboard_handeye.json 2
```

Generate live application validation images:

```bash
cd /home/ligl/realsense_three_cameras
./visualize_handeye_application.sh cam_left /left/gripper_end_pos
```

Outputs are saved under:

```text
/home/ligl/agilex_xpc/calibration/application_validation
```

Before collecting samples, confirm the robot pose topic is live:

```bash
cd /home/ligl/realsense_three_cameras
./check_robot_pose_topic.sh /left/gripper_end_pos
```

## Fixed Board External Camera

Use this path when the checkerboard stays fixed on the table and the external camera, for example `cam_high`, is fixed in the scene.

Capture a full-board image sample:

```bash
cd /home/ligl/realsense_three_cameras
./capture_checkerboard_sample.sh board_static_001
```

Touch three non-collinear inner corners. The examples below assume P0 is inner corner `(col=0,row=0)`, P1 is `(7,0)`, and P2 is `(0,10)`.

```bash
cd /home/ligl/realsense_three_cameras
./capture_touch_point.sh P0 0 0 /left/gripper_end_pos
./capture_touch_point.sh P1 7 0 /left/gripper_end_pos
./capture_touch_point.sh P2 0 10 /left/gripper_end_pos
```

Compute `T_base_camera`:

```bash
cd /home/ligl/realsense_three_cameras
./compute_fixed_board_extrinsic.sh board_static_001 cam_high
```

Results are saved under:

```text
/home/ligl/agilex_xpc/calibration/results/fixed_board
```

## Stop

```bash
cd /home/ligl/realsense_three_cameras
./stop_cameras.sh
./stop_checkerboard_pose.sh
```
