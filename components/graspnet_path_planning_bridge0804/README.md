# GraspNet Path Planning Bridge 0804

This workspace contains only the independent bridge described by
`/home/ligl/path_planning_service0804/graspnet_bridge_path_planning_three_layer_scheme(2).md`.

It does not modify:

- `/home/ligl/path_planning_service0804`
- the 121 GraspNet server

## Interfaces

Bridge services and topics:

- `/grasp_bridge/execute_task`
  - type: `graspnet_bridge_interfaces/srv/ExecuteTask`
  - one request maps to one `/plan_to_pose` request
- `/grasp_bridge/detection_result`
  - type: `graspnet_bridge_interfaces/msg/DetectionResult`
  - caches RGB, dRGB, bbox and mask from the upper state machine
- `/grasp_bridge/get_detection_result`
  - type: `graspnet_bridge_interfaces/srv/GetDetectionResult`
  - returns the last cached detection, or the last cached result for a target

Lower service used by the bridge:

- `/plan_to_pose`
  - type: `path_planning_interfaces/srv/PlanToPose`

## Coordinate Behavior

Every `tcp_target_pose` is the physical MoveIt planning TCP at
`link6 +Z 0.1358 m`, independent of `header.frame_id`. In particular, a raw
121 camera-frame grasp already describes that physical gripper-end center and
must not be shifted by the 76.5 mm difference between the calibration parent
and the planning TCP.

For a right-wrist camera GraspNet result from 121, use:

```text
tcp_target_pose.header.frame_id = cam_right_color_optical_frame
tcp_target_pose.pose = T_cam_right_color_optical_frame_tcp
```

For a left-wrist camera result, use:

```text
tcp_target_pose.header.frame_id = cam_left_color_optical_frame
tcp_target_pose.pose = T_cam_left_color_optical_frame_tcp
```

The bridge computes:

```text
T_world_camera = T_world_link6
               * T_link6_gripper_end
               * T_gripper_end_camera
T_world_tcp = T_world_camera * T_camera_tcp
T_world_link6_target = T_world_tcp * inverse(T_link6_tcp)
```

The hand-eye files provide `T_gripper_end_camera` and were calibrated with
`/<arm>/gripper_end_pos` as their parent:

- `/home/ligl/path_planning_service0804/calibration/extrinsics/cam_left_handeye.json`
- `/home/ligl/path_planning_service0804/calibration/extrinsics/cam_right_handeye.json`

`T_link6_gripper_end` is the 59.3 mm local-Z offset configured by
`link6_to_handeye_parent_m`; it is used only in the camera extrinsic chain.
`T_link6_tcp` is independently configured as the 135.8 mm MoveIt planning TCP
by `left/right_link6_to_tcp_xyz` and is used for every target frame.

## Build

```bash
cd /home/ligl/graspnet_path_planning_bridge0804
source /opt/ros/jazzy/setup.bash
source /home/ligl/path_planning_service0804/install/setup.bash
colcon build --symlink-install
```

## Run

Start the 0804 planning service first. Then run:

```bash
source /opt/ros/jazzy/setup.bash
source /home/ligl/path_planning_service0804/install/setup.bash
source /home/ligl/graspnet_path_planning_bridge0804/install/setup.bash
export ROS_DOMAIN_ID=66
ros2 launch graspnet_path_planning_bridge bridge.launch.py
```

## Example Task

```bash
ros2 service call /grasp_bridge/execute_task graspnet_bridge_interfaces/srv/ExecuteTask "{
  task_id: 'manual_001',
  task_type: 'GRASP',
  target_id: 'AD Milk',
  arm_name: 'right',
  scene_id: 1,
  tcp_target_pose: {
    header: {frame_id: 'cam_right_color_optical_frame'},
    pose: {
      position: {x: -0.03315033, y: -0.00965253, z: 0.31500003},
      orientation: {x: 0.17846218, y: 0.056136, z: -0.66415264, w: 0.72381025}
    }
  },
  gripper_command: true,
  gripper_opening_m: 0.0
}"
```

The bridge sends a `world` frame `right_link6` target to `/plan_to_pose`.
