"""Publish wrist-camera transforms with the calibrated gripper-end parent."""

from launch import LaunchDescription
from launch_ros.actions import Node


def _static_camera_transform(
    *,
    name: str,
    parent_frame: str,
    child_frame: str,
    xyz: tuple[float, float, float],
    xyzw: tuple[float, float, float, float],
) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        output="screen",
        arguments=[
            "--x", str(xyz[0]),
            "--y", str(xyz[1]),
            "--z", str(xyz[2]),
            "--qx", str(xyzw[0]),
            "--qy", str(xyzw[1]),
            "--qz", str(xyzw[2]),
            "--qw", str(xyzw[3]),
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
    )


def generate_launch_description() -> LaunchDescription:
    # The calibration JSON parent was /<arm>/gripper_end_pos at link6 local
    # +Z 0.0593 m.  These camera-link transforms therefore equal
    # T_link6_gripper_end * T_gripper_end_camera_link.  The orientations are
    # unchanged from the calibrated transforms; only local link6 Z gains
    # 0.0593 m compared with the old temporary publishers.
    left_camera = _static_camera_transform(
        name="left_link6_to_cam_left_link",
        parent_frame="left_link6",
        child_frame="cam_left_link",
        xyz=(-0.0803650469349, -0.0132890188904, 0.0370538332498),
        xyzw=(0.0152825309437, -0.566842958459, 0.0169204053293, 0.823510294154),
    )
    right_camera = _static_camera_transform(
        name="right_link6_to_cam_right_link",
        parent_frame="right_link6",
        child_frame="cam_right_link",
        xyz=(-0.0799958965501, -0.0111801199888, 0.0413995453556),
        xyzw=(0.00624794995294, -0.508822217858, 0.0361376478631, 0.86009010234),
    )
    return LaunchDescription([left_camera, right_camera])
