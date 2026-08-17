import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped

from graspnet_path_planning_bridge.coordinate_converter import CoordinateConverter


PACKAGE_ROOT = Path(__file__).parents[1]


class _FakeLogger:
    def info(self, _message: str) -> None:
        pass

    def warn(self, _message: str) -> None:
        pass


class _FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: SimpleNamespace())


class _FakeNode:
    def __init__(self, parameters):
        self._parameters = parameters
        self._logger = _FakeLogger()
        self._clock = _FakeClock()

    def get_parameter(self, name: str):
        return SimpleNamespace(value=self._parameters[name])

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock


class _FakeBuffer:
    def __init__(self, world_from_link6: TransformStamped):
        self._world_from_link6 = world_from_link6
        self.calls = 0

    def lookup_transform(self, _target, _source, _time, timeout=None):
        del timeout
        self.calls += 1
        return self._world_from_link6


def _write_handeye(path: Path, translation=(0.0, 0.0, 0.0)) -> None:
    path.write_text(
        json.dumps(
            {
                "translation_m": list(translation),
                "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        ),
        encoding="utf-8",
    )


def _make_converter(
    tmp_path: Path,
    *,
    world_from_link6=None,
    left_handeye_translation=(0.0, 0.0, 0.0),
    right_handeye_translation=(0.0, 0.0, 0.0),
):
    left_handeye = tmp_path / "left.json"
    right_handeye = tmp_path / "right.json"
    _write_handeye(left_handeye, left_handeye_translation)
    _write_handeye(right_handeye, right_handeye_translation)
    parameters = {
        "world_frame": "world",
        "tf_timeout_s": 1.0,
        "left_link6_frame": "left_link6",
        "right_link6_frame": "right_link6",
        "left_camera_frame": "cam_left_color_optical_frame",
        "right_camera_frame": "cam_right_color_optical_frame",
        "left_handeye_file": str(left_handeye),
        "right_handeye_file": str(right_handeye),
        "link6_to_handeye_parent_m": 0.0593,
        "left_link6_to_tcp_xyz": [0.0, 0.0, 0.1358],
        "left_link6_to_tcp_xyzw": [0.0, 0.0, 0.0, 1.0],
        "right_link6_to_tcp_xyz": [0.0, 0.0, 0.1358],
        "right_link6_to_tcp_xyzw": [0.0, 0.0, 0.0, 1.0],
    }
    if world_from_link6 is None:
        world_from_link6 = TransformStamped()
        world_from_link6.transform.rotation.w = 1.0
    buffer = _FakeBuffer(world_from_link6)
    return CoordinateConverter(_FakeNode(parameters), buffer), buffer


def _identity_pose(frame_id: str) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.orientation.w = 1.0
    return pose


def test_camera_frame_uses_handeye_parent_only_for_camera(tmp_path: Path) -> None:
    converter, buffer = _make_converter(tmp_path)

    output = converter.tcp_pose_to_world_link6(
        "right",
        _identity_pose("cam_right_color_optical_frame"),
    )

    # With identity hand-eye, the camera is at link6 +59.3 mm. The camera-frame
    # target is nevertheless the 135.8 mm planning TCP, so target link6 is
    # 76.5 mm behind the current link6 along local Z.
    assert output.pose.position.x == pytest.approx(0.0)
    assert output.pose.position.y == pytest.approx(0.0)
    assert output.pose.position.z == pytest.approx(0.0593 - 0.1358)
    assert buffer.calls == 1


def test_world_frame_still_uses_135_8mm_planning_tcp(tmp_path: Path) -> None:
    converter, buffer = _make_converter(tmp_path)
    target = _identity_pose("world")
    target.pose.position.z = 0.4

    output = converter.tcp_pose_to_world_link6("right", target)

    assert output.pose.position.z == pytest.approx(0.4 - 0.1358)
    assert buffer.calls == 0


def test_camera_offset_follows_rotated_link6_local_z(tmp_path: Path) -> None:
    world_from_link6 = TransformStamped()
    world_from_link6.transform.translation.x = 0.4
    world_from_link6.transform.translation.y = -0.2
    world_from_link6.transform.translation.z = 0.5
    # +90 degrees about world Y: link6 local +Z points along world +X.
    world_from_link6.transform.rotation.y = 2.0 ** -0.5
    world_from_link6.transform.rotation.w = 2.0 ** -0.5
    converter, _buffer = _make_converter(
        tmp_path,
        world_from_link6=world_from_link6,
    )

    output = converter.tcp_pose_to_world_link6(
        "right",
        _identity_pose("cam_right_color_optical_frame"),
    )

    assert output.pose.position.x == pytest.approx(0.4 + 0.0593 - 0.1358)
    assert output.pose.position.y == pytest.approx(-0.2)
    assert output.pose.position.z == pytest.approx(0.5)
    assert output.pose.orientation.y == pytest.approx(2.0 ** -0.5)
    assert output.pose.orientation.w == pytest.approx(2.0 ** -0.5)


def test_left_and_right_camera_paths_use_their_own_handeye(tmp_path: Path) -> None:
    converter, buffer = _make_converter(
        tmp_path,
        left_handeye_translation=(0.01, 0.0, 0.0),
        right_handeye_translation=(0.0, 0.02, 0.0),
    )

    left = converter.tcp_pose_to_world_link6(
        "left",
        _identity_pose("cam_left_color_optical_frame"),
    )
    right = converter.tcp_pose_to_world_link6(
        "right",
        _identity_pose("cam_right_color_optical_frame"),
    )

    assert left.pose.position.x == pytest.approx(0.01)
    assert left.pose.position.y == pytest.approx(0.0)
    assert left.pose.position.z == pytest.approx(0.0593 - 0.1358)
    assert right.pose.position.x == pytest.approx(0.0)
    assert right.pose.position.y == pytest.approx(0.02)
    assert right.pose.position.z == pytest.approx(0.0593 - 0.1358)
    assert buffer.calls == 2


def test_config_keeps_handeye_parent_and_planning_tcp_separate() -> None:
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "bridge.yaml").read_text(encoding="utf-8")
    )["graspnet_path_planning_bridge"]["ros__parameters"]

    assert parameters["link6_to_handeye_parent_m"] == 0.0593
    assert parameters["left_link6_to_tcp_xyz"] == [0.0, 0.0, 0.1358]
    assert parameters["right_link6_to_tcp_xyz"] == [0.0, 0.0, 0.1358]
