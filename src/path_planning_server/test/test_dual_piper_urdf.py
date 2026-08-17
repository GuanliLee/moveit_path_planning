import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


REPOSITORY_ROOT = Path(__file__).parents[3]
GENERATOR_FILE = REPOSITORY_ROOT / 'tools' / 'generate_dual_piper_urdf.py'
SOURCE_URDF = (
    REPOSITORY_ROOT
    / 'src'
    / 'piper_ros'
    / 'src'
    / 'piper_description'
    / 'urdf'
    / 'piper_description.urdf'
)
GENERATED_URDF = (
    REPOSITORY_ROOT
    / 'src'
    / 'path_planning_server'
    / 'generated'
    / 'dual_piper.urdf'
)
SEMANTIC_URDF = (
    REPOSITORY_ROOT
    / 'src'
    / 'path_planning_server'
    / 'config'
    / 'dual_piper.srdf'
)


def load_generator():
    spec = importlib.util.spec_from_file_location(
        'generate_dual_piper_urdf',
        GENERATOR_FILE,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_vector(element, attribute):
    return tuple(float(value) for value in element.attrib[attribute].split())


def named_elements(root, tag):
    return {
        element.attrib['name']: element
        for element in root.findall(tag)
    }


def transform(xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr,
         cy * sp * cr + sy * sr, xyz[0]),
        (sy * cp, sy * sp * sr + cy * cr,
         sy * sp * cr - cy * sr, xyz[1]),
        (-sp, cp * sr, cp * cr, xyz[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def multiply(left, right):
    return tuple(
        tuple(
            sum(left[row][inner] * right[inner][column]
                for inner in range(4))
            for column in range(4)
        )
        for row in range(4)
    )


def element_transform(element):
    origin = element.find('origin')
    if origin is None:
        return transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    return transform(
        parse_vector(origin, 'xyz'),
        parse_vector(origin, 'rpy'),
    )


def test_generated_urdf_matches_generator_and_preserves_original_arms():
    generator = load_generator()
    source_xml = SOURCE_URDF.read_text(encoding='utf-8')
    expected_xml = generator.generate_dual_urdf(
        source_xml,
        (0.0, 0.3, 0.0),
        (0.0, -0.3, 0.0),
    )
    actual_xml = GENERATED_URDF.read_text(encoding='utf-8').strip()
    assert actual_xml == expected_xml

    source_root = ET.fromstring(source_xml)
    generated_root = ET.fromstring(actual_xml)
    generated_links = named_elements(generated_root, 'link')
    generated_joints = named_elements(generated_root, 'joint')

    for side in ('left', 'right'):
        for source_element in source_root:
            if source_element.tag not in ('link', 'joint'):
                continue
            expected = generator.prefixed_copy(source_element, side)
            generated = (
                generated_links if source_element.tag == 'link'
                else generated_joints
            )[expected.attrib['name']]
            assert ET.tostring(generated) == ET.tostring(expected)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_wrist_d435i_assembly(side):
    root = ET.parse(GENERATED_URDF).getroot()
    links = named_elements(root, 'link')
    joints = named_elements(root, 'joint')

    expected_links = {
        f'{side}_camera_stand',
        f'{side}_d435i_camera_link',
        f'{side}_camera_bottom_screw_frame',
        f'{side}_camera_link',
    }
    assert expected_links <= links.keys()

    stand_joint = joints[f'{side}_camera_stand_joint']
    assert stand_joint.find('parent').attrib['link'] == f'{side}_link6'
    assert stand_joint.find('child').attrib['link'] == f'{side}_camera_stand'
    stand_origin = stand_joint.find('origin')
    assert parse_vector(stand_origin, 'xyz') == pytest.approx(
        (-0.013, 0.0, 0.026)
    )
    assert parse_vector(stand_origin, 'rpy') == pytest.approx(
        (0.0, 0.0, math.pi / 2.0)
    )

    d435i_joint = joints[f'{side}_d435i_camera_joint']
    assert d435i_joint.find('parent').attrib['link'] == (
        f'{side}_camera_stand'
    )
    assert d435i_joint.find('child').attrib['link'] == (
        f'{side}_d435i_camera_link'
    )
    assert parse_vector(d435i_joint.find('origin'), 'xyz') == pytest.approx(
        (-0.0301, 0.0475316082292132, -0.00359836534200768)
    )
    assert parse_vector(d435i_joint.find('origin'), 'rpy') == pytest.approx(
        (0.0, -1.0507963267948965, -math.pi / 2.0)
    )

    camera_link = links[f'{side}_camera_link']
    expected_camera_mesh = (
        'package://path_planning_server/meshes/wrist_camera/d435.dae'
    )
    assert camera_link.find('visual/geometry/mesh').attrib['filename'] == (
        expected_camera_mesh
    )
    assert parse_vector(
        camera_link.find('visual/origin'),
        'xyz',
    ) == pytest.approx((0.0043, -0.0175, 0.0))
    assert parse_vector(
        camera_link.find('visual/origin'),
        'rpy',
    ) == pytest.approx((math.pi / 2.0, 0.0, math.pi / 2.0))
    assert parse_vector(
        camera_link.find('collision/origin'),
        'xyz',
    ) == pytest.approx((-0.0085, -0.0175, 0.0))
    assert parse_vector(
        camera_link.find('collision/geometry/box'),
        'size',
    ) == pytest.approx(
        (0.02505, 0.09, 0.025)
    )

    stand_link = links[f'{side}_camera_stand']
    expected_stand_mesh = (
        'package://path_planning_server/'
        'meshes/wrist_camera/d435_camera_stand.dae'
    )
    assert stand_link.find('visual/geometry/mesh').attrib['filename'] == (
        expected_stand_mesh
    )
    assert stand_link.find('collision/geometry/mesh').attrib['filename'] == (
        expected_stand_mesh
    )


@pytest.mark.parametrize('side', ['left', 'right'])
def test_d435i_body_is_seated_on_aloha_camera_stand(side):
    root = ET.parse(GENERATED_URDF).getroot()
    links = named_elements(root, 'link')
    joints = named_elements(root, 'joint')

    actual = transform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    for joint_name in (
        f'{side}_d435i_camera_joint',
        f'{side}_camera_joint',
        f'{side}_camera_link_joint',
    ):
        actual = multiply(actual, element_transform(joints[joint_name]))
    actual = multiply(
        actual,
        element_transform(links[f'{side}_camera_link'].find('visual')),
    )

    # Preserve the final camera-body pose used by the Aloha wrist mount while
    # keeping the official D435i camera-link-to-mesh coordinate convention.
    expected = transform(
        (-0.045, 0.042, -0.004),
        (0.52, 0.0, 0.0),
    )
    expected = multiply(
        expected,
        transform((0.0106, 0.0175, 0.0125), (0.0, 0.0, 0.0)),
    )
    expected = multiply(
        expected,
        transform((0.0043, 0.0, 0.0), (0.0, 0.0, 0.0)),
    )

    for actual_row, expected_row in zip(actual, expected):
        assert actual_row == pytest.approx(expected_row, abs=1e-12)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_only_wrist_camera_mount_contacts_are_allowed(side):
    root = ET.parse(SEMANTIC_URDF).getroot()
    disabled_pairs = {
        frozenset((entry.attrib['link1'], entry.attrib['link2'])):
        entry.attrib['reason']
        for entry in root.findall('disable_collisions')
    }

    assert disabled_pairs[
        frozenset((
            f'{side}_gripper_base',
            f'{side}_camera_stand',
        ))
    ] == 'Mounted'
    assert disabled_pairs[
        frozenset((
            f'{side}_link5',
            f'{side}_camera_stand',
        ))
    ] == 'Adjacent'
    assert disabled_pairs[
        frozenset((
            f'{side}_camera_link',
            f'{side}_camera_stand',
        ))
    ] == 'Mounted'

    camera_links = {
        f'{side}_camera_stand',
        f'{side}_d435i_camera_link',
        f'{side}_camera_bottom_screw_frame',
        f'{side}_camera_link',
    }
    camera_pairs = {
        pair
        for pair in disabled_pairs
        if pair & camera_links
    }
    assert camera_pairs == {
        frozenset((
            f'{side}_gripper_base',
            f'{side}_camera_stand',
        )),
        frozenset((
            f'{side}_link5',
            f'{side}_camera_stand',
        )),
        frozenset((
            f'{side}_camera_link',
            f'{side}_camera_stand',
        )),
    }
